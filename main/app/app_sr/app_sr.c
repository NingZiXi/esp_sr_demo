/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Unlicense OR CC0-1.0
*/
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "app_sr.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_models.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "app_sr_handler.h"
#include "model_path.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "bsp.h"
#include "driver/i2s_std.h"

#define I2S_CHANNEL_NUM     (1)

static const char *TAG = "app_sr";

static model_iface_data_t       *model_data     = NULL; // 静态模型接口数据指针
static const esp_mn_iface_t     *multinet       = NULL; // 多网络接口指针
static const esp_afe_sr_iface_t *afe_handle     = NULL; // 音频前端接口指针
static QueueHandle_t            g_result_que    = NULL; // 结果队列句柄
static srmodel_list_t           *models         = NULL; // 模型列表指针

const char *cmd_phoneme[12] = {
    "da kai kong qi jing hua qi",      // 打开空气净化器
    "guan bi kong qi jing hua qi",      // 关闭空气净化器
    "da kai tai deng",                  // 打开台灯
    "guan bi tai deng",                 // 关闭台灯
    "tai deng tiao liang",              // 台灯调亮
    "tai deng tiao an",                 // 台灯调暗
    "da kai deng dai",                  // 打开灯带
    "guan bi deng dai",                 // 关闭灯带
    "bo fang yin yue",                  // 播放音乐
    "ting zhi bo fang",                 // 停止播放
    "da kai shi jian",                  // 打开时间
    "da kai ri li"                      // 打开日历
};

static void audio_feed_task(void *pvParam)
{
    size_t bytes_read = 0;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *) pvParam;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data); //获取每次采集的音频数据大小
    ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, 1);

    /* 分配音频缓冲区并检查结果 */
    int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (NULL == audio_buffer) {
        esp_system_abort("No mem for audio buffer");
    }

    while (true) {
        /* 从I2S总线读取音频数据 */
        // esp_err_t read_result = i2s_channel_read(rx_handle, audio_buffer, audio_chunksize * sizeof(int16_t) * I2S_CHANNEL_NUM, &bytes_read, portMAX_DELAY);
        esp_err_t read_result = bsp_i2s_read(audio_buffer, audio_chunksize);

        if (read_result != ESP_OK) {
            ESP_LOGE(TAG, "======== bsp_extra_i2s_read failed ==========");
        }

        /* 馈送音频流的样本到AFE_SR */
        afe_handle->feed(afe_data, audio_buffer);
    }

    /* 清理如果音频馈送结束 */
    afe_handle->destroy(afe_data);


    vTaskDelete(NULL);
}

static void audio_detect_task(void *pvParam)
{
    bool detect_flag = false;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *) pvParam;

    /* 检查音频数据是否与多网络具有相同的块大小 */
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    assert(mu_chunksize == afe_chunksize);  //确认音频的采样大小和模型是否相同

    ESP_LOGI(TAG, "------------detect start------------\n");

    while (true) {
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);  //从afe获取音频数据
        if (!res || res->ret_value == ESP_FAIL) {   //获取失败跳过本次循环
            ESP_LOGE(TAG, "fetch error!");
            continue;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {    //检测到唤醒词通过消息队列发送给处理任务
            ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_GREEN) "Wakeword detected");
            sr_result_t result = {
                .wakenet_mode = WAKENET_DETECTED,
                .state = ESP_MN_STATE_DETECTING,
                .command_id = 0,
            };
            xQueueSend(g_result_que, &result, 10);
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_GREEN) "Channel verified");
            detect_flag = true; //开启命令词识别
            afe_handle->disable_wakenet(afe_data);  //关闭唤醒词识别
        }

        /*命令词识别*/
        if (true == detect_flag) {
            esp_mn_state_t mn_state = ESP_MN_STATE_DETECTING;   //初始化检测状态

            mn_state = multinet->detect(model_data, res->data); //进行命令词匹配

            if (ESP_MN_STATE_DETECTING == mn_state) {
                continue;
            }

            if (ESP_MN_STATE_TIMEOUT == mn_state) { //如果一段时间没有检测到命令词，则重新开启唤醒词识别
                ESP_LOGW(TAG, "Time out");
                sr_result_t result = {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = mn_state,
                    .command_id = 0,
                };
                xQueueSend(g_result_que, &result, 10);  //通过消息队列发送结果
                afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
                continue;
            }

            if (ESP_MN_STATE_DETECTED == mn_state) {    //如果匹配到命令词
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    ESP_LOGI(TAG, "TOP %d, command_id: %d, phrase_id: %d, prob: %f",
                            i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->prob[i]);
                }
                int sr_command_id = mn_result->command_id[0];
                ESP_LOGI(TAG, "Deteted command : %d", sr_command_id);   //打印command_id

                sr_result_t result = {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = mn_state,
                    .command_id = sr_command_id,
                };
                xQueueSend(g_result_que, &result, 10);  //command_id发送给处理任务
#if !SR_CONTINUE_DET
                afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
#endif
                continue;
            }
            ESP_LOGE(TAG, "Exception unhandled");
        }
    }

    /* Clean up if audio feed ends */
    afe_handle->destroy(afe_data);

    /* Task never returns */
    vTaskDelete(NULL);
}

esp_err_t app_sr_start(void)
{

    /*创建afe对象*/
    g_result_que = xQueueCreate(1, sizeof(sr_result_t));    
    ESP_RETURN_ON_FALSE(NULL != g_result_que, ESP_ERR_NO_MEM, TAG, "Failed create result queue");

    models = esp_srmodel_init("model");     //加载模型

    afe_handle = &ESP_AFE_SR_HANDLE;        //初始化声学前端句柄
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();     //afe配置结构体使用默认配置

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);    //过滤出配置好的唤醒词
    afe_config.aec_init = false;    //关掉回声消除
    afe_config.pcm_config.total_ch_num=1;   //总通道数
    afe_config.pcm_config.mic_num=1;        //麦克风通道数
    afe_config.pcm_config.ref_num=0;        //参考回路通道数


    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);  //创建afe对象
    ESP_LOGI(TAG, "load wakenet:%s", afe_config.wakenet_model_name);    //打印唤醒词的名字

    // for (int i = 0; i < models->num; i++) {
    //     ESP_LOGI(TAG, "Current Model:%s", models->model_name[i]);
    // }

    /*加载命令词模型*/
    char *mn_name = esp_srmodel_filter(models, ESP_MN_CHINESE, NULL);   //过滤出之前配置好的命令词模型
    if (NULL == mn_name) {
        ESP_LOGE(TAG, "No multinet model found");
        return ESP_FAIL;
    }

    multinet = esp_mn_handle_from_name(mn_name);    //通过模型名字创建命令词句柄
    model_data = multinet->create(mn_name, 5760);   //加载模型数据5760为触发超时的持续时间
    ESP_LOGI(TAG, "load multinet:%s", mn_name);

    esp_mn_commands_clear();    //清空现有的命令列表

    for (int i = 0; i < sizeof(cmd_phoneme) / sizeof(cmd_phoneme[0]); i++) {    //将数组的指令逐个添加到模型
        esp_mn_commands_add(i, (char *)cmd_phoneme[i]);
    }

    esp_mn_commands_update();   //更新命令词列表

    //打印当前激活的命令列表
    esp_mn_commands_print();    
    multinet->print_active_speech_commands(model_data);

    //音频采集任务
    BaseType_t ret_val = xTaskCreatePinnedToCore(audio_feed_task, "Feed Task", 8 * 1024, afe_data, 5, NULL, 1);
    ESP_RETURN_ON_FALSE(pdPASS == ret_val, ESP_FAIL, TAG,  "Failed create audio feed task");

    //检测音频数据
    ret_val = xTaskCreatePinnedToCore(audio_detect_task, "Detect Task", 4 * 1024, afe_data, 5, NULL, 0);
    ESP_RETURN_ON_FALSE(pdPASS == ret_val, ESP_FAIL, TAG,  "Failed create audio detect task");

    //处理音频检测结果
    ret_val = xTaskCreatePinnedToCore(sr_handler_task, "SR Handler Task", 4 * 1024, g_result_que, 1, NULL, 1);
    ESP_RETURN_ON_FALSE(pdPASS == ret_val, ESP_FAIL, TAG,  "Failed create audio handler task");

    return ESP_OK;
}

esp_err_t app_sr_reset_command_list(char *command_list)
{
    char *err_id = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(NULL != err_id, ESP_ERR_NO_MEM, TAG,  "memory is not enough");
    free(err_id);
    return ESP_OK;
}
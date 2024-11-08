/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_check.h"
#include "app_sr.h"
#include "app_sr_handler.h"
#include "esp_afe_sr_iface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "app_sr_handler";

/* SR命令词处理结果任务 */
void sr_handler_task(void *pvParam)
{
    QueueHandle_t xQueue = (QueueHandle_t) pvParam;

    while (true) {
        sr_result_t result;
        xQueueReceive(xQueue, &result, portMAX_DELAY);

        ESP_LOGI(TAG, "命令ID: %d, 唤醒模式: %d, 状态: %d", result.command_id, result.wakenet_mode, result.state);

        if (ESP_MN_STATE_TIMEOUT == result.state) { // 唤醒词检测超时
            ESP_LOGI(TAG, "超时");

            continue;
        }

        if (WAKENET_DETECTED == result.wakenet_mode) {  // 检测到唤醒词
            ESP_LOGI(TAG, "检测到唤醒词");

            continue;
        }

        if (ESP_MN_STATE_DETECTED & result.state) { // 如果识别到命令词
            ESP_LOGI(TAG, "识别到命令词");

            switch (result.command_id) {
                case 0:
                    ESP_LOGI(TAG, "打开空气净化器");
                    break;
                case 1:
                    ESP_LOGI(TAG, "关闭空气净化器");
                    break;
                case 2:
                    ESP_LOGI(TAG, "打开灯");
                    break;
                case 3:
                    ESP_LOGI(TAG, "关闭灯");
                    break;
                case 4:
                    ESP_LOGI(TAG, "调亮灯");
                    break;
                case 5:
                    ESP_LOGI(TAG, "调暗灯");
                    break;
                case 6:
                    ESP_LOGI(TAG, "打开LED灯带");
                    break;
                case 7:
                    ESP_LOGI(TAG, "关闭LED灯带");
                    break;
                case 8:
                    ESP_LOGI(TAG, "播放音乐");
                    break;
                case 9:
                    ESP_LOGI(TAG, "停止音乐");
                    break;
                case 10:
                    ESP_LOGI(TAG, "显示时间");
                    break;
                case 11:
                    ESP_LOGI(TAG, "显示日历");
                    break;
                default:
                    break;
            }
            /* **************** 在此处注册命令回调 **************** */
        }
    }

    vTaskDelete(NULL);
}
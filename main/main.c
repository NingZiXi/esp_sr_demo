#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_sr.h"
#include "bsp.h"


void app_main(void) {
    bsp_init(); // 初始化硬件
    app_sr_start();
}

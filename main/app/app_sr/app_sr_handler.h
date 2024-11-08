/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handles speech recognition results
 *
 * This task function waits for speech recognition results from a queue, logs the results, and performs actions based on the results.
 * If a timeout occurs, it simply logs the timeout and continues waiting for the next result.
 * If a wakenet detection occurs, it plays a greeting and continues waiting for the next result.
 * If a multinet detection occurs, it performs an action based on the command ID opening the medicine box
 *
 * @param pvParam A pointer to the queue from which to receive speech recognition results. This should be a `QueueHandle_t`.
 */
void sr_handler_task(void *pvParam);

#ifdef __cplusplus
}
#endif
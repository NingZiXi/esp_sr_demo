#ifndef __BSP_H
#define __BSP_H

#include "driver/i2s_std.h"

#define I2S_SAMPLE_RATE 16000

extern i2s_chan_handle_t tx_handle;
extern i2s_chan_handle_t rx_handle;


#ifdef __cplusplus
extern "C" {
#endif


void bsp_i2s_init(void);
void bsp_init(void);
esp_err_t bsp_i2s_read(int16_t* dest, int samples);
#ifdef __cplusplus
}
#endif

#endif
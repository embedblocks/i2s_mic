#pragma once
#include "driver/i2s_common.h"
typedef enum { I2S_SLOT_MODE_MONO = 1, I2S_SLOT_MODE_STEREO = 2 } i2s_slot_mode_t;
typedef int i2s_data_bit_width_t;
#define I2S_GPIO_UNUSED (-1)
typedef struct { uint32_t rate; } i2s_std_clk_config_t;
typedef struct { int bits; i2s_slot_mode_t mode; } i2s_std_slot_config_t;
typedef struct { int mclk, bclk, ws, dout, din; struct { bool mclk_inv, bclk_inv, ws_inv; } invert_flags; } i2s_std_gpio_config_t;
typedef struct { i2s_std_clk_config_t clk_cfg; i2s_std_slot_config_t slot_cfg; i2s_std_gpio_config_t gpio_cfg; } i2s_std_config_t;
#define I2S_STD_CLK_DEFAULT_CONFIG(r) { .rate = (r) }
#define I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(b, m) { .bits = (b), .mode = (m) }
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t h, const i2s_std_config_t *cfg);

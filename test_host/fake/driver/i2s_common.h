#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
typedef struct sim_chan *i2s_chan_handle_t;
typedef enum { I2S_ROLE_MASTER } i2s_role_t;
typedef struct { void *dma_buf; size_t size; } i2s_event_data_t;
typedef bool (*i2s_isr_callback_t)(i2s_chan_handle_t, i2s_event_data_t *, void *);
typedef struct { i2s_isr_callback_t on_recv, on_recv_q_ovf, on_sent, on_send_q_ovf; } i2s_event_callbacks_t;
typedef struct { int id; i2s_role_t role; uint32_t dma_desc_num; uint32_t dma_frame_num; } i2s_chan_config_t;
#define I2S_CHANNEL_DEFAULT_CONFIG(n, r) { .id = (n), .role = (r), .dma_desc_num = 6, .dma_frame_num = 240 }
typedef struct { uint32_t total_dma_buf_size; } i2s_chan_info_t;
esp_err_t i2s_new_channel(const i2s_chan_config_t *cfg, i2s_chan_handle_t *tx, i2s_chan_handle_t *rx);
esp_err_t i2s_del_channel(i2s_chan_handle_t h);
esp_err_t i2s_channel_enable(i2s_chan_handle_t h);
esp_err_t i2s_channel_disable(i2s_chan_handle_t h);
esp_err_t i2s_channel_read(i2s_chan_handle_t h, void *dst, size_t size, size_t *n, uint32_t timeout_ms);
esp_err_t i2s_channel_get_info(i2s_chan_handle_t h, i2s_chan_info_t *info);
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t h, const i2s_event_callbacks_t *cbs, void *ud);

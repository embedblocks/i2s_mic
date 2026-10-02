#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "unity.h"
#include "sdkconfig.h"
#include "test_i2s_mic_common.h"

i2s_mic_config_t test_mic_config(void)
{
    i2s_mic_config_t cfg = {
        .sample_rate = TEST_SAMPLE_RATE,
        .bits_per_sample = 32,
        .channel_count = 2,
        .slot_mode = I2S_SLOT_MODE_STEREO,
        .gpio_bck = CONFIG_I2S_MIC_TEST_GPIO_BCK,
        .gpio_ws = CONFIG_I2S_MIC_TEST_GPIO_WS,
        .gpio_data = CONFIG_I2S_MIC_TEST_GPIO_DATA,
        .port = CONFIG_I2S_MIC_TEST_PORT,
        .dma_buffer_count = TEST_DMA_COUNT,
        .dma_buffer_size = TEST_BUF_BYTES,
    };
    return cfg;
}

void test_mic_up(void)
{
    i2s_mic_config_t cfg = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&cfg));
    TEST_ASSERT_EQUAL(TEST_BUF_BYTES, i2s_mic_get_buffer_size());
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
}

void test_mic_down(void)
{
    esp_err_t e = i2s_mic_stop();
    TEST_ASSERT(e == ESP_OK || e == ESP_ERR_INVALID_STATE);
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

static void reader_task(void *arg)
{
    test_reader_t *r = arg;

    while (1) {
        size_t n = 12345;
        esp_err_t e = i2s_mic_read(r->buf, TEST_BUF_BYTES, &n, r->timeout_ms);
        if (e == ESP_OK) {
            if (n != TEST_BUF_BYTES) {
                r->bad++;
            }
            if (r->ok == 0) {
                r->first_ok_us = esp_timer_get_time();
            }
            r->ok++;
            if (r->stall_after > 0 && r->ok == r->stall_after) {
                vTaskDelay(pdMS_TO_TICKS(r->stall_ms));
            }
            continue;
        }
        if (n != 0) {
            r->bad++;
        }
        if (e == ESP_ERR_TIMEOUT) {
            r->timeouts++;
            continue;
        }
        if (e == ESP_ERR_INVALID_STATE && r->stop_flag && !*r->stop_flag) {
            r->rejected++;
            vTaskDelay(1);
            continue;
        }
        r->last = e;
        break;
    }

    r->exit_us = esp_timer_get_time();
    xSemaphoreGive(r->done);
    vTaskDelete(NULL);
}

void test_reader_start(test_reader_t *r, uint32_t timeout_ms, UBaseType_t prio)
{
    int stall_after = r->stall_after;
    uint32_t stall_ms = r->stall_ms;
    volatile bool *stop_flag = r->stop_flag;
    memset(r, 0, sizeof(*r));
    r->stall_after = stall_after;
    r->stall_ms = stall_ms;
    r->stop_flag = stop_flag;
    r->timeout_ms = timeout_ms;
    r->last = ESP_OK;
    r->done = xSemaphoreCreateBinary();
    r->buf = heap_caps_malloc(TEST_BUF_BYTES, MALLOC_CAP_8BIT);
    TEST_ASSERT_NOT_NULL(r->done);
    TEST_ASSERT_NOT_NULL(r->buf);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(reader_task, "mic_reader", 3072, r,
                                          prio ? prio : TEST_READER_PRIO, NULL));
}

bool test_reader_wait(test_reader_t *r, uint32_t ms)
{
    if (xSemaphoreTake(r->done, pdMS_TO_TICKS(ms)) != pdTRUE) {
        return false;
    }
    vSemaphoreDelete(r->done);
    heap_caps_free(r->buf);
    r->done = NULL;
    r->buf = NULL;
    return true;
}

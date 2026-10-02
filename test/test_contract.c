/*
 * [contract] What each call returns: config validation, lifecycle state
 * errors, the read contract (length, not-running, timeouts, one reader).
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "unity.h"
#include "test_i2s_mic_common.h"

static uint8_t s_buf[TEST_BUF_BYTES];

TEST_CASE("init rejects invalid configs", "[contract]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_init(NULL));

    i2s_mic_config_t c = test_mic_config();
    c.bits_per_sample = 24;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_init(&c));

    c = test_mic_config();
    c.channel_count = 1;                       /* mismatch with STEREO */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_init(&c));

    c = test_mic_config();
    c.sample_rate = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_init(&c));

    c = test_mic_config();
    c.dma_buffer_count = 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_init(&c));

    c = test_mic_config();
    c.dma_buffer_size = TEST_BUF_BYTES + 4;    /* not a whole 8-byte frame */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_init(&c));

    /* none of the failures above may leave the component initialized */
    TEST_ASSERT_EQUAL(0, i2s_mic_get_buffer_size());
}

TEST_CASE("init rejects a DMA buffer the driver would shrink", "[contract]")
{
    i2s_mic_config_t c = test_mic_config();
    c.dma_buffer_size = 4096;                  /* 512 stereo 32-bit frames: over the 4092-byte cap */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, i2s_mic_init(&c));
    TEST_ASSERT_EQUAL(0, i2s_mic_get_buffer_size());

    /* the failed init must have released the I2S channel */
    c.dma_buffer_size = 4088;                  /* 511 frames: fits */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));
    TEST_ASSERT_EQUAL(4088, i2s_mic_get_buffer_size());
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

TEST_CASE("lifecycle calls in the wrong state return INVALID_STATE", "[contract]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_stop());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_deinit());

    i2s_mic_config_t c = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_init(&c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_stop());
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_start());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_deinit());   /* still running */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

TEST_CASE("read returns at once when not running", "[contract]")
{
    size_t n = 7;
    int64_t t0;

    /* not initialized */
    t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, portMAX_DELAY));
    TEST_ASSERT_LESS_THAN_UINT32(2000, (uint32_t)(esp_timer_get_time() - t0));
    TEST_ASSERT_EQUAL(0, n);

    /* initialized, not started */
    i2s_mic_config_t c = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));
    n = 7;
    t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, portMAX_DELAY));
    TEST_ASSERT_LESS_THAN_UINT32(2000, (uint32_t)(esp_timer_get_time() - t0));
    TEST_ASSERT_EQUAL(0, n);

    /* running: one good read */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, 1000));
    TEST_ASSERT_EQUAL(TEST_BUF_BYTES, n);

    /* stopped */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
    n = 7;
    t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, portMAX_DELAY));
    TEST_ASSERT_LESS_THAN_UINT32(2000, (uint32_t)(esp_timer_get_time() - t0));
    TEST_ASSERT_EQUAL(0, n);

    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

TEST_CASE("read rejects a wrong length and a NULL buffer", "[contract]")
{
    test_mic_up();
    size_t n = 7;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, i2s_mic_read(s_buf, TEST_BUF_BYTES - TEST_BYTES_FRAME, &n, 100));
    TEST_ASSERT_EQUAL(0, n);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, i2s_mic_read(s_buf, TEST_BUF_BYTES * 2, &n, 100));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, i2s_mic_read(NULL, TEST_BUF_BYTES, &n, 100));
    /* bytes_read may be NULL */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_read(s_buf, TEST_BUF_BYTES, NULL, 1000));
    test_mic_down();
}

TEST_CASE("zero and short timeouts return TIMEOUT without blocking", "[contract]")
{
    test_mic_up();
    size_t n;

    /* drain whatever is queued, then we are at most one period from the
     * next buffer */
    for (int i = 0; i < TEST_QUEUE_DEPTH + 1; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, 1000));
    }

    int timeouts = 0;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < 20; i++) {
        esp_err_t e = i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, 0);
        TEST_ASSERT(e == ESP_OK || e == ESP_ERR_TIMEOUT);
        if (e == ESP_ERR_TIMEOUT) {
            TEST_ASSERT_EQUAL(0, n);
            timeouts++;
        }
    }
    int64_t dt = esp_timer_get_time() - t0;
    printf("20 zero-timeout reads: %d timeouts in %ld us\n", timeouts, (long)dt);
    TEST_ASSERT_GREATER_OR_EQUAL(15, timeouts);
    TEST_ASSERT_LESS_THAN_UINT32(20000, (uint32_t)dt);          /* nowhere near a buffer period each */

    /* a timeout shorter than one tick must not hang either */
    t0 = esp_timer_get_time();
    esp_err_t e = i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, 1);
    TEST_ASSERT(e == ESP_OK || e == ESP_ERR_TIMEOUT);
    TEST_ASSERT_LESS_THAN_UINT32((uint32_t)TEST_PERIOD_US, (uint32_t)(esp_timer_get_time() - t0));

    /* a timeout longer than one period always gets a buffer */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, 2 * TEST_PERIOD_MS + 20));
    test_mic_down();
}

TEST_CASE("a second concurrent reader is rejected", "[contract]")
{
    test_mic_up();
    volatile bool stopping = false;
    test_reader_t r = { .stop_flag = &stopping };
    test_reader_start(&r, portMAX_DELAY, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* The reader spends almost all its time blocked inside i2s_mic_read(),
     * so nearly every attempt from this task must be rejected. */
    int rejected = 0, total = 50;
    size_t n;
    for (int i = 0; i < total; i++) {
        if (i2s_mic_read(s_buf, TEST_BUF_BYTES, &n, 1) == ESP_ERR_INVALID_STATE) {
            rejected++;
        }
        vTaskDelay(1);
    }
    printf("second reader rejected %d/%d times\n", rejected, total);
    TEST_ASSERT_GREATER_OR_EQUAL(total * 9 / 10, rejected);

    stopping = true;
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
    TEST_ASSERT_TRUE(test_reader_wait(&r, 1000));
    TEST_ASSERT_EQUAL(0, r.bad);
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

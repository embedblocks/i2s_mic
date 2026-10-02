/*
 * Shared configuration and helpers for the i2s_mic hardware tests.
 *
 * All tests use the same capture format: 16 kHz, stereo, 32-bit slots,
 * 480 frames per DMA buffer (3840 bytes, 30 ms), 6 DMA buffers. The driver
 * can therefore hold 5 finished buffers (150 ms) for a reader that falls
 * behind. i2s_mic waits in slices of two buffer periods (60 ms).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "i2s_mic.h"

#define TEST_SAMPLE_RATE   16000
#define TEST_FRAMES        480
#define TEST_BYTES_FRAME   8                                   /* stereo, 32-bit */
#define TEST_BUF_BYTES     (TEST_FRAMES * TEST_BYTES_FRAME)   /* 3840 */
#define TEST_DMA_COUNT     6
#define TEST_QUEUE_DEPTH   (TEST_DMA_COUNT - 1)                /* 5 */
#define TEST_PERIOD_US     ((int64_t)TEST_FRAMES * 1000000 / TEST_SAMPLE_RATE)  /* 30000 */
#define TEST_PERIOD_MS     ((uint32_t)(TEST_PERIOD_US / 1000))                  /* 30 */
#define TEST_SLICE_MS      (2 * TEST_PERIOD_MS)                                 /* 60 */

#define TEST_READER_PRIO   5

/* A config that every test can start from. */
i2s_mic_config_t test_mic_config(void);

/* init() + start() with test_mic_config(), asserting success. */
void test_mic_up(void);

/* stop() (if running) + deinit(), asserting success. */
void test_mic_down(void);

/* ---- Reader task --------------------------------------------------------
 *
 * Calls i2s_mic_read() in a loop until it gets ESP_ERR_INVALID_STATE (the
 * normal way a reader learns capture has stopped) or an unexpected error.
 * Optionally stalls once after `stall_after` successful reads.
 */
typedef struct {
    /* settings */
    uint32_t timeout_ms;
    int stall_after;            /* 0 = never stall */
    uint32_t stall_ms;
    /* If set, ESP_ERR_INVALID_STATE only ends the loop once *stop_flag is
     * true; before that it is counted in `rejected` and retried. Used when
     * another task deliberately competes for the reader slot. */
    volatile bool *stop_flag;

    /* results, valid after test_reader_wait() returns true */
    volatile int ok;            /* full buffers received */
    volatile int timeouts;
    volatile int bad;           /* ESP_OK with a short count, or error with count != 0 */
    volatile int rejected;      /* INVALID_STATE retried because *stop_flag was false */
    volatile esp_err_t last;    /* error that ended the loop */
    volatile int64_t first_ok_us;
    volatile int64_t exit_us;   /* esp_timer time when the task left its loop */

    /* internal */
    SemaphoreHandle_t done;
    uint8_t *buf;
} test_reader_t;

/* Create the reader task at TEST_READER_PRIO (or `prio` if non-zero). */
void test_reader_start(test_reader_t *r, uint32_t timeout_ms, UBaseType_t prio);

/* Wait up to `ms` for the reader to exit. Returns false if it is still
 * running (stuck). Frees the reader's resources only when it has exited. */
bool test_reader_wait(test_reader_t *r, uint32_t ms);

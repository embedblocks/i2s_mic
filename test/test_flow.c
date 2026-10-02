/*
 * [flow] Audio flow and loss accounting on real hardware.
 *
 * Without stamped buffers, loss is checked by bookkeeping: over a run of
 * `elapsed` microseconds the hardware completes about elapsed / period
 * buffers. Every one of them must end up either read by the reader or
 * counted by i2s_mic_get_overflow_count(). So
 *
 *     received + lost  ~=  elapsed / period
 *
 * where the slack covers the buffer being filled at stop() and the few
 * still queued (normally 0-1 when the reader is keeping up at the end).
 * If the counter missed or invented losses, this sum would drift.
 */
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "unity.h"
#include "test_i2s_mic_common.h"

typedef struct {
    int received;
    int lost;
    int expected;       /* elapsed / period */
    int bad;
    int timeouts;
} flow_result_t;

static void run_capture(int stall_after, uint32_t stall_ms, uint32_t run_ms, flow_result_t *res)
{
    i2s_mic_config_t c = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));

    test_reader_t r = { .stall_after = stall_after, .stall_ms = stall_ms };

    int64_t t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
    test_reader_start(&r, portMAX_DELAY, 0);

    vTaskDelay(pdMS_TO_TICKS(run_ms));

    uint32_t lost = i2s_mic_get_overflow_count();
    int64_t t1 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
    TEST_ASSERT_TRUE_MESSAGE(test_reader_wait(&r, 1000), "reader did not exit after stop()");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.last);
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());

    res->received = r.ok;
    res->lost = (int)lost;
    res->expected = (int)((t1 - t0) / TEST_PERIOD_US);
    res->bad = r.bad;
    res->timeouts = r.timeouts;
}

static void check_accounting(const flow_result_t *res)
{
    int sum = res->received + res->lost;
    printf("received %d + lost %d = %d, hardware completed ~%d buffers\n",
           res->received, res->lost, sum, res->expected);
    TEST_ASSERT_EQUAL_MESSAGE(0, res->bad, "a read returned ESP_OK with a short count");
    TEST_ASSERT_EQUAL_MESSAGE(0, res->timeouts, "portMAX_DELAY read timed out");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(res->expected + 1, sum, "more buffers than the hardware produced");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(res->expected - 3, sum, "buffers missing from both counts");
}

TEST_CASE("reader that keeps up loses nothing for 3 s", "[flow]")
{
    flow_result_t res;
    run_capture(0, 0, 3000, &res);
    check_accounting(&res);
    TEST_ASSERT_EQUAL_MESSAGE(0, res.lost, "overflow count rose while the reader kept up");
    TEST_ASSERT_GREATER_OR_EQUAL(res.expected - 3, res.received);
}

/* Model: during a stall of S ms about S / period buffers finish. The
 * driver queue holds TEST_QUEUE_DEPTH of them; the rest are dropped.
 * The tolerance of +-1 covers tick rounding of the stall and where in a
 * period it starts. If a board consistently lands one outside, record the
 * measured values and adjust the tolerance, not the model. */
static void stall_case(uint32_t stall_ms)
{
    flow_result_t res;
    run_capture(10, stall_ms, 10 * TEST_PERIOD_MS + stall_ms + 1000, &res);

    int model = (int)((stall_ms + TEST_PERIOD_MS - 1) / TEST_PERIOD_MS) - TEST_QUEUE_DEPTH;
    if (model < 0) {
        model = 0;
    }
    printf("STALL %u ms: lost %d buffers (model %d)\n", (unsigned)stall_ms, res.lost, model);
    check_accounting(&res);
    TEST_ASSERT_INT_WITHIN_MESSAGE(1, model, res.lost, "loss differs from the queue-depth model");
}

TEST_CASE("stall of 100 ms is absorbed by the driver queue", "[flow]")
{
    stall_case(100);
}

TEST_CASE("stall of 300 ms loses about 5 buffers, all counted", "[flow]")
{
    stall_case(300);
}

TEST_CASE("stall of 600 ms loses about 15 buffers, all counted", "[flow]")
{
    stall_case(600);
}

TEST_CASE("start() resets the overflow count and discards stale audio", "[flow]")
{
    test_mic_up();
    uint8_t *buf = malloc(TEST_BUF_BYTES);
    TEST_ASSERT_NOT_NULL(buf);

    /* nobody reads for 500 ms: about 16 buffers finish, 5 fit in the queue */
    vTaskDelay(pdMS_TO_TICKS(500));
    uint32_t lost = i2s_mic_get_overflow_count();
    printf("unread for 500 ms: overflow %u\n", (unsigned)lost);
    TEST_ASSERT_GREATER_THAN_UINT32(5, lost);

    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
    TEST_ASSERT_EQUAL(lost, i2s_mic_get_overflow_count());   /* survives stop() */
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
    TEST_ASSERT_EQUAL(0, i2s_mic_get_overflow_count());

    /* the queue was emptied by start(): nothing is ready yet */
    size_t n;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, i2s_mic_read(buf, TEST_BUF_BYTES, &n, 0));
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_read(buf, TEST_BUF_BYTES, &n, 1000));

    free(buf);
    test_mic_down();
}

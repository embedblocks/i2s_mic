/*
 * [lifecycle] start/stop/deinit while a reader task is active: stop()
 * latency, stop() under CPU load, many start/stop cycles, deinit refusal.
 *
 * Expected timings (30 ms buffers, 60 ms wait slices):
 *   - stop() waits for a driver copy already in progress; the DMA keeps
 *     running until then, so it returns within about one buffer period.
 *   - A reader blocked with portMAX_DELAY is woken by the next DMA
 *     completion (<= 1 period). In the rare case that its read starts just
 *     after stop() finished, it returns after one wait slice instead.
 *     Worst case is therefore about one period + one slice (~90 ms), plus
 *     scheduling delay.
 */
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   /* xTaskCreatePinnedToCore */
#include "esp_timer.h"
#include "esp_random.h"
#include "unity.h"
#include "sdkconfig.h"
#include "test_i2s_mic_common.h"

#define STOP_LIMIT_US        (TEST_PERIOD_US + 30000)                          /* 60 ms */
#define READER_EXIT_LIMIT_US (TEST_PERIOD_US + TEST_SLICE_MS * 1000LL + 30000)  /* 120 ms */

TEST_CASE("stop() ends a read waiting with portMAX_DELAY", "[lifecycle]")
{
    i2s_mic_config_t c = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));

    int64_t max_stop = 0, max_exit = 0;
    for (int i = 0; i < 20; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
        test_reader_t r = {0};
        test_reader_start(&r, portMAX_DELAY, 0);

        /* stop at a random point within a buffer period */
        vTaskDelay(pdMS_TO_TICKS(100 + esp_random() % TEST_PERIOD_MS));

        int64_t t0 = esp_timer_get_time();
        TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
        int64_t stop_us = esp_timer_get_time() - t0;

        TEST_ASSERT_TRUE_MESSAGE(test_reader_wait(&r, 1000), "reader stayed blocked after stop()");
        int64_t exit_us = r.exit_us - t0;
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.last);
        TEST_ASSERT_EQUAL(0, r.bad);
        TEST_ASSERT_EQUAL(0, r.timeouts);

        if (stop_us > max_stop) max_stop = stop_us;
        if (exit_us > max_exit) max_exit = exit_us;
    }
    printf("20 stops: stop() max %ld us, reader exit max %ld us after stop() began\n",
           (long)max_stop, (long)max_exit);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32((uint32_t)STOP_LIMIT_US, (uint32_t)max_stop);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32((uint32_t)READER_EXIT_LIMIT_US, (uint32_t)max_exit);

    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

/* ---- CPU load ----------------------------------------------------------- */

static volatile bool s_load_run;
static int s_load_alive;

/* Spins for 20 ms, then sleeps one tick: a heavy but well-behaved task at a
 * priority above the reader. */
static void load_task(void *arg)
{
    (void)arg;
    __atomic_fetch_add(&s_load_alive, 1, __ATOMIC_SEQ_CST);
    while (s_load_run) {
        int64_t until = esp_timer_get_time() + 20000;
        while (esp_timer_get_time() < until) {
        }
        vTaskDelay(1);
    }
    __atomic_fetch_sub(&s_load_alive, 1, __ATOMIC_SEQ_CST);
    vTaskDelete(NULL);
}

TEST_CASE("stop() completes under higher-priority CPU load", "[lifecycle]")
{
    i2s_mic_config_t c = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
    test_reader_t r = {0};
    test_reader_start(&r, portMAX_DELAY, 0);

    /* No TEST_ASSERT while the load tasks run: a failing assert would leave
     * them spinning and starve every later test. Collect, then check. */
    s_load_run = true;
    s_load_alive = 0;
    int created = 0;
    for (int core = 0; core < portNUM_PROCESSORS; core++) {
        if (xTaskCreatePinnedToCore(load_task, "load", 2048, NULL,
                                    TEST_READER_PRIO + 2, NULL, core) == pdPASS) {
            created++;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    int64_t t0 = esp_timer_get_time();
    esp_err_t stop_ret = i2s_mic_stop();
    int64_t stop_us = esp_timer_get_time() - t0;
    bool exited = test_reader_wait(&r, 2000);

    s_load_run = false;
    vTaskDelay(1);   /* let both load tasks register before counting down */
    while (__atomic_load_n(&s_load_alive, __ATOMIC_SEQ_CST) > 0) {
        vTaskDelay(1);
    }

    TEST_ASSERT_EQUAL(portNUM_PROCESSORS, created);
    TEST_ASSERT_EQUAL(ESP_OK, stop_ret);
    TEST_ASSERT_TRUE_MESSAGE(exited, "reader stayed blocked after stop() under load");
    int64_t exit_us = r.exit_us - t0;
    printf("under load: stop() %ld us, reader exit %ld us after stop() began, "
           "overflow %u\n", (long)stop_us, (long)exit_us,
           (unsigned)i2s_mic_get_overflow_count());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.last);
    TEST_ASSERT_EQUAL(0, r.bad);
    /* generous: the load task holds the CPU for 20 ms at a time */
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(500000, (uint32_t)exit_us);

    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
}

/* ---- Cycles ------------------------------------------------------------- */

TEST_CASE("start/stop cycles with an active reader", "[lifecycle][cycles]")
{
    const int cycles = CONFIG_I2S_MIC_TEST_CYCLES;
    i2s_mic_config_t c = test_mic_config();
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));

    int64_t max_stop = 0, max_exit = 0;
    for (int k = 0; k < cycles; k++) {
        TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_start());
        TEST_ASSERT_EQUAL(0, i2s_mic_get_overflow_count());

        /* mix infinite and short timeouts; never 0, which would busy-loop */
        uint32_t timeout = (k % 3 == 0) ? portMAX_DELAY : 10 + esp_random() % 90;
        test_reader_t r = {0};
        test_reader_start(&r, timeout, 0);

        vTaskDelay(pdMS_TO_TICKS(esp_random() % 40));

        int64_t t0 = esp_timer_get_time();
        TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
        int64_t stop_us = esp_timer_get_time() - t0;

        if (!test_reader_wait(&r, 1000)) {
            char msg[64];
            snprintf(msg, sizeof(msg), "reader stuck in cycle %d", k);
            TEST_FAIL_MESSAGE(msg);
        }
        int64_t exit_us = r.exit_us - t0;
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.last);
        TEST_ASSERT_EQUAL(0, r.bad);

        if (stop_us > max_stop) max_stop = stop_us;
        if (exit_us > max_exit) max_exit = exit_us;

        if (k % 100 == 99) {
            /* also exercise deinit/init */
            TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());
            TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_init(&c));
            printf("  %d/%d cycles, stop() max %ld us, reader exit max %ld us\n",
                   k + 1, cycles, (long)max_stop, (long)max_exit);
        }
    }
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());

    printf("%d cycles: stop() max %ld us, reader exit max %ld us\n",
           cycles, (long)max_stop, (long)max_exit);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32((uint32_t)STOP_LIMIT_US, (uint32_t)max_stop);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32((uint32_t)READER_EXIT_LIMIT_US, (uint32_t)max_exit);
}

TEST_CASE("deinit is refused while running and works after the reader exits", "[lifecycle]")
{
    test_mic_up();
    test_reader_t r = {0};
    test_reader_start(&r, portMAX_DELAY, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_deinit());

    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_stop());
    TEST_ASSERT_TRUE(test_reader_wait(&r, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, i2s_mic_deinit());

    uint8_t b[16];
    size_t n = 7;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, i2s_mic_read(b, sizeof(b), &n, 10));
    TEST_ASSERT_EQUAL(0, n);
}

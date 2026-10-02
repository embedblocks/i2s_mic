/*
 * i2s_mic test app: runs every TEST_CASE from the i2s_mic test component
 * and prints Unity's summary ("N Tests M Failures K Ignored").
 *
 * setUp/tearDown wrap every test with a heap check, the same pattern
 * ESP-IDF's own driver test apps use, so the start/stop cycle test and the
 * reader tasks also catch leaks.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "sdkconfig.h"
#include "i2s_mic.h"

/* Small negative slack for allocations the driver or FreeRTOS make once
 * and keep (for example on the first channel creation). */
#define LEAK_THRESHOLD (-350)

static size_t s_free_8bit;
static size_t s_free_32bit;

static void check_leak(size_t before, size_t after, const char *type)
{
    int delta = (int)after - (int)before;
    printf("MALLOC_CAP_%s: before %u, after %u (delta %d)\n",
           type, (unsigned)before, (unsigned)after, delta);
    TEST_ASSERT_MESSAGE(delta >= LEAK_THRESHOLD, "memory leak");
}

void setUp(void)
{
    s_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    s_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    /* deleted reader/load tasks are freed by the idle task */
    vTaskDelay(pdMS_TO_TICKS(100));
    check_leak(s_free_8bit, heap_caps_get_free_size(MALLOC_CAP_8BIT), "8BIT");
    check_leak(s_free_32bit, heap_caps_get_free_size(MALLOC_CAP_32BIT), "32BIT");
}

/* Create and delete one channel before the tests, so allocations the
 * driver makes only once don't show up as a leak in the first test. */
static void warm_up_driver(void)
{
    i2s_mic_config_t cfg = {
        .sample_rate = 16000, .bits_per_sample = 32, .channel_count = 2,
        .slot_mode = I2S_SLOT_MODE_STEREO,
        .gpio_bck = CONFIG_I2S_MIC_TEST_GPIO_BCK,
        .gpio_ws = CONFIG_I2S_MIC_TEST_GPIO_WS,
        .gpio_data = CONFIG_I2S_MIC_TEST_GPIO_DATA,
        .port = CONFIG_I2S_MIC_TEST_PORT,
        .dma_buffer_count = 6, .dma_buffer_size = 3840,
    };
    if (i2s_mic_init(&cfg) == ESP_OK && i2s_mic_start() == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
        i2s_mic_stop();
    }
    i2s_mic_deinit();
}

void app_main(void)
{
    printf("\n=== i2s_mic hardware tests ===\n");
    printf("pins: BCK %d, WS %d, DATA %d, port %d (no microphone needed)\n",
           CONFIG_I2S_MIC_TEST_GPIO_BCK, CONFIG_I2S_MIC_TEST_GPIO_WS,
           CONFIG_I2S_MIC_TEST_GPIO_DATA, CONFIG_I2S_MIC_TEST_PORT);
    printf("tick rate: %d Hz\n\n", configTICK_RATE_HZ);

    warm_up_driver();

#if CONFIG_I2S_MIC_TEST_APP_INTERACTIVE
    unity_run_menu();
#else
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
    printf("\n=== done ===\n");
#endif
}

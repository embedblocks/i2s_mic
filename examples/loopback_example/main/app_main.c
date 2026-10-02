/*
 * i2s_mic + i2s_spk loopback example — record with the microphone, then
 * play the recording back through the speaker. No PC, no transport
 * protocol — everything happens entirely on-device.
 *
 * This copy of the example lives inside the i2s_mic repository and pulls
 * in i2s_spk from the ESP Component Registry (see main/idf_component.yml).
 * An identical copy lives inside the i2s_spk repository
 * (examples/loopback), pulling in i2s_mic from the registry instead — the
 * two are intentionally the same demo, cross-referenced from both
 * component's own example sets, matching how each component's README
 * points at "an end-to-end example" without requiring a reader to already
 * have both repositories checked out side by side.
 *
 * ============================================================================
 * WHY RECORD-THEN-PLAYBACK, NOT LIVE/CONTINUOUS LOOPBACK
 * ============================================================================
 * The obvious version of this demo would run i2s_mic and i2s_spk
 * simultaneously — capture a buffer, immediately play it, repeat — for a
 * true live loopback. That's possible on chips with two independent I2S
 * peripherals (ESP32, ESP32-S3): put i2s_mic on I2S_NUM_0 and i2s_spk on
 * I2S_NUM_1, and they never contend for anything.
 *
 * It is NOT possible on ESP32-C3 (or any chip with SOC_I2S_NUM == 1). Both
 * i2s_mic_init() and i2s_spk_init() independently call ESP-IDF's
 * i2s_new_channel(), which claims a physical I2S controller for as long as
 * the component stays INITIALIZED — not just while it's RUNNING. On a
 * single-controller chip, whichever of the two components initializes
 * first gets the only available controller, and the other's init() call
 * fails outright (there's nothing left to allocate), regardless of whether
 * the first one is actively running or merely sitting idle in the
 * INITIALIZED state. This is a hardware/singleton-design constraint, not a
 * bug in either component — see i2s_spk's README ("Chip Support") and the
 * design document's Section 10 for the same limitation stated from each
 * component's own side.
 *
 * Since this example is meant to build and run identically on both ESP32
 * and ESP32-C3 from one shared source file (only sdkconfig.defaults.<target>
 * differs — see that file for GPIO wiring per target), it's written to
 * only ever have ONE of the two components INITIALIZED at a time:
 *
 *   1. RECORD phase: i2s_mic_init() -> i2s_mic_start() -> capture N seconds
 *      into a RAM buffer -> i2s_mic_stop() -> i2s_mic_deinit() (fully
 *      releases the I2S controller)
 *   2. PLAYBACK phase: i2s_spk_init() -> i2s_spk_start() -> play the whole
 *      RAM buffer -> i2s_spk_stop() -> i2s_spk_deinit()
 *   3. repeat
 *
 * This costs a fixed record/playback delay instead of true real-time
 * loopback, but it's the only version of this demo that's actually correct
 * on every chip both components claim to support — including the
 * single-I2S-peripheral ones. If you specifically have a 2-I2S-peripheral
 * chip and want true live loopback, that's a different (and simpler!)
 * example: skip the RAM buffer, run both components RUNNING at once on
 * separate ports, and pipe each captured buffer straight into
 * i2s_spk_send_buffer() as it arrives.
 *
 * ============================================================================
 * MIC CAPTURE: STEREO WORKAROUND AND SAMPLE EXTRACTION
 * ============================================================================
 * Same approach as i2s_mic's own streaming examples: on at least one
 * tested ESP32-C3 / ESP-IDF combination, requesting I2S_SLOT_MODE_MONO for
 * RX did not produce a clean single-slot-per-frame stream, so this example
 * requests STEREO explicitly and discards the unwanted interleaved slot
 * (KEEP_SLOT below) while downconverting INMP441's 32-bit samples to
 * 16-bit. See i2s_mic's own examples for the full diagnosis. If your
 * recording comes out silent or hum-only, try flipping KEEP_SLOT from 0 to
 * 1, or the INMP441's L/R pin from GND to VDD.
 *
 * Wiring:
 *   INMP441 SCK  -> LOOPBACK_MIC_GPIO_BCK   (see sdkconfig.defaults.<target>)
 *   INMP441 WS   -> LOOPBACK_MIC_GPIO_WS
 *   INMP441 SD   -> LOOPBACK_MIC_GPIO_DATA
 *   INMP441 L/R  -> GND
 *   INMP441 VDD  -> 3.3V
 *   INMP441 GND  -> GND
 *
 *   MAX98357A BCLK -> LOOPBACK_SPK_GPIO_BCK
 *   MAX98357A LRC  -> LOOPBACK_SPK_GPIO_WS
 *   MAX98357A DIN  -> LOOPBACK_SPK_GPIO_DATA
 *   MAX98357A SD   -> floating (mono mixdown)
 *   MAX98357A GAIN -> floating
 *   MAX98357A VIN  -> 5V (or 3.3V)
 *   MAX98357A GND  -> GND
 *
 * Both devices are wired at once, permanently — only the *logical* I2S
 * channel toggles between them in firmware; nothing physical needs to be
 * moved between the record and playback phases.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2s_mic.h"
#include "i2s_spk.h"

#define TAG   "LOOPBACK"

/* ---- Mic-side capture configuration (see the big comment above for why
 * this requests stereo for a single mono microphone). ---------------- */
#define SAMPLE_RATE           16000
#define MIC_BITS_PER_SAMPLE   32
#define MIC_CHANNEL_COUNT     2
#define KEEP_SLOT             0

#define DMA_BUF_COUNT   6      /* driver ring; it can hold DMA_BUF_COUNT - 1 unread buffers */
#define BUF_FRAMES      480    /* stereo frames per DMA buffer = 30 ms at 16 kHz.
                                * One DMA buffer is capped at 4092 bytes, so at
                                * 8 bytes per stereo frame the most is 511. */
#define BUF_BYTES       (BUF_FRAMES * MIC_CHANNEL_COUNT * (MIC_BITS_PER_SAMPLE / 8))

/* ---- Recording buffer (RAM, 16-bit mono, post-conversion) ------------- */
#define RECORD_SAMPLES     ((size_t)CONFIG_LOOPBACK_RECORD_SECONDS * SAMPLE_RATE)
#define RECORD_BUF_BYTES   (RECORD_SAMPLES * sizeof(int16_t))

/* ---- Playback chunking (independent of any DMA sizing — see i2s_spk.h's
 * note that send_buffer() callers may use any buffer length). ---------- */
#define SPK_CHUNK_BYTES   4096

/* One capture buffer: each i2s_mic_read() fills it with one DMA buffer,
 * which is converted into s_record_buf straight away. */
static int32_t s_capture_buf[BUF_FRAMES * MIC_CHANNEL_COUNT];
static int16_t *s_record_buf;
static size_t s_record_len;   /* samples recorded in the last record phase */

/* ------------------------------------------------------------------------
 * Record phase
 * ---------------------------------------------------------------------- */

static void run_record_phase(void)
{
    ESP_LOGI(TAG, "Recording %d s...", CONFIG_LOOPBACK_RECORD_SECONDS);

    i2s_mic_config_t mic_cfg = {
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = MIC_BITS_PER_SAMPLE,
        .channel_count = MIC_CHANNEL_COUNT,
        .slot_mode = I2S_SLOT_MODE_STEREO,
        .gpio_bck = CONFIG_LOOPBACK_MIC_GPIO_BCK,
        .gpio_ws = CONFIG_LOOPBACK_MIC_GPIO_WS,
        .gpio_data = CONFIG_LOOPBACK_MIC_GPIO_DATA,
        .port = I2S_NUM_0,
        .dma_buffer_count = DMA_BUF_COUNT,
        .dma_buffer_size = BUF_BYTES,
    };

    /* See the file header comment for why i2s_spk is guaranteed NOT
     * INITIALIZED at this point (run_playback_phase() always deinits it
     * before returning) — that's what makes this init() safe on
     * single-I2S-peripheral chips. */
    ESP_ERROR_CHECK(i2s_mic_init(&mic_cfg));
    ESP_ERROR_CHECK(i2s_mic_start());

    /* Read one DMA buffer at a time, keep one slot of each stereo frame and
     * downconvert 32-bit to 16-bit straight into the recording buffer. This
     * task is the only reader, and it is the same task that calls stop()
     * below, so no shutdown handshake is needed. */
    size_t pos = 0;
    while (pos < RECORD_SAMPLES) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_mic_read(s_capture_buf, BUF_BYTES, &bytes_read, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2s_mic_read failed: %s", esp_err_to_name(ret));
            break;
        }

        size_t frames = bytes_read / (MIC_CHANNEL_COUNT * sizeof(int32_t));
        size_t remaining = RECORD_SAMPLES - pos;
        size_t n = (frames < remaining) ? frames : remaining;
        for (size_t i = 0; i < n; i++) {
            s_record_buf[pos + i] =
                (int16_t)(s_capture_buf[MIC_CHANNEL_COUNT * i + KEEP_SLOT] >> 16);
        }
        pos += n;
    }

    uint32_t lost = i2s_mic_get_overflow_count();

    ESP_ERROR_CHECK(i2s_mic_stop());
    ESP_ERROR_CHECK(i2s_mic_deinit());

    s_record_len = pos;
    ESP_LOGI(TAG, "Recording complete: %u samples, audio lost: %u buffers",
             (unsigned)pos, (unsigned)lost);
}

/* ------------------------------------------------------------------------
 * Playback phase
 * ---------------------------------------------------------------------- */

static void run_playback_phase(void)
{
    ESP_LOGI(TAG, "Playing back...");

    i2s_spk_config_t spk_cfg = {
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .channel_count = 1,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .gpio_bck = CONFIG_LOOPBACK_SPK_GPIO_BCK,
        .gpio_ws = CONFIG_LOOPBACK_SPK_GPIO_WS,
        .gpio_data = CONFIG_LOOPBACK_SPK_GPIO_DATA,
        .port = I2S_NUM_0,
        .dma_buffer_count = 6,
        .dma_buffer_size = 2048,
        .write_timeout = portMAX_DELAY,
        .user_ctx = NULL,
    };

    /* i2s_mic is guaranteed fully deinited by run_record_phase() before we
     * get here — see the file header comment. */
    ESP_ERROR_CHECK(i2s_spk_init(&spk_cfg));
    ESP_ERROR_CHECK(i2s_spk_start());

    uint8_t *pcm_bytes = (uint8_t *)s_record_buf;
    size_t total_bytes = s_record_len * sizeof(int16_t);
    size_t offset = 0;

    while (offset < total_bytes) {
        size_t want = total_bytes - offset;
        if (want > SPK_CHUNK_BYTES) {
            want = SPK_CHUNK_BYTES;
        }

        size_t sent = 0;
        while (sent < want) {
            size_t this_sent = 0;
            esp_err_t ret = i2s_spk_send_buffer(pcm_bytes + offset + sent, want - sent, &this_sent);
            if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "i2s_spk_send_buffer failed: %s", esp_err_to_name(ret));
                offset = total_bytes;   /* stop the outer loop too */
                break;
            }
            sent += this_sent;
            if (this_sent == 0) {
                break;
            }
        }
        offset += want;
    }

    ESP_ERROR_CHECK(i2s_spk_stop());
    ESP_ERROR_CHECK(i2s_spk_deinit());

    ESP_LOGI(TAG, "Playback complete");
}

/* ------------------------------------------------------------------------
 * Entry point
 * ---------------------------------------------------------------------- */

void app_main(void)
{
    s_record_buf = (int16_t *)malloc(RECORD_BUF_BYTES);
    if (s_record_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %u-byte record buffer — lower "
                 "CONFIG_LOOPBACK_RECORD_SECONDS via menuconfig", (unsigned)RECORD_BUF_BYTES);
        return;
    }
    ESP_LOGI(TAG, "Record buffer: %d s (%u bytes)",
             CONFIG_LOOPBACK_RECORD_SECONDS, (unsigned)RECORD_BUF_BYTES);

    while (1) {
        run_record_phase();
        run_playback_phase();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

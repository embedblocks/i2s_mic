/*
 * i2s_mic streaming example — INMP441 -> ESP32-C3 -> USB-Serial-JTAG -> PC
 *
 * Wait for a trigger byte from the PC, send a small binary header
 * describing the audio format, then stream raw PCM continuously — same
 * overall shape as a UART-based handshake, but over the native
 * USB-Serial-JTAG peripheral instead of a physical UART, since that's the
 * only channel connected on boards (like most ESP32-C3 devkits) that have
 * no separate USB-to-UART bridge chip. There's no baud rate to negotiate
 * here (it's a real USB CDC endpoint), which simplifies the handshake
 * versus a true-UART version.
 *
 * The wire format (sync bytes, magic, header layout, and the decision to
 * downconvert INMP441's 32-bit samples to 16-bit before sending) is owned
 * entirely by this example, not by the i2s_mic component. i2s_mic knows
 * nothing about USB, UART, headers, or byte order — see the design
 * document's Section 10 ("component boundary" discussion) for why that
 * split exists.
 *
 * If your board instead has a separate CP2102/CH340-style UART bridge
 * wired to the physical UART0 pins (i.e. a second, distinct COM port shows
 * up when you plug in), you can revert this file to talk to UART0 via
 * driver/uart.h instead — the original approach works fine there, and the
 * PC script's baud-switching logic is meant for exactly that case.
 *
 * Wiring (adjust GPIO_BCK/GPIO_WS/GPIO_DATA below for your board):
 *   INMP441 SCK  -> ESP32 GPIO_BCK
 *   INMP441 WS   -> ESP32 GPIO_WS
 *   INMP441 SD   -> ESP32 GPIO_DATA
 *   INMP441 L/R  -> GND   (selects the LEFT slot, which is the one this
 *                          example keeps — see the note on STEREO below)
 *   INMP441 VDD  -> 3.3V
 *   INMP441 GND  -> GND
 *
 * IMPORTANT — why this requests STEREO for a single mono mic:
 * On at least one tested ESP32-C3 / ESP-IDF combination, requesting
 * I2S_SLOT_MODE_MONO for RX did NOT produce the compact single-slot-per-
 * frame stream ESP-IDF's docs describe for ESP32/S2 — the DMA buffer still
 * contained both slots interleaved, with real audio in only one of every
 * two 32-bit words (confirmed by analyzing a captured WAV: consecutive
 * samples were essentially uncorrelated, while samples two apart were
 * strongly correlated, and the discarded word's own spectrum was
 * dominated by ~50 Hz mains hum rather than speech). Playing that
 * [real, hum, real, hum, ...] stream back as one continuous mono signal
 * produced a heavily distorted, low-pitched "growl."
 *
 * The fix used here sidesteps the ambiguity rather than depending on it:
 * request STEREO explicitly (both slots, unambiguous), and discard the
 * unwanted slot in audio_sender_task() below. This has been verified to
 * fix the growl on the hardware/IDF combination it was diagnosed on; if
 * you hit something similar on different hardware, this same de-interleave
 * approach is the thing to reach for.
 *
 * NOTE on L/R: which physical slot (left/right) ends up first in the
 * interleaved buffer can differ across ESP-IDF versions. This example
 * assumes the real signal is in the first (even-indexed) word of each
 * stereo pair, matching L/R tied to GND. If your recording is silent or
 * hum-only after this fix, swap KEEP_SLOT below from 0 to 1 (or try VDD
 * instead of GND on L/R) before suspecting anything else.
 */
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "i2s_mic.h"

#define TAG            "MIC_UART"

/* ---- Audio / I2S configuration ---------------------------------------- */
#define SAMPLE_RATE     16000
#define GPIO_BCK        7
#define GPIO_WS         8
#define GPIO_DATA       9

/* INMP441 outputs 24-bit samples MSB-justified in a 32-bit slot. i2s_mic
 * only supports 16 or 32-bit slots (24-bit is rejected at init()), so we
 * configure 32 here and downconvert to 16-bit ourselves before sending
 * over the wire — see audio_sender_task(). */
#define MIC_BITS_PER_SAMPLE   32
#define WIRE_BITS_PER_SAMPLE  16   /* what actually goes out over the wire, after c
onversion */

/* We capture STEREO (2 slots) and keep only one — see the big comment at
 * the top of this file for why. KEEP_SLOT selects which of each
 * interleaved pair is real audio: 0 = first/even word, 1 = second/odd
 * word. Change this if your recording comes out silent or hum-only. */
#define KEEP_SLOT       0
#define MIC_CHANNEL_COUNT 2

#define NUM_BUFS        4
#define BUF_FRAMES      512    /* stereo frames per buffer (1 frame = 2 slots = bot
h interleaved words) */
#define BUF_BYTES       (BUF_FRAMES * MIC_CHANNEL_COUNT * (MIC_BITS_PER_SAMPLE / 8))

#define STREAM_SENT 1   /* 1 = actually write PCM to the host, 0 = run the pipeline
 without transmitting (bring-up/debug) */

/* ---- Wire header, read by the companion PC script ---------------------- */
#define AUDIO_MAGIC 0xC0FFEE01u

typedef struct {
    uint8_t  sync[3];          /* 0xAA 0xAA 0xAA */
    uint32_t magic;            /* AUDIO_MAGIC */
    uint32_t sample_rate;
    uint16_t bits_per_sample;  /* wire format, i.e. WIRE_BITS_PER_SAMPLE */
    uint8_t  channel_count;
} __attribute__((packed)) audio_header_t;

/* ---- Buffer pool + inter-task handoff ----------------------------------
 *
 * s_free_q holds buffers that are safe to (re-)post to i2s_mic.
 * s_filled_q holds buffers the ISR has already handed back, paired with
 * how many bytes of captured audio they hold, waiting for the sender task
 * to convert + transmit them.
 *
 * A buffer is only ever in exactly one of: posted-to-i2s_mic, in
 * s_filled_q, being processed by audio_sender_task, or in s_free_q. That
 * invariant is what makes it safe for the ISR to re-post a *different*
 * buffer than the one it just received, rather than the same one — see
 * mic_ready_cb() below.
 * ------------------------------------------------------------------- */
static uint8_t s_bufs[NUM_BUFS][BUF_BYTES];

typedef struct {
    uint8_t *buf;
    size_t   bytes_read;
} filled_item_t;

static QueueHandle_t s_free_q;
static QueueHandle_t s_filled_q;

static volatile uint32_t s_last_overflow_count = 0;
static volatile uint32_t s_last_no_buffer_count = 0;

/* ------------------------------------------------------------------------
 * i2s_mic callbacks (ISR context — see i2s_mic.h's rules before editing)
 * ---------------------------------------------------------------------- */

static void IRAM_ATTR mic_ready_cb(uint8_t *buffer, size_t bytes_read, void *user_ctx)
{
    (void)user_ctx;
    BaseType_t hp_woken = pdFALSE;

    filled_item_t item = { .buf = buffer, .bytes_read = bytes_read };
    xQueueSendFromISR(s_filled_q, &item, &hp_woken);

    /* Re-post a free buffer from the pool (NOT the one we just received —
     * audio_sender_task hasn't read it yet). Non-blocking: if the pool is
     * empty right now, we simply don't re-post this cycle. That means one
     * fewer buffer in flight going forward, which is exactly the
     * best-effort backpressure the component is designed around: a slow
     * consumer causes drops (reported via mic_overflow_cb below), not
     * memory corruption. */
    uint8_t *free_buf;
    if (xQueueReceiveFromISR(s_free_q, &free_buf, &hp_woken) == pdTRUE) {
        i2s_mic_request_buffer(free_buf, BUF_BYTES);
    }

    portYIELD_FROM_ISR(hp_woken);
}

static void mic_overflow_cb(uint32_t total_overflow_count, uint32_t total_no_buffer_count, void *user_ctx)
{
    (void)user_ctx;
    /* ISR context: no logging here. Just record the latest counts; a
     * low-priority task polls and logs them (see overflow_report_task). */
    s_last_overflow_count = total_overflow_count;
    s_last_no_buffer_count = total_no_buffer_count;
}

/* ------------------------------------------------------------------------
 * Ordinary tasks
 * ---------------------------------------------------------------------- */

static void audio_sender_task(void *arg)
{
    (void)arg;
    filled_item_t item;

    while (1) {
        if (xQueueReceive(s_filled_q, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Discard the unwanted interleaved slot, then downconvert INMP441's
         * 32-bit samples to 16-bit PCM — both in place, in one pass.
         *
         * item.buf holds interleaved stereo frames: [slot0, slot1, slot0,
         * slot1, ...] as int32_t words. We keep only src[2*i + KEEP_SLOT]
         * from each frame. This is still safe as an in-place compaction:
         * dst[i] (2 bytes, at byte offset 2*i) is always written strictly
         * before the next frame we read from (starting at byte offset
         * 8*(i+1)) is touched, since 2*i < 8*(i+1) for every i >= 0.
         *
         * The >> 16 keeps the sign and the most significant bits of the
         * sample. This is a starting point, not a calibrated value — if
         * your capture is too quiet or too loud, adjust the shift amount
         * (e.g. >> 14 for more gain) or add explicit scaling here. This is
         * purely an application/transport choice; i2s_mic itself hands you
         * the raw 32-bit samples untouched. */
        int32_t *src = (int32_t *)item.buf;
        int16_t *dst = (int16_t *)item.buf;
        size_t num_frames = item.bytes_read / (MIC_CHANNEL_COUNT * sizeof(int32_t))
;
        for (size_t i = 0; i < num_frames; i++) {
            dst[i] = (int16_t)(src[MIC_CHANNEL_COUNT * i + KEEP_SLOT] >> 16);
        }
        size_t out_bytes = num_frames * sizeof(int16_t);

#if STREAM_SENT
        int written = usb_serial_jtag_write_bytes((const uint8_t *)item.buf, out_bytes, portMAX_DELAY);
        if (written < 0 || (size_t)written != out_bytes) {
            /* Logging is disabled once streaming starts (see
             * host_comm_init), so this is only informative during bring-up
             * with STREAM_SENT temporarily left at 0. */
            ESP_LOGW(TAG, "short USB write: %d/%zu", written, out_bytes);
        }
#endif

        /* The buffer's contents have been fully copied into the driver's
         * own TX ring buffer by the time usb_serial_jtag_write_bytes()
         * returns, so it's safe to return it to the pool now. */
        xQueueSend(s_free_q, &item.buf, portMAX_DELAY);
    }
}

static void overflow_report_task(void *arg)
{
    (void)arg;
    uint32_t last_ovf = 0, last_nobuf = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t ovf = s_last_overflow_count;
        uint32_t nobuf = s_last_no_buffer_count;
        if (ovf != last_ovf || nobuf != last_nobuf) {
            /* Suppressed once streaming starts (esp_log_level_set("*", ESP_LOG_NON
E)
             * below) so it never corrupts the PCM stream; useful during
             * bring-up over a separate console, or if you route logs
             * elsewhere (e.g. USB-CDC on boards with two UARTs/USB). */
            ESP_LOGW(TAG, "drops so far: dma_overflow=%u no_buffer=%u", ovf, nobuf)
;
            last_ovf = ovf;
            last_nobuf = nobuf;
        }
    }
}

/* ------------------------------------------------------------------------
 * Host handshake over USB-Serial-JTAG
 * ---------------------------------------------------------------------- */

static void host_comm_init(void)
{
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT
();
    usb_cfg.rx_buffer_size = 256;
    usb_cfg.tx_buffer_size = BUF_BYTES * 4;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    /* Note: this only installs the driver for our own reads/writes. The
     * console's log output (ESP_LOGI etc.) keeps working over the same
     * physical USB-Serial-JTAG cable via ESP-IDF's separate, lower-level
     * console output path (that's what let you see "READY" below even
     * before this driver was installed) — the two coexist by design; this
     * is the standard pattern for adding bidirectional application I/O on
     * top of a USB-Serial-JTAG console channel. */

    ESP_LOGI(TAG, "=== READY, waiting for trigger ===");
    fflush(stdout);

    uint8_t trigger = 0;
    usb_serial_jtag_read_bytes(&trigger, 1, portMAX_DELAY);

    ESP_LOGI(TAG, "Trigger 0x%02X received", trigger);
    fflush(stdout);

    audio_header_t hdr = {
        .sync = {0xAA, 0xAA, 0xAA},
        .magic = AUDIO_MAGIC,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = WIRE_BITS_PER_SAMPLE,
        .channel_count = 1,
    };

    /* Disable logging before the header/PCM stream starts — any further
     * log line would land inside the binary stream and desync the PC
     * script's parser (same reasoning as the JPEG example). */
    esp_log_level_set("*", ESP_LOG_NONE);
    vTaskDelay(pdMS_TO_TICKS(50));

    usb_serial_jtag_write_bytes((const uint8_t *)&hdr, sizeof(hdr), portMAX_DELAY);
}

/* ------------------------------------------------------------------------
 * Entry point
 * ---------------------------------------------------------------------- */

void app_main(void)
{
    s_free_q = xQueueCreate(NUM_BUFS, sizeof(uint8_t *));
    s_filled_q = xQueueCreate(NUM_BUFS, sizeof(filled_item_t));

    host_comm_init();

    i2s_mic_config_t cfg = {
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = MIC_BITS_PER_SAMPLE,
        .channel_count = MIC_CHANNEL_COUNT,
        .slot_mode = I2S_SLOT_MODE_STEREO,
        .gpio_bck = GPIO_BCK,
        .gpio_ws = GPIO_WS,
        .gpio_data = GPIO_DATA,
        .port = I2S_NUM_0,
        .dma_buffer_count = 6,
        .dma_buffer_size = BUF_BYTES,
        .max_pending_buffers = NUM_BUFS,
        .cb = mic_ready_cb,
        .overflow_cb = mic_overflow_cb,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(i2s_mic_init(&cfg));

    xTaskCreate(audio_sender_task, "audio_sender", 4096, NULL, 5, NULL);
    xTaskCreate(overflow_report_task, "ovf_report", 2048, NULL, 1, NULL);

    /* Post every buffer up front; the free pool starts empty and refills
     * as audio_sender_task finishes with each buffer. */
    for (int i = 0; i < NUM_BUFS; i++) {
        ESP_ERROR_CHECK(i2s_mic_request_buffer(s_bufs[i], BUF_BYTES));
    }

    ESP_ERROR_CHECK(i2s_mic_start());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
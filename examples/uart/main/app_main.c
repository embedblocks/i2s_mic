/*
 * i2s_mic UART streaming example — INMP441 -> ESP32 -> UART0 -> PC
 *
 * FOR BOARDS WITH A SEPARATE USB-TO-UART BRIDGE CHIP — a second, distinct
 * COM port appears when you plug in, wired to the chip's real UART0 pins
 * (true of most classic ESP32 devkits with an onboard CP2102/CH340/etc.).
 * This example reuses that same UART0 — the one your programming/console
 * cable already talks to — for the audio stream too. No extra hardware
 * needed, same single cable.
 *
 * If your board instead has no separate bridge chip (the cable you flash
 * through is the chip's native USB-Serial-JTAG peripheral — common on
 * ESP32-C3/S3/C6 devkits), use the sibling example i2s_mic_usb_jtag_example
 * instead. The two are not interchangeable: this one needs a real UART0
 * wired to something that can negotiate a baud rate change, which a native
 * USB-CDC endpoint is not.
 *
 * Handshake: connect at a safe HANDSHAKE_BAUD (115200), wait for a trigger
 * byte from the PC, switch UART0 to a fast STREAM_BAUD (460800), send a
 * small binary header describing the audio format, then stream raw PCM
 * continuously. Logging is disabled once streaming starts, since it shares
 * the same wire as the PCM data — any log line during streaming would land
 * inside the binary stream and desync the PC script's parser.
 *
 * The wire format (sync bytes, magic, header layout, and the decision to
 * downconvert INMP441's 32-bit samples to 16-bit before sending) is owned
 * entirely by this example, not by the i2s_mic component. i2s_mic knows
 * nothing about UART, headers, or byte order — see the design document's
 * Section 10 ("component boundary" discussion) for why that split exists.
 *
 * Wiring:
 *   INMP441 SCK  -> ESP32 GPIO_BCK
 *   INMP441 WS   -> ESP32 GPIO_WS
 *   INMP441 SD   -> ESP32 GPIO_DATA
 *   INMP441 L/R  -> GND   (selects the slot this example keeps — see
 *                          KEEP_SLOT below if your board differs; also,
 *                          make sure L/R is actually tied to GND or VDD
 *                          and never left floating — a floating L/R pin
 *                          is a common source of hum/noise on its own)
 *   INMP441 VDD  -> 3.3V
 *   INMP441 GND  -> GND
 *
 * The GPIO defaults below (14/15/32) are classic-ESP32 numbers, chosen to
 * avoid UART0's own pins (GPIO1 TX / GPIO3 RX) and the usual SPI-flash
 * pin range. Adjust for your specific board.
 *
 * ON STEREO: this example requests STEREO capture and discards one slot
 * (KEEP_SLOT), the same defensive approach used in i2s_mic_usb_jtag_example,
 * even though the specific mono-RX interleaving issue documented there was
 * diagnosed on ESP32-C3 rather than classic ESP32 — ESP-IDF's own RX table
 * suggests plain MONO capture may in fact work correctly on classic ESP32
 * (its example table is for ESP32/S2 specifically). Stereo-then-discard is
 * kept here anyway for consistency with the other example and because it
 * costs almost nothing; if you've verified plain mono works cleanly on
 * your exact board/IDF version, feel free to simplify.
 *
 * The trigger byte is validated against a specific TRIGGER_BYTE value
 * rather than accepting the first byte received, so a spurious byte can't
 * start streaming prematurely (which would desync the one-shot header and
 * leave the PC script waiting forever for a header that already went out).
 */
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2s_mic.h"

#define TAG            "MIC_UART"

/* ---- I2S / mic configuration ------------------------------------------- */
#define SAMPLE_RATE     16000
#define GPIO_BCK        16
#define GPIO_WS         17
#define GPIO_DATA       18

/* ---- UART configuration (reusing the console/programming UART0) -------- */
#define HANDSHAKE_BAUD  115200
#define STREAM_BAUD     460800

/* Specific expected value for the handshake trigger byte — must match
 * TRIGGER_BYTE in the PC script. */
#define TRIGGER_BYTE    0xA5

/* INMP441 outputs 24-bit samples MSB-justified in a 32-bit slot. i2s_mic
 * only supports 16 or 32-bit slots (24-bit is rejected at init()), so we
 * configure 32 here and downconvert to 16-bit ourselves before sending
 * over UART — see audio_sender_task(). */
#define MIC_BITS_PER_SAMPLE   32
#define WIRE_BITS_PER_SAMPLE  16   /* what actually goes out over the wire, after conversion */

/* We capture STEREO (2 slots) and keep only one — see the header comment
 * for why. KEEP_SLOT selects which of each interleaved pair is real audio:
 * 0 = first/even word, 1 = second/odd word. */
#define KEEP_SLOT       0
#define MIC_CHANNEL_COUNT 2

#define DMA_BUF_COUNT   6      /* driver ring; it can hold DMA_BUF_COUNT - 1 unread buffers */
#define BUF_FRAMES      480    /* stereo frames per DMA buffer = 30 ms at 16 kHz.
                                * One DMA buffer is capped at 4092 bytes, so at
                                * 8 bytes per stereo frame the most is 511. */
#define BUF_BYTES       (BUF_FRAMES * MIC_CHANNEL_COUNT * (MIC_BITS_PER_SAMPLE / 8))

#define STREAM_SENT 1   /* 1 = actually write PCM to UART, 0 = run the pipeline without transmitting (bring-up/debug) */

/* ---- Wire header, read by the companion PC script ---------------------- */
#define AUDIO_MAGIC 0xC0FFEE01u

typedef struct {
    uint8_t  sync[3];          /* 0xAA 0xAA 0xAA */
    uint32_t magic;            /* AUDIO_MAGIC */
    uint32_t sample_rate;
    uint16_t bits_per_sample;  /* wire format, i.e. WIRE_BITS_PER_SAMPLE */
    uint8_t  channel_count;
} __attribute__((packed)) audio_header_t;

/* ---- Capture buffer ----------------------------------------------------
 *
 * One buffer is enough: audio_sender_task reads a DMA buffer into it,
 * converts and sends it, then reads the next. While it is busy, finished
 * DMA buffers wait in the I2S driver's own queue (up to DMA_BUF_COUNT - 1
 * of them, about 150 ms here). Only if the sender falls further behind
 * than that does the driver drop audio, and i2s_mic_get_overflow_count()
 * goes up.
 * ------------------------------------------------------------------- */
static int32_t s_buf[BUF_FRAMES * MIC_CHANNEL_COUNT];

/* ------------------------------------------------------------------------
 * Tasks
 * ---------------------------------------------------------------------- */

static void audio_sender_task(void *arg)
{
    (void)arg;

    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_mic_read(s_buf, BUF_BYTES, &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK) {
            /* Only happens while capture is stopped; this example never
             * stops, so just back off briefly and try again. */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Discard the unwanted interleaved slot, then downconvert INMP441's
         * 32-bit samples to 16-bit PCM — both in place, in one pass.
         *
         * s_buf holds interleaved stereo frames: [slot0, slot1, slot0,
         * slot1, ...] as int32_t words. We keep only src[2*i + KEEP_SLOT]
         * from each frame. This is safe as an in-place compaction: dst[i]
         * (2 bytes, at byte offset 2*i) is always written strictly before
         * the next frame we read from (starting at byte offset 8*(i+1)) is
         * touched, since 2*i < 8*(i+1) for every i >= 0.
         *
         * The >> 16 keeps the sign and the most significant bits of the
         * sample. This is a starting point, not a calibrated value — if
         * your capture is too quiet or too loud, adjust the shift amount
         * (e.g. >> 14 for more gain) or add explicit scaling here. This is
         * purely an application/transport choice; i2s_mic itself hands you
         * the raw 32-bit samples untouched. */
        const int32_t *src = s_buf;
        int16_t *dst = (int16_t *)s_buf;
        size_t num_frames = bytes_read / (MIC_CHANNEL_COUNT * sizeof(int32_t));
        for (size_t i = 0; i < num_frames; i++) {
            dst[i] = (int16_t)(src[MIC_CHANNEL_COUNT * i + KEEP_SLOT] >> 16);
        }
        size_t out_bytes = num_frames * sizeof(int16_t);

#if STREAM_SENT
        int written = uart_write_bytes(UART_NUM_0, (const char *)s_buf, out_bytes);
        if (written < 0 || (size_t)written != out_bytes) {
            /* Logging is disabled once streaming starts (see
             * host_comm_init), so this is only informative during bring-up
             * with STREAM_SENT temporarily left at 0. */
            ESP_LOGW(TAG, "short UART write: %d/%u", written, (unsigned)out_bytes);
        }
#else
        (void)out_bytes;
#endif
    }
}

static void overflow_report_task(void *arg)
{
    (void)arg;
    uint32_t last = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t lost = i2s_mic_get_overflow_count();
        if (lost != last) {
            /* Suppressed once streaming starts (esp_log_level_set("*",
             * ESP_LOG_NONE) in host_comm_init) so it never corrupts the PCM
             * stream. Useful during bring-up with STREAM_SENT set to 0. */
            ESP_LOGW(TAG, "audio lost so far: %u buffers (%u ms)",
                     (unsigned)lost, (unsigned)(lost * BUF_FRAMES * 1000u / SAMPLE_RATE));
            last = lost;
        }
    }
}

/* ------------------------------------------------------------------------
 * Host handshake over UART0 (shared with console/programming)
 * ---------------------------------------------------------------------- */

static void host_comm_init(void)
{
    const int tx_buf_size = BUF_BYTES * 4;

    uart_driver_install(UART_NUM_0, 1024, tx_buf_size, 0, NULL, 0);
    uart_vfs_dev_use_driver(0);

    ESP_LOGI(TAG, "=== READY, waiting for trigger ===");
    fflush(stdout);

    uint8_t rx_byte = 0;
    do {
        uart_read_bytes(UART_NUM_0, &rx_byte, 1, portMAX_DELAY);
    } while (rx_byte != TRIGGER_BYTE);

    ESP_LOGI(TAG, "Trigger 0x%02X received - switching to %d baud", rx_byte, STREAM_BAUD);
    fflush(stdout);

    vTaskDelay(pdMS_TO_TICKS(600));
    uart_wait_tx_done(UART_NUM_0, portMAX_DELAY);

    uart_config_t uart_cfg = {
        .baud_rate  = STREAM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_0, &uart_cfg);

    vTaskDelay(pdMS_TO_TICKS(100));

    audio_header_t hdr = {
        .sync = {0xAA, 0xAA, 0xAA},
        .magic = AUDIO_MAGIC,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = WIRE_BITS_PER_SAMPLE,
        .channel_count = 1,
    };

    /* Disable logging before the header/PCM stream starts — any further
     * log line would land inside the binary stream and desync the PC
     * script's parser. */
    esp_log_level_set("*", ESP_LOG_NONE);
    vTaskDelay(pdMS_TO_TICKS(50));

    uart_write_bytes(UART_NUM_0, (const char *)&hdr, sizeof(hdr));
    uart_wait_tx_done(UART_NUM_0, portMAX_DELAY);
}

/* ------------------------------------------------------------------------
 * Entry point
 * ---------------------------------------------------------------------- */

void app_main(void)
{
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
        .dma_buffer_count = DMA_BUF_COUNT,
        .dma_buffer_size = BUF_BYTES,
    };

    ESP_ERROR_CHECK(i2s_mic_init(&cfg));
    ESP_ERROR_CHECK(i2s_mic_start());

    xTaskCreate(audio_sender_task, "audio_sender", 4096, NULL, 5, NULL);
    xTaskCreate(overflow_report_task, "ovf_report", 2048, NULL, 1, NULL);
}

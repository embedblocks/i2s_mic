/* Host tests for i2s_mic.c against the simulated v6.0 driver. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "i2s_mic.h"
#include "freertos/FreeRTOS.h"

extern int sim_read_entry_delay_ms;
extern volatile int sim_in_entry_delay;

#define RATE      16000
#define FRAMES    480                      /* 30 ms */
#define BUFBYTES  (FRAMES * 2 * 4)         /* stereo 32-bit = 3840 */
#define PERIOD_MS 30

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static double ms_now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

static i2s_mic_config_t cfg_with(int size)
{
    i2s_mic_config_t c = {
        .sample_rate = RATE, .bits_per_sample = 32, .channel_count = 2,
        .slot_mode = I2S_SLOT_MODE_STEREO, .gpio_bck = 1, .gpio_ws = 2, .gpio_data = 3,
        .port = 0, .dma_buffer_count = 6, .dma_buffer_size = size,
    };
    return c;
}

/* ---- reader thread ---------------------------------------------------- */
typedef struct {
    uint32_t timeout_ms;
    volatile int stop_after_stall_ms;   /* if >0, stall once after warmup reads */
    int warmup;
    /* results */
    int ok, timeouts, invalid, other, bad_len, gaps, reads;
    int last_ret;
    double exit_ms;
    volatile int done;
} reader_t;

static uint8_t rbuf[BUFBYTES];

static void *reader(void *arg)
{
    reader_t *r = arg;
    long last_seq = -1;
    while (1) {
        size_t n = 12345;
        esp_err_t e = i2s_mic_read(rbuf, BUFBYTES, &n, r->timeout_ms);
        r->last_ret = e;
        r->reads++;
        if (e == ESP_OK) {
            r->ok++;
            if (n != BUFBYTES) r->bad_len++;
            uint32_t s0, s1;
            memcpy(&s0, rbuf, 4);
            memcpy(&s1, rbuf + BUFBYTES - 4, 4);
            if (s0 != s1) r->bad_len++;      /* torn or partial */
            if (last_seq >= 0 && (long)s0 != last_seq + 1) r->gaps += (int)((long)s0 - last_seq - 1);
            last_seq = s0;
            if (r->stop_after_stall_ms > 0 && r->ok == r->warmup) {
                usleep((useconds_t)r->stop_after_stall_ms * 1000);
                r->stop_after_stall_ms = 0;
            }
        } else {
            if (n != 0) r->bad_len++;
            if (e == ESP_ERR_TIMEOUT) r->timeouts++;
            else if (e == ESP_ERR_INVALID_STATE) { r->invalid++; break; }
            else { r->other++; break; }
        }
    }
    r->exit_ms = ms_now();
    r->done = 1;
    return NULL;
}

static int join_with_timeout(pthread_t th, reader_t *r, int ms)
{
    double t0 = ms_now();
    while (!r->done && ms_now() - t0 < ms) usleep(1000);
    if (!r->done) return -1;
    pthread_join(th, NULL);
    return 0;
}

/* ---- tests ------------------------------------------------------------ */

static void t_init_sizes(void)
{
    printf("T1 init rejects a buffer the driver would shrink\n");
    i2s_mic_config_t c = cfg_with(4096);
    CHECK(i2s_mic_init(&c) == ESP_ERR_INVALID_SIZE, "4096 bytes should be rejected");
    c = cfg_with(4092);  /* 511.5 frames: not whole frames -> INVALID_ARG */
    CHECK(i2s_mic_init(&c) == ESP_ERR_INVALID_ARG, "4092 is not a whole number of 8-byte frames");
    c = cfg_with(4088);
    CHECK(i2s_mic_init(&c) == ESP_OK, "4088 (511 frames) should fit");
    CHECK(i2s_mic_get_buffer_size() == 4088, "buffer size should be 4088");
    CHECK(i2s_mic_deinit() == ESP_OK, "deinit");
    CHECK(i2s_mic_get_buffer_size() == 0, "size 0 after deinit");
}

static void t_not_running(void)
{
    printf("T2 reads before start / wrong length / after stop return at once\n");
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    CHECK(i2s_mic_init(&c) == ESP_OK, "init");
    size_t n = 7;
    double t0 = ms_now();
    CHECK(i2s_mic_read(rbuf, BUFBYTES, &n, 0xffffffff) == ESP_ERR_INVALID_STATE, "read before start");
    CHECK(n == 0, "bytes_read 0");
    CHECK(ms_now() - t0 < 5, "read before start should return immediately");
    CHECK(i2s_mic_start() == ESP_OK, "start");
    CHECK(i2s_mic_read(rbuf, BUFBYTES - 8, &n, 100) == ESP_ERR_INVALID_SIZE, "wrong len");
    CHECK(i2s_mic_read(NULL, BUFBYTES, &n, 100) == ESP_ERR_INVALID_ARG, "NULL dst");
    CHECK(i2s_mic_read(rbuf, BUFBYTES, &n, 1000) == ESP_OK && n == BUFBYTES, "one good read");
    CHECK(i2s_mic_stop() == ESP_OK, "stop");
    t0 = ms_now();
    CHECK(i2s_mic_read(rbuf, BUFBYTES, &n, 0xffffffff) == ESP_ERR_INVALID_STATE, "read after stop");
    CHECK(ms_now() - t0 < 5, "read after stop should return immediately");
    CHECK(i2s_mic_deinit() == ESP_OK, "deinit");
    CHECK(i2s_mic_read(rbuf, BUFBYTES, &n, 10) == ESP_ERR_INVALID_STATE, "read after deinit");
}

static void t_timeouts(void)
{
    printf("T3 short timeouts return ESP_ERR_TIMEOUT and consume nothing\n");
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    i2s_mic_init(&c);
    i2s_mic_start();
    size_t n;
    usleep(200 * 1000);          /* let the 5-entry queue fill (and overflow a bit) */
    for (int i = 0; i < 6; i++) i2s_mic_read(rbuf, BUFBYTES, &n, 1000);  /* drain */
    esp_err_t e0 = i2s_mic_read(rbuf, BUFBYTES, &n, 0);
    esp_err_t e5 = i2s_mic_read(rbuf, BUFBYTES, &n, 5);
    CHECK(e0 == ESP_ERR_TIMEOUT || e0 == ESP_OK, "timeout 0 -> TIMEOUT or OK (got %s)", esp_err_to_name(e0));
    CHECK(e5 == ESP_ERR_TIMEOUT || e5 == ESP_OK, "timeout 5 -> TIMEOUT or OK (got %s)", esp_err_to_name(e5));
    double t0 = ms_now();
    int timeouts = 0;
    for (int i = 0; i < 20; i++) if (i2s_mic_read(rbuf, BUFBYTES, &n, 0) == ESP_ERR_TIMEOUT) timeouts++;
    CHECK(timeouts >= 15, "back-to-back zero-timeout reads mostly time out (%d/20)", timeouts);
    CHECK(ms_now() - t0 < 100, "zero-timeout reads do not block");
    i2s_mic_stop();
    i2s_mic_deinit();
}

static void t_keep_up(void)
{
    printf("T4 reader keeps up for 3 s: no loss\n");
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    i2s_mic_init(&c);
    i2s_mic_start();
    reader_t r = { .timeout_ms = 0xffffffff };
    pthread_t th;
    pthread_create(&th, NULL, reader, &r);
    usleep(3000 * 1000);
    i2s_mic_stop();
    CHECK(join_with_timeout(th, &r, 500) == 0, "reader exits after stop");
    printf("  reads ok=%d gaps=%d overflow=%u\n", r.ok, r.gaps, (unsigned)i2s_mic_get_overflow_count());
    CHECK(r.ok >= 95, "about 100 buffers in 3 s (got %d)", r.ok);
    CHECK(r.gaps == 0 && i2s_mic_get_overflow_count() == 0, "no loss");
    CHECK(r.bad_len == 0, "no short or torn buffers");
    i2s_mic_deinit();
}

static void t_stall(int stall_ms)
{
    printf("T5 one stall of %d ms\n", stall_ms);
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    i2s_mic_init(&c);
    i2s_mic_start();
    reader_t r = { .timeout_ms = 0xffffffff, .stop_after_stall_ms = stall_ms, .warmup = 10 };
    pthread_t th;
    pthread_create(&th, NULL, reader, &r);
    usleep((unsigned)(10 * PERIOD_MS + stall_ms + 1000) * 1000);
    uint32_t ovf = i2s_mic_get_overflow_count();
    i2s_mic_stop();
    join_with_timeout(th, &r, 500);
    int model = (stall_ms + PERIOD_MS - 1) / PERIOD_MS - 5;
    if (model < 0) model = 0;
    printf("  overflow=%u gaps=%d model~%d\n", (unsigned)ovf, r.gaps, model);
    CHECK((int)ovf == r.gaps, "overflow count equals buffers actually missing");
    CHECK(abs((int)ovf - model) <= 1, "within 1 of the model");
    CHECK(r.bad_len == 0, "no short or torn buffers");
    i2s_mic_deinit();
}

static void t_stop_blocked(int entry_delay_ms, const char *what)
{
    printf("T6 stop() while reader waits with portMAX_DELAY (%s)\n", what);
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    i2s_mic_init(&c);
    i2s_mic_start();
    reader_t r = { .timeout_ms = 0xffffffff };
    pthread_t th;
    pthread_create(&th, NULL, reader, &r);
    usleep(200 * 1000);
    sim_read_entry_delay_ms = entry_delay_ms;
    if (entry_delay_ms) {
        /* wait until the reader has passed i2s_mic's running check and is
         * on its way into the driver, then stop() completes before it
         * reaches the driver's semaphore */
        double w0 = ms_now();
        while (!sim_in_entry_delay && ms_now() - w0 < 1000) usleep(200);
        CHECK(sim_in_entry_delay, "reader reached the injected delay");
    } else {
        usleep(5 * 1000);
    }
    double t0 = ms_now();
    CHECK(i2s_mic_stop() == ESP_OK, "stop");
    double stop_ms = ms_now() - t0;
    int j = join_with_timeout(th, &r, 1000);
    sim_read_entry_delay_ms = 0;
    CHECK(j == 0, "reader must not stay blocked after stop()");
    if (j == 0) {
        printf("  stop() took %.1f ms, reader exited %.1f ms after stop() began, last=%s\n",
               stop_ms, r.exit_ms - t0, esp_err_to_name(r.last_ret));
        CHECK(r.exit_ms - t0 < 2 * PERIOD_MS + 2 * PERIOD_MS + entry_delay_ms + 30, "reader exits within about one slice");
    }
    CHECK(r.bad_len == 0, "no short ESP_OK");
    CHECK(i2s_mic_deinit() == ESP_OK, "deinit after reader exit");
}

static void t_two_readers_and_deinit(void)
{
    printf("T7 second reader rejected; deinit refused while reading\n");
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    i2s_mic_init(&c);
    i2s_mic_start();
    reader_t r = { .timeout_ms = 0xffffffff };
    pthread_t th;
    pthread_create(&th, NULL, reader, &r);
    usleep(100 * 1000);
    size_t n;
    int rejected = 0;
    for (int i = 0; i < 50; i++) {
        if (i2s_mic_read(rbuf, BUFBYTES, &n, 1) == ESP_ERR_INVALID_STATE) rejected++;
        usleep(1000);
    }
    CHECK(rejected >= 45, "second reader rejected while first is inside read (%d/50)", rejected);
    CHECK(i2s_mic_deinit() == ESP_ERR_INVALID_STATE, "deinit while running");
    i2s_mic_stop();
    /* the reader may still be inside read for a moment */
    join_with_timeout(th, &r, 1000);
    CHECK(i2s_mic_deinit() == ESP_OK, "deinit after reader exit");
}

static void t_stress(int cycles)
{
    printf("T8 %d start/stop cycles with a live reader and random stop timing\n", cycles);
    srand(1234);
    int hangs = 0, bad = 0;
    i2s_mic_config_t c = cfg_with(BUFBYTES);
    i2s_mic_init(&c);
    for (int k = 0; k < cycles; k++) {
        if (i2s_mic_start() != ESP_OK) { bad++; continue; }
        if (i2s_mic_get_overflow_count() != 0) bad++;
        reader_t r = { .timeout_ms = (k % 3 == 0) ? 0xffffffff : (uint32_t)(rand() % 100) };
        pthread_t th;
        pthread_create(&th, NULL, reader, &r);
        usleep((unsigned)(rand() % 80) * 1000);
        if (k % 7 == 0) sim_read_entry_delay_ms = rand() % 40;
        i2s_mic_stop();
        if (join_with_timeout(th, &r, 1000) != 0) { hangs++; sim_read_entry_delay_ms = 0; break; }
        sim_read_entry_delay_ms = 0;
        bad += r.bad_len + r.other;
        if (k % 25 == 24) { i2s_mic_deinit(); i2s_mic_init(&c); }
    }
    i2s_mic_deinit();
    printf("  hangs=%d bad=%d\n", hangs, bad);
    CHECK(hangs == 0, "no reader left blocked");
    CHECK(bad == 0, "no short/torn ESP_OK, no unexpected errors, counter reset on start");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("TICK_MS=%d\n", TICK_MS);
    t_init_sizes();
    t_not_running();
    t_timeouts();
    t_keep_up();
    t_stall(100);
    t_stall(300);
    t_stall(600);
    t_stop_blocked(0, "reader inside driver wait");
    t_stop_blocked(40, "read starts after stop() finished");
    t_two_readers_and_deinit();
    t_stress(200);
    printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", fails);
    return fails ? 1 : 0;
}

/*
 * Host simulation of the ESP-IDF v6.0 I2S RX driver semantics that i2s_mic
 * relies on (see i2s_common.c on release/v6.0):
 *   - RX ISR per DMA completion: on_recv, then if msg_queue full: drop oldest +
 *     on_recv_q_ovf, then always push.
 *   - msg_queue length = desc_num - 1.
 *   - i2s_channel_read(): take binary semaphore (timeout) else INVALID_STATE;
 *     loop while RUNNING; fetch a new buffer when current is used up, unset, or
 *     queue nearly full; xQueueReceive timeout -> ESP_ERR_TIMEOUT; give sem.
 *     State change mid-read -> loop exits with ESP_OK and a short count.
 *   - i2s_channel_disable(): state = READY, then take sem forever and keep it;
 *     reset curr_ptr; stop DMA.
 *   - i2s_channel_enable(): state = RUNNING, reset RX queue, start DMA, give sem.
 *   - Buffer size capped at 4092 bytes (rounded down to whole frames).
 *   - Timeouts go through pdMS_TO_TICKS, i.e. are rounded down to whole ticks.
 * DMA buffers are stamped with a running sequence number so tests can count
 * gaps (lost buffers) independently of the overflow counter.
 */
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include "driver/i2s_std.h"
#include "freertos/task.h"

#define DMA_MAX 4092
enum { ST_REGISTERED, ST_READY, ST_RUNNING };

struct sim_chan {
    pthread_mutex_t m;
    pthread_cond_t cv;
    volatile int state;
    int sem;
    uint8_t *q[64];
    int qhead, qcount, qcap;
    int desc;
    size_t buf_size;
    uint8_t **bufs;
    int link;
    bool dma_on, alive;
    pthread_t th;
    uint32_t seq;
    uint32_t frames, rate, bpf;
    i2s_event_callbacks_t cbs;
    void *ud;
    uint8_t *curr;
    size_t rw_pos;
};

int sim_read_entry_delay_ms = 0;   /* test hook: delay before taking the semaphore */
volatile int sim_in_entry_delay = 0;
static struct timespec t_start;

static uint64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ull + t.tv_nsec / 1000;
}

TickType_t xTaskGetTickCount(void)
{
    if (t_start.tv_sec == 0) clock_gettime(CLOCK_MONOTONIC, &t_start);
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t ms = (uint64_t)(t.tv_sec - t_start.tv_sec) * 1000 + (t.tv_nsec - t_start.tv_nsec) / 1000000;
    return (TickType_t)(ms / TICK_MS);
}

const char *esp_err_to_name(esp_err_t e)
{
    switch (e) {
    case ESP_OK: return "ESP_OK";
    case ESP_ERR_TIMEOUT: return "ESP_ERR_TIMEOUT";
    case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_INVALID_SIZE: return "ESP_ERR_INVALID_SIZE";
    case ESP_ERR_INVALID_ARG: return "ESP_ERR_INVALID_ARG";
    default: return "ESP_ERR_?";
    }
}

/* pdMS_TO_TICKS rounding, then wait on cv with an absolute deadline. */
static bool deadline_for(uint32_t timeout_ms, struct timespec *ts)
{
    uint64_t ticks = ((uint64_t)timeout_ms * (1000 / TICK_MS)) / 1000;
    if (ticks >= 0xffffffffull) return false;     /* effectively forever */
    uint64_t ms = ticks * TICK_MS;
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (ms % 1000) * 1000000;
    if (ts->tv_nsec >= 1000000000) { ts->tv_sec++; ts->tv_nsec -= 1000000000; }
    return true;
}

static void *dma_thread(void *arg)
{
    struct sim_chan *c = arg;
    uint64_t period_us = (uint64_t)c->frames * 1000000ull / c->rate;
    uint64_t next = now_us() + period_us;
    while (1) {
        uint64_t t = now_us();
        if (t < next) usleep((useconds_t)(next - t));
        next += period_us;
        pthread_mutex_lock(&c->m);
        if (!c->alive) { pthread_mutex_unlock(&c->m); break; }
        if (c->dma_on) {
            uint8_t *fin = c->bufs[c->link];
            uint32_t s = c->seq++;
            for (size_t i = 0; i + 4 <= c->buf_size; i += 4) memcpy(fin + i, &s, 4);
            c->link = (c->link + 1) % c->desc;
            i2s_event_data_t ev = { .dma_buf = fin, .size = c->buf_size };
            if (c->cbs.on_recv) c->cbs.on_recv(c, &ev, c->ud);
            if (c->qcount == c->qcap) {
                c->qhead = (c->qhead + 1) % c->qcap;
                c->qcount--;
                if (c->cbs.on_recv_q_ovf) c->cbs.on_recv_q_ovf(c, &ev, c->ud);
            }
            c->q[(c->qhead + c->qcount) % c->qcap] = fin;
            c->qcount++;
            pthread_cond_broadcast(&c->cv);
        } else {
            next = now_us() + period_us;
        }
        pthread_mutex_unlock(&c->m);
    }
    return NULL;
}

esp_err_t i2s_new_channel(const i2s_chan_config_t *cfg, i2s_chan_handle_t *tx, i2s_chan_handle_t *rx)
{
    (void)tx;
    if (cfg->dma_desc_num < 2) return ESP_ERR_INVALID_ARG;
    struct sim_chan *c = calloc(1, sizeof(*c));
    pthread_mutex_init(&c->m, NULL);
    pthread_cond_init(&c->cv, NULL);
    c->desc = (int)cfg->dma_desc_num;
    c->qcap = c->desc - 1;
    c->frames = cfg->dma_frame_num;
    c->state = ST_REGISTERED;
    *rx = c;
    return ESP_OK;
}

esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t c, const i2s_std_config_t *cfg)
{
    c->rate = cfg->clk_cfg.rate;
    c->bpf = (uint32_t)(cfg->slot_cfg.bits / 8) * (cfg->slot_cfg.mode == I2S_SLOT_MODE_STEREO ? 2 : 1);
    size_t sz = (size_t)c->frames * c->bpf;
    if (sz > DMA_MAX) {
        c->frames = DMA_MAX / c->bpf;
        sz = (size_t)c->frames * c->bpf;
    }
    c->buf_size = sz;
    c->bufs = calloc((size_t)c->desc, sizeof(uint8_t *));
    for (int i = 0; i < c->desc; i++) c->bufs[i] = calloc(1, sz);
    c->state = ST_READY;
    c->alive = true;
    pthread_create(&c->th, NULL, dma_thread, c);
    return ESP_OK;
}

esp_err_t i2s_channel_get_info(i2s_chan_handle_t c, i2s_chan_info_t *info)
{
    info->total_dma_buf_size = c->state >= ST_READY ? (uint32_t)(c->desc * c->buf_size) : 0;
    return ESP_OK;
}

esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t c, const i2s_event_callbacks_t *cbs, void *ud)
{
    if (c->state == ST_RUNNING) return ESP_ERR_INVALID_STATE;
    c->cbs = *cbs;
    c->ud = ud;
    return ESP_OK;
}

esp_err_t i2s_channel_enable(i2s_chan_handle_t c)
{
    pthread_mutex_lock(&c->m);
    if (c->state != ST_READY) { pthread_mutex_unlock(&c->m); return ESP_ERR_INVALID_STATE; }
    c->state = ST_RUNNING;
    c->qhead = c->qcount = 0;           /* xQueueReset */
    c->dma_on = true;
    c->sem = 1;                          /* xSemaphoreGive(binary) */
    pthread_cond_broadcast(&c->cv);
    pthread_mutex_unlock(&c->m);
    return ESP_OK;
}

esp_err_t i2s_channel_disable(i2s_chan_handle_t c)
{
    pthread_mutex_lock(&c->m);
    if (c->state != ST_RUNNING) { pthread_mutex_unlock(&c->m); return ESP_ERR_INVALID_STATE; }
    c->state = ST_READY;
    while (c->sem == 0) pthread_cond_wait(&c->cv, &c->m);   /* take binary, forever */
    c->sem = 0;                                              /* ...and keep it */
    c->curr = NULL;
    c->rw_pos = 0;
    c->dma_on = false;
    pthread_mutex_unlock(&c->m);
    return ESP_OK;
}

esp_err_t i2s_channel_read(i2s_chan_handle_t c, void *dst, size_t size, size_t *n, uint32_t timeout_ms)
{
    if (n) *n = 0;
    if (sim_read_entry_delay_ms) {
        sim_in_entry_delay = 1;
        usleep((useconds_t)sim_read_entry_delay_ms * 1000);
        sim_in_entry_delay = 0;
    }
    struct timespec ts;
    bool timed;

    pthread_mutex_lock(&c->m);
    timed = deadline_for(timeout_ms, &ts);
    while (c->sem == 0) {
        if (!timed) pthread_cond_wait(&c->cv, &c->m);
        else if (pthread_cond_timedwait(&c->cv, &c->m, &ts) == ETIMEDOUT && c->sem == 0) {
            pthread_mutex_unlock(&c->m);
            return ESP_ERR_INVALID_STATE;     /* "The channel is not enabled" */
        }
    }
    c->sem = 0;

    esp_err_t ret = ESP_OK;
    uint8_t *d = dst;
    while (size > 0 && c->state == ST_RUNNING) {
        if (c->rw_pos == c->buf_size || c->curr == NULL || (c->qcap - c->qcount) <= 1) {
            timed = deadline_for(timeout_ms, &ts);
            while (c->qcount == 0) {
                if (!timed) pthread_cond_wait(&c->cv, &c->m);
                else if (pthread_cond_timedwait(&c->cv, &c->m, &ts) == ETIMEDOUT && c->qcount == 0) break;
            }
            if (c->qcount == 0) { ret = ESP_ERR_TIMEOUT; break; }
            c->curr = c->q[c->qhead];
            c->qhead = (c->qhead + 1) % c->qcap;
            c->qcount--;
            c->rw_pos = 0;
        }
        size_t k = c->buf_size - c->rw_pos;
        if (k > size) k = size;
        memcpy(d, c->curr + c->rw_pos, k);
        size -= k; d += k; c->rw_pos += k;
        if (n) *n += k;
    }
    c->sem = 1;                                  /* xSemaphoreGive(binary) */
    pthread_cond_broadcast(&c->cv);
    pthread_mutex_unlock(&c->m);
    return ret;
}

esp_err_t i2s_del_channel(i2s_chan_handle_t c)
{
    pthread_mutex_lock(&c->m);
    if (c->state == ST_RUNNING) { pthread_mutex_unlock(&c->m); return ESP_ERR_INVALID_STATE; }
    c->alive = false;
    pthread_mutex_unlock(&c->m);
    if (c->bufs) pthread_join(c->th, NULL);
    for (int i = 0; c->bufs && i < c->desc; i++) free(c->bufs[i]);
    free(c->bufs);
    free(c);
    return ESP_OK;
}

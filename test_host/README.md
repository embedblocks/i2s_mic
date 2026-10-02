# Host tests

`make` builds `../i2s_mic.c` against `sim_driver.c`, a host simulation of
the ESP-IDF v6.0 I2S RX driver behaviour the component depends on:

- interrupt order (`on_recv`, drop-oldest + `on_recv_q_ovf` when the queue
  is full, always push), queue length `dma_desc_num - 1`
- `i2s_channel_read()`: semaphore, loop while RUNNING, short `ESP_OK` when
  stopped mid-read, `ESP_ERR_TIMEOUT` from the queue wait, timeouts rounded
  down to whole ticks
- `i2s_channel_disable()` waiting forever for an in-progress read and
  keeping the semaphore; `i2s_channel_enable()` emptying the queue
- the 4092-byte DMA buffer cap

It runs at 10 ms and 1 ms tick rates and checks: init size validation,
immediate errors when not running, zero/short timeouts, no loss while the
reader keeps up, stall loss matching the model and the overflow count
matching the buffers actually missing, `stop()` ending a read waiting with
`portMAX_DELAY` (including a read that starts right after `stop()`
finished), single-reader enforcement, `deinit()` refusal during a read, and
200 random start/stop cycles.

This exercises the wrapper's logic and lifecycle races. It does not replace
the hardware acceptance tests: real interrupt latency, DMA timing and
scheduling are not simulated.

# i2s_mic hardware tests

This project runs the Unity tests in `../test` on a real board and prints a
pass/fail summary. The tests live in their own component (`i2s_mic/test`),
so any other project can include them too; this app is just the runner.

```
i2s_mic/
├── i2s_mic.c, i2s_mic.h        the component
├── test/                       test component: Unity TEST_CASEs
│   ├── CMakeLists.txt          (WHOLE_ARCHIVE, requires unity + i2s_mic)
│   ├── Kconfig                 pins, port, cycle count
│   ├── test_i2s_mic_common.*   shared config + reader task helper
│   ├── test_contract.c         [contract]  return values, read contract
│   ├── test_flow.c             [flow]      keep-up, stalls, loss accounting
│   └── test_lifecycle.c        [lifecycle] stop latency, load, cycles, deinit
├── test_app/                   this project: runs the tests
└── test_host/                  PC simulation of the driver (make)
```

## Hardware

No microphone is needed. The ESP32 drives the I2S clocks itself, so DMA
buffers complete on time with nothing connected; the tests check timing and
control flow, not audio content. If a mic is wired, keep it: just make sure
the pins match.

| Target | BCK | WS | DATA |
|---|---|---|---|
| ESP32 | 16 | 17 | 18 |
| ESP32-C3 | 7 | 8 | 9 |

Change them in `idf.py menuconfig` → *i2s_mic tests*.

## Run

```bash
cd i2s_mic/test_app
idf.py set-target esp32c3        # or esp32
idf.py build flash monitor
```

Every test runs on boot (about 1–2 minutes, most of it the 1,000 start/stop
cycles). The log ends with Unity's summary:

```
-----------------------
16 Tests 0 Failures 0 Ignored
OK
```

To pick tests by hand instead, enable *i2s_mic test app* → *Show the Unity
menu* in menuconfig. Then type a test number, or a tag such as `[flow]`.
Fewer cycles: *i2s_mic tests* → *Start/stop cycles*.

## Reading the results

Besides pass/fail, several tests print measurements. Please send the whole
log; these lines are the interesting ones:

| Line | Meaning |
|---|---|
| `received X + lost Y = Z, hardware completed ~W buffers` | Every buffer the hardware produced was either read or counted as lost. Z must be within W−3 … W+1. |
| `STALL 300 ms: lost 5 buffers (model 5)` | Loss after a reader stall, against the queue-depth model (±1 allowed). |
| `20 stops: stop() max … us, reader exit max … us` | How long `stop()` takes and how fast a blocked reader returns. Limits: 60 ms and 120 ms. |
| `under load: stop() … us, reader exit … us` | Same, with a higher-priority task using most of the CPU. |
| `MALLOC_CAP_8BIT: before …, after … (delta …)` | Heap check after every test. |

The stall model and timing limits are first estimates. If a test fails by a
small margin, the printed numbers tell us whether the component or the
tolerance is wrong.

## Not covered here

The race where a read starts in the instant after `stop()` finishes can't be
forced on hardware without hooks; `test_host/` forces it deterministically
in simulation. On hardware the cycle test hits `stop()` at random points
1,000 times.

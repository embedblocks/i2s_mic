# Host-side tests: i2s_mic.c against a simulation of the ESP-IDF v6.0 I2S RX
# driver semantics it relies on (see sim_driver.c). Linux/macOS, needs gcc
# and pthreads. Not part of the ESP-IDF build.
CFLAGS ?= -O1 -g -Wall -Wextra -Wno-unused-parameter
SRCS = ../i2s_mic.c sim_driver.c test.c

all: run

test_tick10: $(SRCS)
	$(CC) $(CFLAGS) -DTICK_MS=10 -Ifake -I.. $(SRCS) -o $@ -lpthread

test_tick1: $(SRCS)
	$(CC) $(CFLAGS) -DTICK_MS=1 -Ifake -I.. $(SRCS) -o $@ -lpthread

run: test_tick10 test_tick1
	./test_tick10
	./test_tick1

clean:
	rm -f test_tick10 test_tick1

.PHONY: all run clean

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#ifndef TICK_MS
#define TICK_MS 10
#endif
typedef uint32_t TickType_t;
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
#define portTICK_PERIOD_MS ((TickType_t)TICK_MS)
typedef struct { pthread_mutex_t m; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED { PTHREAD_MUTEX_INITIALIZER }
#define portENTER_CRITICAL(p) pthread_mutex_lock(&(p)->m)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(&(p)->m)

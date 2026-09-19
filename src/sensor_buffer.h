/*
 * sensor_buffer.h
 *
 * Generic data-history buffer used by the FDI engine. One instance
 * per sensor (US1, US2, IMU1, IMU2) — no combined "redundant pair"
 * buffer; redundancy is interpreted by the FDI engine, not storage.
 *
 * A single sample type (value[] + value_count) covers both
 * single-value sensors (ultrasonic distance) and multi-value sensors
 * (IMU accel + gyro) without needing per-type buffer code.
 */

#ifndef SENSOR_BUFFER_H
#define SENSOR_BUFFER_H

#include <stdint.h>
#include <pthread.h>

#include "mq_names.h"

#define SENSOR_BUFFER_LEN   10   /* samples of history per sensor */
#define SENSOR_MAX_VALUES   6    /* IMU needs 6 (ax,ay,az,gx,gy,gz); ultrasonic uses 1 */

typedef struct {
    sensor_id_t sensor_id;
    uint64_t    timestamp_us;
    uint32_t    sequence;
    uint8_t     value_count;
    float       value[SENSOR_MAX_VALUES];
} sensor_sample_t;

typedef struct {
    sensor_id_t     sensor_id;
    sensor_sample_t samples[SENSOR_BUFFER_LEN];
    uint8_t         write_index;
    uint8_t         sample_count;
    uint32_t        next_sequence;
    uint32_t        expected_period_us;
    pthread_mutex_t lock;
} sensor_buffer_t;

/* Initializes the buffer. expected_period_us is stored for later use
 * by delay/timing detection — the buffer itself does not enforce it. */
int  sensor_buffer_init(sensor_buffer_t *buf, sensor_id_t sensor_id, uint32_t expected_period_us);
void sensor_buffer_destroy(sensor_buffer_t *buf);

/* Writes one new sample. value_count must be <= SENSOR_MAX_VALUES.
 * Overwrites the oldest slot once the buffer is full. Assigns and
 * increments the sequence number internally. Returns 0 on success. */
int  sensor_buffer_write(sensor_buffer_t *buf, uint64_t timestamp_us,
                          const float *values, uint8_t value_count);

/* Copies up to max_out samples, oldest-first, into out. Writes the
 * actual count into *out_count. Returns 0 on success. */
int  sensor_buffer_read_history(sensor_buffer_t *buf, sensor_sample_t *out,
                                 uint8_t max_out, uint8_t *out_count);

/* Copies the most recent sample into out. Returns -1 if the buffer is empty. */
int  sensor_buffer_get_latest(sensor_buffer_t *buf, sensor_sample_t *out);

uint8_t  sensor_buffer_count(sensor_buffer_t *buf);
uint32_t sensor_buffer_expected_period(sensor_buffer_t *buf);
uint64_t sensor_buffer_last_timestamp(sensor_buffer_t *buf);

#endif /* SENSOR_BUFFER_H */

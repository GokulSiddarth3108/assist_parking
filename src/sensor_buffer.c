/*
 * sensor_buffer.c
 *
 * Storage layer only — no sleep, no timer scheduling, no sensor
 * acquisition, no fault detection. Newest sample overwrites oldest
 * once full (slot 10 -> slot 0, per README section 10).
 */

#include <string.h>

#include "sensor_buffer.h"

int sensor_buffer_init(sensor_buffer_t *buf, sensor_id_t sensor_id, uint32_t expected_period_us)
{
    if (buf == NULL) {
        return -1;
    }

    memset(buf, 0, sizeof(*buf));
    buf->sensor_id           = sensor_id;
    buf->expected_period_us  = expected_period_us;

    if (pthread_mutex_init(&buf->lock, NULL) != 0) {
        return -1;
    }

    return 0;
}

void sensor_buffer_destroy(sensor_buffer_t *buf)
{
    if (buf == NULL) {
        return;
    }

    pthread_mutex_destroy(&buf->lock);
}

int sensor_buffer_write(sensor_buffer_t *buf, uint64_t timestamp_us,
                         const float *values, uint8_t value_count)
{
    if (buf == NULL || values == NULL || value_count == 0 ||
        value_count > SENSOR_MAX_VALUES)
    {
        return -1;
    }

    pthread_mutex_lock(&buf->lock);

    sensor_sample_t *slot = &buf->samples[buf->write_index];

    slot->sensor_id     = buf->sensor_id;
    slot->timestamp_us  = timestamp_us;
    slot->sequence      = buf->next_sequence++;
    slot->value_count   = value_count;
    memcpy(slot->value, values, value_count * sizeof(float));

    buf->write_index = (uint8_t)((buf->write_index + 1) % SENSOR_BUFFER_LEN);

    if (buf->sample_count < SENSOR_BUFFER_LEN) {
        buf->sample_count++;
    }

    pthread_mutex_unlock(&buf->lock);

    return 0;
}

int sensor_buffer_read_history(sensor_buffer_t *buf, sensor_sample_t *out,
                                uint8_t max_out, uint8_t *out_count)
{
    if (buf == NULL || out == NULL || out_count == NULL) {
        return -1;
    }

    pthread_mutex_lock(&buf->lock);

    uint8_t count = buf->sample_count;
    if (count > max_out) {
        count = max_out;
    }

    /* oldest-first: start "count" slots behind write_index */
    uint8_t start = (uint8_t)((buf->write_index + SENSOR_BUFFER_LEN - count) % SENSOR_BUFFER_LEN);

    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (uint8_t)((start + i) % SENSOR_BUFFER_LEN);
        out[i] = buf->samples[idx];
    }

    *out_count = count;

    pthread_mutex_unlock(&buf->lock);

    return 0;
}

int sensor_buffer_get_latest(sensor_buffer_t *buf, sensor_sample_t *out)
{
    if (buf == NULL || out == NULL) {
        return -1;
    }

    pthread_mutex_lock(&buf->lock);

    if (buf->sample_count == 0) {
        pthread_mutex_unlock(&buf->lock);
        return -1;
    }

    uint8_t latest_idx = (uint8_t)((buf->write_index + SENSOR_BUFFER_LEN - 1) % SENSOR_BUFFER_LEN);
    *out = buf->samples[latest_idx];

    pthread_mutex_unlock(&buf->lock);

    return 0;
}

uint8_t sensor_buffer_count(sensor_buffer_t *buf)
{
    if (buf == NULL) {
        return 0;
    }

    pthread_mutex_lock(&buf->lock);
    uint8_t count = buf->sample_count;
    pthread_mutex_unlock(&buf->lock);

    return count;
}

uint32_t sensor_buffer_expected_period(sensor_buffer_t *buf)
{
    if (buf == NULL) {
        return 0;
    }

    return buf->expected_period_us;
}

uint64_t sensor_buffer_last_timestamp(sensor_buffer_t *buf)
{
    if (buf == NULL) {
        return 0;
    }

    pthread_mutex_lock(&buf->lock);

    uint64_t ts = 0;
    if (buf->sample_count > 0) {
        uint8_t latest_idx = (uint8_t)((buf->write_index + SENSOR_BUFFER_LEN - 1) % SENSOR_BUFFER_LEN);
        ts = buf->samples[latest_idx].timestamp_us;
    }

    pthread_mutex_unlock(&buf->lock);

    return ts;
}

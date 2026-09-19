/*
 * mq_names.h
 *
 * Central system-contract header. Defines the identifiers shared
 * across acquisition, buffering and (eventually) fault detection so
 * no module invents its own sensor IDs or state values.
 *

 */

#ifndef MQ_NAMES_H
#define MQ_NAMES_H

typedef enum {
    SENSOR_US1 = 0,
    SENSOR_US2,
    SENSOR_IMU1,
    SENSOR_IMU2,
    SENSOR_COUNT
} sensor_id_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_DELAYED,
    FAULT_STUCK,
    FAULT_NOISY,
    FAULT_CONTRADICTORY
} fault_type_t;

typedef enum {
    STATE_HEALTHY = 0,
    STATE_SUSPECT,
    STATE_FAULTED,
    STATE_EXCLUDED,
    STATE_PROBING,
    STATE_VALIDATING
} sensor_state_t;

static inline sensor_id_t sensor_redundant_pair(sensor_id_t id)
{
    switch (id) {
        case SENSOR_US1:  return SENSOR_US2;
        case SENSOR_US2:  return SENSOR_US1;
        case SENSOR_IMU1: return SENSOR_IMU2;
        case SENSOR_IMU2: return SENSOR_IMU1;
        default:          return SENSOR_COUNT;
    }
}

static inline const char *sensor_name(sensor_id_t id)
{
    switch (id) {
        case SENSOR_US1:  return "US1";
        case SENSOR_US2:  return "US2";
        case SENSOR_IMU1: return "IMU1";
        case SENSOR_IMU2: return "IMU2";
        default:          return "UNKNOWN";
    }
}

#endif /* MQ_NAMES_H */

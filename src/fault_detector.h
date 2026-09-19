/*
 * fault_detection.h
 */

#ifndef FAULT_DETECTION_H
#define FAULT_DETECTION_H

#include <stdint.h>

#include "mq_names.h"
#include "sensor_buffer.h"

typedef struct {
    uint32_t delay_timeout_us;        /* e.g. 500000 */
    float    stuck_threshold;         /* e.g. 0.5 (US) or 0.01 (IMU) */
    float    noise_threshold;         /* e.g. 30 (US) or 5 (IMU) */
    float    contradiction_threshold; /* e.g. 70 (US) or per-axis for IMU */
    uint8_t  fault_confirm_limit;     /* e.g. 3 */
    uint8_t  recovery_confirm_limit;  /* e.g. 3 */
} fdi_config_t;

typedef struct {
    sensor_id_t     id;
    sensor_id_t     redundant_id;
    fdi_config_t    cfg;

    sensor_state_t  state;
    fault_type_t    last_fault;

    uint8_t         fault_streak;      /* consecutive fault evaluations (HEALTHY->SUSPECT->FAULTED) */
    uint8_t         recovery_streak;   /* consecutive healthy evaluations (SUSPECT->HEALTHY) */
    uint8_t         validation_streak; /* consecutive healthy evaluations during VALIDATING */
} fdi_context_t;

void fdi_context_init(fdi_context_t *ctx, sensor_id_t id, const fdi_config_t *cfg);

/*
 * Runs DELAYED -> STUCK -> NOISY -> CONTRADICTORY checks in order
 * against self_buf (and redundant_buf for CONTRADICTORY) as of now_us.
 * Returns the first detected fault, or FAULT_NONE.
 */
fault_type_t fdi_detect(const fdi_context_t *ctx, sensor_buffer_t *self_buf,
                         sensor_buffer_t *redundant_buf, uint64_t now_us);

/*
 * Feeds one detection result into the state machine and returns the
 * resulting state. Handles HEALTHY<->SUSPECT<->FAULTED via
 * fault_confirm_limit/recovery_confirm_limit. Does NOT advance
 * EXCLUDED/PROBING/VALIDATING — those are driven explicitly below.
 */
sensor_state_t fdi_update_state(fdi_context_t *ctx, fault_type_t detected);

/* Isolation: caller (isolation manager) moves a FAULTED sensor to EXCLUDED. */
void fdi_isolate(fdi_context_t *ctx);

/* Recovery: begin probing an EXCLUDED sensor. */
void fdi_start_probe(fdi_context_t *ctx);

/*
 * Call once per evaluation while state == PROBING or VALIDATING, with
 * the latest fdi_detect() result for this sensor. On PROBING: any
 * FAULT_NONE moves to VALIDATING; any fault moves back to EXCLUDED.
 * On VALIDATING: recovery_confirm_limit consecutive FAULT_NONE moves
 * to HEALTHY (reintegrated); any fault moves back to EXCLUDED.
 * Returns the resulting state.
 */
sensor_state_t fdi_probe_evaluate(fdi_context_t *ctx, fault_type_t detected);

#endif /* FAULT_DETECTION_H */

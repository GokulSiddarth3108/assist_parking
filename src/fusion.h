/*
 * fusion.h
 *
 * Degraded-operation distance fusion for the parking-assist FDI.
 * Implements a graded degradation ladder for sensor failover.
 *
 * Zone decisions are made on the CONSERVATIVE edge of the estimate
 * (distance minus uncertainty) to strictly enforce the safety envelope.
 */

#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>

#include "sensor_manager.h"
#include "fault_detector.h"

/* System health and trust hierarchy */
typedef enum {
    OP_FULL = 0,
    OP_DEGRADED_SINGLE,
    OP_DEGRADED_HOLDOVER,
    OP_UNSAFE
} op_mode_t;

/* Fused telemetry and dynamic safety envelope */
typedef struct {
    op_mode_t mode;
    int       valid;            /* 0 = critical failure, command STOP */
    float     distance_cm;
    float     uncertainty_cm;
    float     safe_distance_cm; /* Computed as: distance - uncertainty */
    int       contributors;
    uint64_t  age_us;
    int       disagreement;     /* 1 = pair disagreed, took the closer reading */
} fusion_result_t;

/* Context for holdover (dead-reckoning) mode */
typedef struct {
    float    last_good_cm;
    uint64_t last_good_us;
    float    velocity_mps;
    uint64_t last_integrate_us;
    int      have_last_good;
} fusion_state_t;

void fusion_init(fusion_state_t *fs);

/*
 * Core fusion engine.
 * Guarantees a result; worst-case failover is OP_UNSAFE (valid == 0).
 */
void fusion_compute(fusion_state_t *fs,
                    sensor_manager_t *mgr,
                    const fdi_context_t *us1,
                    const fdi_context_t *us2,
                    const fdi_context_t *imu1,
                    const fdi_context_t *imu2,
                    uint64_t now_us,
                    fusion_result_t *out);

const char *op_mode_name(op_mode_t m);

/* Returns zone based on the conservative safe_distance_cm */
const char *fusion_zone_name(const fusion_result_t *r);

#endif /* FUSION_H */

/**
 * @file fault_detection.c
 * @brief Fault Detection and Isolation (FDI) module for sensor management.
 *
 * ARCHITECTURE & DATA FLOW:
 * This module implements an active health-monitoring state machine for sensors.
 * Data flows from hardware -> sensor_buffer_t -> FDI checks -> System State.
 *
 *
 * - Sensors begin HEALTHY.
 * - Any fault anomaly moves them to SUSPECT.
 * - Repeated anomalies transition them to FAULTED (except for symmetrical contradictions).
 * - External isolation managers move FAULTED sensors to EXCLUDED.
 * - Probes can be sent to EXCLUDED sensors to test for recovery (PROBING -> VALIDATING -> HEALTHY).
 */

#include <math.h>
#include <stddef.h>

#include "fault_detector.h"

/**
 * @brief Initializes the FDI context for a specific sensor.
 * @param ctx Pointer to the state context to initialize.
 * @param id The unique identifier for this sensor.
 * @param cfg Pointer to the threshold/limit configurations for this sensor type.
 */
void fdi_context_init(fdi_context_t *ctx, sensor_id_t id, const fdi_config_t *cfg)
{
    if (ctx == NULL || cfg == NULL) {
        return;
    }

    ctx->id                = id;
    ctx->redundant_id      = sensor_redundant_pair(id); // Automatically maps to physical twin if it exists
    ctx->cfg               = *cfg;


    ctx->state             = STATE_HEALTHY;
    ctx->last_fault        = FAULT_NONE;

    // Reset all confirmation counters
    ctx->fault_streak      = 0;
    ctx->recovery_streak   = 0;
    ctx->validation_streak = 0;
}

/**
 * @brief Validates if the sensor data is stale/delayed.
 * @return 1 if delayed beyond timeout, 0 if fresh.
 */
static int check_delayed(const fdi_context_t *ctx, sensor_buffer_t *buf, uint64_t now_us)
{
    if (sensor_buffer_count(buf) == 0) {
        return 1; /* Fault: Buffer is completely empty; no data ever arrived. */
    }

    uint64_t last = sensor_buffer_last_timestamp(buf);

    // Guard against timer rollover or future-stamped anomalous data
    if (now_us < last) {
        return 0;
    }

    // Fault: Time since last sample exceeds hardware-specific allowable delay
    return (now_us - last) > ctx->cfg.delay_timeout_us;
}

/**
 * @brief Evaluates if a sensor has "flatlined" (stuck at a single value).
 * @note Requires a fully populated window (SENSOR_BUFFER_LEN).
 * @return 1 if signal is stuck, 0 if healthy variation exists.
 */
static int check_stuck(const fdi_context_t *ctx, sensor_buffer_t *buf)
{
    sensor_sample_t hist[SENSOR_BUFFER_LEN];
    uint8_t n = 0;

    sensor_buffer_read_history(buf, hist, SENSOR_BUFFER_LEN, &n);
    if (n < SENSOR_BUFFER_LEN) {
        return 0;
    }

    uint8_t vc = hist[0].value_count;

    // Check every spatial/data component (e.g., X, Y, Z axes of an IMU)
    for (uint8_t c = 0; c < vc; c++) {
        float mn = hist[0].value[c];
        float mx = hist[0].value[c];

        for (uint8_t i = 1; i < n; i++) {
            float v = hist[i].value[c];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }

        // If even ONE data component varies by more than the threshold, the sensor is actively tracking.
        if ((mx - mn) > ctx->cfg.stuck_threshold) {
            return 0;
        }
    }

    // Fault: ALL components across the entire window remained deadlocked within the stuck threshold.
    return 1;
}


static int check_noisy(const fdi_context_t *ctx, sensor_buffer_t *buf)
{
    sensor_sample_t hist[SENSOR_BUFFER_LEN];
    uint8_t n = 0;

    sensor_buffer_read_history(buf, hist, SENSOR_BUFFER_LEN, &n);
    if (n < SENSOR_BUFFER_LEN) {
        return 0;
    }

    uint8_t vc = hist[0].value_count;

    for (uint8_t c = 0; c < vc; c++) {
        for (uint8_t i = 1; i < n; i++) {
            // Calculate absolute difference between immediate neighbor samples
            float delta = hist[i].value[c] - hist[i - 1].value[c];
            if (delta < 0.0f) {
                delta = -delta;
            }

            // Fault: Single jump exceeds physical possibility for this sample rate.
            if (delta > ctx->cfg.noise_threshold) {
                return 1;
            }
        }
    }

    return 0;
}

/**
 * @brief Compares this sensor against its physical redundant twin.
 * @return 1 if sensors disagree beyond acceptable error, 0 if they match.
 */
static int check_contradictory(const fdi_context_t *ctx, sensor_buffer_t *self_buf,
                                sensor_buffer_t *redundant_buf)
{
    if (redundant_buf == NULL) {
        return 0; /* Hardware configuration lacks a redundant pair; ignore check. */
    }

    sensor_sample_t a, b;
    if (sensor_buffer_get_latest(self_buf, &a) != 0) return 0;
    if (sensor_buffer_get_latest(redundant_buf, &b) != 0) return 0;

    // Handle mismatched sensor types gracefully by comparing only shared dimensions
    uint8_t vc = (a.value_count < b.value_count) ? a.value_count : b.value_count;
    float sumsq = 0.0f;

    // Calculate Euclidean distance (L2 norm) between the two sensor vectors
    for (uint8_t i = 0; i < vc; i++) {
        float d = a.value[i] - b.value[i];
        sumsq += d * d;
    }

    // Fault: The spatial/value difference between the two sensors exceeds allowed margin
    return sqrtf(sumsq) > ctx->cfg.contradiction_threshold;
}

/**
 * @brief Master detection orchestrator. Executes all fault checks in priority order.
 * @return The specific fault type detected, or FAULT_NONE.
 */
fault_type_t fdi_detect(const fdi_context_t *ctx, sensor_buffer_t *self_buf,
                         sensor_buffer_t *redundant_buf, uint64_t now_us)
{
    if (ctx == NULL || self_buf == NULL) {
        return FAULT_NONE;
    }

    // Priority hierarchy: A delayed sensor will fail all other checks;
    // a stuck sensor shouldn't be evaluated for noise. Order matters.
    if (check_delayed(ctx, self_buf, now_us))              return FAULT_DELAYED;
    if (check_stuck(ctx, self_buf))                        return FAULT_STUCK;
    if (check_noisy(ctx, self_buf))                        return FAULT_NOISY;
    if (check_contradictory(ctx, self_buf, redundant_buf)) return FAULT_CONTRADICTORY;

    return FAULT_NONE;
}

/**
 * @brief Primary state machine tick. Updates sensor status based on the latest detection.
 * @param detected The fault type returned by fdi_detect().
 * @return The new state of the sensor.
 */
sensor_state_t fdi_update_state(fdi_context_t *ctx, fault_type_t detected)
{
    if (ctx == NULL) {
        return STATE_HEALTHY;
    }

    ctx->last_fault = detected;

    switch (ctx->state) {
        case STATE_HEALTHY:
            if (detected != FAULT_NONE) {
                // First sign of trouble. Downgrade to suspect immediately.
                ctx->fault_streak = 1;
                ctx->state = STATE_SUSPECT;
            }
            break;

        case STATE_SUSPECT:
            if (detected != FAULT_NONE) {

                /*
                 * SYMMETRIC CONTRADICTION GUARD:
                 * If two paired sensors disagree, BOTH will read FAULT_CONTRADICTORY.
                 * We do not know which one is lying. If we allow contradiction to push
                 * the streak to `fault_confirm_limit`, BOTH sensors will hit FAULTED
                 * simultaneously, leaving the system with 0 usable data streams.
                 *
                 * Action: Reset recovery, but DO NOT increment the fault streak.
                 * We remain in SUSPECT (visible to logs/telemetry) until a definitive,
                 * asymmetric fault (Delayed, Stuck, Noisy) condemns one of the sensors.
                 */
                if (detected == FAULT_CONTRADICTORY) {
                    ctx->recovery_streak = 0;
                    break;
                }

                // Definitive fault (Delayed, Stuck, Noisy). Advance towards isolation.
                ctx->fault_streak++;
                ctx->recovery_streak = 0;

                if (ctx->fault_streak >= ctx->cfg.fault_confirm_limit) {
                    ctx->state = STATE_FAULTED;
                }
            } else {
                // Good data received. Advance towards recovery.
                ctx->recovery_streak++;

                if (ctx->recovery_streak >= ctx->cfg.recovery_confirm_limit) {
                    ctx->state = STATE_HEALTHY;
                    ctx->fault_streak = 0;
                    ctx->recovery_streak = 0;
                }
            }
            break;

        case STATE_FAULTED:
            /*
             * Sensor is condemned. State is frozen here until the higher-level
             * system architecture (Isolation Manager) explicitly calls fdi_isolate().
             */
            break;

        default:
            /* EXCLUDED, PROBING, and VALIDATING are handled exclusively by fdi_probe_evaluate() */
            break;
    }

    return ctx->state;
}

/**
 * @brief Invoked by the Isolation Manager to officially pull a FAULTED sensor from the data pool.
 */
void fdi_isolate(fdi_context_t *ctx)
{
    if (ctx != NULL && ctx->state == STATE_FAULTED) {
        ctx->state = STATE_EXCLUDED;
    }
}

/**
 * @brief Wakes up an EXCLUDED sensor to check if hardware has recovered (e.g., rebooted).
 */
void fdi_start_probe(fdi_context_t *ctx)
{
    if (ctx != NULL && ctx->state == STATE_EXCLUDED) {
        ctx->state = STATE_PROBING;
        ctx->validation_streak = 0;
    }
}

/**
 * @brief Secondary state machine tick for recovering sensors.
 * @note This bypasses fdi_update_state to prevent a recovering sensor from accidentally
 * mixing with healthy data pipelines before it is fully vetted.
 */
sensor_state_t fdi_probe_evaluate(fdi_context_t *ctx, fault_type_t detected)
{
    if (ctx == NULL) {
        return STATE_HEALTHY;
    }

    ctx->last_fault = detected;

    if (ctx->state == STATE_PROBING) {
        if (detected == FAULT_NONE) {
            // Survived the initial probe. Move to strict validation.
            ctx->state = STATE_VALIDATING;
            ctx->validation_streak = 1;
        } else {
            // Probe failed immediately. Send back to isolation.
            ctx->state = STATE_EXCLUDED;
        }
    } else if (ctx->state == STATE_VALIDATING) {
        if (detected == FAULT_NONE) {
            // Continues to provide good data. Advance towards full reinstatement.
            ctx->validation_streak++;
            if (ctx->validation_streak >= ctx->cfg.recovery_confirm_limit) {
                ctx->state = STATE_HEALTHY;
                ctx->fault_streak = 0;
                ctx->recovery_streak = 0;
                ctx->validation_streak = 0;
            }
        } else {
            // Failed validation partway through. Untrustworthy. Send back to isolation.
            ctx->state = STATE_EXCLUDED;
        }
    }

    return ctx->state;
}

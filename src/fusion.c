/*
 * fusion.c  — see fusion.h for the degradation ladder.
 */

#include <math.h>
#include <string.h>

#include "fusion.h"
#include "sensor_buffer.h"



/* Nominal single-shot accuracy of the ultrasonic ranger. */
#define US_BASE_UNCERTAINTY_CM      2.0f

/* A SUSPECT sensor is unconfirmed, not condemned: keep using it, but
 * inflate its contribution's uncertainty. */
#define SUSPECT_UNCERTAINTY_MULT    3.0f

/* Losing redundancy costs you the cross-check, so a lone sensor is
 * trusted less than the same sensor inside a healthy pair. */
#define SINGLE_UNCERTAINTY_MULT     2.5f

/* Newer than this or the sample does not count as a contributor.
 * 3 sample periods at 10 Hz. */
#define STALE_SAMPLE_US             300000ULL

/* If the pair differs by more than this, we stop averaging and take
 * the closer reading. */
#define PAIR_DISAGREE_CM            20.0f

/* How long we will dead-reckon before declaring OP_UNSAFE. */
#define HOLDOVER_MAX_US             1500000ULL

/* IMU dead-reckoning error growth, charged against the estimate. */
#define HOLDOVER_GROWTH_CM_PER_S    25.0f

/* Below this the accelerometer is reporting noise, not motion. */
#define ACCEL_DEADBAND_G            0.03f

#define G_TO_MPS2                   9.80665f

/* Zone thresholds (cm), matching the original main.c ladder. */
#define ZONE_SAFE_CM                100.0f
#define ZONE_NORMAL_CM              60.0f
#define ZONE_CAUTION_CM             30.0f
#define ZONE_SLOW_CM                15.0f

/* ---- Helpers ---- */

static int state_is_usable(sensor_state_t s)
{
    /* SUSPECT still contributes — that is the whole point of a graded
     * degrade. EXCLUDED/PROBING/VALIDATING/FAULTED do not. */
    return (s == STATE_HEALTHY || s == STATE_SUSPECT);
}

typedef struct {
    int      present;
    float    cm;
    float    sigma_cm;
    uint64_t age_us;
} contrib_t;

static void read_contributor(sensor_manager_t *mgr,
                             const fdi_context_t *ctx,
                             uint64_t now_us,
                             contrib_t *c)
{
    sensor_sample_t s;
    sensor_buffer_t *buf;

    memset(c, 0, sizeof(*c));

    if (!state_is_usable(ctx->state))
        return;

    buf = sensor_manager_get_buffer(mgr, ctx->id);
    if (buf == NULL)
        return;

    if (sensor_buffer_get_latest(buf, &s) != 0)
        return;

    /* NOTE: field name below must match sensor_sample_t in
     * sensor_buffer.h. Adjust if yours is named differently. */
    c->age_us = (now_us > s.timestamp_us) ? (now_us - s.timestamp_us) : 0ULL;

    if (c->age_us > STALE_SAMPLE_US)
        return;                       /* too old to be a contributor */

    c->cm       = s.value[0];
    c->sigma_cm = US_BASE_UNCERTAINTY_CM;
    if (ctx->state == STATE_SUSPECT)
        c->sigma_cm *= SUSPECT_UNCERTAINTY_MULT;

    c->present = 1;
}

/* Longitudinal acceleration in m/s^2 from whichever IMU is usable.
 * Returns 0 if no IMU can be trusted. */
static int read_long_accel(sensor_manager_t *mgr,
                           const fdi_context_t *imu1,
                           const fdi_context_t *imu2,
                           uint64_t now_us,
                           float *accel_mps2)
{
    const fdi_context_t *cands[2] = { imu1, imu2 };

    for (int i = 0; i < 2; i++) {
        sensor_sample_t s;
        sensor_buffer_t *buf;
        uint64_t age;
        float g;

        if (!state_is_usable(cands[i]->state))
            continue;

        buf = sensor_manager_get_buffer(mgr, cands[i]->id);
        if (buf == NULL)
            continue;
        if (sensor_buffer_get_latest(buf, &s) != 0)
            continue;

        age = (now_us > s.timestamp_us) ? (now_us - s.timestamp_us) : 0ULL;
        if (age > STALE_SAMPLE_US)
            continue;

        /* value[0] assumed to be longitudinal (vehicle X) accel in g.
         * Change the index if your mounting orientation differs. */
        g = s.value[0];
        if (fabsf(g) < ACCEL_DEADBAND_G)
            g = 0.0f;

        *accel_mps2 = g * G_TO_MPS2;
        return 1;
    }

    return 0;
}

/* ---- API ---- */

void fusion_init(fusion_state_t *fs)
{
    memset(fs, 0, sizeof(*fs));
}

void fusion_compute(fusion_state_t *fs,
                    sensor_manager_t *mgr,
                    const fdi_context_t *us1,
                    const fdi_context_t *us2,
                    const fdi_context_t *imu1,
                    const fdi_context_t *imu2,
                    uint64_t now_us,
                    fusion_result_t *out)
{
    contrib_t a, b;

    memset(out, 0, sizeof(*out));

    read_contributor(mgr, us1, now_us, &a);
    read_contributor(mgr, us2, now_us, &b);

    // Both ultrasonics contributing
    if (a.present && b.present) {
        float spread = fabsf(a.cm - b.cm);

        out->mode         = OP_FULL;
        out->valid        = 1;
        out->contributors = 2;
        out->age_us       = (a.age_us < b.age_us) ? a.age_us : b.age_us;

        if (spread > PAIR_DISAGREE_CM) {
            /* They contradict each other and FDI has not yet resolved
             * which one is lying. Believe the CLOSER one — a false
             * near reading costs a needless stop, a false far reading
             * costs a collision. */
            out->disagreement    = 1;
            out->distance_cm     = (a.cm < b.cm) ? a.cm : b.cm;
            out->uncertainty_cm  = spread;
        } else {
            /* Inverse-variance weighted mean. */
            float wa = 1.0f / (a.sigma_cm * a.sigma_cm);
            float wb = 1.0f / (b.sigma_cm * b.sigma_cm);

            out->distance_cm    = (a.cm * wa + b.cm * wb) / (wa + wb);
            out->uncertainty_cm = sqrtf(1.0f / (wa + wb)) + spread * 0.5f;
        }

        fs->last_good_cm      = out->distance_cm;
        fs->last_good_us      = now_us;
        fs->have_last_good    = 1;
        fs->velocity_mps      = 0.0f;
        fs->last_integrate_us = now_us;
    }
    /* ---------- One ultrasonic left ---------- */
    else if (a.present || b.present) {
        const contrib_t *c = a.present ? &a : &b;

        out->mode           = OP_DEGRADED_SINGLE;
        out->valid          = 1;
        out->contributors   = 1;
        out->age_us         = c->age_us;
        out->distance_cm    = c->cm;
        out->uncertainty_cm = c->sigma_cm * SINGLE_UNCERTAINTY_MULT;

        fs->last_good_cm      = out->distance_cm;
        fs->last_good_us      = now_us;
        fs->have_last_good    = 1;
        fs->velocity_mps      = 0.0f;
        fs->last_integrate_us = now_us;
    }
    /* ---------- No ultrasonic: holdover / dead reckoning ---------- */
    else {
        uint64_t held_us;
        float    accel;
        float    dt;

        if (!fs->have_last_good) {
            /* Never had a good reading — nothing to hold over from. */
            out->mode  = OP_UNSAFE;
            out->valid = 0;
            return;
        }

        held_us = now_us - fs->last_good_us;
        if (held_us > HOLDOVER_MAX_US) {
            out->mode   = OP_UNSAFE;
            out->valid  = 0;
            out->age_us = held_us;
            return;
        }

        dt = (float)(now_us - fs->last_integrate_us) / 1000000.0f;
        fs->last_integrate_us = now_us;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > 0.5f) dt = 0.5f;          /* guard against scheduling gaps */

        if (read_long_accel(mgr, imu1, imu2, now_us, &accel))
            fs->velocity_mps += accel * dt;
        /* No usable IMU: velocity holds its last value rather than
         * decaying to zero — decaying would optimistically imply the
         * vehicle stopped. */

        /* Sign of travel relative to the obstacle is not observable
         * from one accel axis alone, so assume the worst case: any
         * motion closes the gap. */
        out->distance_cm = fs->last_good_cm
                         - fabsf(fs->velocity_mps) * dt * 100.0f;
        fs->last_good_cm = out->distance_cm;   /* carry the decay forward */

        if (out->distance_cm < 0.0f)
            out->distance_cm = 0.0f;

        out->mode           = OP_DEGRADED_HOLDOVER;
        out->valid          = 1;
        out->contributors   = 0;
        out->age_us         = held_us;
        out->uncertainty_cm = US_BASE_UNCERTAINTY_CM
                            + HOLDOVER_GROWTH_CM_PER_S
                              * ((float)held_us / 1000000.0f);
    }

    out->safe_distance_cm = out->distance_cm - out->uncertainty_cm;
    if (out->safe_distance_cm < 0.0f)
        out->safe_distance_cm = 0.0f;
}

const char *op_mode_name(op_mode_t m)
{
    switch (m) {
        case OP_FULL:              return "FULL";
        case OP_DEGRADED_SINGLE:   return "DEGRADED/SINGLE";
        case OP_DEGRADED_HOLDOVER: return "DEGRADED/HOLDOVER";
        case OP_UNSAFE:            return "UNSAFE";
        default:                   return "UNKNOWN";
    }
}

const char *fusion_zone_name(const fusion_result_t *r)
{
    if (!r->valid) return "CRITICAL/STOP";

    /* Conservative edge, not the point estimate. */
    if (r->safe_distance_cm > ZONE_SAFE_CM)    return "SAFE";
    if (r->safe_distance_cm > ZONE_NORMAL_CM)  return "NORMAL";
    if (r->safe_distance_cm > ZONE_CAUTION_CM) return "CAUTION";
    if (r->safe_distance_cm > ZONE_SLOW_CM)    return "SLOW";
    return "CRITICAL/STOP";
}

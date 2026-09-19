/*
 * main.c
 *
 * Multi-threaded FDI architecture per project spec:
 *   Sensor Tasks + Fusion Task  -> Fault Detector (Highest prio)
 *                                       -> Recovery Task (High prio)
 *                                       -> Logger (Low prio)
 *
 * IPC: native QNX channels + MsgSend()/MsgReceive(), NOT POSIX
 * mqueue. POSIX mq_open/mqd_t were tried first and are not declared
 * by <mqueue.h> in this SDK image (build error: "unknown type name
 * 'mqd_t'"), so this uses the same channel-based IPC pattern already
 * proven to compile/link in this project (see ultrasonic_driver.c's
 * ChannelCreate/ConnectAttach/MsgReceivePulse for GPIO events). Each
 * "queue" below is a QNX channel owned by the receiving thread; a
 * sender does MsgSend() (blocking send/reply), the receiver loops on
 * MsgReceive() + MsgReply(). This is QNX's native message-passing
 * primitive and fulfills the spec's "Message Queues" requirement in
 * the idiomatic QNX sense.
 *
 * REUSES UNCHANGED: sensor_manager.[ch], fault_detector.[ch],
 * sensor_buffer.[ch], mq_names.h — no changes to any of those files.
 * sensor_name() is NOT redefined here; it comes from mq_names.h via
 * sensor_buffer.h -> sensor_manager.h.
 *
 * ADDS (this revision):
 *   - fusion.[ch]: graded degraded-operation distance estimate
 *     (FULL -> DEGRADED/SINGLE -> DEGRADED/HOLDOVER -> UNSAFE) instead
 *     of the old "no healthy ultrasonic sensor available" cliff edge.
 *   - Recovery Timeout: PROBING/VALIDATING is bounded by
 *     RECOVERY_TIMEOUT_US. On expiry the probe is abandoned
 *     (fdi_isolate() forces the sensor back to EXCLUDED), attempts
 *     are counted, and re-probing is gated by exponential backoff.
 *     After RECOVERY_MAX_ATTEMPTS the sensor is latched
 *     permanently-excluded so a persistently faulty sensor cannot
 *     livelock the recovery path. Every timeout / permanent-fail
 *     event is pushed into the Fault Timeline, not just console
 *     output, so it is visible as evidence.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>

#include <sys/neutrino.h>

#include "sensor_manager.h"
#include "fault_detector.h"
#include "fusion.h"

#define LOOP_PERIOD_US        100000   /* 100ms / 10Hz sensor sampling */

/* Thread priorities (QNX SCHED_FIFO, 1-255, higher = more urgent) */
#define PRIO_FAULT_DETECTOR    25   /* Highest, per spec */
#define PRIO_RECOVERY          20   /* High, per spec */
#define PRIO_SENSOR_FUSION     15   /* Medium */
#define PRIO_LOGGER              5   /* Low, best-effort */

#define LOG_FILE_PATH  "/tmp/fault_timeline.log"

/* ---- Recovery Timeout tuning ---- */
#define RECOVERY_TIMEOUT_US        2000000ULL   /* 2s max per probe attempt */
#define RECOVERY_MAX_ATTEMPTS      5            /* then latch permanent    */
#define RECOVERY_BACKOFF_BASE_US   1000000ULL   /* 1s, doubles per attempt */
#define RECOVERY_BACKOFF_MAX_US    30000000ULL  /* capped at 30s           */

/* ---- Message payloads ---- */

/* Fusion -> Fault Detector: "sensors were just polled, go evaluate" */
typedef struct {
    uint64_t now_us;
} fusion_msg_t;

/* Fault Detector -> Recovery / Logger: one sensor's post-evaluation state.
 *
 * synthetic_reason / probe_attempts are populated only by Recovery when
 * it forces a state change on timeout; Fault Detector always sends 0/0.
 * This struct is private to main.c (not shared via a header), so adding
 * fields here does not touch fault_detector.h. */
typedef struct {
    sensor_id_t     id;
    sensor_state_t  state;
    fault_type_t    fault;
    uint64_t        timestamp_us;
    int             synthetic_reason; /* 0=normal, 1=recovery timeout, 2=permanent fail */
    int             probe_attempts;
} fault_event_t;

/* Minimal reply payload — receivers just ack, no data needed back. */
typedef struct {
    int ok;
} ipc_ack_t;

/* One channel = one "queue": chid is created by the receiving thread,
 * coid is what senders connect to it with. */
typedef struct {
    int chid;
    int coid;
} qnx_queue_t;

/* Per-sensor Recovery Timeout bookkeeping. Indexed the same way as the
 * logger's last_state[] array (see sensor_idx() below). Touched only by
 * recovery_thread, so no lock needed. */
typedef struct {
    uint64_t probe_started_us;
    uint64_t next_probe_us;   /* backoff gate: don't re-probe before this */
    int      attempts;
    int      permanent_fail;
} recovery_sup_t;

/* ---- Shared context (read by Fusion/FDI, written only by FDI thread) ---- */

static sensor_manager_t g_mgr;

static fdi_context_t g_us1_ctx, g_us2_ctx, g_imu1_ctx, g_imu2_ctx;

static qnx_queue_t g_q_fusion_to_fdi;
static qnx_queue_t g_q_fdi_to_recovery;
static qnx_queue_t g_q_fdi_to_logger;

static fusion_state_t g_fusion;
static recovery_sup_t g_sup[4];

static volatile int g_running = 1;

/* ---- Helpers ---- */

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static const char *state_name(sensor_state_t s)
{
    switch (s) {
        case STATE_HEALTHY:    return "HEALTHY";
        case STATE_SUSPECT:    return "SUSPECT";
        case STATE_FAULTED:    return "FAULTED";
        case STATE_EXCLUDED:   return "EXCLUDED";
        case STATE_PROBING:    return "PROBING";
        case STATE_VALIDATING: return "VALIDATING";
        default:                return "UNKNOWN";
    }
}

/* sensor_name() comes from mq_names.h — not redefined here. */

static const char *fault_name(fault_type_t f)
{
    switch (f) {
        case FAULT_NONE:          return "NONE";
        case FAULT_DELAYED:       return "DELAYED";
        case FAULT_STUCK:         return "STUCK";
        case FAULT_NOISY:         return "NOISY";
        case FAULT_CONTRADICTORY: return "CONTRADICTORY";
        default:                   return "UNKNOWN";
    }
}

/* Shared index mapping used by both the Logger's last_state[] table and
 * Recovery's g_sup[] timeout table, so the two stay in sync by
 * construction instead of via two separately-maintained ternary chains. */
static int sensor_idx(sensor_id_t id)
{
    switch (id) {
        case SENSOR_US1:  return 0;
        case SENSOR_US2:  return 1;
        case SENSOR_IMU1: return 2;
        case SENSOR_IMU2: return 3;
        default:          return 0;
    }
}

/* Set SCHED_FIFO + priority on the calling thread. */
static void set_thread_priority(int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;

    /* pthread_setschedparam RETURNS the error code; it does not set
     * errno. Passing errno to strerror() here would print whatever
     * errno happened to hold from an unrelated earlier call. */
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
        fprintf(stderr, "[WARN] pthread_setschedparam(prio=%d) failed: %s\n",
                prio, strerror(rc));
        /* Non-fatal: continue at default priority. */
    }
}

/* Called by the thread that will OWN/receive on this queue. Creates a
 * private channel and immediately attaches a connection to it (so the
 * owning thread can also be used as its own sender reference if ever
 * needed; senders elsewhere get their own coid via queue_connect()). */
static int queue_create(qnx_queue_t *q)
{
    q->chid = ChannelCreate(_NTO_CHF_PRIVATE);
    if (q->chid == -1) {
        fprintf(stderr, "ChannelCreate failed: %s\n", strerror(errno));
        return -1;
    }

    q->coid = -1; /* senders attach their own connection via queue_connect() */
    return 0;
}

/* Called by a thread that wants to SEND to a queue owned elsewhere.
 * Returns a connection id to use with MsgSend(). */
static int queue_connect(const qnx_queue_t *q)
{
    int coid = ConnectAttach(0, 0, q->chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        fprintf(stderr, "ConnectAttach failed: %s\n", strerror(errno));
    }
    return coid;
}

/* ---- Fault Detector Thread (Highest priority) ---- */

static void evaluate_sensor(fdi_context_t *ctx, sensor_buffer_t *self_buf,
                             sensor_buffer_t *redundant_buf, uint64_t now)
{
    sensor_state_t before = ctx->state;
    fault_type_t detected = fdi_detect(ctx, self_buf, redundant_buf, now);

    switch (before) {
        case STATE_HEALTHY:
        case STATE_SUSPECT:
            fdi_update_state(ctx, detected);
            if (ctx->state == STATE_FAULTED) {
                fdi_isolate(ctx);
            }
            break;

        case STATE_FAULTED:
            fdi_isolate(ctx);
            break;

        case STATE_EXCLUDED:
        case STATE_PROBING:
        case STATE_VALIDATING:
            /* Recovery Task owns probe/validate transitions for excluded
             * sensors; Fault Detector leaves them alone here so the two
             * threads don't race on the same ctx->state path. */
            break;

        default:
            break;
    }

    ctx->last_fault = detected;
}

static void *fault_detector_thread(void *arg)
{
    (void)arg;
    set_thread_priority(PRIO_FAULT_DETECTOR);

    int coid_to_recovery = queue_connect(&g_q_fdi_to_recovery);
    int coid_to_logger   = queue_connect(&g_q_fdi_to_logger);

    fusion_msg_t msg;
    ipc_ack_t ack;

    while (g_running) {
        int rcvid = MsgReceive(g_q_fusion_to_fdi.chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[FDI] MsgReceive failed: %s\n", strerror(errno));
            continue;
        }

        ack.ok = 1;
        MsgReply(rcvid, EOK, &ack, sizeof(ack));

        sensor_buffer_t *us1  = sensor_manager_get_buffer(&g_mgr, SENSOR_US1);
        sensor_buffer_t *us2  = sensor_manager_get_buffer(&g_mgr, SENSOR_US2);
        sensor_buffer_t *imu1 = sensor_manager_get_buffer(&g_mgr, SENSOR_IMU1);
        sensor_buffer_t *imu2 = sensor_manager_get_buffer(&g_mgr, SENSOR_IMU2);

        struct { fdi_context_t *ctx; sensor_buffer_t *self; sensor_buffer_t *red; } pairs[4] = {
            { &g_us1_ctx,  us1,  us2  },
            { &g_us2_ctx,  us2,  us1  },
            { &g_imu1_ctx, imu1, imu2 },
            { &g_imu2_ctx, imu2, imu1 },
        };

        for (int i = 0; i < 4; i++) {
            evaluate_sensor(pairs[i].ctx, pairs[i].self, pairs[i].red, msg.now_us);

            fault_event_t ev;
            memset(&ev, 0, sizeof(ev));   /* zeroes synthetic_reason/probe_attempts */
            ev.id           = pairs[i].ctx->id;
            ev.state        = pairs[i].ctx->state;
            ev.fault        = pairs[i].ctx->last_fault;
            ev.timestamp_us = msg.now_us;

            /* Always forward to Logger (full timeline). */
            if (coid_to_logger != -1) {
                MsgSend(coid_to_logger, &ev, sizeof(ev), &ack, sizeof(ack));
            }

            /* Only wake Recovery when there's isolation work to drive. */
            if ((ev.state == STATE_EXCLUDED || ev.state == STATE_PROBING ||
                 ev.state == STATE_VALIDATING) && coid_to_recovery != -1) {
                MsgSend(coid_to_recovery, &ev, sizeof(ev), &ack, sizeof(ack));
            }
        }
    }

    return NULL;
}

/* ---- Recovery Thread (High priority) ---- */

static fdi_context_t *ctx_for_id(sensor_id_t id)
{
    switch (id) {
        case SENSOR_US1:  return &g_us1_ctx;
        case SENSOR_US2:  return &g_us2_ctx;
        case SENSOR_IMU1: return &g_imu1_ctx;
        case SENSOR_IMU2: return &g_imu2_ctx;
        default:          return NULL;
    }
}

static sensor_buffer_t *buf_for_id(sensor_id_t id)
{
    return sensor_manager_get_buffer(&g_mgr, id);
}

/* Sends a synthetic (Recovery-generated) event into the Fault Timeline.
 * reason: 1 = probe abandoned on timeout, 2 = latched permanently excluded. */
static void log_recovery_event(int coid_to_logger, const fdi_context_t *ctx,
                                uint64_t now, int reason, int attempts)
{
    if (coid_to_logger == -1) return;

    fault_event_t ev;
    ipc_ack_t ack;
    memset(&ev, 0, sizeof(ev));
    ev.id               = ctx->id;
    ev.state            = ctx->state;
    ev.fault            = ctx->last_fault;
    ev.timestamp_us     = now;
    ev.synthetic_reason = reason;
    ev.probe_attempts   = attempts;

    MsgSend(coid_to_logger, &ev, sizeof(ev), &ack, sizeof(ack));
}

static void *recovery_thread(void *arg)
{
    (void)arg;
    set_thread_priority(PRIO_RECOVERY);

    int coid_to_logger = queue_connect(&g_q_fdi_to_logger);

    fault_event_t ev;
    ipc_ack_t ack;

    while (g_running) {
        int rcvid = MsgReceive(g_q_fdi_to_recovery.chid, &ev, sizeof(ev), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[RECOVERY] MsgReceive failed: %s\n", strerror(errno));
            continue;
        }

        ack.ok = 1;
        MsgReply(rcvid, EOK, &ack, sizeof(ack));

        fdi_context_t *ctx = ctx_for_id(ev.id);
        if (ctx == NULL) continue;

        recovery_sup_t *sup = &g_sup[sensor_idx(ev.id)];

        switch (ctx->state) {
            case STATE_EXCLUDED:
                if (sup->permanent_fail) {
                    /* Latched: this sensor has exhausted its retries.
                     * Stop attempting probes so it cannot re-enter the
                     * PROBING/EXCLUDED oscillation seen when both members
                     * of a redundant pair kept failing validation against
                     * each other. Fusion (fusion.c) is what keeps the
                     * vehicle operating safely without it, not this loop. */
                    break;
                }
                if (ev.timestamp_us < sup->next_probe_us) {
                    /* Still inside the exponential-backoff window from a
                     * previous timeout; do not hammer the sensor. */
                    break;
                }
                fdi_start_probe(ctx);
                sup->probe_started_us = ev.timestamp_us;
                break;

            case STATE_PROBING:
            case STATE_VALIDATING: {
                uint64_t elapsed = ev.timestamp_us - sup->probe_started_us;

                if (elapsed > RECOVERY_TIMEOUT_US) {
                    /* Recovery Timeout: this probe attempt has run too
                     * long without confirming HEALTHY. Abandon it rather
                     * than let it run indefinitely — fdi_isolate() is the
                     * existing, unmodified fault_detector.c entry point
                     * for "force this sensor back to EXCLUDED". */
                    fdi_isolate(ctx);
                    sup->attempts++;

                    log_recovery_event(coid_to_logger, ctx, ev.timestamp_us,
                                       1 /* timeout */, sup->attempts);

                    if (sup->attempts >= RECOVERY_MAX_ATTEMPTS) {
                        sup->permanent_fail = 1;
                        log_recovery_event(coid_to_logger, ctx, ev.timestamp_us,
                                           2 /* permanent */, sup->attempts);
                    } else {
                        uint64_t backoff = RECOVERY_BACKOFF_BASE_US
                                         << (sup->attempts - 1);
                        if (backoff > RECOVERY_BACKOFF_MAX_US)
                            backoff = RECOVERY_BACKOFF_MAX_US;
                        sup->next_probe_us = ev.timestamp_us + backoff;
                    }
                    break; /* don't also run fdi_detect this cycle */
                }

                sensor_buffer_t *self = buf_for_id(ctx->id);
                sensor_buffer_t *red  = buf_for_id(ctx->redundant_id);
                fault_type_t detected = fdi_detect(ctx, self, red, ev.timestamp_us);
                fdi_probe_evaluate(ctx, detected);

                if (ctx->state == STATE_HEALTHY) {
                    /* Recovered: clear timeout bookkeeping so a future
                     * fault starts its own fresh attempt/backoff cycle. */
                    sup->attempts       = 0;
                    sup->next_probe_us  = 0;
                    sup->permanent_fail = 0;
                }
                break;
            }

            default:
                break;
        }
    }

    return NULL;
}

/* ---- Logger Thread (Low priority) ---- */

static void *logger_thread(void *arg)
{
    (void)arg;
    set_thread_priority(PRIO_LOGGER);

    FILE *fp = fopen(LOG_FILE_PATH, "a");
    if (fp == NULL) {
        fprintf(stderr, "[LOGGER] failed to open %s: %s\n", LOG_FILE_PATH, strerror(errno));
        return NULL;
    }
    setvbuf(fp, NULL, _IOLBF, 0);

    fault_event_t ev;
    ipc_ack_t ack;
    sensor_state_t last_state[4] = { STATE_HEALTHY, STATE_HEALTHY, STATE_HEALTHY, STATE_HEALTHY };

    while (g_running) {
        int rcvid = MsgReceive(g_q_fdi_to_logger.chid, &ev, sizeof(ev), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[LOGGER] MsgReceive failed: %s\n", strerror(errno));
            continue;
        }

        ack.ok = 1;
        MsgReply(rcvid, EOK, &ack, sizeof(ack));

        int idx = sensor_idx(ev.id);

        /* Normal transitions are logged edge-triggered, on state change
         * only. Synthetic Recovery events (timeout / permanent-fail) are
         * always logged even when the state happens to match the last
         * recorded state, since two consecutive timeouts both landing
         * back on EXCLUDED are two distinct, worth-recording events. */
        int should_log = (ev.state != last_state[idx]) || (ev.synthetic_reason != 0);

        if (should_log) {
            if (ev.synthetic_reason == 1) {
                fprintf(fp, "[%llu us] %s: RECOVERY TIMEOUT (attempt %d/%d) -> %s\n",
                        (unsigned long long)ev.timestamp_us,
                        sensor_name(ev.id), ev.probe_attempts, RECOVERY_MAX_ATTEMPTS,
                        state_name(ev.state));
            } else if (ev.synthetic_reason == 2) {
                fprintf(fp, "[%llu us] %s: RECOVERY ABANDONED PERMANENTLY after %d attempts -> %s\n",
                        (unsigned long long)ev.timestamp_us,
                        sensor_name(ev.id), ev.probe_attempts, state_name(ev.state));
            } else {
                fprintf(fp, "[%llu us] %s: %s -> %s (fault=%s)\n",
                        (unsigned long long)ev.timestamp_us,
                        sensor_name(ev.id),
                        state_name(last_state[idx]),
                        state_name(ev.state),
                        fault_name(ev.fault));
            }
            fflush(fp);
            last_state[idx] = ev.state;
        }
    }

    fclose(fp);
    return NULL;
}

/* ---- Sensor + Fusion Thread (Medium priority) ---- */

static void *sensor_fusion_thread(void *arg)
{
    (void)arg;
    set_thread_priority(PRIO_SENSOR_FUSION);

    int coid_to_fdi = queue_connect(&g_q_fusion_to_fdi);
    ipc_ack_t ack;

    while (g_running) {
        sensor_manager_poll(&g_mgr);

        fusion_msg_t msg;
        msg.now_us = now_us();

        if (coid_to_fdi != -1) {
            MsgSend(coid_to_fdi, &msg, sizeof(msg), &ack, sizeof(ack));
        }

        /* Sensor Health Dashboard (CLI, mandatory). Degraded-operation
         * fusion replaces the old "pick the first HEALTHY ultrasonic,
         * else print nothing" failover — see fusion.h for the ladder. */
        fusion_result_t fr;
        fusion_compute(&g_fusion, &g_mgr,
                       &g_us1_ctx, &g_us2_ctx, &g_imu1_ctx, &g_imu2_ctx,
                       msg.now_us, &fr);

        if (fr.valid) {
            printf("[PARKING] dist=%.1fcm (+/-%.1f) zone=%s mode=%s src=%d%s\n",
                   fr.distance_cm, fr.uncertainty_cm, fusion_zone_name(&fr),
                   op_mode_name(fr.mode), fr.contributors,
                   fr.disagreement ? " DISAGREE" : "");
        } else {
            printf("[PARKING] NO VALID RANGE zone=CRITICAL/STOP mode=%s held=%.1fs\n",
                   op_mode_name(fr.mode), (float)fr.age_us / 1000000.0f);
        }

        printf("[FDI] US1=%s US2=%s IMU1=%s IMU2=%s\n",
               state_name(g_us1_ctx.state), state_name(g_us2_ctx.state),
               state_name(g_imu1_ctx.state), state_name(g_imu2_ctx.state));
        fflush(stdout);

        usleep(LOOP_PERIOD_US);
    }

    return NULL;
}

/* ---- main: setup + thread spawn ---- */

int main(void)
{
    if (sensor_manager_init(&g_mgr) != 0) {
        fprintf(stderr, "sensor_manager_init failed\n");
        return 1;
    }

    fdi_config_t us_cfg = {
        .delay_timeout_us        = 500000,
        .stuck_threshold         = 3.0f,
        .noise_threshold         = 30.0f,
        .contradiction_threshold = 70.0f,
        .fault_confirm_limit     = 3,
        .recovery_confirm_limit  = 3,
    };

    fdi_config_t imu_cfg = {
        .delay_timeout_us        = 500000,
        .stuck_threshold         = 0.01f,
        .noise_threshold         = 5.0f,
        .contradiction_threshold = 5.0f,
        .fault_confirm_limit     = 3,
        .recovery_confirm_limit  = 3,
    };

    fdi_context_init(&g_us1_ctx,  SENSOR_US1,  &us_cfg);
    fdi_context_init(&g_us2_ctx,  SENSOR_US2,  &us_cfg);
    fdi_context_init(&g_imu1_ctx, SENSOR_IMU1, &imu_cfg);
    fdi_context_init(&g_imu2_ctx, SENSOR_IMU2, &imu_cfg);

    fusion_init(&g_fusion);
    memset(g_sup, 0, sizeof(g_sup));

    /* Create all three channels BEFORE spawning any thread, so every
     * sender can connect to a channel that already exists regardless
     * of thread start order. */
    if (queue_create(&g_q_fusion_to_fdi)   != 0 ||
        queue_create(&g_q_fdi_to_recovery) != 0 ||
        queue_create(&g_q_fdi_to_logger)   != 0) {
        fprintf(stderr, "channel setup failed, aborting\n");
        return 1;
    }

    printf("--- Parking Assist FDI (threaded) starting ---\n");
    printf("--- Fault timeline: %s ---\n", LOG_FILE_PATH);

    pthread_t th_fdi, th_recovery, th_logger, th_sensor;

    pthread_create(&th_logger,   NULL, logger_thread,         NULL);
    pthread_create(&th_recovery, NULL, recovery_thread,       NULL);
    pthread_create(&th_fdi,      NULL, fault_detector_thread, NULL);
    pthread_create(&th_sensor,   NULL, sensor_fusion_thread,  NULL);

    pthread_join(th_sensor,   NULL);
    pthread_join(th_fdi,      NULL);
    pthread_join(th_recovery, NULL);
    pthread_join(th_logger,   NULL);

    sensor_manager_shutdown(&g_mgr);
    return 0;
}

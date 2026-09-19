/*
 * sensor_manager.c
 */

#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stddef.h>

#include "sensor_manager.h"

/* GPIO pins per QNX_Parking_Assist_README.md section 2.
 * NOTE: verify against actual wiring — an earlier note had US2 as
 * TRIG25/ECHO26; this README states TRIG26/ECHO25. Using the README
 * values here since it's the architecture-of-record. */
#define US1_TRIG_PIN 23
#define US1_ECHO_PIN 24
#define US2_TRIG_PIN 26
#define US2_ECHO_PIN 25

#define I2C_BUS_PATH "/dev/i2c1"

#define SENSOR_PERIOD_US 100000  /* 100ms / 10Hz, per README section 29 */

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

int sensor_manager_init(sensor_manager_t *mgr)
{
    if (mgr == NULL) {
        return -1;
    }

    if (ultrasonic_init(&mgr->us1, US1_TRIG_PIN, US1_ECHO_PIN, SENSOR_US1) != 0) {
        return -1;
    }
    if (ultrasonic_init(&mgr->us2, US2_TRIG_PIN, US2_ECHO_PIN, SENSOR_US2) != 0) {
        return -1;
    }

    mgr->i2c_fd = open(I2C_BUS_PATH, O_RDWR);
    if (mgr->i2c_fd == -1) {
        return -1;
    }

    if (mpu6050_init(&mgr->imu1, mgr->i2c_fd, MPU6050_PRIMARY_ADDR, SENSOR_IMU1) != 0) {
        return -1;
    }
    if (mpu6050_init(&mgr->imu2, mgr->i2c_fd, MPU6050_BACKUP_ADDR, SENSOR_IMU2) != 0) {
        return -1;
    }

    /* Startup calibration — same as original mpu6050_thread() behavior */
    mpu6050_calibrate(&mgr->imu1);
    mpu6050_calibrate(&mgr->imu2);

    if (sensor_buffer_init(&mgr->us1_buf,  SENSOR_US1,  SENSOR_PERIOD_US) != 0) return -1;
    if (sensor_buffer_init(&mgr->us2_buf,  SENSOR_US2,  SENSOR_PERIOD_US) != 0) return -1;
    if (sensor_buffer_init(&mgr->imu1_buf, SENSOR_IMU1, SENSOR_PERIOD_US) != 0) return -1;
    if (sensor_buffer_init(&mgr->imu2_buf, SENSOR_IMU2, SENSOR_PERIOD_US) != 0) return -1;

    return 0;
}

void sensor_manager_shutdown(sensor_manager_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    ultrasonic_shutdown(&mgr->us1);
    ultrasonic_shutdown(&mgr->us2);

    if (mgr->i2c_fd != -1) {
        close(mgr->i2c_fd);
        mgr->i2c_fd = -1;
    }

    sensor_buffer_destroy(&mgr->us1_buf);
    sensor_buffer_destroy(&mgr->us2_buf);
    sensor_buffer_destroy(&mgr->imu1_buf);
    sensor_buffer_destroy(&mgr->imu2_buf);
}

void sensor_manager_poll(sensor_manager_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    /* --- Ultrasonic: existing driver + generic buffer, unmodified read path --- */
    float distance;

    if (ultrasonic_read(&mgr->us1, &distance) == 0) {
        sensor_buffer_write(&mgr->us1_buf, now_us(), &distance, 1);
    }

    if (ultrasonic_read(&mgr->us2, &distance) == 0) {
        sensor_buffer_write(&mgr->us2_buf, now_us(), &distance, 1);
    }

    /* --- IMU: existing driver + generic buffer, unmodified read path --- */
    imu_sample_t sample;

    if (mpu6050_read(&mgr->imu1, &sample) == 0) {
        float values[6] = { sample.ax, sample.ay, sample.az,
                             sample.gx, sample.gy, sample.gz };
        sensor_buffer_write(&mgr->imu1_buf, now_us(), values, 6);
    }

    if (mpu6050_read(&mgr->imu2, &sample) == 0) {
        float values[6] = { sample.ax, sample.ay, sample.az,
                             sample.gx, sample.gy, sample.gz };
        sensor_buffer_write(&mgr->imu2_buf, now_us(), values, 6);
    }
}

sensor_buffer_t *sensor_manager_get_buffer(sensor_manager_t *mgr, sensor_id_t id)
{
    if (mgr == NULL) {
        return NULL;
    }

    switch (id) {
        case SENSOR_US1:  return &mgr->us1_buf;
        case SENSOR_US2:  return &mgr->us2_buf;
        case SENSOR_IMU1: return &mgr->imu1_buf;
        case SENSOR_IMU2: return &mgr->imu2_buf;
        default:          return NULL;
    }
}


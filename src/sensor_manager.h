/*
 * sensor_manager.h
 *
 * Integration layer, he only place that
 * touches both "acquisition" and "buffer". Neither ultrasonic_driver
 * nor imu_driver know sensor_buffer_t exists — this file is what
 * connects them, so acquisition stays frozen and buffer stays generic.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensor_buffer.h"
#include "ultrasonic_driver.h"
#include "imu_driver.h"

typedef struct {
    Ultrasonic us1;
    Ultrasonic us2;
    MPU6050    imu1;
    MPU6050    imu2;
    int        i2c_fd;

    sensor_buffer_t us1_buf;
    sensor_buffer_t us2_buf;
    sensor_buffer_t imu1_buf;
    sensor_buffer_t imu2_buf;
} sensor_manager_t;

/* Initializes GPIO/I2C for all four sensors (via the existing driver
 * init calls, unmodified), runs IMU calibration, and sets up all four
 * sensor_buffer_t instances. Returns 0 on success. */
int sensor_manager_init(sensor_manager_t *mgr);

void sensor_manager_shutdown(sensor_manager_t *mgr);

/* One acquisition pass over all four sensors: calls the existing
 * ultrasonic_read()/mpu6050_read() functions unmodified, and writes
 * each successful reading into its matching sensor_buffer_t. A failed
 * read is skipped — no sample is published for that sensor this cycle.
 * Call this once per sensor period (currently 100ms per README section
 * 29) from whichever thread/task drives acquisition timing. */
void sensor_manager_poll(sensor_manager_t *mgr);

sensor_buffer_t *sensor_manager_get_buffer(sensor_manager_t *mgr, sensor_id_t id);

#endif /* SENSOR_MANAGER_H */

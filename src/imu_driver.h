
/*
 * imu_driver.h
 *
 * Hardware acquisition module for the MPU6050 6-DOF IMU over I2C.
 *
 * Operates purely as a sensor driver. Data acquisition is handled here,
 * while Fault Detection and Isolation (FDI) buffer handoff is managed
 * externally by the sensor manager.
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

/* I2C hardware addresses for dual-sensor redundancy */
#define MPU6050_PRIMARY_ADDR   0x68   /* AD0 pin pulled low */
#define MPU6050_BACKUP_ADDR    0x69   /* AD0 pin pulled high */

/* Zero-state offsets for gravity and gyroscope bias */
typedef struct {
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
    int   is_calibrated;
} imu_calib_t;

/* Processed sensor telemetry in physical units */
typedef struct {
    float ax, ay, az;   /* Acceleration (g) */
    float gx, gy, gz;   /* Angular velocity (deg/s) */
} imu_sample_t;

/* Device context for a single MPU6050 instance */
typedef struct {
    int         fd;         /* Shared I2C bus fd (managed externally) */
    uint8_t     addr;       /* MPU6050_PRIMARY_ADDR or MPU6050_BACKUP_ADDR */
    int         sensor_id;  /* 1 = primary, 2 = backup */
    imu_calib_t calib;      /* Device-specific calibration baseline */
} MPU6050;

/*
 * Wakes the device and verifies I2C communication via WHO_AM_I register.
 * Requires an already-open I2C file descriptor.
 * Returns 0 on success, -1 on failure.
 */
int mpu6050_init(MPU6050 *imu, int fd, uint8_t addr, int sensor_id);

/*
 * Stationary baseline calibration.
 * Averages 50 samples to establish zero-state offsets.
 * Returns 0 on success, -1 on failure.
 */
int mpu6050_calibrate(MPU6050 *imu);

/*
 * Fetches a 14-byte data burst, converts to physical units (g, deg/s),
 * applies calibration offsets, and populates the sample struct.
 * Returns 0 on success, -1 on I2C read failure.
 */
int mpu6050_read(MPU6050 *imu, imu_sample_t *out);

/*
 * Standalone execution thread for continuous sensor polling.
 * Opens the I2C bus, initializes/calibrates both sensors, and runs
 * a dual-sensor heartbeat loop at a fixed 100ms interval.
 */
void *mpu6050_thread(void *arg);

#endif /* IMU_DRIVER_H */


/*
 * imu_driver.c
 *
 * Hardware acquisition module for the MPU6050 6-DOF IMU over I2C.
 * Restructured from a monolith into modular init/calibrate/read functions.
 *
 * Operates purely as a sensor driver. Data acquisition is handled here,
 * while Fault Detection and Isolation (FDI) buffer handoff is managed externally.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <errno.h>

#include "imu_driver.h"

#define I2C_BUS_PATH        "/dev/i2c1"
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75  /* expect 0x68 */
#define REG_ACCEL_XOUT_H    0x3B  /* start of 14-byte burst (Accel + Temp + Gyro) */

struct i2c_recv_data_msg_t {
    i2c_sendrecv_t hdr;
    uint8_t        bytes[0];
};

/* --- I2C Hardware Abstraction Layer (QNX devctl) --- */

/* Write a single register address + value */
static int i2c_write_reg(int fd, uint8_t addr, uint8_t reg, uint8_t val)
{
    struct i2c_recv_data_msg_t *msg = malloc(sizeof(*msg) + 2);
    if (!msg) {
        perror("malloc failed");
        return -1;
    }

    msg->bytes[0] = reg;
    msg->bytes[1] = val;

    msg->hdr.slave.addr = addr;
    msg->hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg->hdr.send_len   = 2;
    msg->hdr.recv_len   = 0;
    msg->hdr.stop       = 1;

    int status;
    int err = devctl(fd, DCMD_I2C_SENDRECV, msg, sizeof(*msg) + 2, &status);

    free(msg);

    if (err != EOK) {
        fprintf(stderr, "i2c_write_reg(0x%02X) failed: %s\n", reg, strerror(err));
        return -1;
    }
    return 0;
}

/* Read len bytes starting at requested register */
static int i2c_read_bytes(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    struct i2c_recv_data_msg_t *msg = malloc(sizeof(*msg) + len);
    if (!msg) {
        perror("malloc failed");
        return -1;
    }

    msg->bytes[0] = reg;

    msg->hdr.slave.addr = addr;
    msg->hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg->hdr.send_len   = 1;      /* send register address */
    msg->hdr.recv_len   = len;    /* expect len bytes back */
    msg->hdr.stop       = 1;

    int status;
    int err = devctl(fd, DCMD_I2C_SENDRECV, msg, sizeof(*msg) + len, &status);

    if (err != EOK) {
        fprintf(stderr, "i2c_read_bytes(0x%02X, len=%zu) failed: %s\n",
                reg, len, strerror(err));
        free(msg);
        return -1;
    }

    memcpy(buf, msg->bytes, len);
    free(msg);
    return 0;
}

/* --- Sensor Driver API --- */

/* Wakes up the device and verifies communication via WHO_AM_I */
int mpu6050_init(MPU6050 *imu, int fd, uint8_t addr, int sensor_id)
{
    if (imu == NULL) {
        return -1;
    }

    memset(imu, 0, sizeof(*imu));
    imu->fd        = fd;
    imu->addr      = addr;
    imu->sensor_id = sensor_id;

    /* Wake up the device by clearing the sleep bit in power management */
    if (i2c_write_reg(fd, addr, REG_PWR_MGMT_1, 0x00) != 0) {
        fprintf(stderr, "MPU wake failed (addr 0x%02X)\n", addr);
        return -1;
    }

    usleep(10000); /* 10ms hardware settle time */

    uint8_t who = 0;
    i2c_read_bytes(fd, addr, REG_WHO_AM_I, &who, 1);
    printf("Sensor 0x%02X WHO_AM_I = 0x%02X\n", addr, who);

    return 0;
}

/*
 * Accumulates 50 samples to calculate zero-state offsets.
 * Note: Z-axis removes 1.0g to compensate for standard Earth gravity.
 * Assumes sensor is perfectly level and stationary during this process.
 */
int mpu6050_calibrate(MPU6050 *imu)
{
    if (imu == NULL) {
        return -1;
    }

    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    int samples = 50;

    printf("[CALIB] Keep vehicle still... Calibrating sensor at 0x%02X\n", imu->addr);
    fflush(stdout);

    for (int i = 0; i < samples; i++) {
        uint8_t raw[14];
        if (i2c_read_bytes(imu->fd, imu->addr, REG_ACCEL_XOUT_H, raw, 14) != 0) {
            fprintf(stderr, "[CALIB] Read failed at 0x%02X during sample %d\n", imu->addr, i);
            return -1;
        }

        /* Bitwise merge of high and low bytes */
        int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
        int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
        int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

        /* Convert raw LSB to physical units based on default sensitivity:
         * Accel: +/- 2g range -> 16384 LSB/g
         * Gyro: +/- 250 deg/s range -> 131 LSB/deg/s
         */
        ax_sum += (ax / 16384.0f);
        ay_sum += (ay / 16384.0f);
        az_sum += (az / 16384.0f);

        gx_sum += (gx / 131.0f);
        gy_sum += (gy / 131.0f);
        gz_sum += (gz / 131.0f);

        usleep(20000); /* 20ms between calibration samples */
    }

    imu->calib.ax_offset = ax_sum / samples;
    imu->calib.ay_offset = ay_sum / samples;
    imu->calib.az_offset = (az_sum / samples) - 1.0f;

    imu->calib.gx_offset = gx_sum / samples;
    imu->calib.gy_offset = gy_sum / samples;
    imu->calib.gz_offset = gz_sum / samples;
    imu->calib.is_calibrated = 1;

    printf("[CALIB] Done 0x%02X -> Accel Offsets (g) | X: %.3f | Y: %.3f | Z: %.3f\n",
           imu->addr, imu->calib.ax_offset, imu->calib.ay_offset, imu->calib.az_offset);
    printf("[CALIB] Done 0x%02X -> Gyro Offsets (deg/s)  | X: %.3f | Y: %.3f | Z: %.3f\n",
           imu->addr, imu->calib.gx_offset, imu->calib.gy_offset, imu->calib.gz_offset);
    fflush(stdout);

    return 0;
}

/*
 * Burst reads 14 consecutive bytes starting from ACCEL_XOUT_H:
 * [0-5: Accel], [6-7: Temp], [8-13: Gyro].
 * Converts to physical units (g and deg/s) and applies calibration offsets.
 */
int mpu6050_read(MPU6050 *imu, imu_sample_t *out)
{
    if (imu == NULL || out == NULL) {
        return -1;
    }

    uint8_t raw[14];
    if (i2c_read_bytes(imu->fd, imu->addr, REG_ACCEL_XOUT_H, raw, 14) != 0) {
        return -1;
    }

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    /* Apply sensitivity scale factor and calibration offsets */
    out->ax = (ax / 16384.0f) - imu->calib.ax_offset;
    out->ay = (ay / 16384.0f) - imu->calib.ay_offset;
    out->az = (az / 16384.0f) - imu->calib.az_offset;

    out->gx = (gx / 131.0f) - imu->calib.gx_offset;
    out->gy = (gy / 131.0f) - imu->calib.gy_offset;
    out->gz = (gz / 131.0f) - imu->calib.gz_offset;

    return 0;
}

/* --- Execution Thread --- */

/* Standalone heartbeat loop for continuous dual-sensor polling */
void *mpu6050_thread(void *arg)
{
    (void)arg;

    int fd = open(I2C_BUS_PATH, O_RDWR);
    if (fd == -1) {
        perror("open " I2C_BUS_PATH " failed in thread");
        return NULL;
    }

    MPU6050 primary;
    MPU6050 backup;

    if (mpu6050_init(&primary, fd, MPU6050_PRIMARY_ADDR, 1) != 0) {
        fprintf(stderr, "Primary MPU init failed\n");
    }
    if (mpu6050_init(&backup, fd, MPU6050_BACKUP_ADDR, 2) != 0) {
        fprintf(stderr, "Backup MPU init failed\n");
    }

    /* ---- Run Startup Calibration ---- */
    mpu6050_calibrate(&primary);
    mpu6050_calibrate(&backup);

    printf("\n--- Starting Live IMU Heartbeat Loop ---\n");
    fflush(stdout);

    /* ---- Dual Sensor Heartbeat Loop ---- */
    for (;;) {
        imu_sample_t sample;

        /* Fetch and print primary sensor telemetry */
        if (mpu6050_read(&primary, &sample) == 0) {
            printf("[PRIMARY 0x68] Accel (g): %+1.3f %+1.3f %+1.3f | Gyro (deg/s): %+6.1f %+6.1f %+6.1f\n",
                   sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz);
        }

        /* Fetch and print backup sensor telemetry */
        if (mpu6050_read(&backup, &sample) == 0) {
            printf("[BACKUP  0x69] Accel (g): %+1.3f %+1.3f %+1.3f | Gyro (deg/s): %+6.1f %+6.1f %+6.1f\n",
                   sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz);
        }

        fflush(stdout);
        usleep(100000); /* 100ms interval (~10Hz polling rate) */
    }

    close(fd);
    return NULL;
}

/*
 * ultrasonic_driver.c
 *
 *  Created on: 18-Sep-2026
 *      Author: Naseem
 */

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <sys/neutrino.h>
#include <unistd.h>

#include "ultrasonic_driver.h"

#include "rpi_gpio.h"

#define SPEED_OF_SOUND_CM_PER_US  0.0343f
#define EVENT_ECHO                1
#define ECHO_TIMEOUT_MS           30
#define MAX_DISTANCE_CM           500.0f


static int receive_echo_event(Ultrasonic *sensor,
                              struct _pulse *pulse)
{
    uint64_t timeout_ns;

    timeout_ns =
        (uint64_t)ECHO_TIMEOUT_MS * 1000000ULL;

    if (TimerTimeout(
            CLOCK_MONOTONIC,
            _NTO_TIMEOUT_RECEIVE,
            NULL,
            &timeout_ns,
            NULL) == -1)
    {
        return -1;
    }

    if (MsgReceivePulse(
            sensor->chid,
            pulse,
            sizeof(*pulse),
            NULL) == -1)
    {
        return -1;
    }

    if (pulse->code != _PULSE_CODE_MINAVAIL)
    {
        return -1;
    }

    if (pulse->value.sival_int != EVENT_ECHO)
    {
        return -1;
    }

    return 0;
}


static int64_t elapsed_time_ns(
    const struct timespec *start,
    const struct timespec *end)
{
    int64_t seconds;
    int64_t nanoseconds;

    seconds =
        (int64_t)end->tv_sec -
        (int64_t)start->tv_sec;

    nanoseconds =
        (int64_t)end->tv_nsec -
        (int64_t)start->tv_nsec;

    return (seconds * 1000000000LL) + nanoseconds;
}


static int send_trigger(Ultrasonic *sensor)
{
    struct timespec delay;

    if (rpi_gpio_output(
            sensor->trig_pin,
            GPIO_HIGH) != GPIO_SUCCESS)
    {
        return -1;
    }

    /*
     * HC-SR04 trigger pulse:
     * HIGH for 10 microseconds.
     */
    delay.tv_sec = 0;
    delay.tv_nsec = 10000;

    if (nanosleep(&delay, NULL) == -1)
    {
        rpi_gpio_output(
            sensor->trig_pin,
            GPIO_LOW);

        return -1;
    }

    if (rpi_gpio_output(
            sensor->trig_pin,
            GPIO_LOW) != GPIO_SUCCESS)
    {
        return -1;
    }

    return 0;
}


int ultrasonic_init(Ultrasonic *sensor,
                    int trig_pin,
                    int echo_pin,
                    int sensor_id)
{
    if (sensor == NULL)
    {
        return -1;
    }

    sensor->trig_pin = trig_pin;
    sensor->echo_pin = echo_pin;
    sensor->sensor_id = sensor_id;

    sensor->chid = -1;
    sensor->coid = -1;


    /*
     * Configure TRIG as output.
     */
    if (rpi_gpio_setup(
            sensor->trig_pin,
            GPIO_OUT) != GPIO_SUCCESS)
    {
        return -1;
    }


    /*
     * Configure ECHO as input.
     *
     * No internal pull resistor.
     */
    if (rpi_gpio_setup_pull(
            sensor->echo_pin,
            GPIO_IN,
            GPIO_PUD_OFF) != GPIO_SUCCESS)
    {
        return -1;
    }


    /*
     * Create private QNX channel.
     */
    sensor->chid =
        ChannelCreate(_NTO_CHF_PRIVATE);

    if (sensor->chid == -1)
    {
        return -1;
    }


    /*
     * Attach connection to our channel.
     */
    sensor->coid =
        ConnectAttach(
            0,
            0,
            sensor->chid,
            _NTO_SIDE_CHANNEL,
            0);

    if (sensor->coid == -1)
    {
        ChannelDestroy(sensor->chid);
        sensor->chid = -1;

        return -1;
    }


    /*
     * Receive both rising and falling ECHO edges.
     */
    if (rpi_gpio_add_event_detect(
            sensor->echo_pin,
            sensor->coid,
            GPIO_RISING | GPIO_FALLING,
            EVENT_ECHO) != GPIO_SUCCESS)
    {
        ConnectDetach(sensor->coid);
        ChannelDestroy(sensor->chid);

        sensor->coid = -1;
        sensor->chid = -1;

        return -1;
    }


    /*
     * Ensure TRIG starts LOW.
     */
    if (rpi_gpio_output(
            sensor->trig_pin,
            GPIO_LOW) != GPIO_SUCCESS)
    {
        ConnectDetach(sensor->coid);
        ChannelDestroy(sensor->chid);

        sensor->coid = -1;
        sensor->chid = -1;

        return -1;
    }

    return 0;
}


int ultrasonic_read(Ultrasonic *sensor,
                    float *distance)
{
    struct _pulse pulse;

    struct timespec start;
    struct timespec end;

    int64_t pulse_time_ns;


    if (sensor == NULL || distance == NULL)
    {
        return -1;
    }

    *distance = 0.0f;


    /*
     * Remove stale events from the channel.
     */
    while (1)
    {
        uint64_t zero_timeout = 0;

        if (TimerTimeout(
                CLOCK_MONOTONIC,
                _NTO_TIMEOUT_RECEIVE,
                NULL,
                &zero_timeout,
                NULL) == -1)
        {
            break;
        }

        if (MsgReceivePulse(
                sensor->chid,
                &pulse,
                sizeof(pulse),
                NULL) == -1)
        {
            break;
        }
    }


    /*
     * Generate the trigger pulse.
     */
    if (send_trigger(sensor) != 0)
    {
        return -1;
    }


    /*
     * Wait for ECHO rising edge.
     */
    if (receive_echo_event(
            sensor,
            &pulse) != 0)
    {
        return -1;
    }


    /*
     * Timestamp rising edge.
     */
    if (clock_gettime(
            CLOCK_MONOTONIC,
            &start) == -1)
    {
        return -1;
    }


    /*
     * Wait for ECHO falling edge.
     */
    if (receive_echo_event(
            sensor,
            &pulse) != 0)
    {
        return -1;
    }


    /*
     * Timestamp falling edge.
     */
    if (clock_gettime(
            CLOCK_MONOTONIC,
            &end) == -1)
    {
        return -1;
    }


    /*
     * Calculate ECHO HIGH duration.
     */
    pulse_time_ns =
        elapsed_time_ns(&start, &end);

    if (pulse_time_ns <= 0)
    {
        return -1;
    }


    /*
     * Convert pulse width to distance.
     *
     * Distance:
     *
     *     time(us) * 0.0343
     *     ----------------
     *            2
     */
    *distance =
        ((float)pulse_time_ns / 1000.0f)
        * SPEED_OF_SOUND_CM_PER_US
        / 2.0f;


    /*
     * Basic validity check.
     */
    if (*distance <= 0.0f ||
        *distance > MAX_DISTANCE_CM)
    {
        *distance = 0.0f;
        return -1;
    }


    return 0;
}


void ultrasonic_shutdown(Ultrasonic *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    if (sensor->coid != -1)
    {
        ConnectDetach(sensor->coid);
        sensor->coid = -1;
    }

    if (sensor->chid != -1)
    {
        ChannelDestroy(sensor->chid);
        sensor->chid = -1;
    }

    rpi_gpio_cleanup();
}

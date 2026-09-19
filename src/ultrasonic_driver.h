#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#include <stdint.h>

typedef struct
{
    int trig_pin;
    int echo_pin;
    int sensor_id;

    int chid;
    int coid;

} Ultrasonic;


/*
  trig_pin : BCM GPIO used for TRIG
  echo_pin : BCM GPIO used for ECHO
  sensor_id: application-defined sensor identifier

  Returns:
    0  -> success
   -1  -> failure
 */
int ultrasonic_init(Ultrasonic *sensor,
                    int trig_pin,
                    int echo_pin,
                    int sensor_id);


/*
 * Acquire one distance measurement.
 *
 * distance : returned distance in centimeters
 *
 * Returns:
 *   0  -> valid measurement
 *  -1  -> acquisition/measurement failure
 */
int ultrasonic_read(Ultrasonic *sensor,
                    float *distance);

/*
 * Release resources associated with the sensor.
 */
void ultrasonic_shutdown(Ultrasonic *sensor);

#endif

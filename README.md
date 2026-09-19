# Automotive Sensor Fault Detection & Isolation (FDI)

### QNX 8.0 • Raspberry Pi 5 • Real-Time Automotive Safety Prototype

> **A real-time automotive sensor monitoring and fault-tolerant
> parking-assist prototype that detects abnormal sensor behaviour,
> isolates unreliable sensors, attempts recovery, and maintains degraded
> operation using redundant sensing.**

------------------------------------------------------------------------

## 1. Project Overview

Modern automotive systems depend on sensors whose measurements directly
influence safety-critical decisions. A sensor may fail without
completely stopping: it can become **stuck, stale, noisy, delayed, or
contradictory** while continuing to produce apparently valid data.

This project implements a **Sensor Fault Detection & Isolation (FDI)
system** on **QNX 8.0 running on a Raspberry Pi 5**.

The prototype uses:

-   **2 × Ultrasonic sensors** for redundant distance measurement
-   **2 × IMUs** for redundant motion sensing
-   QNX real-time threads and priority scheduling
-   Message-queue based communication
-   A small sensor history buffer
-   Fault detection and classification
-   Sensor health state management
-   Isolation and recovery/probing
-   Degraded and holdover operation
-   Fault timeline logging
-   QNX System Profiler for runtime analysis

The system was developed by starting with a small two-sensor prototype
and progressively building the complete FDI architecture around it.

------------------------------------------------------------------------

# 2. Problem Statement

### Automotive Sensor Fault Detection & Isolation

**Detect failed, stale, or inconsistent sensors and maintain safe
operation.**

The system must:

1.  Monitor redundant vehicle sensors.
2.  Identify abnormal sensor behaviour.
3.  Classify faults.
4.  Isolate unreliable sensors.
5.  Attempt bounded recovery.
6.  Continue operation using available healthy information.
7.  Maintain a record of the fault timeline.
8.  Demonstrate deterministic, real-time task execution on QNX.

------------------------------------------------------------------------

# 3. Objectives

The prototype is designed to demonstrate:

-   Real-time sensor acquisition
-   Redundant sensor monitoring
-   Sensor calibration
-   Timestamped sensor data
-   Sensor health tracking
-   Fault classification
-   Fault isolation
-   Recovery and probing
-   Sensor reintegration
-   Fault-tolerant sensor fusion
-   Degraded operation
-   QNX IPC using message queues
-   Priority-based real-time scheduling
-   Runtime fault logging
-   System profiling

------------------------------------------------------------------------

# 4. System Architecture

``` text
                         QNX 8.0
                      Raspberry Pi 5
                            │
             ┌──────────────┴──────────────┐
             │                             │
       SENSOR ACQUISITION             APPLICATION
             │                             │
      ┌──────┼────────┐                    │
      │      │        │                    │
     US1    US2     IMU1 ───── IMU2        │
      │      │        │          │         │
      └──────┴────────┴──────────┘         │
                    │                      │
                    ▼                      │
             Sensor Data Layer             │
                    │                      │
                    ▼                      │
             Small Sensor Buffer            │
                    │                      │
                    ▼                      │
             Sensor / Fusion                │
                    │                      │
                    ▼                      │
              Fault Detector                │
                    │                      │
          ┌─────────┼──────────┐           │
          │         │          │           │
       Detection Classification Health     │
          │                      │           │
          ▼                      │           │
       SUSPECT                   │           │
          │                      │           │
          ▼                      │           │
       PROBING                   │           │
          │                      │           │
       ┌──┴──────┐               │           │
       │         │               │           │
   Recovered   Failed            │           │
       │         │               │           │
       ▼         ▼               │           │
    HEALTHY   EXCLUDED           │           │
                   │             │           │
                   └──────► DEGRADED ◄─────┘
                              │
                              ▼
                           HOLDOVER

          Fault / State Events
                   │
                   ▼
                Logger
                   │
                   ▼
             Fault Timeline
```

------------------------------------------------------------------------

# 5. Sensor Redundancy

The current prototype uses **identical redundancy**.

### Ultrasonic redundancy

``` text
ULTRASONIC 1 ─────┐
                  ├──► Redundant distance information
ULTRASONIC 2 ─────┘
```

### IMU redundancy

``` text
IMU 1 ────────────┐
                  ├──► Redundant motion information
IMU 2 ────────────┘
```

The architecture was designed with the broader concept of redundant
sensing in mind, including functional redundancy. The demonstrated
prototype focuses on **identical redundancy**, where two sensors of the
same type provide comparable information.

------------------------------------------------------------------------

# 6. Fault Detection

The FDI engine monitors sensor behaviour and maintains an individual
health state for each sensor.

The target fault classes are:

  -----------------------------------------------------------------------
  Fault                               Description
  ----------------------------------- -----------------------------------
  **STUCK-AT**                        Sensor output remains effectively
                                      unchanged for consecutive samples

  **STALE / DELAYED**                 Expected sensor updates stop or
                                      become too old

  **NOISY**                           Sensor variation exceeds the
                                      configured noise threshold

  **CONTRADICTORY**                   Redundant sensors provide
                                      inconsistent measurements
  -----------------------------------------------------------------------

The detector does not treat every abnormal reading as an immediate
permanent failure.

Instead, the system uses sensor states.

------------------------------------------------------------------------

# 7. Sensor Health State Machine

``` text
                 ┌───────────┐
                 │  HEALTHY  │
                 └─────┬─────┘
                       │
                 abnormal evidence
                       │
                       ▼
                 ┌───────────┐
                 │  SUSPECT  │
                 └─────┬─────┘
                       │
                       ▼
                 ┌───────────┐
                 │  PROBING  │
                 └─────┬─────┘
                       │
                ┌──────┴──────┐
                │             │
            Recovered       Failed
                │             │
                ▼             ▼
           ┌─────────┐   ┌──────────┐
           │ HEALTHY │   │ EXCLUDED │
           └─────────┘   └──────────┘
```

This allows the system to distinguish between:

-   a temporary abnormal condition,
-   an active recovery attempt,
-   and a sensor that should no longer participate in normal operation.

------------------------------------------------------------------------

# 8. Fault Isolation & Recovery

When a sensor becomes unreliable, the Fault Detector sends a recovery
command.

``` text
Fault Detected
      │
      ▼
   SUSPECT
      │
      ▼
   PROBING
      │
      ▼
 Check sensor health
      │
 ┌────┴─────┐
 │          │
Healthy    Failed
 │          │
 ▼          ▼
Reintegrate EXCLUDE
            │
            ▼
       Degraded Mode
```

Recovery is bounded by a finite number of attempts and a finite recovery
window.

The recovery mechanism checks whether the sensor resumes publishing
valid measurements instead of inventing a hardware reset that may not
exist for the sensor.

------------------------------------------------------------------------

# 9. Fault-Tolerant Parking Operation

The FDI system is integrated with the parking-assist application.

During a sensor fault, the application can continue using available
sensor information.

Example runtime output:

``` text
[PARKING] dist=40.3cm (+/-5.0)
zone=CAUTION
mode=DEGRADED/SINGLE
src=1

[FDI] US1=HEALTHY US2=SUSPECT
      IMU1=HEALTHY IMU2=HEALTHY
```

The fault then progresses:

``` text
US2 = SUSPECT
        ↓
US2 = PROBING
        ↓
US2 = EXCLUDED
```

The parking application continues operating in degraded mode.

When both ultrasonic sensors become unavailable, the system demonstrates
a holdover state:

``` text
[FDI] US1=EXCLUDED US2=EXCLUDED
      IMU1=HEALTHY IMU2=HEALTHY

[PARKING] mode=DEGRADED/HOLDOVER
```

This demonstrates that fault handling is connected to application
behaviour rather than being only a diagnostic printout.

------------------------------------------------------------------------

# 10. IMU Identification & Calibration

The two IMUs are independently identified and calibrated.

Example:

``` text
Sensor 0x68 WHO_AM_I = 0x70
Sensor 0x69 WHO_AM_I = 0x68
```

Calibration is performed while the vehicle is stationary.

Example calibration output:

``` text
[CALIB] Done 0x68 -> Accel Offsets (g)
X: 0.009 | Y: -0.014 | Z: 0.006

[CALIB] Done 0x68 -> Gyro Offsets (deg/s)
X: -0.541 | Y: 4.222 | Z: -1.595
```

The second IMU is calibrated independently.

Calibration was an important early stage of the project because
redundant sensors cannot simply be assumed to produce identical
numerical measurements.

------------------------------------------------------------------------

# 11. Data Architecture

One of the major design decisions was simplifying the sensor data path.

An early approach considered using shared memory and mutex-protected
access.

``` text
Sensor
   ↓
Shared Memory
   ↓
Mutex
   ↓
Consumer
```

This introduced unnecessary synchronization complexity and became a
bottleneck during prototyping.

The architecture was therefore simplified to:

``` text
Sensor
   ↓
Acquire
   ↓
Normalize
   ↓
Timestamp
   ↓
Sensor Buffer
   ↓
Fault/Fusion Processing
```

The sensor buffer maintains a small history of recent readings.

This was sufficient for temporal fault checks while keeping the data
path bounded and easier to reason about.

------------------------------------------------------------------------

# 12. QNX Real-Time Architecture

The project uses QNX mechanisms directly for the real-time system
structure.

### Main mechanisms

-   POSIX threads
-   `SCHED_FIFO`
-   Thread priorities
-   POSIX message queues
-   Timed message operations
-   QNX device interfaces
-   Real-time clock/timestamping
-   System Profiler tracing

### Logical task structure

``` text
Sensor Tasks
     │
     ▼
Sensor / Fusion
     │
     ▼
Fault Detector
     │
     ├────────► Recovery
     │
     └────────► Logger
```

The Fault Detector is treated as the highest-priority application task,
with Recovery below it and Logger at a lower priority.

This allows fault handling to receive scheduling priority over less
critical logging work.

------------------------------------------------------------------------

# 13. Inter-Task Communication

The architecture uses message queues for task communication.

Conceptually:

``` text
Sensor Tasks
     │
     │ sensor readings
     ▼
/mq_sensor_data
     │
     ▼
Fault Detector
     │
     │ recovery command
     ▼
/mq_recovery_cmd
     │
     ▼
Recovery Task
     │
     │ recovery result
     ▼
/mq_recovery_result
     │
     ▼
Fault Detector

Fault Detector / Recovery
     │
     │ log event
     ▼
/mq_log_events
     │
     ▼
Logger
```

The queues provide clear communication boundaries between the major
components.

------------------------------------------------------------------------

# 14. Fault Timeline

The Logger records sensor state transitions and fault events.

Example:

``` text
US2 → SUSPECT
US2 → PROBING
US2 → EXCLUDED

US1 → SUSPECT
US1 → PROBING

US1 → EXCLUDED
US2 → EXCLUDED
```

The timeline provides an auditable sequence of what happened during a
fault event.

------------------------------------------------------------------------

# 15. QNX System Profiler

The completed system was also analysed using the QNX System Profiler.

A captured `.kev` trace provided:

  Metric              Observed value
  ----------------- ----------------
  Trace duration             3.000 s
  CPUs                             4
  Total events               154,872
  Dropped buffers                  0
  Idle activity                98.9%
  User activity                 0.3%
  Kernel activity               0.8%

The captured trace therefore showed approximately **98.9% system idle
time** during that measurement window.

The profiler was also used to inspect:

-   CPU activity
-   Per-CPU execution
-   Thread activity
-   Inter-CPU communication
-   System timeline behaviour

> **Note:** The values above describe the captured system trace and
> should not be interpreted as the isolated CPU consumption of the
> `assist_parking` application.

------------------------------------------------------------------------

# 16. Hardware & Software

## Hardware

-   Raspberry Pi 5
-   2 × Ultrasonic sensors
-   2 × IMU sensors
-   Automotive/parking-assist prototype setup

## Software

-   QNX 8.0
-   QNX Momentics IDE
-   QNX System Profiler
-   C
-   QNX/POSIX APIs
-   Message Queues
-   Real-time threads

------------------------------------------------------------------------

# 17. Project Structure

A simplified logical structure is:

``` text
assist_parking/
│
├── src/
│   ├── main.c
│   ├── sensor_manager.c
│   ├── sensor_buffer.c
│   ├── ultrasonic_driver.c
│   ├── imu_driver.c
│   ├── fusion.c
│   ├── fault_detector.c
│   ├── recovery_task.c
│   └── logger_task.c
│
├── include/
│   ├── sensor_common.h
│   ├── fault_common.h
│   ├── sensor_buffer.h
│   ├── sensor_manager.h
│   ├── ultrasonic_driver.h
│   ├── imu_driver.h
│   └── fusion.h
│
└── README.md
```

> The exact source layout may differ from this logical representation
> depending on the Momentics project configuration.

------------------------------------------------------------------------

# 18. Runtime Flow

A normal system cycle looks like:

``` text
1. Sensor acquisition
        ↓
2. Timestamp and normalize
        ↓
3. Store recent samples
        ↓
4. Sensor/fusion processing
        ↓
5. Fault evaluation
        ↓
6. Update sensor health
        ↓
7. If fault:
        SUSPECT
          ↓
        PROBING
          ↓
     RECOVER / EXCLUDE
        ↓
8. Continue in normal or degraded mode
        ↓
9. Log the event
```

------------------------------------------------------------------------

# 19. Demonstrated Scenario

A representative fault scenario is:

### Step 1 --- Normal operation

``` text
US1 = HEALTHY
US2 = HEALTHY
IMU1 = HEALTHY
IMU2 = HEALTHY
```

### Step 2 --- US2 becomes suspicious

``` text
US1 = HEALTHY
US2 = SUSPECT
```

### Step 3 --- Recovery begins

``` text
US2 = PROBING
```

### Step 4 --- Recovery fails

``` text
US2 = EXCLUDED
```

### Step 5 --- System continues

``` text
mode = DEGRADED/SINGLE
```

### Step 6 --- Additional degradation

If both ultrasonic sensors become unavailable:

``` text
US1 = EXCLUDED
US2 = EXCLUDED
```

the system enters:

``` text
mode = DEGRADED/HOLDOVER
```

while the available healthy sensing remains visible to the system.

------------------------------------------------------------------------

# 20. Engineering Journey

This project was deliberately developed incrementally.

We did not begin with the final architecture.

We began with **two sensors** and tried to make the smallest useful
system work.

The first challenges were not advanced algorithms. They were practical
engineering problems:

-   sensor calibration,
-   synchronization,
-   data abstraction,
-   communication between stages,
-   synchronization overhead,
-   process interference,
-   debugging real hardware,
-   and understanding what the RTOS was actually doing.

An early shared-memory/mutex approach introduced more complexity than
necessary. Replacing it with a small bounded sensor buffer made the data
path considerably simpler.

That simplification became a central architectural principle:

> **Keep the sensor interface simple, keep the data path bounded, and
> let each subsystem have one clear responsibility.**

Once that foundation worked, the rest of the system could be built
around it:

``` text
Acquisition
    ↓
Buffer
    ↓
Fusion
    ↓
FDI
    ↓
Recovery
    ↓
Isolation
    ↓
Degraded Operation
    ↓
Logging
```

The most satisfying stage was when every part of that architecture
became observable on the actual Raspberry Pi:

``` text
sensor fault
    ↓
SUSPECT
    ↓
PROBING
    ↓
EXCLUDED
    ↓
DEGRADED OPERATION
```

At that point, the architecture was no longer just a diagram.

It was a running system.

------------------------------------------------------------------------

# 21. Key Technical Takeaways

### 1. Redundancy needs comparable data

Two physical sensors do not automatically produce identical numerical
values. Calibration, timing, and sensor characteristics matter.

### 2. Abstraction should reduce complexity

The final sensor interface intentionally hides sensor-specific
acquisition details from downstream processing.

### 3. Bounded data structures matter in real-time systems

A small sensor history buffer provides the temporal context needed for
fault detection without introducing an unnecessarily complicated
shared-memory architecture.

### 4. Detection and recovery are different responsibilities

The Fault Detector identifies abnormal behaviour. The Recovery Task
performs bounded probing and reports the result.

### 5. Fault handling must affect the application

A fault is meaningful only when the system responds to it. The
parking-assist application therefore changes operating mode as sensor
availability changes.

### 6. Profiling validates assumptions

The QNX System Profiler provides visibility into CPU activity, thread
execution, and system behaviour instead of relying only on theoretical
estimates.

------------------------------------------------------------------------

# 22. Current Prototype Status

  Capability                   Status
  --------------------------- --------
  QNX 8.0 on Raspberry Pi 5      ✅
  Ultrasonic sensor 1            ✅
  Ultrasonic sensor 2            ✅
  IMU 1                          ✅
  IMU 2                          ✅
  IMU identification             ✅
  IMU calibration                ✅
  Sensor buffering               ✅
  Sensor/fusion pipeline         ✅
  Fault detection                ✅
  Stuck-at detection             ✅
  Stale/delayed detection        ✅
  Noisy detection                ✅
  Contradictory detection        ✅
  Sensor health states           ✅
  Fault isolation                ✅
  Recovery/probing               ✅
  Sensor exclusion               ✅
  Reintegration path             ✅
  Degraded operation             ✅
  Holdover operation             ✅
  Fault timeline                 ✅
  QNX message queues             ✅
  Real-time scheduling           ✅
  QNX System Profiler            ✅

------------------------------------------------------------------------

# 23. Future Extensions

The prototype can be extended with:

-   Functional redundancy across different sensor modalities
-   Additional automotive sensors
-   Hardware-level sensor reset/reinitialization
-   More sophisticated statistical fault models
-   Persistent fault storage
-   Sensor-health dashboard
-   Python/Qt/Web visualization
-   More extensive stress and fault-injection testing
-   Longer-duration CPU and scheduling measurements
-   Additional QNX trace analysis

------------------------------------------------------------------------

# 24. Conclusion

This project demonstrates a complete prototype of an automotive **Sensor
Fault Detection & Isolation** architecture on QNX.

The central idea is simple:

``` text
Don't just detect the failure.

Detect it.
Understand it.
Isolate it.
Try to recover it.
And keep the vehicle function operating with what remains healthy.
```

The project began as a small two-sensor experiment and evolved into a
real-time system containing sensor acquisition, buffering, redundancy,
fault detection, recovery, isolation, degraded operation, logging, and
system profiling.

The final result is not intended to represent a production automotive
safety system. It is a **functional real-time prototype demonstrating
the architecture and engineering principles required for fault-tolerant
automotive sensing**.

------------------------------------------------------------------------

## Platform

**Target:** Raspberry Pi 5\
**RTOS:** QNX 8.0\
**Development Environment:** QNX Momentics IDE\
**Language:** C\
**Application:** Automotive Parking Assist with Sensor FDI\
**Redundancy Demonstrated:** Identical sensor redundancy\
**Sensors:** 2 × Ultrasonic + 2 × IMU\
**IPC:** POSIX/QNX Message Queues\
**Profiling:** QNX System Profiler

------------------------------------------------------------------------

## License

Add the project's applicable license here.

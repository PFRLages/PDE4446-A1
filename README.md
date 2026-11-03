# Autonomous Robot Navigation System

## Project Overview

This project was developed for the PDE4446 Sensing and Motion Control assessment.

The aim was to design and implement an autonomous robot capable of:

- Driving in a straight line
- Stopping at five evenly spaced checkpoints
- Continuing automatically after each stop
- Reducing speed while travelling underneath a bridge
- Returning to normal speed after leaving the bridge
- Displaying its operating state using LEDs

The system uses an ESP32 and a Mirto robot controller. The ESP32 performs distance measurement and bridge detection, while the Mirto controls movement, steering correction and checkpoint stopping.

---

## Hardware Components

- Mirto Robot Platform
- ESP32 Development Board
- VL53L4CX Time-of-Flight Sensor
- HC-SR04 Ultrasonic Sensor
- Two Wheel Encoders
- DC Motors
- Red LED
- Yellow LED
- Green LED
- 220Ω Resistors for LEDs
- 1kΩ and 2kΩ Resistors for HC-SR04 Voltage Divider

---

## System Design

### ESP32 Responsibilities

- Initialise the VL53L4CX sensor
- Collect distance measurements
- Apply median filtering
- Send the measured distance to the Mirto controller
- Detect bridge sections using the HC-SR04
- Signal bridge detection to the Mirto

### Mirto Responsibilities

- Receive the measured distance
- Divide the route into six equal sections
- Control the drive motors
- Maintain straight-line travel using wheel encoders
- Stop at each checkpoint
- Adjust speed when a bridge is detected
- Control status LEDs

---

## Wiring Connections

### VL53L4CX to ESP32

```text
VCC    -> 3.3V
GND    -> GND
SDA    -> GPIO 8
SCL    -> GPIO 9
XSHUT  -> GPIO 18
```

### HC-SR04 to ESP32

```text
VCC    -> 5V
GND    -> GND
TRIG   -> GPIO 4

ECHO -> 1kΩ resistor -> GPIO 5
                      |
                    2kΩ
                      |
                     GND
```

A voltage divider is used because the HC-SR04 ECHO pin outputs 5V while the ESP32 GPIO pins are 3.3V tolerant.

### ESP32 to Mirto

```text
GPIO 17 (TX) -> Pin 11
GPIO 16      -> Pin 12
GND          -> GND
```

### Encoders to Mirto

```text
Left Encoder  -> Pin 3
Right Encoder -> Pin 2
```

### Motors to Mirto

```text
Left Motor  -> Pin 5
Right Motor -> Pin 6
```

### LEDs

```text
A1 -> 220Ω resistor -> Red LED -> GND

A2 -> 220Ω resistor -> Yellow LED -> GND

A3 -> 220Ω resistor -> Green LED -> GND
```

---

## System Overview

```text
VL53L4CX
   |
   +--> ESP32 GPIO 8 (SDA)
   +--> ESP32 GPIO 9 (SCL)
   +--> ESP32 GPIO 18 (XSHUT)

HC-SR04
   |
   +--> ESP32 GPIO 4 (TRIG)
   +--> ESP32 GPIO 5 (ECHO via voltage divider)

ESP32
   |
   +--> GPIO 17 -----> Mirto Pin 11
   +--> GPIO 16 -----> Mirto Pin 12

Mirto
   |
   +--> Pin 3  (Left Encoder)
   +--> Pin 2  (Right Encoder)
   +--> Pin 5  (Left Motor)
   +--> Pin 6  (Right Motor)
   +--> A1     (Red LED)
   +--> A2     (Yellow LED)
   +--> A3     (Green LED)
```

---

## Operation

1. The ESP32 starts and initialises the VL53L4CX sensor.
2. Twenty valid distance measurements are collected.
3. A median value is calculated.
4. The measured distance is sent to the Mirto controller.
5. The Mirto divides the route into six equal sections.
6. The robot starts moving.
7. Wheel encoders are used to maintain straight-line travel.
8. The robot stops at each checkpoint for five seconds.
9. The HC-SR04 continuously checks the height beneath the robot.
10. When a bridge is detected, the robot slows down.
11. When the bridge ends, normal speed is restored.
12. The robot stops after reaching the final checkpoint.

---

## LED Operating Modes

| LED | Meaning |
|------|---------|
| Green | Normal driving |
| Yellow | Bridge detected, reduced speed |
| Red | Robot stopped |

---

## Important Design Decisions

### Median Distance Filtering

Twenty valid VL53L4CX measurements are collected and sorted.

The median value is used instead of the average because it removes the influence of occasional incorrect readings and produces more reliable measurements.

### Encoder-Based Steering Correction

Wheel encoder counts are continuously compared.

A proportional controller adjusts motor speed based on encoder error to keep the robot travelling in a straight line.

### Bridge Detection

The HC-SR04 sensor continuously measures the distance below the robot.

Distances between 25 cm and 80 cm are classified as a bridge section.

### Smooth Speed Changes

Motor speed changes are applied gradually rather than instantly.

This reduces wheel slip and improves stability when changing between operating modes.

---

## Testing and Evaluation

### VL53L4CX Distance Measurement Test

The VL53L4CX sensor was tested at known distances.

Results:

- Stable measurements obtained after median filtering
- Outliers successfully rejected
- Consistent distance values transmitted to the Mirto controller

### Straight-Line Motion Test

The robot was driven repeatedly along the test route.

Results:

- Encoder feedback improved straight-line accuracy
- Minor drift remained due to wheel slip and motor differences

### Checkpoint Stopping Test

Multiple runs were performed to verify stopping locations.

Results:

- Robot consistently stopped at six evenly distributed positions
- Small cumulative errors occurred over longer distances

### Bridge Detection Test

The robot was driven beneath the bridge section multiple times.

Results:

- Bridge detection operated reliably
- Speed reduction activated correctly
- Normal speed resumed after leaving the bridge

---

## Results

The completed system successfully achieved all required objectives:

- Straight-line navigation
- Distance-based checkpoint stopping
- Automatic restart after each stop
- Bridge detection
- Automatic bridge speed reduction
- LED status indication

The system operated reliably across multiple test runs.

---

## Limitations

- Wheel slip affects stopping accuracy.
- Small encoder errors accumulate over long distances.
- HC-SR04 readings may become inaccurate when sound waves reflect away from the sensor due to angled surfaces.
- VL53L4CX measurements may vary depending on target reflectivity.
- The proportional controller cannot completely eliminate drift.

---

## Contributors

#### Pedro Lages
#### Sutharsan Ramesh

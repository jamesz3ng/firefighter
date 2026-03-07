# Firefighter Robot

An Arduino Mega-based robot that autonomously navigates an arena, detects fires using phototransistors, drives toward them, and extinguishes them with a fan. Built for MECHENG 706 at the University of Auckland.

## Hardware

| Component | Details | Pins |
|-----------|---------|------|
| Arduino Mega 2560 | Main controller | - |
| 4x Vex Motor Controllers | Mecanum drive (via Servo library PWM) | 46, 47, 50, 51 |
| HC-SR04 Ultrasonic | Front distance | Trig: 48, Echo: 49 |
| 2x Sharp IR (long range) | Front-left / front-right distance | A4, A1 |
| 2x Sharp IR (short range) | Left / right side distance | A2, A3 |
| 2x Phototransistors | Fire detection (left/right) | A9, A10 |
| Servo | Fire-tracking scanner | 37 |
| Gyroscope | Heading reference | A14 |
| DC Fan | Fire extinguisher | 52 |
| LiPo Battery | Power supply | A0 (voltage sense) |

## State Machine

```
INITIALISING → DETECT_FIRE → DRIVE_TO_FIRE → EXTINGUISH
                    ↑               |              |
                    |               ↓              |
                    <--------- AVOID  ←-------------
                    |
                    -→ STOPPED (after 2 fires extinguished)
```

- **DETECT_FIRE**: Rotates in place, reads phototransistors. When both sensors read >0.4V with balanced readings, transitions to DRIVE_TO_FIRE.
- **DRIVE_TO_FIRE**: Servo tracks the fire source. Proportional steering correction steers the robot toward the servo heading. Transitions to EXTINGUISH when close, or AVOID if an obstacle is detected.
- **AVOID / LEFT / RIGHT**: Obstacle avoidance using strafing (mecanum wheels). Returns to DETECT_FIRE after clearing the obstacle.
- **EXTINGUISH**: Aligns servo to fire, activates fan. After phototransistor readings drop below 0.4V, increments fire count. After 2 fires, stops.

## Key Software Features

- **Kalman filtering** on all distance sensors and phototransistors for noise rejection
- **NaN guard** on every Kalman filter output to prevent cascading invalid readings
- **Servo-based fire tracking** with proportional control (5.0 Kp) for smooth steering
- **Mecanum drive** support (forward, reverse, strafe, rotate)
- **Battery monitoring** with 5-reading low-voltage lockout

## Project Structure

```
├── src/firefighter/firefighter.ino   # Final working code
├── tests/
│   ├── KalmanReading.ino             # Kalman filter comparison tool
│   └── sensor_test/sensor_test.ino   # Sensor calibration suite
├── experimental/
│   ├── firefighter_modular/          # Modular architecture attempt (multi-file)
│   └── behaviour/behaviour.ino       # Behaviour-based architecture attempt
└── archive/                          # 15 earlier iterations (see archive/HISTORY.md)
```

## Build & Upload

1. Open `src/firefighter/firefighter.ino` in the Arduino IDE
2. Select **Board**: Arduino Mega 2560
3. Required libraries: `Servo` (built-in), `SoftwareSerial` (built-in)
4. Upload

## Development History

This project was developed over 6 weeks with limited access to git -- most versioning was done by copying the working folder. The 15 archived iterations trace the evolution from basic obstacle avoidance to the final dual-fire-extinguishing robot. See [`archive/HISTORY.md`](archive/HISTORY.md) for the full timeline.

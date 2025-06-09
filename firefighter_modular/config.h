#ifndef CONFIG_H
#define CONFIG_H

#include <Servo.h>
#include <SoftwareSerial.h>

// Pin definitions - Serial Communication
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11
#define STARTUP_DELAY 3

// Pin definitions - Fire Detection
#define SERVO_PIN 9
#define IR_SENSOR_PIN A0
#define LEFT_PHOTOTRANSISTOR A9
#define RIGHT_PHOTOTRANSISTOR A10

// Pin definitions - Motors (Updated for standard Arduino)
#define LEFT_MOTOR_PIN1 2
#define LEFT_MOTOR_PIN2 3
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 5
#define LEFT_MOTOR_EN 6
#define RIGHT_MOTOR_EN 7

// Pin definitions - Ultrasonic
#define TRIG_PIN 12
#define ECHO_PIN 13

// Pin definitions - Additional sensors
#define BATTERY_PIN A7
#define GYRO_PIN A6
#define BUZZER_PIN 8  // Optional

// Pin definitions - IR Sensors
#define RIGHT_FRONT_IR_PIN A1
#define LEFT_FRONT_IR_PIN A2
#define RIGHT_IR_PIN A3
#define LEFT_IR_PIN A4

// Motor parameters
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_NEUTRAL 1500
#define DEFAULT_SPEED 200
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 100

// Servo parameters
#define SERVO_CENTER 90
#define SERVO_MIN 0
#define SERVO_MAX 180
#define SERVO_STEP 5
#define SERVO_DEADBAND 10
#define SERVO_SWEEP_DELAY 100  // ms

// Fire detection parameters
#define FIRE_THRESHOLD 500  // IR sensor threshold
#define PHOTOTRANSISTOR_THRESHOLD 0.10
#define ACTIVATION_THRESHOLD 0.5
#define START_ANGLE 90
#define ANGLE_INCREMENT 1
#define MIN_FIRE_VOLTAGE 0.25
#define FIRE_LOST_TIMEOUT 2000  // ms
#define FIRE_TRACK_TIMEOUT 5000  // ms
#define FIRE_EXTINGUISH_CONFIRM_TIME 3000  // ms

// Navigation thresholds
#define OBSTACLE_DISTANCE 20  // cm
#define OBSTACLE_THRESHOLD 20  // cm (legacy compatibility)
#define SIDE_OBSTACLE_THRESHOLD 15  // cm
#define FIRE_APPROACH_DISTANCE 15  // cm
#define EMERGENCY_DISTANCE 5  // cm

// Timing parameters
#define LOOP_TIME 100  // ms
#define MAIN_LOOP_DELAY 50  // ms
#define SENSOR_CHECK_INTERVAL 200  // ms
#define STATUS_PRINT_INTERVAL 5000  // ms
#define AUTO_SEARCH_TIMEOUT 30000  // ms
#define SEARCH_TURN_INTERVAL 3000  // ms
#define MAX_SEARCH_TIME 60000  // ms
#define MAX_EXTINGUISH_TIME 10000  // ms
#define MAX_AVOIDANCE_TIME 5000  // ms
#define CALIBRATION_TIMEOUT 5000  // ms

// Gyro parameters
#define GYRO_SUPPLY_VOLTAGE 5.0
#define GYRO_SENSITIVITY 0.007
#define ROTATION_THRESHOLD 1.5

// Kalman filter noise parameters
#define PROCESS_NOISE_ULTRASONIC 1.0
#define SENSOR_NOISE_ULTRASONIC 1.0
#define PROCESS_NOISE_IR 1.0
#define SENSOR_NOISE_IR 1.0

// Battery monitoring parameters
#define BATTERY_MIN_LEVEL 717
#define BATTERY_MAX_LEVEL 860
#define BATTERY_SCALE_FACTOR 143
#define LOW_BATTERY_VOLTAGE 6.5  // V
#define CRITICAL_BATTERY_VOLTAGE 6.0  // V

// Sensor parameters
#define MAX_DIST 23200
#define MAX_DISTANCE 400  // cm
#define MIN_DISTANCE 2   // cm

// Mission parameters
#define MAX_FIRES_TO_EXTINGUISH 2
#define EXTINGUISH_TIMEOUT 10000  // ms
#define FIRE_SEARCH_TIMEOUT 10000  // ms

#endif // CONFIG_H

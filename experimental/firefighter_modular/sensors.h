#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include "kalman.h"
#include "utils.h"

// Global sensor variables
extern double front_distance;
extern double frontRightDist;
extern double frontLeftDist;
extern double rightDist;
extern double leftDist;
extern float currentAngle;
extern float gyroZeroVoltage;

// Sensor function declarations
void ReadUltrasonic();
void ReadRightFront();
void ReadLeftFront();
void ReadRight();
void ReadLeft();
void calibrateGyro();
void ReadGyro();
boolean is_battery_voltage_OK();

// External reference to serial communication
extern SoftwareSerial *SerialCom;

#endif // SENSORS_H

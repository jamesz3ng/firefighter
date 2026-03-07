#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

// Motor control function declarations
void initializeMotors();
void stopMotors();
void forward();
void backward();
void left();
void right();
void enableMotors();
void disableMotors();

// Advanced motor functions
void forwardWithCorrection(float correctionFactor);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void emergencyBrake();

// Motor speed control
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);

#endif // MOTORS_H

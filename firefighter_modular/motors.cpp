#include "motors.h"
#include "sensors.h"
#include "utils.h"

void initializeMotors() {
    // Set motor pins as outputs
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
    pinMode(LEFT_MOTOR_EN, OUTPUT);
    pinMode(RIGHT_MOTOR_EN, OUTPUT);
    
    // Stop all motors initially
    stopMotors();
    
    Serial.println("Motors initialized");
}

void stopMotors() {
    // Stop left motor
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    analogWrite(LEFT_MOTOR_EN, 0);
    
    // Stop right motor
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, 0);
}

void forward() {
    // Check battery safety (from original code analysis)
    float voltage = getVoltage();
    if (voltage < LOW_BATTERY_VOLTAGE && voltage > 0) {
        Serial.println("Battery too low for forward movement");
        return;
    }
    
    // Left motor forward
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    analogWrite(LEFT_MOTOR_EN, DEFAULT_SPEED);
    
    // Right motor forward
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, DEFAULT_SPEED);
}

void backward() {
    // Check battery safety
    float voltage = getVoltage();
    if (voltage < LOW_BATTERY_VOLTAGE && voltage > 0) {
        Serial.println("Battery too low for backward movement");
        return;
    }
    
    // Left motor backward
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, DEFAULT_SPEED);
    
    // Right motor backward
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    analogWrite(RIGHT_MOTOR_EN, DEFAULT_SPEED);
}

void left() {
    // Check battery safety - this function had commented battery check in original
    float voltage = getVoltage();
    if (voltage < LOW_BATTERY_VOLTAGE && voltage > 0) {
        Serial.println("Battery too low for left turn");
        return;
    }
    
    // Left motor backward, right motor forward
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, DEFAULT_SPEED);
    
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, DEFAULT_SPEED);
}

void right() {
    // Check battery safety
    float voltage = getVoltage();
    if (voltage < LOW_BATTERY_VOLTAGE && voltage > 0) {
        Serial.println("Battery too low for right turn");
        return;
    }
    
    // Left motor forward, right motor backward
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    analogWrite(LEFT_MOTOR_EN, DEFAULT_SPEED);
    
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    analogWrite(RIGHT_MOTOR_EN, DEFAULT_SPEED);
}

void enableMotors() {
    // Motors are enabled by default when moving
    Serial.println("Motors enabled");
}

void disableMotors() {
    stopMotors();
    Serial.println("Motors disabled");
}

void forwardWithCorrection(float correctionFactor) {
    // Apply gyroscope correction to maintain straight line
    int leftSpeed = DEFAULT_SPEED;
    int rightSpeed = DEFAULT_SPEED;
    
    // Adjust speeds based on correction factor
    if (correctionFactor > 0) {
        // Robot drifting right, slow down right motor
        rightSpeed = DEFAULT_SPEED * (1.0 - abs(correctionFactor));
    } else if (correctionFactor < 0) {
        // Robot drifting left, slow down left motor
        leftSpeed = DEFAULT_SPEED * (1.0 - abs(correctionFactor));
    }
    
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Constrain speeds to safe range
    leftSpeed = constrain(leftSpeed, 0, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_MOTOR_SPEED);
    
    // Check battery safety
    float voltage = getVoltage();
    if (voltage < LOW_BATTERY_VOLTAGE && voltage > 0) {
        leftSpeed = 0;
        rightSpeed = 0;
    }
    
    // Set left motor
    if (leftSpeed > 0) {
        digitalWrite(LEFT_MOTOR_PIN1, HIGH);
        digitalWrite(LEFT_MOTOR_PIN2, LOW);
    } else {
        digitalWrite(LEFT_MOTOR_PIN1, LOW);
        digitalWrite(LEFT_MOTOR_PIN2, LOW);
    }
    analogWrite(LEFT_MOTOR_EN, abs(leftSpeed));
    
    // Set right motor
    if (rightSpeed > 0) {
        digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
        digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    } else {
        digitalWrite(RIGHT_MOTOR_PIN1, LOW);
        digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    }
    analogWrite(RIGHT_MOTOR_EN, abs(rightSpeed));
}

void emergencyBrake() {
    // Immediate stop for emergency situations
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    analogWrite(LEFT_MOTOR_EN, 0);
    analogWrite(RIGHT_MOTOR_EN, 0);
    
    Serial.println("EMERGENCY BRAKE ACTIVATED!");
}

void setLeftMotorSpeed(int speed) {
    speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    if (speed > 0) {
        digitalWrite(LEFT_MOTOR_PIN1, HIGH);
        digitalWrite(LEFT_MOTOR_PIN2, LOW);
        analogWrite(LEFT_MOTOR_EN, speed);
    } else if (speed < 0) {
        digitalWrite(LEFT_MOTOR_PIN1, LOW);
        digitalWrite(LEFT_MOTOR_PIN2, HIGH);
        analogWrite(LEFT_MOTOR_EN, -speed);
    } else {
        digitalWrite(LEFT_MOTOR_PIN1, LOW);
        digitalWrite(LEFT_MOTOR_PIN2, LOW);
        analogWrite(LEFT_MOTOR_EN, 0);
    }
}

void setRightMotorSpeed(int speed) {
    speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    if (speed > 0) {
        digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
        digitalWrite(RIGHT_MOTOR_PIN2, LOW);
        analogWrite(RIGHT_MOTOR_EN, speed);
    } else if (speed < 0) {
        digitalWrite(RIGHT_MOTOR_PIN1, LOW);
        digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
        analogWrite(RIGHT_MOTOR_EN, -speed);
    } else {
        digitalWrite(RIGHT_MOTOR_PIN1, LOW);
        digitalWrite(RIGHT_MOTOR_PIN2, LOW);
        analogWrite(RIGHT_MOTOR_EN, 0);
    }
}

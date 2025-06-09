/*
 * Firefighter Robot - Modular Implementation
 * 
 * A comprehensive firefighter robot that can:
 * - Search for fires using IR sensor and servo-mounted detection
 * - Navigate autonomously while avoiding obstacles
 * - Track and approach fire sources
 * - Extinguish fires (mechanism to be implemented)
 * 
 * This modular implementation separates concerns into different files:
 * - config.h: All constants and pin definitions
 * - sensors.h/.cpp: Sensor reading and management
 * - motors.h/.cpp: Motor control and movement
 * - fire_detection.h/.cpp: Fire detection and tracking
 * - kalman.h/.cpp: Kalman filtering for sensor noise
 * - states.h/.cpp: State machine implementation
 * - utils.h/.cpp: Utility functions
 * 
 * Hardware Requirements:
 * - Arduino Uno/Nano
 * - Ultrasonic sensor (HC-SR04)
 * - IR flame sensor
 * - Servo motor for sensor scanning
 * - 4 DC motors with motor driver
 * - MPU6050 gyroscope (optional)
 * - Voltage divider for battery monitoring
 * - Buzzer for alerts (optional)
 * 
 * Author: GitHub Copilot & User
 * Version: 2.0 (Modular)
 * Date: 2024
 */

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "fire_detection.h"
#include "kalman.h"
#include "states.h"
#include "utils.h"
#include <Servo.h>

// Global servo object for fire detection
Servo myservo;

// Main setup function
void setup() {
    Serial.begin(9600);
    Serial.println("Firefighter Robot v2.0 Starting...");
    
    // Initialize all subsystems
    initializePins();
    initializeSensors();
    initializeMotors();
    initializeFireDetection();
    initializeKalmanFilters();
    
    // Start in calibration state
    changeState(CALIBRATION);
    
    Serial.println("Initialization complete. Robot ready.");
    delay(1000);
}

// Main loop function
void loop() {
    // Process serial commands for manual control
    processSerialCommands();
    
    // Update all sensors
    updateAllSensors();
    
    // Execute current state
    executeCurrentState();
    
    // Print status periodically
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > STATUS_PRINT_INTERVAL) {
        printStatus();
        lastStatusPrint = millis();
    }
    
    // Small delay for stability
    delay(MAIN_LOOP_DELAY);
}

// Initialize all pins
void initializePins() {
    // Motor pins
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
    pinMode(LEFT_MOTOR_EN, OUTPUT);
    pinMode(RIGHT_MOTOR_EN, OUTPUT);
    
    // Sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(BATTERY_PIN, INPUT);
    
    // Optional pins
    #ifdef BUZZER_PIN
    pinMode(BUZZER_PIN, OUTPUT);
    #endif
    
    Serial.println("Pins initialized");
}

// Print system status
void printStatus() {
    Serial.println("=== FIREFIGHTER ROBOT STATUS ===");
    Serial.print("State: ");
    Serial.println(getStateName(getCurrentState()));
    Serial.print("Battery: ");
    Serial.print(getVoltage());
    Serial.println("V");
    Serial.print("Distance: ");
    Serial.print(getDistance());
    Serial.println("cm");
    Serial.print("Fire Detected: ");
    Serial.println(fireDetected ? "YES" : "NO");
    if (fireDetected) {
        Serial.print("Fire Direction: ");
        int dir = getFireDirection();
        if (dir < 0) Serial.println("LEFT");
        else if (dir > 0) Serial.println("RIGHT");
        else Serial.println("CENTER");
    }
    Serial.print("Servo Position: ");
    Serial.println(servoPosition);
    Serial.println("==============================");
}

// Emergency stop function (can be called via serial or external interrupt)
void emergencyStop() {
    changeState(EMERGENCY_STOP);
    Serial.println("EMERGENCY STOP ACTIVATED!");
}

// Manual control functions (for testing via serial commands)
void processSerialCommands() {
    if (Serial.available()) {
        char command = Serial.read();
        
        switch (command) {
            case 'w': // Forward
                if (getCurrentState() == IDLE) {
                    forward();
                    delaySeconds(0.5);
                    stopMotors();
                }
                break;
            case 's': // Backward
                if (getCurrentState() == IDLE) {
                    backward();
                    delaySeconds(0.5);
                    stopMotors();
                }
                break;
            case 'a': // Left
                if (getCurrentState() == IDLE) {
                    left();
                    delaySeconds(0.5);
                    stopMotors();
                }
                break;
            case 'd': // Right
                if (getCurrentState() == IDLE) {
                    right();
                    delaySeconds(0.5);
                    stopMotors();
                }
                break;
            case 'f': // Start fire search
                changeState(SEARCH_FIRE);
                break;
            case 'i': // Return to idle
                changeState(IDLE);
                break;
            case 'e': // Emergency stop
                emergencyStop();
                break;
            case 'c': // Calibrate
                changeState(CALIBRATION);
                break;
            case 'p': // Print status
                printStatus();
                break;
            case 'h': // Help
                printHelp();
                break;
        }
    }
}

void printHelp() {
    Serial.println("=== FIREFIGHTER ROBOT COMMANDS ===");
    Serial.println("w - Move forward (if idle)");
    Serial.println("s - Move backward (if idle)");
    Serial.println("a - Turn left (if idle)");
    Serial.println("d - Turn right (if idle)");
    Serial.println("f - Start fire search");
    Serial.println("i - Return to idle");
    Serial.println("e - Emergency stop");
    Serial.println("c - Calibrate sensors");
    Serial.println("p - Print status");
    Serial.println("h - Show this help");
    Serial.println("=================================");
}

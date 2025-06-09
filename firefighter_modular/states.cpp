#include "states.h"
#include "sensors.h"
#include "motors.h"
#include "fire_detection.h"
#include "utils.h"

// State variables
RobotState currentState = IDLE;
RobotState previousState = IDLE;
unsigned long stateStartTime = 0;
unsigned long lastStateChange = 0;

void changeState(RobotState newState) {
    if (newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();
        lastStateChange = millis();
        
        Serial.print("State changed from ");
        Serial.print(getStateName(previousState));
        Serial.print(" to ");
        Serial.println(getStateName(currentState));
        
        // Stop motors during state transitions for safety
        stopMotors();
    }
}

void executeCurrentState() {
    // Check for emergency conditions first
    if (shouldEmergencyStop() && currentState != EMERGENCY_STOP) {
        changeState(EMERGENCY_STOP);
        return;
    }
    
    // Check for low battery
    if (isBatteryLow() && currentState != BATTERY_LOW && currentState != EMERGENCY_STOP) {
        changeState(BATTERY_LOW);
        return;
    }
    
    // Execute current state
    switch (currentState) {
        case IDLE:
            stateIdle();
            break;
        case SEARCH_FIRE:
            stateSearchFire();
            break;
        case DRIVE_TO_FIRE:
            stateDriveToFire();
            break;
        case EXTINGUISH_FIRE:
            stateExtinguishFire();
            break;
        case NAVIGATE_OBSTACLE:
            stateNavigateObstacle();
            break;
        case EMERGENCY_STOP:
            stateEmergencyStop();
            break;
        case BATTERY_LOW:
            stateBatteryLow();
            break;
        case CALIBRATION:
            stateCalibration();
            break;
        default:
            changeState(IDLE);
            break;
    }
}

RobotState getCurrentState() {
    return currentState;
}

String getStateName(RobotState state) {
    switch (state) {
        case IDLE: return "IDLE";
        case SEARCH_FIRE: return "SEARCH_FIRE";
        case DRIVE_TO_FIRE: return "DRIVE_TO_FIRE";
        case EXTINGUISH_FIRE: return "EXTINGUISH_FIRE";
        case NAVIGATE_OBSTACLE: return "NAVIGATE_OBSTACLE";
        case EMERGENCY_STOP: return "EMERGENCY_STOP";
        case BATTERY_LOW: return "BATTERY_LOW";
        case CALIBRATION: return "CALIBRATION";
        default: return "UNKNOWN";
    }
}

// State implementations
void stateIdle() {
    static unsigned long idleStartTime = millis();
    
    // Stop all motors
    stopMotors();
    
    // Check sensors periodically
    static unsigned long lastSensorCheck = 0;
    if (millis() - lastSensorCheck > SENSOR_CHECK_INTERVAL) {
        updateAllSensors();
        lastSensorCheck = millis();
    }
    
    // Transition conditions
    if (shouldSearchForFire()) {
        changeState(SEARCH_FIRE);
    } else if (millis() - idleStartTime > AUTO_SEARCH_TIMEOUT) {
        // Automatically start searching after idle timeout
        changeState(SEARCH_FIRE);
    }
}

void stateSearchFire() {
    static unsigned long searchStartTime = millis();
    
    // Update sensors
    updateAllSensors();
    
    // Check for obstacles while searching
    if (shouldNavigateObstacle()) {
        changeState(NAVIGATE_OBSTACLE);
        return;
    }
    
    // Sweep servo to look for fire
    sweepForFire();
    
    // If fire is detected, transition to drive state
    if (shouldDriveToFire()) {
        changeState(DRIVE_TO_FIRE);
        return;
    }
    
    // Continue search pattern - slow forward movement with turning
    static unsigned long lastTurn = 0;
    static int turnDirection = 1;
    
    if (millis() - lastTurn > SEARCH_TURN_INTERVAL) {
        // Turn slightly to search new area
        if (turnDirection > 0) {
            right();
        } else {
            left();
        }
        delaySeconds(0.5);
        stopMotors();
        
        turnDirection *= -1; // Alternate turn direction
        lastTurn = millis();
    } else {
        // Move slowly forward
        forward();
        delaySeconds(0.3);
        stopMotors();
        delaySeconds(0.2);
    }
    
    // Return to idle if searching too long
    if (millis() - searchStartTime > MAX_SEARCH_TIME) {
        changeState(IDLE);
    }
}

void stateDriveToFire() {
    // Update sensors
    updateAllSensors();
    
    // Check for obstacles
    if (shouldNavigateObstacle()) {
        changeState(NAVIGATE_OBSTACLE);
        return;
    }
    
    // Track fire continuously
    trackFire();
    
    // Get fire direction
    int fireDir = getFireDirection();
    
    if (!fireDetected) {
        // Lost fire, go back to searching
        changeState(SEARCH_FIRE);
        return;
    }
    
    // Check if close enough to extinguish
    if (shouldExtinguishFire()) {
        changeState(EXTINGUISH_FIRE);
        return;
    }
    
    // Navigate towards fire
    if (fireDir < 0) {
        // Fire is on the left
        left();
        delaySeconds(0.2);
    } else if (fireDir > 0) {
        // Fire is on the right
        right();
        delaySeconds(0.2);
    } else {
        // Fire is straight ahead
        forward();
        delaySeconds(0.3);
    }
    
    stopMotors();
    delaySeconds(0.1);
}

void stateExtinguishFire() {
    static unsigned long extinguishStartTime = millis();
    
    // Stop moving
    stopMotors();
    
    // Point directly at fire and extinguish
    extinguishFire();
    
    // Check if fire is extinguished
    if (isFireExtinguished()) {
        Serial.println("Fire successfully extinguished!");
        changeState(IDLE);
        return;
    }
    
    // Timeout if taking too long
    if (millis() - extinguishStartTime > MAX_EXTINGUISH_TIME) {
        Serial.println("Extinguish timeout - returning to search");
        changeState(SEARCH_FIRE);
    }
}

void stateNavigateObstacle() {
    static unsigned long avoidanceStartTime = millis();
    
    // Simple obstacle avoidance - back up and turn
    backward();
    delaySeconds(0.5);
    
    // Turn away from obstacle
    if (getDistance() < OBSTACLE_DISTANCE) {
        // Choose turn direction based on which side is clearer
        // For now, just turn right
        right();
        delaySeconds(1.0);
    }
    
    stopMotors();
    
    // Check if obstacle is cleared
    updateAllSensors();
    if (!shouldNavigateObstacle()) {
        // Return to previous state
        if (fireDetected) {
            changeState(DRIVE_TO_FIRE);
        } else {
            changeState(SEARCH_FIRE);
        }
    }
    
    // Timeout protection
    if (millis() - avoidanceStartTime > MAX_AVOIDANCE_TIME) {
        changeState(IDLE);
    }
}

void stateEmergencyStop() {
    // Stop all motors immediately
    stopMotors();
    
    // Flash LED or make sound to indicate emergency
    static unsigned long lastFlash = 0;
    if (millis() - lastFlash > 500) {
        // Toggle LED if available
        lastFlash = millis();
    }
    
    Serial.println("EMERGENCY STOP ACTIVE");
    
    // Only exit emergency stop manually or if condition clears
    if (!shouldEmergencyStop()) {
        delaySeconds(2); // Wait before resuming
        changeState(IDLE);
    }
}

void stateBatteryLow() {
    // Stop all motors
    stopMotors();
    
    // Warn about low battery
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 5000) {
        Serial.println("WARNING: BATTERY LOW!");
        lastWarning = millis();
    }
    
    // If battery recovers, return to idle
    if (!isBatteryLow()) {
        changeState(IDLE);
    }
}

void stateCalibration() {
    // Calibration routine for sensors
    static bool calibrationComplete = false;
    static unsigned long calibrationStart = millis();
    
    stopMotors();
    
    if (!calibrationComplete) {
        Serial.println("Calibrating sensors...");
        
        // Calibrate gyroscope
        initializeGyro();
        
        // Calibrate servo position
        initializeFireDetection();
        
        // Calibration timeout
        if (millis() - calibrationStart > CALIBRATION_TIMEOUT) {
            calibrationComplete = true;
            Serial.println("Calibration complete");
        }
    } else {
        changeState(IDLE);
    }
}

// State transition condition functions
bool shouldSearchForFire() {
    return !fireDetected && currentState == IDLE;
}

bool shouldDriveToFire() {
    return fireDetected && getDistance() > FIRE_APPROACH_DISTANCE;
}

bool shouldExtinguishFire() {
    return fireDetected && getDistance() <= FIRE_APPROACH_DISTANCE;
}

bool shouldNavigateObstacle() {
    return getDistance() < OBSTACLE_DISTANCE && getDistance() > 0;
}

bool shouldEmergencyStop() {
    // Emergency conditions
    float voltage = getVoltage();
    return (voltage < CRITICAL_BATTERY_VOLTAGE) || 
           (getDistance() < EMERGENCY_DISTANCE && getDistance() > 0);
}

bool shouldReturnToIdle() {
    return !fireDetected && currentState != IDLE && currentState != EMERGENCY_STOP;
}

bool isBatteryLow() {
    float voltage = getVoltage();
    return voltage < LOW_BATTERY_VOLTAGE && voltage > 0;
}

#include "fire_detection.h"
#include "sensors.h"
#include "utils.h"
#include <Servo.h>

// External servo object (defined in main file)
extern Servo myservo;

// Fire detection state variables
bool fireDetected = false;
int fireDirection = 0;
int servoPosition = SERVO_CENTER;
unsigned long lastFireDetectionTime = 0;
bool isTracking = false;

void initializeFireDetection() {
    myservo.attach(SERVO_PIN);
    myservo.write(SERVO_CENTER);
    delay(500);
    fireDetected = false;
    fireDirection = 0;
    servoPosition = SERVO_CENTER;
    isTracking = false;
}

bool detectFire() {
    int irValue = analogRead(IR_SENSOR_PIN);
    
    // Update fire detection status
    if (irValue > FIRE_THRESHOLD) {
        fireDetected = true;
        lastFireDetectionTime = millis();
        return true;
    }
    
    // Check if fire was lost recently
    if (millis() - lastFireDetectionTime > FIRE_LOST_TIMEOUT) {
        fireDetected = false;
    }
    
    return fireDetected;
}

int getFireDirection() {
    if (!fireDetected) {
        return 0; // No fire detected
    }
    
    // Calculate direction based on servo position where fire was detected
    if (servoPosition < SERVO_CENTER - SERVO_DEADBAND) {
        return -1; // Fire on the left
    } else if (servoPosition > SERVO_CENTER + SERVO_DEADBAND) {
        return 1;  // Fire on the right
    } else {
        return 0;  // Fire straight ahead
    }
}

void trackFire() {
    static unsigned long lastSweepTime = 0;
    static int sweepDirection = 1;
    static bool foundFire = false;
    
    // If fire was recently detected, track it
    if (fireDetected && millis() - lastFireDetectionTime < FIRE_TRACK_TIMEOUT) {
        isTracking = true;
        
        // Fine-tune servo position for better tracking
        int irValue = analogRead(IR_SENSOR_PIN);
        
        if (irValue > FIRE_THRESHOLD) {
            // Fire is strong, we're pointing at it
            foundFire = true;
            fireDirection = getFireDirection();
        } else {
            // Search around current position
            if (millis() - lastSweepTime > SERVO_SWEEP_DELAY) {
                servoPosition += sweepDirection * SERVO_STEP;
                
                // Constrain servo position
                if (servoPosition >= SERVO_MAX) {
                    servoPosition = SERVO_MAX;
                    sweepDirection = -1;
                } else if (servoPosition <= SERVO_MIN) {
                    servoPosition = SERVO_MIN;
                    sweepDirection = 1;
                }
                
                myservo.write(servoPosition);
                lastSweepTime = millis();
            }
        }
    } else {
        isTracking = false;
        foundFire = false;
    }
}

void extinguishFire() {
    // TODO: Implement actual fire extinguishing mechanism
    // This could control a fan, water pump, or CO2 system
    
    // For now, just simulate extinguishing action
    Serial.println("EXTINGUISHING FIRE!");
    
    // Point servo directly at fire
    if (fireDetected) {
        myservo.write(servoPosition);
        delay(100);
        
        // TODO: Activate extinguishing mechanism
        // digitalWrite(FAN_PIN, HIGH);
        // delay(EXTINGUISH_DURATION);
        // digitalWrite(FAN_PIN, LOW);
    }
}

bool isFireExtinguished() {
    // Check if fire is no longer detected
    int irValue = analogRead(IR_SENSOR_PIN);
    
    // Fire is extinguished if IR reading is below threshold for extended time
    static unsigned long noFireTime = 0;
    
    if (irValue < FIRE_THRESHOLD) {
        if (noFireTime == 0) {
            noFireTime = millis();
        } else if (millis() - noFireTime > FIRE_EXTINGUISH_CONFIRM_TIME) {
            fireDetected = false;
            return true;
        }
    } else {
        noFireTime = 0;
    }
    
    return false;
}

void sweepForFire() {
    static unsigned long lastSweepTime = 0;
    static int sweepDirection = 1;
    
    if (millis() - lastSweepTime > SERVO_SWEEP_DELAY) {
        servoPosition += sweepDirection * SERVO_STEP;
        
        // Constrain and reverse direction at limits
        if (servoPosition >= SERVO_MAX) {
            servoPosition = SERVO_MAX;
            sweepDirection = -1;
        } else if (servoPosition <= SERVO_MIN) {
            servoPosition = SERVO_MIN;
            sweepDirection = 1;
        }
        
        myservo.write(servoPosition);
        lastSweepTime = millis();
        
        // Check for fire at current position
        delaySeconds(0.1); // Small delay for servo to settle
        if (detectFire()) {
            fireDirection = getFireDirection();
            Serial.print("Fire detected at servo position: ");
            Serial.println(servoPosition);
        }
    }
}

int findBestFireDirection() {
    int bestDirection = 0;
    int maxFireIntensity = 0;
    int bestServoPosition = SERVO_CENTER;
    
    // Sweep through full range to find strongest fire signal
    for (int pos = SERVO_MIN; pos <= SERVO_MAX; pos += SERVO_STEP) {
        myservo.write(pos);
        delay(SERVO_SWEEP_DELAY);
        
        int irValue = analogRead(IR_SENSOR_PIN);
        
        if (irValue > maxFireIntensity && irValue > FIRE_THRESHOLD) {
            maxFireIntensity = irValue;
            bestServoPosition = pos;
            
            // Calculate direction
            if (pos < SERVO_CENTER - SERVO_DEADBAND) {
                bestDirection = -1; // Left
            } else if (pos > SERVO_CENTER + SERVO_DEADBAND) {
                bestDirection = 1;  // Right
            } else {
                bestDirection = 0;  // Center
            }
        }
    }
    
    // Return to best position
    if (maxFireIntensity > 0) {
        servoPosition = bestServoPosition;
        myservo.write(servoPosition);
        fireDetected = true;
        lastFireDetectionTime = millis();
    } else {
        // No fire found, return to center
        servoPosition = SERVO_CENTER;
        myservo.write(SERVO_CENTER);
        fireDetected = false;
    }
    
    return bestDirection;
}

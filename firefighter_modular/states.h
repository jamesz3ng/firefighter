#ifndef STATES_H
#define STATES_H

#include "config.h"

// State machine states
enum RobotState {
    IDLE,
    SEARCH_FIRE,
    DRIVE_TO_FIRE,
    EXTINGUISH_FIRE,
    NAVIGATE_OBSTACLE,
    EMERGENCY_STOP,
    BATTERY_LOW,
    CALIBRATION
};

// State management functions
void changeState(RobotState newState);
void executeCurrentState();
RobotState getCurrentState();
String getStateName(RobotState state);

// Individual state implementations
void stateIdle();
void stateSearchFire();
void stateDriveToFire();
void stateExtinguishFire();
void stateNavigateObstacle();
void stateEmergencyStop();
void stateBatteryLow();
void stateCalibration();

// State transition conditions
bool shouldSearchForFire();
bool shouldDriveToFire();
bool shouldExtinguishFire();
bool shouldNavigateObstacle();
bool shouldEmergencyStop();
bool shouldReturnToIdle();
bool isBatteryLow();

// State variables
extern RobotState currentState;
extern RobotState previousState;
extern unsigned long stateStartTime;
extern unsigned long lastStateChange;

#endif

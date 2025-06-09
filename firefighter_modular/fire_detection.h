#ifndef FIRE_DETECTION_H
#define FIRE_DETECTION_H

#include "config.h"

// Fire detection and tracking functions
void initializeFireDetection();
bool detectFire();
int getFireDirection();
void trackFire();
void extinguishFire();
bool isFireExtinguished();
void sweepForFire();
int findBestFireDirection();

// Fire detection state variables
extern bool fireDetected;
extern int fireDirection;
extern int servoPosition;
extern unsigned long lastFireDetectionTime;
extern bool isTracking;

#endif

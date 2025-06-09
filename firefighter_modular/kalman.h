#ifndef KALMAN_H
#define KALMAN_H

#include "config.h"

// Global variables for Kalman filters
extern double last_var_ultrasonic;
extern double last_var_ir_right;
extern double last_var_ir_left;
extern double last_var_ir_back;
extern double last_var_ir_short_left;

// Kalman filter function declarations
double Kalman_ultrasonic(double rawdata, double prev_est);
double Kalman_ir_right(double rawdata, double prev_est);
double Kalman_ir_left(double rawdata, double prev_est);
double Kalman_ir_back(double rawdata, double prev_est);
double Kalman_ir_short_left(double rawdata, double prev_est);

#endif // KALMAN_H

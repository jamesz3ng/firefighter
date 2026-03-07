#include "kalman.h"

// Global variables for Kalman filters
double last_var_ultrasonic = 1.0;
double last_var_ir_right = 1.0;
double last_var_ir_left = 1.0;
double last_var_ir_back = 1.0;
double last_var_ir_short_left = 1.0;

double Kalman_ultrasonic(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ultrasonic + PROCESS_NOISE_ULTRASONIC;

  kalman_gain = a_priori_var / (a_priori_var + SENSOR_NOISE_ULTRASONIC);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ultrasonic = a_post_var;
  return a_post_est;
}

double Kalman_ir_right(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_right + PROCESS_NOISE_IR;

  kalman_gain = a_priori_var / (a_priori_var + SENSOR_NOISE_IR);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_right = a_post_var;
  return a_post_est;
}

double Kalman_ir_left(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_left + PROCESS_NOISE_IR;

  kalman_gain = a_priori_var / (a_priori_var + SENSOR_NOISE_IR);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_left = a_post_var;
  return a_post_est;
}

double Kalman_ir_back(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_back + PROCESS_NOISE_IR;

  kalman_gain = a_priori_var / (a_priori_var + SENSOR_NOISE_IR);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_back = a_post_var;
  return a_post_est;
}

double Kalman_ir_short_left(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_short_left + PROCESS_NOISE_IR;

  kalman_gain = a_priori_var / (a_priori_var + SENSOR_NOISE_IR);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_short_left = a_post_var;
  return a_post_est;
}

#include <Servo.h>
#include <SoftwareSerial.h>

/*
 * Firefighter Robot - Comprehensive Sensor Testing
 * 
 * This file tests all sensors used in the firefighter robot:
 * - HC-SR04 Ultrasonic Distance Sensor
 * - Sharp IR Distance Sensors (Front Left, Front Right, Left, Right)
 * - Gyroscope for orientation tracking
 * - Photo transistors for fire detection
 * - Battery voltage monitoring
 * 
 * All sensor readings are processed through Kalman filters for noise reduction
 */

// Serial Communication Setup
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11
#define STARTUP_DELAY 3
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
SoftwareSerial *SerialCom;

// Test mode selection
enum TEST_MODE {
  ALL_SENSORS,
  ULTRASONIC_ONLY,
  IR_SENSORS_ONLY,
  GYRO_ONLY,
  FIRE_DETECTION_ONLY,
  BATTERY_ONLY
};

TEST_MODE current_test_mode = ALL_SENSORS;

// Timing control
int T = 50; // Main loop timing (ms)
unsigned long previous_millis = 0;

// ========== ULTRASONIC SENSOR ==========
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const unsigned int MAX_DIST = 23200;
double front_distance = 30.0;
double last_var_ultrasonic = 1.0;
double process_noise_ultrasonic = 0.5;
double sensor_noise_ultrasonic = 3.0;

// ========== IR SENSORS ==========
// Front Right IR
const int rightFrontIrPin = A1;
int rightIrSensorValue = 0;
float rightIrDistance = 0.0;
double frontRightDist = 30.0;
double last_var_ir_right = 1.0;
double process_noise_ir_right = 1.0;
double sensor_noise_ir_right = 1.0;

// Front Left IR
const int leftFrontIrPin = A4;
int leftIrSensorValue = 0;
float leftIrDistance = 0.0;
double frontLeftDist = 30.0;
double last_var_ir_left = 1.0;
double process_noise_ir_left = 1.0;
double sensor_noise_ir_left = 1.0;

// Right Side IR
const int rightIrPin = A3;
int backIrSensorValue = 0;
float backIrDistance = 0.0;
double rightDist = 30.0;
double last_var_ir_back = 1.0;
double process_noise_ir_back = 1.0;
double sensor_noise_ir_back = 1.0;

// Left Side IR
const int leftIrPin = A2;
int shortleftIrSensorValue = 0;
float shortleftIrDistance = 0.0;
double leftDist = 30.0;
double last_var_ir_short_left = 1.0;
double process_noise_ir_short_left = 1.0;
double sensor_noise_ir_short_left = 1.0;

// ========== GYROSCOPE ==========
int gyroPin = A14;
int gyroSensorValue = 0;
float gyroSupplyVoltage = 5.0;
float gyroZeroVoltage = 0.0;
float gyroSensitivity = 0.007;
float rotationThreshold = 1.5;
float gyroRate = 0.0;
float currentAngle = 0.0;

// ========== FIRE DETECTION ==========
#define LEFT_PHOTOTRANSISTOR A9
#define RIGHT_PHOTOTRANSISTOR A10
#define PHOTOTRANSISTOR_THRESHOLD 0.10

// ========== FUNCTION PROTOTYPES ==========
void setup_sensors();
void test_all_sensors();
void test_ultrasonic();
void test_ir_sensors();
void test_gyro();
void test_fire_detection();
void test_battery();
void print_test_menu();
void handle_serial_input();

// Sensor reading functions
void ReadUltrasonic();
void ReadRightFront();
void ReadLeftFront();
void ReadRight();
void ReadLeft();
void ReadGyro();
void calibrateGyro();
float readPhotoTransistor(int analog_pin);
boolean is_battery_voltage_OK();

// Kalman filter functions
double Kalman_ultrasonic(double rawdata, double prev_est);
double Kalman_ir_right(double rawdata, double prev_est);
double Kalman_ir_left(double rawdata, double prev_est);
double Kalman_ir_back(double rawdata, double prev_est);
double Kalman_ir_short_left(double rawdata, double prev_est);

// Utility functions
void delaySeconds(int seconds);
void print_separator();

void setup() {
  // Initialize serial communication
  SerialCom = &BluetoothSerial;
  SerialCom->begin(115200);
  Serial.begin(115200);
  
  delaySeconds(STARTUP_DELAY);
  
  SerialCom->println(F("=== FIREFIGHTER ROBOT SENSOR TEST ==="));
  SerialCom->println(F("Initializing sensors..."));
  
  setup_sensors();
  
  SerialCom->println(F("Sensor initialization complete!"));
  print_test_menu();
}

void loop() {
  // Handle serial input for test mode selection
  handle_serial_input();
  
  // Run tests based on current mode
  if (millis() - previous_millis > T) {
    previous_millis = millis();
    
    switch (current_test_mode) {
      case ALL_SENSORS:
        test_all_sensors();
        break;
      case ULTRASONIC_ONLY:
        test_ultrasonic();
        break;
      case IR_SENSORS_ONLY:
        test_ir_sensors();
        break;
      case GYRO_ONLY:
        test_gyro();
        break;
      case FIRE_DETECTION_ONLY:
        test_fire_detection();
        break;
      case BATTERY_ONLY:
        test_battery();
        break;
    }
    
    // delay(100); // Small delay between readings
  }
}

void setup_sensors() {
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // IR sensor pins (analog, no setup needed)
  // Gyro pin (analog, no setup needed)
  // Photo transistor pins (analog, no setup needed)
  
  // Calibrate gyroscope
  SerialCom->println(F("Calibrating gyroscope..."));
  calibrateGyro();
  SerialCom->println(F("Gyroscope calibration complete."));
  
  // Initialize all distance variables
  front_distance = 30.0;
  frontRightDist = 30.0;
  frontLeftDist = 30.0;
  rightDist = 30.0;
  leftDist = 30.0;
  
  // Initialize Kalman filter variances
  last_var_ultrasonic = 1.0;
  last_var_ir_right = 1.0;
  last_var_ir_left = 1.0;
  last_var_ir_back = 1.0;
  last_var_ir_short_left = 1.0;
}

void test_all_sensors() {
  static int counter = 0;
  counter++;
  
  // Read all sensors
  ReadUltrasonic();
  ReadRightFront();
  ReadLeftFront();
  ReadRight();
  ReadLeft();
  ReadGyro();
  
  // Print results every 20 readings (about 1 second)
  if (counter >= 20) {
    counter = 0;
    print_separator();
    SerialCom->println(F("=== ALL SENSORS TEST ==="));
    
    // Distance sensors
    SerialCom->print(F("Ultrasonic (front): "));
    SerialCom->print(front_distance, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("IR Front Right: "));
    SerialCom->print(frontRightDist, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("IR Front Left: "));
    SerialCom->print(frontLeftDist, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("IR Right Side: "));
    SerialCom->print(rightDist, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("IR Left Side: "));
    SerialCom->print(leftDist, 2);
    SerialCom->println(F(" cm"));
    
    // Gyroscope
    SerialCom->print(F("Gyro Angle: "));
    SerialCom->print(currentAngle, 2);
    SerialCom->println(F(" degrees"));
    
    // Fire detection
    float leftFire = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
    float rightFire = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);
    SerialCom->print(F("Fire Left: "));
    SerialCom->print(leftFire, 3);
    SerialCom->print(F("V, Right: "));
    SerialCom->print(rightFire, 3);
    SerialCom->println(F("V"));
    
    // Battery
    boolean battery_ok = is_battery_voltage_OK();
    SerialCom->print(F("Battery Status: "));
    SerialCom->println(battery_ok ? F("OK") : F("LOW"));
    
    print_separator();
  }
}

void test_ultrasonic() {
  static int counter = 0;
  counter++;
  
  ReadUltrasonic();
  
  if (counter >= 10) {
    counter = 0;
    SerialCom->print(F("[ULTRASONIC] Distance: "));
    SerialCom->print(front_distance, 2);
    SerialCom->println(F(" cm"));
  }
}

void test_ir_sensors() {
  static int counter = 0;
  counter++;
  
  ReadRightFront();
  ReadLeftFront();
  ReadRight();
  ReadLeft();
  
  if (counter >= 10) {
    counter = 0;
    SerialCom->println(F("=== IR SENSORS ==="));
    SerialCom->print(F("Front Right: "));
    SerialCom->print(frontRightDist, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("Front Left: "));
    SerialCom->print(frontLeftDist, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("Right Side: "));
    SerialCom->print(rightDist, 2);
    SerialCom->println(F(" cm"));
    
    SerialCom->print(F("Left Side: "));
    SerialCom->print(leftDist, 2);
    SerialCom->println(F(" cm"));
    print_separator();
  }
}

void test_gyro() {
  static int counter = 0;
  counter++;
  
  ReadGyro();
  
  if (counter >= 10) {
    counter = 0;
    SerialCom->print(F("[GYRO] Angle: "));
    SerialCom->print(currentAngle, 2);
    SerialCom->print(F(" degrees, Rate: "));
    SerialCom->print(gyroRate, 4);
    SerialCom->println(F(" V"));
  }
}

void test_fire_detection() {
  static int counter = 0;
  counter++;
  
  if (counter >= 10) {
    counter = 0;
    float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
    float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);
    
    SerialCom->println(F("=== FIRE DETECTION ==="));
    SerialCom->print(F("Left Photo: "));
    SerialCom->print(leftVoltage, 3);
    SerialCom->print(F("V "));
    SerialCom->println(leftVoltage > PHOTOTRANSISTOR_THRESHOLD ? F("(FIRE!)") : F("(no fire)"));
    
    SerialCom->print(F("Right Photo: "));
    SerialCom->print(rightVoltage, 3);
    SerialCom->print(F("V "));
    SerialCom->println(rightVoltage > PHOTOTRANSISTOR_THRESHOLD ? F("(FIRE!)") : F("(no fire)"));
    print_separator();
  }
}

void test_battery() {
  static int counter = 0;
  counter++;
  
  if (counter >= 20) { // Check every second
    counter = 0;
    boolean battery_ok = is_battery_voltage_OK();
    SerialCom->print(F("[BATTERY] Status: "));
    SerialCom->println(battery_ok ? F("OK") : F("LOW"));
  }
}

void print_test_menu() {
  SerialCom->println(F("\n=== TEST MODE SELECTION ==="));
  SerialCom->println(F("Send one of these commands:"));
  SerialCom->println(F("'a' - Test ALL sensors"));
  SerialCom->println(F("'u' - Test ULTRASONIC only"));
  SerialCom->println(F("'i' - Test IR sensors only"));
  SerialCom->println(F("'g' - Test GYRO only"));
  SerialCom->println(F("'f' - Test FIRE detection only"));
  SerialCom->println(F("'b' - Test BATTERY only"));
  SerialCom->println(F("'m' - Show this menu"));
  print_separator();
}

void handle_serial_input() {
  if (SerialCom->available() > 0) {
    char command = SerialCom->read();
    
    switch (command) {
      case 'a':
      case 'A':
        current_test_mode = ALL_SENSORS;
        SerialCom->println(F("Mode: Testing ALL sensors"));
        break;
      case 'u':
      case 'U':
        current_test_mode = ULTRASONIC_ONLY;
        SerialCom->println(F("Mode: Testing ULTRASONIC only"));
        break;
      case 'i':
      case 'I':
        current_test_mode = IR_SENSORS_ONLY;
        SerialCom->println(F("Mode: Testing IR sensors only"));
        break;
      case 'g':
      case 'G':
        current_test_mode = GYRO_ONLY;
        SerialCom->println(F("Mode: Testing GYRO only"));
        break;
      case 'f':
      case 'F':
        current_test_mode = FIRE_DETECTION_ONLY;
        SerialCom->println(F("Mode: Testing FIRE detection only"));
        break;
      case 'b':
      case 'B':
        current_test_mode = BATTERY_ONLY;
        SerialCom->println(F("Mode: Testing BATTERY only"));
        break;
      case 'm':
      case 'M':
        print_test_menu();
        break;
    }
  }
}

// ========== SENSOR READING FUNCTIONS ==========

void ReadUltrasonic() {
  unsigned long t1, t2, pulse_width;
  
  // Trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Wait for echo start
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println(F("HC-SR04: NOT found"));
      return;
    }
  }
  
  // Measure echo duration
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println(F("HC-SR04: Out of range"));
      return;
    }
  }
  
  t2 = micros();
  pulse_width = t2 - t1;
  
  // Calculate distance and apply Kalman filter
  double distance = pulse_width / 58.0;
  double est = Kalman_ultrasonic(distance, front_distance);
  front_distance = est;
}

void ReadRightFront() {
  rightIrSensorValue = analogRead(rightFrontIrPin);
  float voltage = rightIrSensorValue * (5.0 / 1023.0);
  rightIrDistance = (25.5 * pow(voltage, -1.10));
  
  double est = Kalman_ir_right(rightIrDistance, frontRightDist);
  frontRightDist = est;
}

void ReadLeftFront() {
  leftIrSensorValue = analogRead(leftFrontIrPin);
  float voltage = leftIrSensorValue * (5.0 / 1023.0);
  leftIrDistance = (25.5 * pow(voltage, -1.10));
  
  double est = Kalman_ir_left(leftIrDistance, frontLeftDist);
  frontLeftDist = est;
}

void ReadRight() {
  backIrSensorValue = analogRead(rightIrPin);
  float voltage = backIrSensorValue * (5.0 / 1023.0);
  
  if (voltage < 0.1) {
    backIrDistance = 100.0;
  } else if (voltage > 4.5) {
    backIrDistance = 5.0;
  } else {
    backIrDistance = 11.35795124 * pow(voltage, -0.7897);
    if (backIrDistance > 100.0) backIrDistance = 100.0;
    if (backIrDistance < 5.0) backIrDistance = 5.0;
  }
  
  if (isnan(rightDist) || isnan(last_var_ir_back)) {
    rightDist = backIrDistance;
    last_var_ir_back = 1.0;
  }
  
  double est = Kalman_ir_back(backIrDistance, rightDist);
  if (!isnan(est)) {
    rightDist = est;
  } else {
    rightDist = backIrDistance;
    last_var_ir_back = 1.0;
  }
}

void ReadLeft() {
  shortleftIrSensorValue = analogRead(leftIrPin);
  float voltage = shortleftIrSensorValue * (5.0 / 1023.0);
  
  if (voltage < 0.1) {
    shortleftIrDistance = leftDist;
  } else {
    shortleftIrDistance = 11.35795124 * pow(voltage, -0.7897);
  }
  
  if (!isnan(shortleftIrDistance)) {
    double est = Kalman_ir_short_left(shortleftIrDistance, leftDist);
    leftDist = est - 1.25;
  }
}

void ReadGyro() {
  gyroRate = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023;
  gyroRate -= (gyroZeroVoltage / 1023 * 5);
  
  float angularVelocity = gyroRate / gyroSensitivity;
  
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }
  
  // Keep angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }
}

void calibrateGyro() {
  float sum = 0;
  pinMode(gyroPin, INPUT);
  
  for (int i = 0; i < 100; i++) {
    gyroSensorValue = analogRead(gyroPin);
    sum += gyroSensorValue;
    delay(5);
  }
  
  gyroZeroVoltage = sum / 100;
}

float readPhotoTransistor(int analog_pin) {
  return analogRead(analog_pin) * 5.0 / 1024.0;
}

boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;
  
  int Lipo_level_cal;
  int raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;
  
  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println(F("Lipo is Disconnected or Power Switch is OFF!"));
    else if (Lipo_level_cal > 160)
      SerialCom->println(F("Lipo is Overcharged!"));
    else {
      SerialCom->println(F("Lipo voltage too LOW"));
      SerialCom->print(F("Battery level: "));
      SerialCom->print(Lipo_level_cal);
      SerialCom->println(F("%"));
    }
    
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}

// ========== KALMAN FILTER FUNCTIONS ==========

double Kalman_ultrasonic(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;
  
  a_priori_est = prev_est;
  a_priori_var = last_var_ultrasonic + process_noise_ultrasonic;
  
  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ultrasonic);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ultrasonic = a_post_var;
  
  return a_post_est;
}

double Kalman_ir_right(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;
  
  a_priori_est = prev_est;
  a_priori_var = last_var_ir_right + process_noise_ir_right;
  
  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_right);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_right = a_post_var;
  
  return a_post_est;
}

double Kalman_ir_left(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;
  
  a_priori_est = prev_est;
  a_priori_var = last_var_ir_left + process_noise_ir_left;
  
  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_left);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_left = a_post_var;
  
  return a_post_est;
}

double Kalman_ir_back(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;
  
  a_priori_est = prev_est;
  a_priori_var = last_var_ir_back + process_noise_ir_back;
  
  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_back);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_back = a_post_var;
  
  return a_post_est;
}

double Kalman_ir_short_left(double rawdata, double prev_est) {
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;
  
  a_priori_est = prev_est;
  a_priori_var = last_var_ir_short_left + process_noise_ir_short_left;
  
  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_short_left);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_short_left = a_post_var;
  
  return a_post_est;
}

// ========== UTILITY FUNCTIONS ==========

void delaySeconds(int seconds) {
  for (int i = 0; i < seconds; i++) {
    delay(1000);
  }
}

void print_separator() {
  SerialCom->println(F("----------------------------------------"));
}

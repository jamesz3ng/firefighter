#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

// Investment thing
// https://kernelwealth.co.nz/funds/kernel-high-growth-fund
// schneider opens July

enum STATE {
  INITIALISING,
  DETECT_FIRE,
  NAVIGATE,
  EXTINGUISH,
  CHECK_GYRO,
  STOPPED,
  LEFT,
  RIGHT
};

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
#define STARTUP_DELAY 3  // Seconds
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

#define SERVO_PIN 37
#define LEFT_PHOTOTRANSISTOR A9
#define RIGHT_PHOTOTRANSISTOR A10
#define PHOTOTRANSISTOR_THRESHOLD 0.10
// Serial Pointer

// HardwareSerial *SerialCom;

SoftwareSerial *SerialCom;

// Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 51;
const byte right_front = 50;
// Motor
Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
int speed_val = 200;

// Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;
double front_distance = 0;
double last_var_ultrasonic = 1;
double process_noise_ultrasonic = 1;
double sensor_noise_ultrasonic = 1;  // Change the value of sensor noise to get different KF performance

// Gyro
int gyroPin = A14;              // define the pin that gyro is connected
int T = 100;                    // T is the time of one loop
int gyroSensorValue = 0;        // read out value of sensor
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;      // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity less than this value will not be ignored
float gyroRate = 0;             // read out value of sensor in voltage
float currentAngle = 0;         // current angle calculated by angular velocity integral on

// IR
const int rightFrontIrPin = A1;  // Analog input pin for the sensor's output
int rightIrSensorValue = 0;       // Variable to store the sensor reading
float rightIrDistance = 0.0;      // Variable to store the calculated distance
double frontRightDist = 0;
double last_var_ir_right = 1;
double process_noise_ir_right = 1;
double sensor_noise_ir_right = 1;  // Change the value of sensor noise to get different KF performance

const int leftFrontIrPin = A0;  // Analog input pin for the sensor's output
int leftIrSensorValue = 0;      // Variable to store the sensor reading
float leftIrDistance = 0.0;     // Variable to store the calculated distance
double frontLeftDist = 0;
double last_var_ir_left = 1;
double process_noise_ir_left = 1;
double sensor_noise_ir_left = 1;  // Change the value of sensor noise to get different KF performance

const int rightIrPin = A3;   // Analog input pin for the sensor's output
int backIrSensorValue = 0;   // Variable to store the sensor reading
float backIrDistance = 0.0;  // Variable to store the calculated distance
double rightDist = 0;
double last_var_ir_back = 1;
double process_noise_ir_back = 1;
double sensor_noise_ir_back = 1;  // Change the value of sensor noise to get different KF performance
float target_dist = 0;

const int leftIrPin = A2;        // Analog input pin for the sensor's output
int shortleftIrSensorValue = 0;   // Variable to store the sensor reading
float shortleftIrDistance = 0.0;  // Variable to store the calculated distance
double leftDist = 0;
double last_var_ir_short_left = 1;
double process_noise_ir_short_left = 1;
double sensor_noise_ir_short_left = 1;

int current_servo_angle = 45;

Servo myservo;
// Function prototypes
void calibrateGyro();
void delaySeconds(int TimedDelaySeconds);
void ReadGyro();
void ReadUltrasonic();
void ReadRightFront();
void ReadLeftFront();
void ReadRight();
void ReadLeft();
void enable_motors();
void disable_motors();
void stop();
void forward();
void reverse();
void ccw();
void cw();
void strafe_left();
void strafe_right();
boolean is_battery_voltage_OK();
double Kalman_ultrasonic(double rawdata, double prev_est);
double Kalman_ir_right(double rawdata, double prev_est);
double Kalman_ir_left(double rawdata, double prev_est);
double Kalman_ir_back(double rawdata, double prev_est);
double Kalman_ir_short_left(double rawdata, double prev_est);
STATE initialising();
STATE stopped();
STATE navigate();
STATE extinguish();
STATE check_gyro();

void setup(void) {

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &BluetoothSerial;
  // SerialCom = &Serial;

  Serial.begin(115200);
  // SerialCom->begin(115200);
  delaySeconds(STARTUP_DELAY);
  // SerialCom->println("MECHENG706");
  // SerialCom->println("Setup....");
  calibrateGyro();

  delay(50);
}

void loop(void) {

  static STATE machine_state = INITIALISING;
  // Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      // SerialCom->println("INITIALISING....");
      machine_state = initialising();
      break;
    case DETECT_FIRE:
      machine_state = detect_fire();

      break;
    case STOPPED:
      // SerialCom->println("STOPPED....");
      machine_state = stopped();
      break;
    case EXTINGUISH:
      machine_state = extinguish();
      break;
    case NAVIGATE:
      machine_state = navigate();
      break;
    case LEFT:
      machine_state = left();
      break;
    case RIGHT:
      machine_state = right();
      break;
    case CHECK_GYRO:
      machine_state = check_gyro();
  };
}

//-------------------------------States---------------------------------

STATE initialising() {
  SerialCom->println("Enabling Motors...");
  enable_motors();

  for (int i = 0; i < 10; i++) {
    ReadUltrasonic();
    delay(100);
  }
  // return CHECK_GYRO;  // Changed from RIGHT to SEARCH_WALL
  return NAVIGATE;
}

STATE detect_fire() {
  float left_photo_reading;
  float right_photo_reading;


  // TODO: look around until fire is found

  // TODO: rotate to face the fire

  // TODO: when facing the fire, return NAVIGATE
  
  static unsigned long previous_millis;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    if (!is_battery_voltage_OK())
      return STOPPED;

    ReadLeftFront();
    ReadRightFront();
    ReadLeft();
    ReadUltrasonic();

    left_photo_reading = getVoltage(LEFT_PHOTOTRANSISTOR);
    right_photo_reading = getVoltage(RIGHT_PHOTOTRANSISTOR);
    if (left_photo_reading > right_photo_reading) {
      current_servo_angle += 3;
      myservo.write(current_servo_angle);
    } else if (right_photo_reading > left_photo_reading) {
      current_servo_angle -= 3;
      myservo.write(current_servo_angle);
    }
    Serial.print("Left Reading  ");
    Serial.println(left_photo_reading);

    Serial.print("Right Reading  ");
    Serial.println(right_photo_reading);
  }
  return LEFT;
}

STATE navigate() {
  static unsigned long previous_millis;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    if (!is_battery_voltage_OK())
      return STOPPED;

    ReadUltrasonic();
    ReadLeftFront();
    ReadRightFront();
    Serial.println("in navigate.");
    Serial.print("ultrasonic distance  ");
    Serial.println(front_distance);

    if (frontLeftDist < 20) {
      stop();
      return RIGHT;
    } else if (frontRightDist < 20) {
      Serial.println("left IR hit");
      stop();
      return LEFT;
    } else if (front_distance < 20) {
      Serial.println("ultrasonic hit");
      stop();
      return LEFT;
    } else {
      Serial.println("hello");
      forward();
    }
  }
  return NAVIGATE;
  // TODO: find fire
}

STATE left() {
  static unsigned long previous_millis;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    // if (!is_battery_voltage_OK())
    //   return STOPPED;

    ReadLeftFront();
    ReadRightFront();
    ReadLeft();
    ReadUltrasonic();
    Serial.print("ultrasonic distance  ");
    Serial.println(front_distance);

    if (leftDist < 15) {
      stop();
      return RIGHT;
    }

    if ((frontLeftDist > 20) && (frontRightDist > 20) && (front_distance > 20)) {
    stop();
      return NAVIGATE;
    } else {
      strafe_left();
    }
  }
  return LEFT;
}

STATE right() {
  static unsigned long previous_millis;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    if (!is_battery_voltage_OK())
      return STOPPED;

    ReadLeftFront();
    ReadRightFront();
    ReadRight();
    ReadUltrasonic();
    Serial.print("ultrasonic distance  ");
    Serial.println(front_distance);

    // if (rightDist < 15) {
    //   return LEFT;
    // }
    if ((frontLeftDist > 20) && (frontRightDist > 20) && (front_distance > 20)) {
      return NAVIGATE;
    } else {
      strafe_right();
    }
  }
  return RIGHT;
}

STATE extinguish() {
  // TODO: turn the fan on for 10 seconds

  // TODO: turn the fan off if fire is extinguished before 10 seconds

  // TODO: find another fire, or stop the robot if two fire have been extinguished.
  return EXTINGUISH;
}

//----------------------STOPPED STATE------------------------
STATE stopped() {
  disable_motors();
  return STOPPED;
}

STATE check_gyro() {
  static unsigned long previous_millis;
  static int counter = 0;

  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    if (!is_battery_voltage_OK())
      return STOPPED;

    ReadGyro();

    counter++;

    if (counter > 10) {
      // SerialCom->println(currentAngle);
    }
  }
  return CHECK_GYRO;
}

//-------------------------------Helper Functions---------------------------------

//----------------------Wireless------------------------
void delaySeconds(int TimedDelaySeconds) {
  for (int i = 0; i < TimedDelaySeconds; i++) {
    delay(1000);
  }
}

float getVoltage(int analog_pin) {
  float volts = analogRead(analog_pin) * 5.0 / 1024.0;
  return volts;
}

//----------------------Kalman Filter------------------------
double Kalman_ultrasonic(double rawdata, double prev_est) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ultrasonic + process_noise_ultrasonic;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ultrasonic);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ultrasonic = a_post_var;
  return a_post_est;
}

double Kalman_ir_right(double rawdata, double prev_est) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_right + process_noise_ir_right;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_right);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_right = a_post_var;
  return a_post_est;
}

double Kalman_ir_left(double rawdata, double prev_est) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_left + process_noise_ir_left;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_left);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_left = a_post_var;
  return a_post_est;
}

double Kalman_ir_back(double rawdata, double prev_est) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_back + process_noise_ir_back;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_back);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_back = a_post_var;
  return a_post_est;
}

//----------------------Battery------------------------
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  // to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  // Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}

//----------------------Ultrasonic------------------------
void ReadUltrasonic() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  double distance = pulse_width / 58.0;

  // medianFilter2.AddValue(distance);
  double est = Kalman_ultrasonic(distance, front_distance);
  front_distance = est;
}

//----------------------IR------------------------
void ReadRightFront() {
  rightIrSensorValue = analogRead(rightFrontIrPin);

  // Convert the sensor value to a voltage (assuming 5V reference voltage)
  float voltage = rightIrSensorValue * (5.0 / 1023.0);

  // Calculate the distance using the voltage (for Sharp GP2Y0A21YK0F, this is a typical formula)
  // Note: this is a rough estimate based on sensor's datasheet.
  rightIrDistance = (25.5 * pow(voltage, -1.10));

  double est = Kalman_ir_right(rightIrDistance, frontRightDist);
  frontRightDist = est;
}

void ReadLeftFront() {
  leftIrSensorValue = analogRead(leftFrontIrPin);

  // Convert the sensor value to a voltage (assuming 5V reference voltage)
  float voltage = leftIrSensorValue * (5.0 / 1023.0);

  // Calculate the distance using the voltage (for Sharp GP2Y0A21YK0F, this is a typical formula)
  // Note: this is a rough estimate based on sensor's datasheet.
  leftIrDistance = (25.5 * pow(voltage, -1.10));

  double est = Kalman_ir_left(leftIrDistance, frontLeftDist);
  frontLeftDist = est;
}

void ReadRight() {
  backIrSensorValue = analogRead(rightIrPin);

  // Convert the sensor value to a voltage (assuming 5V reference voltage)
  float voltage = backIrSensorValue * (5.0 / 1023.0);

  // Calculate the distance using the voltage (for Sharp GP2Y0A21YK0F, this is a typical formula)
  // Note: this is a rough estimate based on sensor's datasheet.
  backIrDistance = 11.35795124 * pow(voltage, -0.7897);

  double est = Kalman_ir_back(backIrDistance, rightDist);
  rightDist = est;
}

void ReadLeft() {
  // 1. Read ADC value
  shortleftIrSensorValue = analogRead(leftIrPin);

  // 2. Convert to voltage
  float voltage = shortleftIrSensorValue * (5.0 / 1023.0);

  // 3. Compute raw distance OR fallback if voltage is too low
  if (voltage < 0.1) {
    shortleftIrDistance = leftDist;  // fallback to last estimate
  } else {
    shortleftIrDistance = 11.35795124 * pow(voltage, -0.7897);  // normal calculation
  }

  // 4. Kalman filter only if value is valid
  if (!isnan(shortleftIrDistance)) {
    double est = Kalman_ir_left(shortleftIrDistance, leftDist);
    leftDist = est - 1.25;
  }
}

double Kalman_ir_short_left(double rawdata, double prev_est) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var_ir_short_left + process_noise_ir_short_left;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise_ir_short_left);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var_ir_short_left = a_post_var;
  return a_post_est;
}

// void Short_Left_IR_range2() {
//   shortleftIrSensorValue = analogRead(shortleftIrPin);

//   // Convert the sensor value to a voltage (assuming 5V reference voltage)
//   float voltage = shortleftIrSensorValue * (5.0 / 1023.0);

//   // Calculate the distance using the voltage (for Sharp GP2Y0A21YK0F, this is a typical formula)
//   // Note: this is a rough estimate based on sensor's datasheet.
//   shortleftIrDistance = (25.5 * pow(voltage, -1.10));

//   double est = Kalman_ir_short_left(shortleftIrDistance, sideLeftDist);
//   sideLeftDist = est;
// }

//----------------------Gyro------------------------
void calibrateGyro() {
  int i;
  float sum = 0;
  pinMode(gyroPin, INPUT);
  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    gyroSensorValue = analogRead(gyroPin);
    sum += gyroSensorValue;
    // SerialCom->println(gyroSensorValue);
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
}

void ReadGyro() {
  gyroRate = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023;
  // SerialCom->println(gyroRate);
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5);
  // SerialCom->println(gyroZeroVoltage*5/1023);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;
  //  SerialCom->println(angularVelocity);
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }
  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }
  // SerialCom->println(currentAngle);
}

//----------------------Motor------------------------
void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

void stop() {
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  SerialCom->println("MECHENG706");
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

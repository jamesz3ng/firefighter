#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

// Investment thing
// https://kernelwealth.co.nz/funds/kernel-high-growth-fund
// schneider opens July

enum STATE {
  INITIALISING,
  DETECT_FIRE,
  DRIVE_TO_FIRE,
  AVOID,
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

// Fire tracking servo parameters
const float ACTIVATION_THRESHOLD = 0.5;  // Minimum voltage difference to react
const int START_ANGLE = 90;              // Initial servo position
const int ANGLE_INCREMENT = 1;           // Degree change per adjustment
const int SERVO_MIN = 0;                 // Minimum servo angle
const int SERVO_MAX = 180;               // Maximum servo angle
const float MIN_FIRE_VOLTAGE = 0.25;     // Minimum voltage to consider as fire

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

int current_servo_angle = 90;  // Changed initial value to match START_ANGLE
Servo myservo;  // Fire tracking servo

// Add these global variables for fire tracking
bool fire_detected = false;
int fire_servo_angle = 90;  // Track the angle where fire is centered

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
STATE Avoid();
STATE extinguish();
STATE check_gyro();
STATE detect_fire();
STATE left();
STATE right();

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

  // Initialize fire tracking servo
  myservo.attach(SERVO_PIN);
  myservo.write(START_ANGLE);
  current_servo_angle = START_ANGLE;

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
    case AVOID:
      machine_state = Avoid();
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
  return DETECT_FIRE;  // Start by looking for fire
}
STATE detect_fire() {
  static unsigned long previous_millis;
  static unsigned int counter = 0;

  if (millis() - previous_millis > T) {
    previous_millis = millis();
    counter++;
    if (counter > 10) {
      float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
      float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);

      cw();

      if (leftVoltage > 0.25 && rightVoltage > 0.25){
        if (abs(leftVoltage - rightVoltage) < 0.5){
          stop();
          counter = 0;
          return DRIVE_TO_FIRE;
        }
      }
    }
  }
  return DETECT_FIRE;
}

STATE drive_to_fire() {
  static unsigned long previous_millis;
  static int currentAngle = START_ANGLE;  // Preserve angle between iterations
  static unsigned int counter = 0;
  
  if (millis() - previous_millis > T) {
    previous_millis = millis();
    counter++;
    // Read light sensors
    float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
    float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);
    float difference = leftVoltage - rightVoltage;

    readUltrasonic();

    if (counter > 10) {
      // Only adjust if difference exceeds threshold
      if (abs(difference) > ACTIVATION_THRESHOLD) {
        if (difference > 0) {
          currentAngle = constrain(currentAngle + ANGLE_INCREMENT, SERVO_MIN, SERVO_MAX);
        } else {
          currentAngle = constrain(currentAngle - ANGLE_INCREMENT, SERVO_MIN, SERVO_MAX);
        }
        trackerServo.write(currentAngle);
      }

      if (front_distance > 10) {
        float correction_kp = 5.0;
        float error = currentAngle-90;

        float correction_factor = correction_kp * error;
        left_font_motor.writeMicroseconds(constrain(1500 + speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
        left_rear_motor.writeMicroseconds(constrain(1500 + speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
        right_font_motor.writeMicroseconds(constrain(1500 - speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
        right_rear_motor.writeMicroseconds(constrain(1500 - speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
      } else {
        stop();
        counter = 0;
        return STOPPED;
      }  
    }
  }
  return DRIVE_TO_FIRE;
}


STATE Avoid() {
  static unsigned long previous_millis;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    if (!is_battery_voltage_OK())
      return STOPPED;

    ReadUltrasonic();
    ReadLeftFront();
    ReadRightFront();
    
    // If fire is detected, also track it with servo while navigating
    if (fire_detected) {
      float left_photo = getVoltage(LEFT_PHOTOTRANSISTOR);
      float right_photo = getVoltage(RIGHT_PHOTOTRANSISTOR);
      float diff = left_photo - right_photo;
      
      // Continue tracking fire with servo
      if (abs(diff) > ACTIVATION_THRESHOLD) {
        if (diff > 0) {
          current_servo_angle = constrain(current_servo_angle + ANGLE_INCREMENT, SERVO_MIN, SERVO_MAX);
        } else {
          current_servo_angle = constrain(current_servo_angle - ANGLE_INCREMENT, SERVO_MIN, SERVO_MAX);
        }
        myservo.write(current_servo_angle);
      }
      
      // Check if we're close enough to extinguish
      if (front_distance < 10 && front_distance > 0) {
        Serial.println("Close to fire! Ready to extinguish.");
        stop();
        return EXTINGUISH;
      }
    }
    
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
      Serial.println("moving forward");
      forward();
    }
  }
  return AVOID;
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
      return AVOID;
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
      return AVOID;
    } else {
      strafe_right();
    }
  }
  return RIGHT;
}

STATE extinguish() {
  // TODO: turn the fan on for 10 seconds
  static unsigned long extinguish_start_time = 0;
  static bool extinguishing = false;
  static int fires_extinguished = 0;
  
  if (!extinguishing) {
    extinguish_start_time = millis();
    extinguishing = true;
    Serial.println("Starting fire extinguishment...");
    // TODO: Turn on fan here
    // digitalWrite(FAN_PIN, HIGH);
  }
  
  // Check if 10 seconds have passed or fire is out
  if (millis() - extinguish_start_time > 10000) {  // 10 seconds
    extinguishing = false;
    fires_extinguished++;
    fire_detected = false;  // Reset fire detection
    current_servo_angle = START_ANGLE;  // Reset servo
    myservo.write(current_servo_angle);
    Serial.println("Fire extinguished!");
    // TODO: Turn off fan here
    // digitalWrite(FAN_PIN, LOW);
    
    if (fires_extinguished >= 2) {
      Serial.println("Two fires extinguished. Mission complete!");
      return STOPPED;
    } else {
      Serial.println("Looking for next fire...");
      return DETECT_FIRE;  // Look for another fire
    }
  }
  
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
    double est = Kalman_ir_short_left(shortleftIrDistance, leftDist);
    leftDist = est - 1.25;
  }
}

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
  // Apply correction based on servo angle when tracking fire
  if (fire_detected) {
    float correction_kp = 2.0;
    float error = current_servo_angle - 90;  // Error from center position
    float correction_factor = correction_kp * error;
    
    left_font_motor.writeMicroseconds(constrain(1500 + speed_val - correction_factor, 1000, 2000));
    left_rear_motor.writeMicroseconds(constrain(1500 + speed_val - correction_factor, 1000, 2000));
    right_font_motor.writeMicroseconds(constrain(1500 - speed_val - correction_factor, 1000, 2000));
    right_rear_motor.writeMicroseconds(constrain(1500 - speed_val - correction_factor, 1000, 2000));
  } else {
    // Normal forward motion
    left_font_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_font_motor.writeMicroseconds(1500 - speed_val);
  }
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
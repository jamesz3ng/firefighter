#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

// Investment thing
// https://kernelwealth.co.nz/funds/kernel-high-growth-fund
// schneider opens July

enum STATE {
  INITIALISING,
  DETECT_FIRE,
  AVOID,
  EXTINGUISH,
  CHECK_GYRO,
  STOPPED,
  LEFT,
  RIGHT,
  DRIVE_TO_FIRE
};

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

#define SERVO_PIN 37
#define LEFT_PHOTOTRANSISTOR A9
#define RIGHT_PHOTOTRANSISTOR A10
#define PHOTOTRANSISTOR_THRESHOLD 0.10
// Serial Pointer

// HardwareSerial *SerialCom;

SoftwareSerial *SerialCom;

const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;

// System parameters

const float ACTIVATION_THRESHOLD = 0.5;  // Minimum voltage difference to react
const int START_ANGLE = 90;              // Initial servo position
const int ANGLE_INCREMENT = 1;           // Degree change per adjustment
const int SERVO_MIN = 0;                 // Minimum servo angle
const int SERVO_MAX = 180;               // Maximum servo angle



Servo trackerServo;  // Servo controller object



int T = 50;

double process_noise_ultrasonic = 0.5;

double sensor_noise_ultrasonic = 3.0;  // Change the value of sensor noise to get different KF performance

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield

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


// Gyro
int gyroPin = A14;              // define the pin that gyro is connected                  // T is the time of one loop
int gyroSensorValue = 0;        // read out value of sensor
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;      // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity less than this value will not be ignored
float gyroRate = 0;             // read out value of sensor in voltage
float currentAngle = 0;         // current angle calculated by angular velocity integral on

// IR
const int rightFrontIrPin = A1;  // Analog input pin for the sensor's output
int rightIrSensorValue = 0;      // Variable to store the sensor reading
float rightIrDistance = 0.0;     // Variable to store the calculated distance
double frontRightDist = 0;
double last_var_ir_right = 1;
double process_noise_ir_right = 1;
double sensor_noise_ir_right = 1;  // Change the value of sensor noise to get different KF performance

const int leftFrontIrPin = A4;  // Analog input pin for the sensor's output
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

const int leftIrPin = A2;         // Analog input pin for the sensor's output
int shortleftIrSensorValue = 0;   // Variable to store the sensor reading
float shortleftIrDistance = 0.0;  // Variable to store the calculated distance
double leftDist = 0;
double last_var_ir_short_left = 1;
double process_noise_ir_short_left = 1;
double sensor_noise_ir_short_left = 1;

// Objective related parametres
const int fan_pin = 53;
int fire_extinguished = 0;

int current_servo_angle = 45;
bool rotate_cw = true;  // for detecting fire

Servo myservo;
// Function prototypes
STATE left();
STATE right();
STATE detect_fire();
float readPhotoTransistor(int analog_pin);
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

void setup(void) {

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &BluetoothSerial;
  // SerialCom = &Serial;

  // Serial.begin(9600);
  SerialCom->begin(115200);
  // SerialCom->begin(115200);
  trackerServo.attach(SERVO_PIN);
  trackerServo.write(START_ANGLE);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // SerialCom->println("MECHENG706");
  // SerialCom->println("Setup....");
  calibrateGyro();

  // Initialise fan pin
  pinMode(fan_pin, OUTPUT);

  // Initialize distance variables to prevent NaN
  rightDist = 30.0;  // reasonable default
  frontLeftDist = 30.0;
  frontRightDist = 30.0;
  leftDist = 30.0;
  front_distance = 30.0;

  // Initialize Kalman variances
  last_var_ir_back = 1.0;
  last_var_ir_left = 1.0;
  last_var_ir_right = 1.0;
  last_var_ir_short_left = 1.0;
  last_var_ultrasonic = 1.0;

  delay(50);


  // while (1){
  //   float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
  //   float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);
  //   Serial.print("Left Voltage ");
  //   Serial.println(leftVoltage);
  //   Serial.print("Right Voltage ");
  //   Serial.println(rightVoltage);
  //   delay(1000);
  // }
}

void loop(void) {
  // float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
  // float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);
  // SerialCom->print("leftVoltage  ");
  // SerialCom->print(leftVoltage);
  // SerialCom->print("rightVoltage  ");
  // SerialCom->println(rightVoltage);
  ReadUltrasonic();
  // SerialCom->println(front_distance);
  ReadLeftFront();
  ReadRightFront();
  ReadLeft();
  ReadRight();

  // delay(100);
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
    case DRIVE_TO_FIRE:
      machine_state = drive_to_fire();
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
    ReadLeft();
    ReadRight();

    ReadUltrasonic();
    delay(100);
  }
  // return CHECK_GYRO;  // Changed from RIGHT to SEARCH_WALL
  return DETECT_FIRE;
}

STATE detect_fire() {
  static unsigned long previous_millis;
  static unsigned int counter = 0;

  if (millis() - previous_millis > T) {
    if (counter < 10) {
      SerialCom->println("Detecting fire...");
    }
    previous_millis = millis();
    counter++;
    if (counter > 10) {
      float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
      float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);
      speed_val = 150;
      rotate_cw ? cw() : ccw();
      // cw();
      if (leftVoltage > 0.20 && rightVoltage > 0.20) {
        if (abs(leftVoltage - rightVoltage) < 0.3) {
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

    ReadLeftFront();
    ReadRightFront();
    ReadUltrasonic();
    if ((frontLeftDist < 10 || frontRightDist < 10 || front_distance < 10) && (leftVoltage < 1.0 && rightVoltage < 1.0)) {
      SerialCom->print("leftVoltage  ");
      SerialCom->print(leftVoltage);
      SerialCom->print("rightVoltage  ");
      SerialCom->println(rightVoltage);
      return AVOID;
    }

    if (counter > 10) {
      // Only adjust if difference exceeds threshold
      if (difference > 0) {
        currentAngle = constrain(currentAngle + ANGLE_INCREMENT, SERVO_MIN, SERVO_MAX);
      } else {
        currentAngle = constrain(currentAngle - ANGLE_INCREMENT, SERVO_MIN, SERVO_MAX);
      }
      trackerServo.write(currentAngle);

      if (front_distance > 10) {
        float correction_kp = 5.0;
        float error = currentAngle - 90;

        float correction_factor = correction_kp * error;
        speed_val = 150;
        left_font_motor.writeMicroseconds(constrain(1500 + speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
        left_rear_motor.writeMicroseconds(constrain(1500 + speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
        right_font_motor.writeMicroseconds(constrain(1500 - speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));
        right_rear_motor.writeMicroseconds(constrain(1500 - speed_val - correction_factor, MOTOR_MIN, MOTOR_MAX));

      } else {
        stop();
        counter = 0;
        SerialCom -> println("Activate Extinguish");
        return EXTINGUISH;
      }
    }
  }
  return DRIVE_TO_FIRE;
}

STATE Avoid() {
  static unsigned long previous_millis;
  static unsigned long front_count = 0;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    // if (!is_battery_voltage_OK())
    // return STOPPED;

    ReadUltrasonic();
    ReadLeftFront();
    ReadRightFront();
    SerialCom->println("in navigate.");
    // SerialCom->print("ultrasonic distance  ");
    // SerialCom->print(front_distance);
    // SerialCom->print("  frontRightDist  ");
    // SerialCom->print(frontRightDist);
    // SerialCom->print("   frontLeftDist  ");
    // SerialCom->println(frontLeftDist);

    if (frontLeftDist < 10) {
      stop();
      SerialCom->println("front left hit");
      return RIGHT;
    } else if (frontRightDist < 10) {
      SerialCom->println("right IR hit");
      stop();
      return LEFT;
    } else if (front_distance < 10) {
      SerialCom->println("ultrasonic hit");
      stop();
      return LEFT;
    } else {
      SerialCom->println("nothing within 10cm");
      front_count++;
      forward();

      if (front_count > 10) {
        // after avoid the obstacle spin again face the fire
        // logic here to rotate in the correct direction
        SerialCom->println("detect fire activate");
        stop();
        front_count = 0;
        SerialCom->println("Returning to fire detection");
        return DETECT_FIRE;
      }
    }
    return AVOID;
    // TODO: find fire
  }
}

STATE left() {
  static unsigned long previous_millis;
  static int counting_time = 0;
  rotate_cw = true;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    // if (!is_battery_voltage_OK())
    //   return STOPPED;

    ReadLeftFront();
    ReadRightFront();
    ReadLeft();
    ReadUltrasonic();
    SerialCom->print("In Left  ");
    // SerialCom->print("ultrasonic distance  ");
    // SerialCom->print(front_distance);
    // SerialCom->print("  frontRightDist  ");
    // SerialCom->print(frontRightDist);
    // SerialCom->print("   frontLeftDist  ");
    // SerialCom->print(frontLeftDist);
    // SerialCom->print("  leftDist  ");
    // SerialCom->println(leftDist);
    if (leftDist < 15) {
      stop();
      SerialCom->println("left side hit");
      counting_time = 0;
      return RIGHT;
    }

    if ((frontLeftDist > 20) && (frontRightDist > 20) && (front_distance > 20)) {

      counting_time++;
      if (counting_time > 10) {
        counting_time = 0;
        stop();
        return AVOID;
      }
    } else {
      strafe_left();
    }
  }
  return LEFT;
}

STATE right() {
  static unsigned long previous_millis;
  static unsigned long right_counter = 0;
  rotate_cw = false;
  if (millis() - previous_millis > T) {  // Arduino style 100ms timed execution statement
    previous_millis = millis();

    if (!is_battery_voltage_OK()) {
      SerialCom->println("battery stop");
      // return STOPPED;
    }
    // SerialCom->print("rightDist ");
    // SerialCom->println(rightDist);
    if (rightDist < 15) {
      stop();
      SerialCom->println("right hit ");
      right_counter = 0;
      return LEFT;
    }


    ReadLeftFront();
    ReadRightFront();
    ReadRight();
    ReadUltrasonic();
    SerialCom->print("In Right  ");
    // SerialCom->print("ultrasonic distance  ");
    // SerialCom->print(front_distance);
    // SerialCom->print("  frontRightDist  ");
    // SerialCom->print(frontRightDist);
    // SerialCom->print("   frontLeftDist  ");
    // SerialCom->println(frontLeftDist);

    // if (rightDist < 15) {
    //   return LEFT;
    // }
    if ((frontLeftDist > 20) && (frontRightDist > 20) && (front_distance > 20)) {
      right_counter++;
      if (right_counter > 10) {

        SerialCom->println("Activate Navigate");
        right_counter = 0;
        return AVOID;
      }
    } else {
      strafe_right();
    }
  }
  // SerialCom->print(" return right");
  return RIGHT;
}

STATE extinguish() {
  unsigned long current_time = millis();
  static unsigned long start_time = 0;
  static bool extinguished = false;
  static bool timer_started = false;

  if (!timer_started) {
    start_time = current_time;
    timer_started = true;
  }

  unsigned long elapsed_time = current_time - start_time;


  if (elapsed_time < 10000) {
    SerialCom -> print("elapsed time");
    stop();
    digitalWrite(fan_pin, HIGH);

    float leftVoltage = readPhotoTransistor(LEFT_PHOTOTRANSISTOR);
    float rightVoltage = readPhotoTransistor(RIGHT_PHOTOTRANSISTOR);

    if (leftVoltage < 0.20 && rightVoltage < 0.20) {
      fire_extinguished += 1;
      SerialCom->print("Fire extinguished count: ");
      SerialCom->println(fire_extinguished);
      digitalWrite(fan_pin, LOW);
      timer_started = false;  // Reset for next time
      // delay(1000);

      if (fire_extinguished >= 2) {
        SerialCom->println("Fire extinguished, stopping.");
        return STOPPED;
      }
      reverse();
      delay(300);
      stop();
      extinguished = true;  // Fire extinguished, exit loop
      return DETECT_FIRE;
    }
  } else {
    digitalWrite(fan_pin, LOW);
    extinguished = true;
    timer_started = false;  // Reset for next time
  }

  if (extinguished) {
    reverse();
    delay(300);
    stop();
    return DETECT_FIRE;
  }

  return EXTINGUISH;  // Stay in extinguish state if still trying
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

    // if (!is_battery_voltage_OK())
    //   return STOPPED;

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


float readPhotoTransistor(int analog_pin) {

  float volts = analogRead(analog_pin) * 5.0 / 1024.0;

  return volts;
}

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
  // 1. Read ADC value
  rightIrSensorValue = analogRead(rightFrontIrPin);

  // 2. Convert the sensor value to a voltage (assuming 5V reference voltage)
  float voltage = rightIrSensorValue * (5.0 / 1023.0);

  // 3. Calculate the distance using voltage with validation
  if (voltage < 0.1) {
    // Too low voltage, sensor might be disconnected - use fallback
    rightIrDistance = frontRightDist;  // fallback to last estimate
  } else {
    // Calculate distance using the voltage (for Sharp GP2Y0A21YK0F)
    rightIrDistance = (25.5 * pow(voltage, -1.10));

    // Clamp distance to reasonable range (5-100 cm)
    if (rightIrDistance > 100.0) {
      rightIrDistance = 100.0;
    } else if (rightIrDistance < 5.0) {
      rightIrDistance = 5.0;
    }
  }

  // 4. Check for NaN before Kalman filter
  if (isnan(frontRightDist) || isnan(last_var_ir_right)) {
    frontRightDist = rightIrDistance;  // Reset to current reading
    last_var_ir_right = 1.0;           // Reset variance
  }

  // 5. Kalman filter with NaN check
  double est = Kalman_ir_right(rightIrDistance, frontRightDist);

  if (!isnan(est)) {
    frontRightDist = est;
  } else {
    // Kalman returned NaN, use raw value and reset
    frontRightDist = rightIrDistance;
    last_var_ir_right = 1.0;  // Reset variance
  }

  // SerialCom->print("right  ");
  // SerialCom->println(frontRightDist);
}

void ReadLeftFront() {
  // 1. Read ADC value
  leftIrSensorValue = analogRead(leftFrontIrPin);

  // 2. Convert the sensor value to a voltage (assuming 5V reference voltage)
  float voltage = leftIrSensorValue * (5.0 / 1023.0);

  // 3. Calculate the distance using voltage with validation
  if (voltage < 0.1) {
    // Too low voltage, sensor might be disconnected - use fallback
    leftIrDistance = frontLeftDist;  // fallback to last estimate
  } else {
    // Calculate distance using the voltage (for Sharp GP2Y0A21YK0F)
    leftIrDistance = (25.5 * pow(voltage, -1.10));

    // Clamp distance to reasonable range (5-100 cm)
    if (leftIrDistance > 100.0) {
      leftIrDistance = 100.0;
    } else if (leftIrDistance < 5.0) {
      leftIrDistance = 5.0;
    }
  }

  // 4. Check for NaN before Kalman filter
  if (isnan(frontLeftDist) || isnan(last_var_ir_left)) {
    frontLeftDist = leftIrDistance;  // Reset to current reading
    last_var_ir_left = 1.0;          // Reset variance
  }

  // 5. Kalman filter with NaN check
  double est = Kalman_ir_left(leftIrDistance, frontLeftDist);

  if (!isnan(est)) {
    frontLeftDist = est;
  } else {
    // Kalman returned NaN, use raw value and reset
    frontLeftDist = leftIrDistance;
    last_var_ir_left = 1.0;  // Reset variance
  }

  // SerialCom->print("left  ");
  // SerialCom->println(frontLeftDist);
}

void ReadRight() {
  /* 1. Raw ADC reading */
  backIrSensorValue = analogRead(rightIrPin);
  // SerialCom->print(F("[IR-Back] raw ADC = "));
  // SerialCom->println(backIrSensorValue);

  /* 2. Convert to voltage */
  float voltage = backIrSensorValue * (5.0 / 1023.0);
  // SerialCom->print(F("[IR-Back] voltage  = "));
  // SerialCom->println(voltage, 3);  // 3 decimals

  /* 3. Voltage → distance with validation */
  if (voltage < 0.1) {       // Too low voltage, sensor might be disconnected
    backIrDistance = 100.0;  // Default to max range
    // SerialCom->println(F("[IR-Back] WARNING: Voltage too low, using default"));
  } else if (voltage > 4.5) {  // Too high voltage, possible error
    backIrDistance = 5.0;      // Default to min range
    // SerialCom->println(F("[IR-Back] WARNING: Voltage too high, using default"));
  } else {
    backIrDistance = 11.35795124 * pow(voltage, -0.7897);

    // Clamp distance to reasonable range
    if (backIrDistance > 100.0) {
      backIrDistance = 100.0;
    } else if (backIrDistance < 5.0) {
      backIrDistance = 5.0;
    }
  }

  // SerialCom->print(F("[IR-Back] dist calc = "));
  // SerialCom->println(backIrDistance, 2);  // cm, 2 decimals

  /* 4. Check for NaN before Kalman filter */
  if (isnan(rightDist) || isnan(last_var_ir_back)) {
    // SerialCom->println(F("[IR-Back] Resetting Kalman filter"));
    rightDist = backIrDistance;  // Reset to current reading
    last_var_ir_back = 1.0;      // Reset variance
  }

  /* 5. Kalman filter with NaN check */
  double est = Kalman_ir_back(backIrDistance, rightDist);

  if (!isnan(est)) {
    rightDist = est;
  } else {
    // SerialCom->println(F("[IR-Back] Kalman returned NaN, using raw value"));
    rightDist = backIrDistance;
    last_var_ir_back = 1.0;  // Reset variance
  }

  // SerialCom->print(F("[IR-Back] dist KF   = "));
  // SerialCom->println(rightDist, 2);  // filtered distance

  // SerialCom->println();  // blank line for readability
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
  // SerialCom->println("MECHENG706");
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

#include "sensors.h"

// Global sensor variables
double front_distance = 0;
double frontRightDist = 0;
double frontLeftDist = 0;
double rightDist = 0;
double leftDist = 0;
float currentAngle = 0;
float gyroZeroVoltage = 0;

// Local sensor variables
static int rightIrSensorValue = 0;
static float rightIrDistance = 0.0;
static int leftIrSensorValue = 0;
static float leftIrDistance = 0.0;
static int backIrSensorValue = 0;
static float backIrDistance = 0.0;
static int shortleftIrSensorValue = 0;
static float shortleftIrDistance = 0.0;
static int gyroSensorValue = 0;
static float gyroRate = 0;

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

  // Calculate distance in centimeters
  double distance = pulse_width / 58.0;
  double est = Kalman_ultrasonic(distance, front_distance);
  front_distance = est;
}

void ReadRightFront() {
  rightIrSensorValue = analogRead(RIGHT_FRONT_IR_PIN);
  float voltage = rightIrSensorValue * (5.0 / 1023.0);
  rightIrDistance = (25.5 * pow(voltage, -1.10));
  double est = Kalman_ir_right(rightIrDistance, frontRightDist);
  frontRightDist = est;
}

void ReadLeftFront() {
  leftIrSensorValue = analogRead(LEFT_FRONT_IR_PIN);
  float voltage = leftIrSensorValue * (5.0 / 1023.0);
  leftIrDistance = (25.5 * pow(voltage, -1.10));
  double est = Kalman_ir_left(leftIrDistance, frontLeftDist);
  frontLeftDist = est;
}

void ReadRight() {
  backIrSensorValue = analogRead(RIGHT_IR_PIN);
  float voltage = backIrSensorValue * (5.0 / 1023.0);
  backIrDistance = 11.35795124 * pow(voltage, -0.7897);
  double est = Kalman_ir_back(backIrDistance, rightDist);
  rightDist = est;
}

void ReadLeft() {
  shortleftIrSensorValue = analogRead(LEFT_IR_PIN);
  float voltage = shortleftIrSensorValue * (5.0 / 1023.0);
  
  if (voltage < 0.1) {
    shortleftIrDistance = leftDist;  // fallback to last estimate
  } else {
    shortleftIrDistance = 11.35795124 * pow(voltage, -0.7897);
  }
  
  if (!isnan(shortleftIrDistance)) {
    double est = Kalman_ir_short_left(shortleftIrDistance, leftDist);
    leftDist = est - 1.25;
  }
}

void calibrateGyro() {
  int i;
  float sum = 0;
  pinMode(GYRO_PIN, INPUT);
  for (i = 0; i < 100; i++) {
    gyroSensorValue = analogRead(GYRO_PIN);
    sum += gyroSensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;
}

void ReadGyro() {
  gyroRate = (analogRead(GYRO_PIN) * GYRO_SUPPLY_VOLTAGE) / 1023;
  gyroRate -= (gyroZeroVoltage / 1023 * 5);
  float angularVelocity = gyroRate / GYRO_SENSITIVITY;
  
  if (angularVelocity >= ROTATION_THRESHOLD || angularVelocity <= -ROTATION_THRESHOLD) {
    float angleChange = angularVelocity / (1000 / LOOP_TIME);
    currentAngle += angleChange;
  }
  
  // Keep angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }
}

boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - BATTERY_MIN_LEVEL);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / BATTERY_SCALE_FACTOR;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overcharged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo will be damaged");
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

#include <Servo.h>

enum STATE {
  INITIALISING,
  DETECT_FIRE,
  DRIVE_TO_FIRE,
  STOPPED,
};

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 51;
const byte right_front = 50;
int speed_val = 70;

// Hardware configuration
const int SERVO_PIN = 37;
const int LEFT_PHOTOTRANSISTOR = A9;
const int RIGHT_PHOTOTRANSISTOR = A10;

// System parameters
const float ACTIVATION_THRESHOLD = 0.5;  // Minimum voltage difference to react
const int START_ANGLE = 90;               // Initial servo position
const int ANGLE_INCREMENT = 1;            // Degree change per adjustment
const int SERVO_MIN = 0;                  // Minimum servo angle
const int SERVO_MAX = 180;                // Maximum servo angle

Servo trackerServo;  // Servo controller object

int T = 50;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;
double front_distance = 0;
double last_var_ultrasonic = 1;
double process_noise_ultrasonic = 0.5;
double sensor_noise_ultrasonic = 3.0;  // Change the value of sensor noise to get different KF performance

float readPhotoTransistor(int analog_pin){
  float volts = analogRead(analog_pin)*5.0/1024.0;
  return volts;
}

void readUltrasonic() {
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
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  double distance = pulse_width / 58.0;

  // medianFilter2.AddValue(distance);
  double est = Kalman_ultrasonic(distance, front_distance);
  front_distance = est;
}

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

void stop() {
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val + 70);
  left_rear_motor.writeMicroseconds(1500 + speed_val+ 70);
  right_rear_motor.writeMicroseconds(1500 + speed_val+ 70);
  right_font_motor.writeMicroseconds(1500 + speed_val+ 70);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

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

void setup() {
  Serial.begin(9600);
  
  trackerServo.attach(SERVO_PIN);
  trackerServo.write(START_ANGLE);
  enable_motors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop(void) {

  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case DETECT_FIRE:
      machine_state = detect_fire();
      break;
    case DRIVE_TO_FIRE:
      machine_state = drive_to_fire();
      break;
    case STOPPED:
      machine_state = stopped();
      break;
  };
}

STATE initialising() {
  enable_motors();

  for (int i = 0; i < 10; i++) {
    readUltrasonic();
    delay(50);
  }

  return DETECT_FIRE;
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

STATE stopped() {
  disable_motors();
  trackerServo.detach();
  return STOPPED;
}
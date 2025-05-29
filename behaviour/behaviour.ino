#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

// Define motion states for behavior outputs
enum MOTION {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT_TURN,
  RIGHT_TURN,
  STRAFE_LEFT,
  STRAFE_RIGHT,
  ROTATE_CW,
  ROTATE_CCW
};

// Serial Data pins
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11
#define STARTUP_DELAY 3  // Seconds
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Sensors
#define SERVO_PIN 37
#define LEFT_PHOTOTRANSISTOR A9
#define RIGHT_PHOTOTRANSISTOR A10
#define TRIG_PIN 48
#define ECHO_PIN 49
#define FAN_PIN 38  // Add your fan control pin

// Fire tracking parameters
const float FIRE_THRESHOLD = 0.25;      // Minimum voltage to detect fire
const float ALIGNMENT_THRESHOLD = 0.1;   // Threshold for fire alignment
const int SERVO_CENTER = 90;            // Center position for servo

// Serial Pointer
SoftwareSerial *SerialCom;

// Motor pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 51;
const byte right_front = 50;

// Motor servos
Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;
int speed_val = 200;

// Servo for fire tracking
Servo fire_servo;
int current_servo_angle = 90;

// Ultrasonic sensor
const unsigned int MAX_DIST = 23200;
double front_distance = 100;
double last_var_ultrasonic = 1;
double process_noise_ultrasonic = 1;
double sensor_noise_ultrasonic = 1;

// Gyro
int gyroPin = A14;
int T = 100;  // Loop time in ms
float gyroZeroVoltage = 0;
float gyroSensitivity = 0.007;
float rotationThreshold = 1.5;
float currentAngle = 0;

// IR sensors
const int rightFrontIrPin = A1;
const int leftFrontIrPin = A0;
const int rightIrPin = A3;
const int leftIrPin = A2;
double frontRightDist = 100;
double frontLeftDist = 100;
double rightDist = 100;
double leftDist = 100;

// Kalman filter variables
double last_var_ir_right = 1;
double process_noise_ir_right = 1;
double sensor_noise_ir_right = 1;
double last_var_ir_left = 1;
double process_noise_ir_left = 1;
double sensor_noise_ir_left = 1;
double last_var_ir_back = 1;
double process_noise_ir_back = 1;
double sensor_noise_ir_back = 1;
double last_var_ir_short_left = 1;
double process_noise_ir_short_left = 1;
double sensor_noise_ir_short_left = 1;

// Global state variables
bool fire_detected = false;
bool fire_extinguished = false;
int fires_extinguished_count = 0;
unsigned long extinguish_start_time = 0;
bool currently_extinguishing = false;

// Behavior outputs and flags
MOTION cruise_command;
int cruise_output_flag;

MOTION search_fire_command;
int search_fire_output_flag;

MOTION track_fire_command;
int track_fire_output_flag;

MOTION avoid_command;
int avoid_output_flag;

MOTION wall_follow_command;
int wall_follow_output_flag;

MOTION extinguish_command;
int extinguish_output_flag;

MOTION battery_low_command;
int battery_low_output_flag;

MOTION motor_input;

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
void executeMotorCommand(MOTION cmd);
boolean is_battery_voltage_OK();
float getVoltage(int analog_pin);
double Kalman_ultrasonic(double rawdata, double prev_est);
double Kalman_ir_right(double rawdata, double prev_est);
double Kalman_ir_left(double rawdata, double prev_est);
double Kalman_ir_back(double rawdata, double prev_est);
double Kalman_ir_short_left(double rawdata, double prev_est);

// Behavior functions
void cruise();
void search_fire();
void track_fire();
void avoid();
void wall_follow();
void extinguish();
void battery_check();
void arbitrate();

void setup(void) {
  SerialCom = &BluetoothSerial;
  Serial.begin(115200);
  SerialCom->begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  
  delaySeconds(STARTUP_DELAY);
  SerialCom->println("MECHENG706 - Behavior Based Fire Fighting Robot");
  
  calibrateGyro();
  
  // Initialize fire tracking servo
  fire_servo.attach(SERVO_PIN);
  fire_servo.write(SERVO_CENTER);
  current_servo_angle = SERVO_CENTER;
  
  enable_motors();
  
  // Initialize sensors
  for (int i = 0; i < 10; i++) {
    ReadUltrasonic();
    delay(50);
  }
  
  delay(50);
}

void loop(void) {
  static unsigned long previous_millis = 0;
  
  // Run behaviors at fixed time intervals
  if (millis() - previous_millis > T) {
    previous_millis = millis();
    
    // Read all sensors first
    ReadUltrasonic();
    ReadLeftFront();
    ReadRightFront();
    ReadLeft();
    ReadRight();
    
    // Execute all behaviors in parallel
    // Each behavior reads sensors and sets its command + flag
    cruise();         // Lowest priority
    search_fire();    
    track_fire();     
    avoid();          
    wall_follow();    
    extinguish();     
    battery_check();  // Highest priority
    
    // Arbitrate between behaviors based on priority
    arbitrate();
    
    // Execute the selected motor command
    executeMotorCommand(motor_input);
  }
}

//---------------------------- BEHAVIOR FUNCTIONS ----------------------------

// CRUISE - Default behavior, just move forward
void cruise() {
  cruise_command = FORWARD;
  cruise_output_flag = 1;
}

// SEARCH_FIRE - Rotate to search for fire
void search_fire() {
  float left_photo = getVoltage(LEFT_PHOTOTRANSISTOR);
  float right_photo = getVoltage(RIGHT_PHOTOTRANSISTOR);
  
  // Only active if no fire detected yet
  if (!fire_detected && (left_photo < FIRE_THRESHOLD && right_photo < FIRE_THRESHOLD)) {
    search_fire_command = ROTATE_CW;
    search_fire_output_flag = 1;
  } else {
    search_fire_output_flag = 0;
  }
}

// TRACK_FIRE - Track and move toward detected fire
void track_fire() {
  float left_photo = getVoltage(LEFT_PHOTOTRANSISTOR);
  float right_photo = getVoltage(RIGHT_PHOTOTRANSISTOR);
  
  // Check if we see fire
  if (left_photo > FIRE_THRESHOLD || right_photo > FIRE_THRESHOLD) {
    fire_detected = true;
    float diff = left_photo - right_photo;
    
    // Adjust servo to track fire
    if (abs(diff) > ALIGNMENT_THRESHOLD) {
      if (diff > 0) {
        current_servo_angle = constrain(current_servo_angle + 2, 0, 180);
      } else {
        current_servo_angle = constrain(current_servo_angle - 2, 0, 180);
      }
      fire_servo.write(current_servo_angle);
    }
    
    // Determine motion based on servo angle
    int angle_error = current_servo_angle - SERVO_CENTER;
    
    if (abs(angle_error) > 30) {
      // Fire is far to the side, rotate robot
      if (angle_error > 0) {
        track_fire_command = ROTATE_CW;
      } else {
        track_fire_command = ROTATE_CCW;
      }
    } else if (abs(angle_error) > 10) {
      // Fire is slightly to the side, move forward with turn
      if (angle_error > 0) {
        track_fire_command = RIGHT_TURN;
      } else {
        track_fire_command = LEFT_TURN;
      }
    } else {
      // Fire is centered, move forward
      track_fire_command = FORWARD;
    }
    
    track_fire_output_flag = 1;
  } else {
    track_fire_output_flag = 0;
  }
}

// AVOID - Avoid obstacles using IR sensors
void avoid() {
  // Check for obstacles
  bool left_obstacle = (frontLeftDist < 20);
  bool right_obstacle = (frontRightDist < 20);
  bool front_obstacle = (front_distance < 20);
  
  if (front_obstacle || (left_obstacle && right_obstacle)) {
    avoid_command = BACKWARD;
    avoid_output_flag = 1;
  } else if (left_obstacle) {
    avoid_command = STRAFE_RIGHT;
    avoid_output_flag = 1;
  } else if (right_obstacle) {
    avoid_command = STRAFE_LEFT;
    avoid_output_flag = 1;
  } else {
    avoid_output_flag = 0;
  }
}

// WALL_FOLLOW - Follow walls when avoiding obstacles
void wall_follow() {
  static unsigned long wall_follow_start = 0;
  static bool wall_following = false;
  
  // Activate wall following if we've been avoiding for too long
  if (avoid_output_flag == 1 && !wall_following) {
    if (wall_follow_start == 0) {
      wall_follow_start = millis();
    } else if (millis() - wall_follow_start > 2000) {  // 2 seconds of avoiding
      wall_following = true;
    }
  }
  
  if (wall_following) {
    // Simple wall following - keep wall on left side
    if (leftDist < 10) {
      wall_follow_command = STRAFE_RIGHT;
    } else if (leftDist > 25) {
      wall_follow_command = STRAFE_LEFT;
    } else {
      wall_follow_command = FORWARD;
    }
    wall_follow_output_flag = 1;
    
    // Exit wall following if we see fire again
    if (fire_detected) {
      wall_following = false;
      wall_follow_start = 0;
      wall_follow_output_flag = 0;
    }
  } else {
    wall_follow_start = 0;
    wall_follow_output_flag = 0;
  }
}

// EXTINGUISH - Extinguish fire when close enough
void extinguish() {
  float left_photo = getVoltage(LEFT_PHOTOTRANSISTOR);
  float right_photo = getVoltage(RIGHT_PHOTOTRANSISTOR);
  
  // Check if we're close to fire and it's centered
  if (fire_detected && front_distance < 15 && front_distance > 0 &&
      left_photo > FIRE_THRESHOLD && right_photo > FIRE_THRESHOLD &&
      abs(current_servo_angle - SERVO_CENTER) < 20) {
    
    if (!currently_extinguishing) {
      currently_extinguishing = true;
      extinguish_start_time = millis();
      digitalWrite(FAN_PIN, HIGH);  // Turn on fan
      SerialCom->println("Starting fire extinguishment!");
    }
    
    extinguish_command = STOP;
    extinguish_output_flag = 1;
    
    // Check if extinguishment is complete (10 seconds or fire out)
    if (millis() - extinguish_start_time > 10000 ||
        (left_photo < FIRE_THRESHOLD && right_photo < FIRE_THRESHOLD)) {
      digitalWrite(FAN_PIN, LOW);  // Turn off fan
      currently_extinguishing = false;
      fire_detected = false;
      fires_extinguished_count++;
      current_servo_angle = SERVO_CENTER;
      fire_servo.write(current_servo_angle);
      SerialCom->println("Fire extinguished!");
      
      if (fires_extinguished_count >= 2) {
        SerialCom->println("Mission complete! Two fires extinguished.");
        extinguish_command = STOP;
        extinguish_output_flag = 2;  // Special flag to indicate mission complete
      } else {
        extinguish_output_flag = 0;
      }
    }
  } else {
    extinguish_output_flag = 0;
  }
}

// BATTERY_CHECK - Stop if battery is low
void battery_check() {
  if (!is_battery_voltage_OK()) {
    battery_low_command = STOP;
    battery_low_output_flag = 1;
  } else {
    battery_low_output_flag = 0;
  }
}

// ARBITRATE - Select behavior based on priority (highest to lowest)
void arbitrate() {
  // Start with lowest priority
  motor_input = STOP;  // Default
  
  if (cruise_output_flag == 1) {
    motor_input = cruise_command;
  }
  
  if (search_fire_output_flag == 1) {
    motor_input = search_fire_command;
  }
  
  if (track_fire_output_flag == 1) {
    motor_input = track_fire_command;
  }
  
  if (avoid_output_flag == 1) {
    motor_input = avoid_command;
  }
  
  if (wall_follow_output_flag == 1) {
    motor_input = wall_follow_command;
  }
  
  if (extinguish_output_flag == 1) {
    motor_input = extinguish_command;
    if (extinguish_output_flag == 2) {  // Mission complete
      disable_motors();
    }
  }
  
  if (battery_low_output_flag == 1) {
    motor_input = battery_low_command;
    disable_motors();
  }
}

// Execute motor commands
void executeMotorCommand(MOTION cmd) {
  switch(cmd) {
    case STOP:
      stop();
      break;
    case FORWARD:
      forward();
      break;
    case BACKWARD:
      reverse();
      break;
    case LEFT_TURN:
      forward_left();
      break;
    case RIGHT_TURN:
      forward_right();
      break;
    case STRAFE_LEFT:
      strafe_left();
      break;
    case STRAFE_RIGHT:
      strafe_right();
      break;
    case ROTATE_CW:
      cw();
      break;
    case ROTATE_CCW:
      ccw();
      break;
  }
}

//----------------------Helper Functions---------------------------------

void delaySeconds(int TimedDelaySeconds) {
  for (int i = 0; i < TimedDelaySeconds; i++) {
    delay(1000);
  }
}

float getVoltage(int analog_pin) {
  float volts = analogRead(analog_pin) * 5.0 / 1024.0;
  return volts;
}

//----------------------Kalman Filters------------------------
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

//----------------------Battery------------------------
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
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

//----------------------Sensor Reading Functions------------------------
void ReadUltrasonic() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      return;
    }
  }

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
  double distance = pulse_width / 58.0;
  double est = Kalman_ultrasonic(distance, front_distance);
  front_distance = est;
}

void ReadRightFront() {
  int sensorValue = analogRead(rightFrontIrPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = (25.5 * pow(voltage, -1.10));
  double est = Kalman_ir_right(distance, frontRightDist);
  frontRightDist = est;
}

void ReadLeftFront() {
  int sensorValue = analogRead(leftFrontIrPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = (25.5 * pow(voltage, -1.10));
  double est = Kalman_ir_left(distance, frontLeftDist);
  frontLeftDist = est;
}

void ReadRight() {
  int sensorValue = analogRead(rightIrPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 11.35795124 * pow(voltage, -0.7897);
  double est = Kalman_ir_back(distance, rightDist);
  rightDist = est;
}

void ReadLeft() {
  int sensorValue = analogRead(leftIrPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  if (voltage < 0.1) {
    leftDist = leftDist;  // Keep previous value
  } else {
    float distance = 11.35795124 * pow(voltage, -0.7897);
    if (!isnan(distance)) {
      double est = Kalman_ir_short_left(distance, leftDist);
      leftDist = est - 1.25;
    }
  }
}

//----------------------Gyro------------------------
void calibrateGyro() {
  int i;
  float sum = 0;
  pinMode(gyroPin, INPUT);
  for (i = 0; i < 100; i++) {
    int sensorValue = analogRead(gyroPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;
}

void ReadGyro() {
  int sensorValue = analogRead(gyroPin);
  float gyroRate = (sensorValue * 5.0) / 1023;
  gyroRate -= (gyroZeroVoltage / 1023 * 5);
  float angularVelocity = gyroRate / gyroSensitivity;
  
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }
  
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }
}

//----------------------Motor Control------------------------
void disable_motors() {
  left_font_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_font_motor.detach();
  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_font_motor.attach(right_front);
}

void stop() {
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
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

void forward_left() {
  // Forward with left turn - reduce left side speed
  left_font_motor.writeMicroseconds(1500 + speed_val/2);
  left_rear_motor.writeMicroseconds(1500 + speed_val/2);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void forward_right() {
  // Forward with right turn - reduce right side speed
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val/2);
  right_font_motor.writeMicroseconds(1500 - speed_val/2);
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
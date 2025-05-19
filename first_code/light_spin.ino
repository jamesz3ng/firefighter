#include <Servo.h>
Servo myservo;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 51;
const byte right_front = 50;

const int servo_pin = 37;
const int LEFT_PHOTOTRANSISTOR = A9;
const int RIGHT_PHOTOTRANSISTOR = A10;
const float PHOTOTRANSISTOR_THRESHOLD = 0.10;

const int speed_val = 100;

const int start_servo_angle = 45;
bool found_light = false;


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


float getVoltage(int analog_pin){
  float volts = analogRead(analog_pin)*5.0/1024.0;
  return volts;
}

void setup()
  {
  Serial.begin(9600);
  Serial.println("start.....");
  myservo.attach(servo_pin); //servo signal pin
  myservo.write(start_servo_angle); // set servo to mid-point

  enable_motors();
  }

void loop() {
  float left_photo_reading;
  float right_photo_reading;
  
  while (!found_light){
    left_font_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_font_motor.writeMicroseconds(1500 + speed_val);

    left_photo_reading = getVoltage(LEFT_PHOTOTRANSISTOR);
    right_photo_reading = getVoltage(RIGHT_PHOTOTRANSISTOR);

    if (left_photo_reading > 0.25 && right_photo_reading > 0.25){
      Serial.print("Insidde initial check, left reading: ");
      Serial.println(left_photo_reading);
      Serial.print(" right reading :");
      Serial.println(right_photo_reading);

      if (abs(left_photo_reading - right_photo_reading) < 0.5){
        found_light = true;
        stop();
      }
    }
  }

  
  // int current_servo_angle = start_servo_angle;
  // while (1) {
  //   left_photo_reading = getVoltage(LEFT_PHOTOTRANSISTOR);
  //   right_photo_reading = getVoltage(RIGHT_PHOTOTRANSISTOR);
  //   if (left_photo_reading > right_photo_reading){
  //     current_servo_angle += 3; 
  //     myservo.write(current_servo_angle);
  //   } 
  //   else if (right_photo_reading > left_photo_reading){
  //     current_servo_angle -= 3; 
  //     myservo.write(current_servo_angle);
  //   }
  //   Serial.print("Left Reading  ");
  //   Serial.println(left_photo_reading);

  //   // Serial.print("Right Reading  ");
  //   // Serial.println(right_photo_reading);
  //   delay(50);


  // }

} 



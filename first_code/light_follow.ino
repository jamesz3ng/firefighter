#include <Servo.h>
Servo myservo;

const int servo_pin = 37;
const int LEFT_PHOTOTRANSISTOR = A9;
const int RIGHT_PHOTOTRANSISTOR = A10;
const float PHOTOTRANSISTOR_THRESHOLD = 0.10;

const int start_servo_angle = 45;


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
  }

void loop() {
  float left_photo_reading;
  float right_photo_reading;
  int current_servo_angle = start_servo_angle;
  while (1) {
    left_photo_reading = getVoltage(LEFT_PHOTOTRANSISTOR);
    right_photo_reading = getVoltage(RIGHT_PHOTOTRANSISTOR);
    if (left_photo_reading > right_photo_reading){
      current_servo_angle += 3; 
      myservo.write(current_servo_angle);
    } 
    else if (right_photo_reading > left_photo_reading){
      current_servo_angle -= 3; 
      myservo.write(current_servo_angle);
    }
    Serial.print("Left Reading  ");
    Serial.println(left_photo_reading);

    // Serial.print("Right Reading  ");
    // Serial.println(right_photo_reading);
    delay(50);


  }

} 




#include "utils.h"

void delaySeconds(int TimedDelaySeconds) {
  for (int i = 0; i < TimedDelaySeconds; i++) {
    delay(1000);
  }
}

float getVoltage(int analog_pin) {
  float volts = analogRead(analog_pin) * 5.0 / 1024.0;
  return volts;
}

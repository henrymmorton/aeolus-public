#include "sensors.h"

void Tachometer::initTachometer() {
  pinMode(hall_pin, INPUT); // initialize the hall pin as a input:
}

void Tachometer::simpleRead() {
  Serial.println(digitalRead(hall_pin));
}

void Tachometer::updateTachometer() {

  // Reset timers and rev counts upon a new measurement
  if (new_dat) {
    rev_count = 0;
    new_dat = false;
    t1 = micros();
  }

  // Check if the sensor is registering the magnet
  if (digitalRead(hall_pin) == 1) {
    // Ensure this is a new signal (the sensor wasn't registering the magnet last read)
    if (hall_trpd == false) {
      hall_trpd = true;
      rev_count += 1;
    }
  } else {
    hall_trpd = false;
  }

  // Measure the time since the last update
  dt_micros = micros() - t1;

  // If we have surpassed the revolution threshold, measure rpm
  if (rev_count>=rev_thrsh) {
    rev_count = rev_count / 2;
    dt_s = dt_micros / 1000000;
    rpm = (rev_count / dt_s) * 60;
    new_dat = true;

  // If we have surpassed the time threshold, measure rpm
  } else if (dt_micros > 1000000) {
    rev_count = rev_count / 2;
    dt_s = dt_micros / 1000000;
    rpm = (rev_count / dt_s) * 60;
    new_dat = true;
  }
}

void Tachometer::printRPM() {
  if (new_dat) {
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
}

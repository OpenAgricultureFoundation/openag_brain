/**
 *  \file openag_am2315.cpp
 *  \brief Air temperature and air humidity sensor.
 */
/***************************************************
  This is a library for the AM2315 Humidity & Temp Sensor

  Designed specifically to work with the AM2315 sensor from Adafruit
  ----> https://www.adafruit.com/products/1293

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  Modified for OpenAg
 ****************************************************/
#include "openag_binary_sensor.h"

BinarySensor::BinarySensor(int pin, bool is_active_low) {
  _pin = pin;
  _is_active_low = is_active_low;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

void BinarySensor::begin() {
  pinMode(_pin, OUTPUT);
  _is_on = false;
  _time_of_last_reading = 0;
  _send_is_on = true;
}

void BinarySensor::update() {
  if (millis() - _time_of_last_reading > _min_update_interval) {
    readData();
    _time_of_last_reading = millis();
  }
}

bool BinarySensor::get_is_on(std_msgs::Bool &msg) {
  msg.data = _is_on;
  bool res = _send_is_on;
  _send_is_on = false;
  return res;
}

void BinarySensor::readData() {
  _is_on = digitalRead(_pin) ^ _is_active_low;
  _send_is_on = true;
}

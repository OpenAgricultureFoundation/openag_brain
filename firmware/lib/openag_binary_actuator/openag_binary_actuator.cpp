#include "openag_binary_actuator.h"

BinaryActuator::BinaryActuator(int pin, bool is_active_low, int shutoff_ms) {
  _pin = pin;
  _is_active_low = is_active_low;
  _shutoff_ms = shutoff_ms;
  _last_cmd = 0;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

void BinaryActuator::begin() {
  pinMode(_pin, OUTPUT);
  if (_is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
}

void BinaryActuator::update() {
  uint32_t curr_time = millis();
  if ((curr_time - _last_cmd) > _shutoff_ms) {
    if (_is_active_low) {
      digitalWrite(_pin, HIGH);
    }
    else {
      digitalWrite(_pin, LOW);
    }
  }
}

void BinaryActuator::set_cmd(std_msgs::Bool cmd) {
  _last_cmd = millis();
  if (cmd.data ^ _is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
}

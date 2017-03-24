#include "openag_pulse_actuator.h"

PulseActuator::PulseActuator(int pin, bool is_active_low, int pulse_ms, int update_ms) {
  _pin = pin;
  _is_active_low = is_active_low;
  _pulse_ms = pulse_ms;
  _update_ms = update_ms;
  _last_cmd = 0;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

void PulseActuator::begin() {
  pinMode(_pin, OUTPUT);
  if (_is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
}

void PulseActuator::update() {}

void PulseActuator::set_cmd(std_msgs::Bool cmd) {
  uint32_t curr_time = millis();
  if ((curr_time - _last_cmd) > _update_ms) { // Only pulse once every update_ms
    _last_cmd = curr_time;
    if (_is_active_low) {
      if (cmd.data) {
        digitalWrite(_pin, LOW);
        delay(_pulse_ms);
        digitalWrite(_pin, HIGH);
      }
      else {
        digitalWrite(_pin, HIGH);
      }
    }
    else {
      if (cmd.data) {
        digitalWrite(_pin, HIGH);
        delay(_pulse_ms);
        digitalWrite(_pin, LOW);
      }
      else {
        digitalWrite(_pin, LOW);
      }
    }
  }
}

#include "openag_doser_pump.h"

DoserPump::DoserPump(int pin, bool is_active_low) {
  _pin = pin;
  _is_active_low = is_active_low;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

void DoserPump::begin() {
  pinMode(_pin, OUTPUT);
  if (_is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
}

void DoserPump::update() {}

void DoserPump::set_cmd(std_msgs::Float32 cmd) {
  if (cmd.data != _prev_cmd) {
    // Assign cmd as prev, so we can compare it next time.
    _prev_cmd = cmd.data;

    // Determine dosing rate
    float rate = 0; // ml/ms
    if (cmd.data < 100 * _rate100) {
      rate = _rate100;
    }
    else if (cmd.data < 1000 * _rate1000) {
      float slope = (_rate1000 - _rate100) / (_rate1000 * 1000 - _rate100 * 100);
      rate = _rate100 + slope * (cmd.data - _rate100 * 100);
    }
    else {
      rate = _rate1000;
    }

    // Set pump on for duration then turn off
    float duration = cmd.data / rate;
    if (_is_active_low) {
      digitalWrite(_pin, LOW);
      delay(duration);
      digitalWrite(_pin, HIGH);
    }
    else {
      digitalWrite(_pin, HIGH);
      delay(duration);
      digitalWrite(_pin, LOW);
    }
  }
}

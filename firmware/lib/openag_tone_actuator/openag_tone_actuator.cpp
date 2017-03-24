#include "openag_tone_actuator.h"

ToneActuator::ToneActuator(int pin, bool is_active_low, int tone_frequency, int _tone_duration) {
  _pin = pin;
  _is_active_low = is_active_low;
  _tone_frequency = tone_frequency;
  _tone_duration = _tone_duration;
  _last_cmd = 0;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

void ToneActuator::begin() {
}

void ToneActuator::update() {
  uint32_t curr_time = millis();
  if ((curr_time - _last_cmd) > _shutoff_ms) {
    noTone(_pin);
  }
}

void ToneActuator::set_cmd(std_msgs::Bool cmd) {
  _last_cmd = millis();
  if (cmd.data) {
    tone(_pin, _tone_frequency);
  }
  else {
    noTone(_pin);
  }
}

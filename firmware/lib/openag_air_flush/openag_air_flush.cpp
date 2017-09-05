#include "openag_air_flush.h"

AirFlush::AirFlush(int pin, bool is_active_low, float cfm) {
  _pin = pin;
  _is_active_low = is_active_low;
  _maxCFM = cfm
  _cmd_start_time = 0;
  _previous_command_time = 0;
  _is_on = false;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

uint8_t AirFlush::begin() {
//   Serial1.begin(9600);
//   Serial1.println("Air Flush Transmitting to Serial1");
  pinMode(_pin, OUTPUT);
  if (_is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
  return status_level;
}

uint8_t AirFlush::update() {
  uint32_t curr_time = millis();

  if(curr_time - _cmd_start_time > _shutoff_ms){
    _is_on = false;
    digitalWrite(_pin, bool2command(_isOn));
  }

  // Flush or not flush based on the current set frequencies
  if(_isOn && curr_time - _last_pulse > _on_duration){
    _isOn = false;
    _last_pulse = curr_time;
  }
  if(!_isOn && curr_time - _last_pulse > _off_duration){
    _isOn = true;
    _last_pulse = curr_time;
  }

  digitalWrite(_pin, bool2command(_isOn));
  return status_level;
}

uint8_t AirFlush::set_cmd(float volume) {
  // cmd is in terms of Air volume per minute
  _cmd_start_time = millis();

  if(volume > _maxCFM){
    volume = _maxCFM;
  }
  float onRate = volume / _maxCFM;

  _on_duration = (uint32_t) onRatio * _cycle_ms;
  _off_duration = (uint32_t)_cycle_ms - _on_duration;

  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
  return status_level;
}

// Output a HIGH/LOW signal based on _is_active_low and the boolean passed to
// always turn on when passed true.
uint8_t AirFlush::bool2command(bool isOn){
  bool realValue = _is_active_low ? !isOn : isOn;
  return realValue ? HIGH : LOW;
}

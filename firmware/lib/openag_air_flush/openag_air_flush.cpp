#include "openag_air_flush.h"

AirFlush::AirFlush(int pin, bool is_active_low) {
  _pin = pin;
  _is_active_low = is_active_low;
  _cmd_start_time = 0;
  _prev_cmd = 0;
  _is_on = false;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

void AirFlush::begin() {
//   Serial1.begin(9600);
//   Serial1.println("Air Flush Transmitting to Serial1");
  pinMode(_pin, OUTPUT);
  if (_is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
}

void AirFlush::update() {
  uint32_t curr_time = millis();
  if (_is_on && ((curr_time - _cmd_start_time) > _prev_cmd)) { // turn off once exceeded on duration
    _is_on = false;
//     Serial1.print("Turning OFF by setting: ");
    if (_is_active_low) {
      digitalWrite(_pin, HIGH);
//       Serial1.println("HIGH");
    }
    else {
      digitalWrite(_pin, LOW);
//       Serial1.println("LOW");
    }
  }
}

void AirFlush::set_cmd(std_msgs::Float32 cmd) {
//   Serial1.print("Received cmd="); Serial1.print(cmd.data);
  uint32_t cmd_ms = uint32_t(cmd.data * 60000); // convert minutes to milliseconds
//   Serial1.print(", cmd_ms="); Serial1.print(cmd_ms);
//   Serial1.print(", _prev_cmd="); Serial1.println(_prev_cmd);
  if ((cmd_ms != _prev_cmd) && (cmd_ms > 0)) { // only turn on if receive new command
    _is_on = true;
    _cmd_start_time = millis();
//     Serial1.print("Turning ON by setting: ");
    if (_is_active_low) {
      digitalWrite(_pin, LOW);
//       Serial1.println("LOW");
    }
    else {
      digitalWrite(_pin, HIGH);
//       Serial1.println("HIGH");
    }
  }
  _prev_cmd = cmd_ms;
}

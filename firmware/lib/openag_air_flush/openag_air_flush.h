#ifndef OPENAG_AIR_FLUSH_H
#define OPENAG_AIR_FLUSH_H

#include "Arduino.h"
#include <openag_module.h>

class AirFlush : public Module {
  public:
    // Constructor
    AirFlush(int pin, bool is_active_low, float cfm=11.7);

    // Public functions
    uint8_t begin();
    uint8_t update();
    uint8_t set_cmd(float cmd);

  private:
    // Private variables
    int _pin;
    uint32_t _cmd_start_time;
    bool _is_active_low;
    bool _is_on;
    float _maxCFM;
    uint32_t _on_duration;
    uint32_t _off_duration;
    uint32_t _last_pulse;
    uint32_t _cycle_ms = 60000; // 1 on/off cycle is 1 minute
    uint32_t _shutoff_ms = 10000;

    uint8_t bool2command(bool isOn);
};

#endif

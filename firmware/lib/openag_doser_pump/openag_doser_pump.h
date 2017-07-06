#ifndef OPENAG_DOSER_PUMP_H
#define OPENAG_DOSER_PUMP_H

#include "Arduino.h"
#include <openag_module.h>

class DoserPump : public Module {
  public:
    // Constructor
    // TODO: Test the pumps and find their lower and upper bounds
    DoserPump(int pin, bool is_active_low, float lowerBound=10, float upperBound=4764);

    // Public functions
    uint8_t begin();
    uint8_t update();
    uint8_t set_cmd(float cmd);

  private:
    // Private variables
    int _pin;
    uint32_t _shutoff_ms = 10000;
    uint32_t _on_duration;
    uint32_t _off_duration;
    uint32_t _last_cmd = 0;
    uint32_t _last_pulse = 0;
    bool _is_active_low;
    bool _isOn;
    float _dosingFreq = 60000; // dose once every minute
    float _lowerBound = 100;
    float _upperBound = 1000;

    uint8_t bool2command(bool isOn);

    const static uint8_t CODE_STALE_COMMAND = 1;
    const static uint8_t CODE_LOWER_BOUND = 2;
    const static uint8_t CODE_UPPER_BOUND = 3;

};

#endif

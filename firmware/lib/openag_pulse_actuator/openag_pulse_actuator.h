#ifndef OPENAG_PULSE_ACTUATOR_H
#define OPENAG_PULSE_ACTUATOR_H

#include "Arduino.h"
#include <std_msgs/Bool.h>
#include <openag_module.h>

class PulseActuator : public Module {
  public:
    // Constructor
    PulseActuator(int pin, bool is_active_low, int pulse_ms, int update_ms);

    // Public functions
    void begin();
    void update();
    void set_cmd(std_msgs::Bool cmd);

  private:
    // Private variables
    int _pin;
    bool _is_active_low;
    int _pulse_ms = 50;
    int _update_ms = 4000;
    uint32_t _last_cmd;
};

#endif

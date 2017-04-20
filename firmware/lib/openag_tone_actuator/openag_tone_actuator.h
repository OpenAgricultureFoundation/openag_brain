#ifndef OPENAG_TONE_ACTUATOR_H
#define OPENAG_TONE_ACTUATOR_H

#include "Arduino.h"
#include <std_msgs/Bool.h>
#include <openag_module.h>

class ToneActuator : public Module {
  public:
    // Constructor
    ToneActuator(int pin, bool is_active_low, int tone_frequency, int tone_duration);

    // Public functions
    void begin();
    void update();
    void set_cmd(std_msgs::Bool cmd);

  private:
    // Private variables
    int _pin;
    bool _is_active_low;
    int _tone_frequency = 140;
    int _tone_duration = -1;
    int _shutoff_ms = 10000;
    uint32_t _last_cmd;
};

#endif

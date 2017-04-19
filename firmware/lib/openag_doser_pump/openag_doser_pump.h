#ifndef OPENAG_DOSER_PUMP_H
#define OPENAG_DOSER_PUMP_H

#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <openag_module.h>

class DoserPump : public Module {
  public:
    // Constructor
    DoserPump(int pin, bool is_active_low);

    // Public functions
    void begin();
    void update();
    void set_cmd(std_msgs::Float32 cmd);

  private:
    // Private variables
    int _pin;
    bool _is_active_low;
    float _prev_cmd;
    float _rate100 = 0.0019;
    float _rate1000 = 0.0015;
};

#endif

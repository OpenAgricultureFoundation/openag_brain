#ifndef OPENAG_AIR_FLUSH_H
#define OPENAG_AIR_FLUSH_H

#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <openag_module.h>

class AirFlush : public Module {
  public:
    // Constructor
    AirFlush(int pin, bool is_active_low);

    // Public functions
    void begin();
    void update();
    void set_cmd(std_msgs::Float32 cmd);

  private:
    // Private variables
    int _pin;
    uint32_t _cmd_start_time;
    bool _is_active_low;
    bool _is_on;
    uint32_t _prev_cmd;
};

#endif

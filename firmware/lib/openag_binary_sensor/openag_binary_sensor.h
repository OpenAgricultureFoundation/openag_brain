#ifndef OPENAG_BINARY_SENSOR_H
#define OPENAG_BINARY_SENSOR_H

#include "Arduino.h"
#include <std_msgs/Bool.h>
#include <openag_module.h>

class BinarySensor : public Module {
  public:
    // Constructor
    BinarySensor(int pin, bool is_active_low);

    // Methods
    void begin();
    void update();
    bool get_is_on(std_msgs::Bool &msg);

  private:
    // Private variables
    float _is_on;
    int _pin;
    bool _is_active_low;
    bool _send_is_on;
    uint32_t _time_of_last_reading;
    const static uint32_t _min_update_interval = 2000;

    // Private methods
    void readData();
};

#endif

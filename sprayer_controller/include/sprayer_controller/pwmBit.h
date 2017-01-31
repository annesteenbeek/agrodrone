#pragma once

#include <algorithm>
#include "ros/ros.h"

#define DEFAULT_FREQ 500 
#define DEFAULT_PWM 0
#define DEFAULT_TICKS 255 

class pwmBit {

public:
    pwmBit();
    void setPWM(int pwm_val);
    void setMicroseconds(int microseconds);
    int getPWM(){return ticks_high;};
    int getState(int tick_nr);

private:
    int ticks_high;
    int ticks;
    int freq;
};

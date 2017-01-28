#pragma once

#include <algorithm>

#define DEFAULT_PWM 0

class pwmBit {

public:
    pwmBit();
    void setPWM(int pwm_val);
    int getPWM(){return pwm_ticks_high;};
    int getState(int tick_nr);

private:
    int pwm_ticks_high;
};

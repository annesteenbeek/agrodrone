#pragma once

#include <algorithm>

#define DEFAULT_PWM 0

class pwmBit {

public:
    pwmBit();
    void setPWM(int pwm_val);
    int getState(int tick_nr);

private:
    int my_pwm;
};

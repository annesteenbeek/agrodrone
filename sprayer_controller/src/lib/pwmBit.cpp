#include "sprayer_controller/pwmBit.h"

/* @brief this class is used to simulate a PWM signal on digital ports
 */

pwmBit::pwmBit() {
    setPWM(DEFAULT_PWM);
    ros::param::param<int>("/sprayer_controller/pwm_freq", freq, DEFAULT_FREQ);
    ros::param::param<int>("/sprayer_controller/ticks", ticks, DEFAULT_TICKS);
}

// Calculate how many of the ticks in each period should be high
void pwmBit::setPWM(int pwm_val) {
    ticks_high = std::min(ticks, std::max(0, pwm_val));
}

void pwmBit::setMicroseconds(int microseconds) {
    int sec_to_microsec = 1 * 1000 * 1000;
    // calc duration of tick in microsec
    float duration = sec_to_microsec/((float)(freq*ticks));
    setPWM(microseconds/duration);
    
}

int pwmBit::getState(int tick_nr) {
    return ticks_high > tick_nr;
}

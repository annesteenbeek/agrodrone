#include "sprayer_controller/pwmBit.h"

/* @brief this class is used to simulate a PWM signal on digital ports
 */

pwmBit::pwmBit() {
    setPWM(DEFAULT_PWM);
}

// Calculate how many of the 256 ticks in each period should be high
void pwmBit::setPWM(int pwm_val) {
    pwm_ticks_high = std::min(256, std::max(0, pwm_val));
}

int pwmBit::getState(int tick_nr) {
    return pwm_ticks_high > tick_nr;
}


// Compile : gcc -o <create excute file name> <source file name> -lwiringPi -lwiringPiDev -lpthread

#include "sprayer_controller/pwmBit.h"
#include "sprayer_controller/ShiftReg.h"
#include "ros/ros.h"

#define clockPin 0 
#define latchPin 2
#define dataPin 3 
#define ledPin 12

#define DEFAULT_FREQ 500

class SprayerController{
    ShiftReg shiftReg(clockPin, dataPin, latchPin);
    pwmBit pwmArray[8]; 
    int tick_nr = 256; 
    pwmArray[1].setPWM(100); // to test pwm
}

void SprayerController::sprayerSendPWM(const ros::TimerEvent&) {
    unsigned char data = 0;
    tick_nr = ++tick_nr % 256; // 256 ticks per period

    for(int i=8; i>=0; --i) { // walkt rough array reversed
        data <<= 1; // shift the right most bit 1 place left 
        data |= pwmArray[i].getState(tick_nr); // add new state to the right
    }
    shiftReg.sendData(data);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sprayer_controller");
    ros::NodeHandle nh;

    float freq;
    nh.param("/pwm_freq", freq, DEFAULT_FREQ);

    SprayerController sc;
    ros::Timer pwm_timer = nh.createTimer(ros::Duration(1/freq), sc.sprayerSendPWM);

    ros::spin();
    return 0 ;
}


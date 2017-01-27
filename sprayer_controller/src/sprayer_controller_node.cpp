
// Compile : gcc -o <create excute file name> <source file name> -lwiringPi -lwiringPiDev -lpthread

#include "sprayer_controller/pwmBit.h"
#include "sprayer_controller/ShiftReg.h"
#include "ros/ros.h"
#include "ros/console.h"

#define clockPin 0 
#define latchPin 2
#define dataPin 3 

#define DEFAULT_FREQ 500
#define TICKS 256


ShiftReg shiftReg(clockPin, dataPin, latchPin);
pwmBit pwmArray[8]; 
int tick_nr = TICKS; 
    
void sprayerSendPWM(const ros::TimerEvent&) {
    unsigned char data = 0;
    tick_nr = ++tick_nr % TICKS; // 256 ticks per period

    for(int i=8; i>=0; --i) { // walkt rough array reversed
        data <<= 1; // shift the right most bit 1 place left 
        data |= pwmArray[i].getState(tick_nr); // add new state to the right
    }
    shiftReg.sendData(data);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sprayer_controller");
    ros::NodeHandle nh;

    int freq;
    nh.param("/pwm_freq", freq, DEFAULT_FREQ);

    pwmArray[5].setPWM(100); // to test pwm
    pwmArray[7].setPWM(200);
    pwmArray[6].setPWM(1);
    freq *= TICKS; // multiply the pwm frequency with the ticks per phase
    ros::Timer pwm_timer = nh.createTimer(ros::Duration(1/((float) freq)), sprayerSendPWM);
    ROS_DEBUG("pwm duration: %f", 1/((float) freq) );

    ros::spin();
    return 0 ;
}

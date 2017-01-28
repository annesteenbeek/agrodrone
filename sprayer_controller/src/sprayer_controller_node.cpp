/*
* @brief node to contorol sprayer motor speeds
* @file sprayer_controller_node.cpp
* @author Anne Steenbeek <annesteenbeek@gmail.com>
*/

#include "sprayer_controller/pwmBit.h"
#include "sprayer_controller/ShiftReg.h"
#include "sprayer_controller/MotorSpeeds.h"
#include "ros/ros.h"

#define clockPin 0 
#define latchPin 2
#define dataPin 3 

#define DEFAULT_FREQ 500
#define TICKS 255


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

// TODO add ability to send less then 8 bytes, maybe only send 1 message at a time
void pwmCallback(const sprayer_controller::MotorSpeeds::ConstPtr &msg) {
	for(int i=0; i<8; ++i){
		pwmArray[i].setPWM(msg->speeds[i]);
	}
	return; 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sprayer_controller");
    ros::NodeHandle nh;

    int freq;
    nh.param("/pwm_freq", freq, DEFAULT_FREQ);

    freq *= TICKS; // multiply the pwm frequency with the ticks per phase
    ros::Timer pwm_timer = nh.createTimer(ros::Duration(1/((float) freq)), sprayerSendPWM);
    ros::Subscriber sub = nh.subscribe("motor_speed", 3, pwmCallback);

    ros::spin();
    return 0 ;
}

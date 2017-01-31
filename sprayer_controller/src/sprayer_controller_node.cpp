/*
* @brief node to contorol sprayer motor speeds
* @file sprayer_controller_node.cpp
* @author Anne Steenbeek <annesteenbeek@gmail.com>
*/

#include "sprayer_controller/pwmBit.h"
#include "sprayer_controller/MotorSpeeds.h"
#include <wiringPi.h>
#include <wiringShift.h>
#include "ros/ros.h"
#include "ros/console.h"

class SprayerController {
    pwmBit pwmArray[8]; 
    int clockPin;
    int latchPin;
    int dataPin;

    int freq;
    int ticks;
    int tick_nr;

    public:
    SprayerController (ros::NodeHandle nh) {

        ros::param::param<int>("/sprayer_controller/pwm_freq", freq, DEFAULT_FREQ);
        ros::param::param<int>("/sprayer_controller/ticks", ticks, DEFAULT_TICKS);
        bool got_params = true;
        got_params = got_params && ros::param::get("/sprayer_controller/clockPin", clockPin);
        got_params = got_params && ros::param::get("/sprayer_controller/dataPin", dataPin);
        got_params = got_params && ros::param::get("/sprayer_controller/latchPin", latchPin);

        pwmArray[7].setMicroseconds(700);

        if (!got_params) {
            ROS_FATAL("Unable to retrieve GPIO pins from parameter server"); 
        } else {
            pinSetup();
            freq *= ticks; // multiply the pwm frequency with the ticks per phase
            tick_nr = ticks;
            ros::Timer pwm_timer = nh.createTimer(ros::Duration(1/((float) freq)), &SprayerController::sprayerSendPWM, this);
            ros::Subscriber sub = nh.subscribe("motor_speed", 3, &SprayerController::pwmCallback, this);

            ros::spin();
        }
    }

    void sprayerSendPWM(const ros::TimerEvent& event) {
        unsigned char data = 0;
        tick_nr = ++tick_nr % ticks; // 256 ticks per period

        for(int i=8; i>=0; --i) { // walkt rough array reversed
            data <<= 1; // shift the right most bit 1 place left 
            data |= pwmArray[i].getState(tick_nr); // add new state to the right
        }
        digitalWrite(latchPin, 0);
        shiftOut(dataPin, clockPin, 1, data);
        digitalWrite(latchPin, 1);
    }

    void pwmCallback(const sprayer_controller::MotorSpeeds::ConstPtr &msg) {
        for(int i=0; i<8; ++i){
            pwmArray[i].setPWM(msg->speeds[i]);
        }
    }

    void pinSetup() {
        wiringPiSetup();

        pinMode(clockPin, OUTPUT);
        pinMode(dataPin, OUTPUT);
        pinMode(latchPin, OUTPUT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sprayer_controller");
    ros::NodeHandle nh;
    SprayerController sc(nh);
    return 0 ;
}

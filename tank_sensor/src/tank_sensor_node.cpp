/***********************************************************************
 * mcp3008SpiTest.cpp. Sample program that tests the mcp3008Spi class.
 * an mcp3008Spi class object (a2d) is created. the a2d object is instantiated
 * using the overloaded constructor. which opens the spidev0.0 device with
 * SPI_MODE_0 (MODE 0) (defined in linux/spi/spidev.h), speed = 1MHz &
 * bitsPerWord=8.
 *
 * call the spiWriteRead function on the a2d object 20 times. Each time make sure
 * that conversion is configured for single ended conversion on CH0
 * i.e. transmit ->  byte1 = 0b00000001 (start bit)
 *                   byte2 = 0b1000000  (SGL/DIF = 1, D2=D1=D0=0)
 *                   byte3 = 0b00000000  (Don't care)
 *      receive  ->  byte1 = junk
 *                   byte2 = junk + b8 + b9
 *                   byte3 = b7 - b0
 *
 * after conversion must merge data[1] and data[2] to get final result
 *
 *
 *
 * *********************************************************************/
#include "tank_sensor/mcp3008Spi.h"
#include <ros/ros.h>
#include "mavros_agrodrone/TankLevel.h"

#define DEFAULT_RATE 100

using namespace std;

int get_sensor_value(mcp3008Spi& mcp, int channel) {
        unsigned char data[3];
        data[0] = 1;  //  first byte transmitted -> start bit
        data[1] = 0b10000000 |( ((channel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
        data[2] = 0; // third byte transmitted....don't care

        mcp.spiWriteRead(data, sizeof(data));

        int a2dVal = 0;
        a2dVal = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
        a2dVal |=  (data[2] & 0xff);
        return a2dVal;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tank_sensor");
    ros::NodeHandle nh;

    int rate;
    nh.param("/tank_rate", rate, DEFAULT_RATE);
    ros::Rate tank_rate(rate);
    ros::Publisher tank_pub = nh.advertise<mavros_agrodrone::TankLevel>("tank_level", 10);

    mcp3008Spi a2d("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
    
    int a2dChannel = 0;

    while(ros::ok()) {
        
        int tank_raw = get_sensor_value(a2d, a2dChannel);

        mavros_agrodrone::TankLevel msg;
        msg.raw = tank_raw;

        tank_pub.publish(msg);

        ros::spinOnce();
        tank_rate.sleep();
    }

    return 0;
}

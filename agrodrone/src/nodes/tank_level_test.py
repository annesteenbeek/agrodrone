#!/usr/bin/env python

import rospy

from mavros_agrodrone.msg import TankLevel


def run():
    rospy.init_node("tank_level_test")
    rate = rospy.Rate(100)
    tank_pub = rospy.Publisher("mavros/tank_level", TankLevel, queue_size=10) 
    
    tank_level = 10
    msg = TankLevel()
    msg.percentage = tank_level
    msg.raw = 400

    while not rospy.is_shutdown():
        tank_pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    run()



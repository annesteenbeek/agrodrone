#!/usr/bin/env python

import rospy

from agrodrone.msg import CompanionMode
from mavros_agrodrone.msg import TankLevel

"""
The copter waits until it is in tracking mode.
After a set amount of time the tank level will be set to 5.
When the copter is in docked mode. The level will be set to 100.
"""

class FieldTest():
    
    def __init__(self):
        self.companion_state = None
        self.docked_string = "Docked"
        self.track_string = "TrackingSpray"
        self.rate = rospy.Rate(10)
        self.tank_time = 40 # time in secs until tank lvl is set down
        self.tank_level = self.full_tank_level = 100
        self.empty_tank_level = 5

        rospy.Subscriber("/commander/companion_mode",
                CompanionMode,
                self.companion_callback)
        self.tank_pub = rospy.Publisher("mavros/tank_level",
                TankLevel,
                queue_size=10)

    def set_tank(self, tank_level):
        msg = TankLevel()
        msg.percentage = tank_level
        msg.raw = 0
        self.tank_pub.publish(msg)

    def empty_tank(self, event):
        self.set_tank(self.empty_tank_level)
        rospy.loginfo("Emptying the tank.")

    def companion_callback(self, data):
        # go from docked -> tracking
        if self.companion_state is self.docked_string  and \
                data.state is self.track_string:
                    # create timer object that empties tank
            rospy.Timer(rospy.Duration(self.tank_time),
                        self.empty_tank,
                        True)

        # from not docked -> docked fills up the tank
        if self.companion_state is not self.docked_string and \
                data.state is self.docked_string:
            self.set_tank(self.full_tank_level)
            rospy.loginfo("Filling the tank up")
        self.companion_state = data.state

if __name__ == "__main__":
    ft = FieldTest()
    rospy.spin()

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
        self.companion_state = ""
        self.docked_string = "Docked"
        self.track_string = "TrackSpray"
        self.freq = 1 # freq of tank publisher
        self.tank_time = 40 # time in secs until tank lvl is set down
        self.tank_level = self.full_tank_level = 100
        self.empty_tank_level = 5

        rospy.init_node('field_test_node', anonymous=True)

        rospy.Subscriber("commander/companion_mode",
                CompanionMode,
                self.companion_callback)
        self.tank_pub = rospy.Publisher("mavros/tank_level",
                TankLevel,
                queue_size=10)

        rospy.Timer(rospy.Duration(1/self.freq), self.send_tank)
        rospy.loginfo("Starting field test.")

    def send_tank(self, event):
        msg = TankLevel()
        msg.percentage = self.tank_level
        msg.raw = 0
        self.tank_pub.publish(msg)

    def empty_tank(self, event):
        self.tank_level = self.empty_tank_level
        rospy.loginfo("Emptying the tank.")

    def companion_callback(self, data):
        # go from docked -> tracking
        #  rospy.loginfo("Cur state: %s, new state: %s" % (self.companion_state, data.state))
        if self.companion_state == self.docked_string  and \
                data.state == self.track_string:
                    # create timer object that empties tank
            rospy.Timer(rospy.Duration(self.tank_time),
                        self.empty_tank,
                        True)
            rospy.loginfo("Emptying tank in %d seconds." % self.tank_time)

        # from not docked -> docked fills up the tank
        if self.companion_state != self.docked_string and \
                data.state == self.docked_string:
            self.tank_level = self.full_tank_level
            rospy.loginfo("Filling the tank up.")

        self.companion_state = data.state

if __name__ == "__main__":
    ft = FieldTest()
    rospy.spin()

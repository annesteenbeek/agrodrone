#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import mavros.msg

from agrodrone.modes import Modes
from agrodrone.lib.vehicle import Vehicle

DEFAULT_CONTROL_LOOP_RATE = 100

class CommanderNode(object):
    """
    This node controls states and monitors the copter
    """

    def __init__(self, vehicle):
        rospy.init_node("commander")
        self.vehicle = vehicle

        control_loop_rate = rospy.get_param("~control_loop_rate", DEFAULT_CONTROL_LOOP_RATE)
        self.control_loop_rate = rospy.Rate(control_loop_rate)
        # initialize the modes
        self.modes = Modes(self.vehicle)

    def run(self):
        """
        Spin the loop
        """
        while not rospy.is_shutdown():
            self.modes.current_mode.run()
            self.control_loop_rate.sleep()

if __name__ == "__main__":
    vehicle = Vehicle()
    node = CommanderNode(vehicle)
    node.run()

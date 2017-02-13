#!/usr/bin/env python

import rospy
from transitions import State

class FlightState(State):
    """
    This is the helper class for the states.
    States are a subcomponent of modes, each taking care of specific task
    States are controlled by the state machine in a mode
    """

    def enter(self, event_data):
        rospy.loginfo("Entered state: %s" % self.name)
        super(FlightState, self).enter(event_data)

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.name = self.__class__.__name__     # inherit the class name as state name
        super(FlightState, self).__init__(self.name)

    def is_state_complete(self):
        """
        This function returns True when the state has completed its task
        """
        return True
        
    def run(self):
        """
        This function is called on every loop itteration of the main controll loop when this function is active
        """
        pass


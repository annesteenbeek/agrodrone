#!/usr/bin/env python

from transitions import Machine
from src.modes import Inactive, RTD, Autospray
import rospy
from agrodrone.srv import SetCompanionMode
from agrodrone.msg import CompanionMode

DEFAULT_MODE_PUBLISH_RATE = 1

class Modes(Machine):
    """
    This class holds all the modes and also functions as a state machine
    Transitions to a new mode are triggered by self.to_'mode name'()
    """
    def set_new_mode(self):
        if self.cur_mode is not None:
            rospy.loginfo("Companion mode switch: from [%s] -> [%s]"
                          % (self.cur_mode.name, self.state))
        self.cur_mode = self.states[self.state]

    def __init__(self, vehicle):
        self.cur_mode = None
        self.mode_pub = None
        self.mode_pub_rate = None
        self.prev_publish_time = None
        self.set_mode_service = None
        self.vehicle = vehicle

        modes = [
                Inactive(self.vehicle),
                RTD(self.vehicle),
                Autospray(self.vehicle)
                ]

        self.initial_state = modes[0].name
        Machine.__init__(self,
                        states=modes,
                        initial=self.initial_state,
                        after_state_change='set_new_mode')
        self.set_new_mode()
        self.setup_services()
        self.setup_publisher()
        # TODO could be included as a transition, however, this might be more
        #       resource friendly
        rospy.Timer(rospy.Duration(0.5), self.check_manual_mode_change)

    def check_manual_mode_change(self, event):
        """
        This method is used to regulary check if the fcu flight mode has been changed
        by the user manually instead of by the software self.
        If this is the case the state should be interrupted and set back to pending.
        """
        if self.vehicle.get_manual_mode_change(reset=True):
            data = lambda: None
            data.mode_to_set = "Inactive"
            self.set_companion_mode(data)


    def setup_publisher(self):
        self.mode_pub = rospy.Publisher('/commander/companion_mode', CompanionMode, queue_size=3)
        mode_pub_rate = rospy.get_param("~mode_pub_rate", DEFAULT_MODE_PUBLISH_RATE)
        self.mode_pub_rate = rospy.Duration(1/mode_pub_rate)
        self.prev_publish_time = rospy.get_rostime()

    def setup_services(self):
        self.set_mode_service = rospy.Service(
            '/commander/set_companion_mode',
            SetCompanionMode, self.set_companion_mode
            )

    def publish_mode(self):
        now = rospy.get_rostime()
        if now - self.prev_publish_time > self.mode_pub_rate:
            info = CompanionMode()
            info.mode = self.cur_mode.name
            info.state = self.cur_mode.cur_state.name
            self.mode_pub.publish(info)
            self.prev_publish_time = now

    def set_companion_mode(self, data):
        """
        Service callback to set companion computer modes
        :param data: String that represents a mode
        :return: True/False when mode switch has taken place
        """
        mode_name = data.mode_to_set
        if self.cur_mode.name is not mode_name:
            if mode_name == "Inactive":
                result = self.to_Inactive()
            elif mode_name == "RTD":
                result = self.to_RTD()
            elif mode_name == "Autospray":
                result = self.to_Autospray()
            else:
                rospy.logerr("Service mode transition: Mode (%s) not found." % mode_name)
                result = False
        else:
            rospy.logerr("Service mode transition: Already in this mode, not transitioning.")
            result = False

        return result

    def run(self):
        self.publish_mode()
        self.cur_mode.run()


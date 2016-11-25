#!/usr/bin/env python
from transitions import Machine
from src.modes import Inactive, RTD, Autospray
import rospy
from agrodrone.srv import SetCompanionMode
from agrodrone.msg import CompanionMode

DEFAULT_MODE_PUBLISH_RATE = 10


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
        self.initialState = modes[0].name
        Machine.__init__(self, states=modes, initial=self.initialState, after_state_change='set_new_mode')
        self.set_new_mode()
        self.setup_services()
        self.setup_publisher()

    def setup_publisher(self):
        self.mode_pub = rospy.Publisher('companion_mode', CompanionMode, queue_size=3)
        mode_pub_rate = rospy.get_param("~mode_pub_rate", DEFAULT_MODE_PUBLISH_RATE)
        self.mode_pub_rate = rospy.Duration(1/mode_pub_rate)
        self.prev_publish_time = rospy.get_rostime()

    def setup_services(self):
        self.set_mode_service = rospy.Service('set_companion_mode', SetCompanionMode, self.set_companion_mode)

    def publish_mode(self):
        now = rospy.get_rostime()
        if now - self.prev_publish_time > self.mode_pub_rate:
            info = CompanionMode
            info.mode = self.cur_mode.name
            info.state = self.cur_mode.cur_state.name
            self.mode_pub.publish(info)
            self.prev_publish_time = now

    def set_companion_mode(self, mode_name):
        """
        Service callback to set companion computer modes
        :param input: String that represents a mode
        :return: True/False when mode switch has taken place
        """
        if self.cur_mode.name is not mode_name:
            function_name = "to_" + mode_name
            has_function = hasattr(self, function_name)
            if has_function:
                result = eval(self, mode_name)
            else:
                rospy.logerr("Service mode transition: Mode not found.")
                result = False
        else:
            rospy.logerr("Service mode transition: Already in this mode, not transitioning.")
            result = False

        return result

    def run(self):
        self.publish_mode()
        self.cur_mode.run()





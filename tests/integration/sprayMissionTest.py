#!/usr/bin/env python

PKG = 'agro_test'

import unittest
import rospy
import mavros
import os
import sys

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from src.lib.vehicle import Vehicle
from src.nodes.commander import CommanderNode
from agrodrone.srv import SetCompanionMode

class AgrodroneSprayMissionTest(unittest.TestCase):
    """
    A test for a spray mission
    """

    def setUp(self):
        # rospy.init_node('test_node', anonymous=True)
        self.vehicle = Vehicle()
        self.commander = CommanderNode(self.vehicle)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        self.rate = rospy.Rate(10) # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()
        self.armed = False
        self.filename = 'sprayTest.mission'
        self._srv_wp_push = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
        rospy.Subscriber("")

    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def test_spray_mission(self):
        """Test offboard position control"""

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        mission_file = os.path.dirname(os.path.realpath(__file__)) + "/" + self.filename
        rospy.loginfo("reading mission %s", mission_file)
        actualMissionFile = '/home/anne/catkin_ws/src/agrodrone/tests/integration/' + self.filename
        self.assertEqual(mission_file, actualMissionFile, "Incorrect file: %s" % mission_file)
        # TODO insert waypoints from file
        mode_srv = rospy.ServiceProxy('set_companion_mode', SetCompanionMode)

        while self.vehicle.mission_list is None:
            self.rate.sleep()

        mode_srv("Autospray")
        self.assertEqual()
        # self.vehicle.set_mode("AUTO.MISSION")
        self.vehicle.set_arm()
        # self.commander.run()
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'agrodrone_spray_mission_test', AgrodroneSprayMissionTest)
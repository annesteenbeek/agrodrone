#!/usr/bin/env python

PKG = 'agro_test'

import unittest
import rospy
import mavros
import os
import sys
import time
import json

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint, WaypointList
from src.lib.vehicle import Vehicle
from src.nodes.commander import CommanderNode
from agrodrone.srv import SetCompanionMode, SetTankLevel

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
        self.filename = 'sprayTestSmall.mission'
        self._srv_wp_push = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
        # rospy.Subscriber("~companion_mode")

    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def upload_missions(self):
        actualMissionFile = '/home/anne/catkin_ws/src/agrodrone/tests/integration/' + self.filename
        wpl = []
        with open(actualMissionFile) as mission_file:
            data = json.load(mission_file)

        for item in data["items"]:
            waypoint = Waypoint()
            waypoint.command = item["command"]
            waypoint.autocontinue = item["autoContinue"]
            waypoint.frame = item["frame"]
            waypoint.x_lat = item["coordinate"][0]
            waypoint.y_long = item["coordinate"][1]
            waypoint.z_alt = item["coordinate"][2]
            wpl.append(waypoint)

        res = self._srv_wp_push(wpl)
        self.assertTrue(res.success, "mission could not be transfered" )



    def test_spray_mission(self):
        """Test offboard position control"""

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        self.upload_missions()
        mode_srv = rospy.ServiceProxy('set_companion_mode', SetCompanionMode)

        while self.vehicle.mission_list is None:
            self.rate.sleep()

        mode_srv("Autospray")
        self.vehicle.set_arm()
        startTime = rospy.Time.now()
        setTank = rospy.ServiceProxy("set_tank_level", SetTankLevel)
        tankFlag = False
        while not rospy.is_shutdown():
            self.commander.modes.run()
            if rospy.Time.now() - startTime >= rospy.Duration(30) and not tankFlag:
                tankResult = setTank(5)
                tankFlag = True
            self.rate.sleep()

        self.assertTrue(tankResult,"Tank not set to 5")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'agrodrone_spray_mission_test', AgrodroneSprayMissionTest)
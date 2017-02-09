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
from mavros_msgs.srv import WaypointPush, CommandTOL
from mavros_msgs.msg import Waypoint, WaypointList
from src.lib.vehicle import Vehicle
from src.nodes.commander import CommanderNode
from agrodrone.srv import SetCompanionMode
from mavros.mission import QGroundControlWP
from mavros_agrodrone.msg import TankLevel


# TODO complete misison test
# TODO include ardupilot auto test
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

        self.tank_pub = rospy.Publisher("mavros/tank_level", TankLevel, queue_size=10)

        self.mode_srv = rospy.ServiceProxy('/commander/set_companion_mode', SetCompanionMode)
        self.rate = rospy.Rate(10) # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()
        self.armed = False
        self.filename = 'arduMissionTest.txt'
        self._srv_wp_push = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)

    def set_tank(self, tank_level):
        msg = TankLevel()
        msg.percentage = tank_level
        msg.raw = 0
        self.tank_pub.publish(msg)


    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def upload_missions(self):
        actualMissionFile = '/home/anne/catkin_ws/src/agrodrone/agrodrone/tests/integration/' + self.filename
        wpl = []
        if self.vehicle.fcu_type == "PX4":
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
        elif self.vehicle.fcu_type == "Ardupilot":
            mission = QGroundControlWP()
            for waypoint in mission.read(open(actualMissionFile, 'r')):
                wpl.append(waypoint)
        else:
            rospy.logerr("Unknown fcu type set")

        res = self._srv_wp_push(wpl)
        return res.success


    def test_spray_mission(self):
        """Test offboard position control"""

        # hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        upload_result = self.upload_missions()
        self.assertTrue(upload_result, "mission could not be transfered")
        rospy.sleep(1)

        while self.vehicle.mission_list is None:
            self.rate.sleep()

        self.set_tank(100)
        self.vehicle.set_arm()
        rospy.sleep(1) # wait to receive arming

        #  if self.vehicle.fcu_type == "Ardupilot":
            #  self.vehicle.set_mode("GUIDED")
            #  rospy.sleep(0.5)
            #  takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            #  lat = self.vehicle.global_position.latitude
            #  lng = self.vehicle.global_position.longitude
            #  takeoff(0, 0, lat, lng, 600)
            #  rospy.sleep(10)
        self.mode_srv("Autospray")
	rospy.sleep(1)
	self.vehicle.command_mission_start()
        start_time = rospy.Time.now()
        tank_flag = False
        wait_time = rospy.Duration(25)
        while not rospy.is_shutdown():
            self.commander.modes.run()
            if rospy.Time.now() - start_time >= wait_time and not tank_flag:
                self.set_tank(5)
                tank_flag = True

            if tank_flag and \
                not self.vehicle.is_armed and \
                self.vehicle.tank_level != 100:
                # after landing, fill tank up, resume mission
                self.set_tank(100)
                self.vehicle.set_arm()

            self.rate.sleep()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'agrodrone_spray_mission_test',
                    AgrodroneSprayMissionTest)

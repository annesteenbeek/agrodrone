#!/usr/bin/env python

PKG = 'agro_test'

import unittest
import rospy
import rosbag

from numpy import linalg
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix

from src.lib.vehicle import Vehicle

class MavrosOffboardPosctlTest(unittest.TestCase):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        self.vehicle = Vehicle()
        self.rate = rospy.Rate(10) # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()
        self.armed = False


    def tearDown(self):
        #self.helper.tearDown()
        pass

    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def is_at_position(self, x, y, z, offset):
        rospy.logdebug("current position %f, %f, %f" %
                       (self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        # does it reach the position in X seconds?
        count = 1
        while count < timeout:
            # update timestamp for each published SP
            self.vehicle.set_local_setpoint([x, y, z, 0])

            # (need to wait the first few rounds until PX4 has the offboard stream)
            if not self.armed and count > 5:
                self.vehicle.set_arm()
                self.vehicle.set_mode_offboard()
                self.armed = True

            if self.is_at_position(x, y, z, 1):
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count < timeout, "took too long to get to position")

    def test_posctl(self):
        """Test offboard position control"""
        rospy.loginfo("TESTING")
        print("testing")

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        positions = (
            (0, 0, 0),
            (2, 2, 2),
            (2, -2, 2),
            (-2, -2, 2),
            (2, 2, 2))

        for i in range(0, len(positions)):
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], 180)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest)

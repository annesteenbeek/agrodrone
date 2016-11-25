#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler
import math
from agrodrone.srv import SetTankLevel
from std_msgs.msg import Header
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode, State

DEFAULT_FCU_TYPE = "PX4"
DEFAULT_CONTROL_LOOP_RATE = 100
DEFAULT_MIN_TANK_LEVEL = 10


class Vehicle(object):
    """
    Class that stores all vehicle information and handles the control
    """
    # TODO create helper class to setup subscribers, publishers etc...
    def __init__(self):
        self.position = None
        self.orientation = None
        self.is_armed = None
        self.is_offboard = None
        self.firmware = None
        self.fcu_mode = None    # mode the FCU is currently in
        self.tank_level = 100 # TODO implement tank and change to None
        self.mission_list = None
        self.offb_modes = ['OFFBOARD', 'GUIDED']
        # TODO find better method to detect fcu type
        self.fcu_type = rospy.get_param("~fcu_type", DEFAULT_FCU_TYPE)
        self.min_tank_level = rospy.get_param("~min_tank_level", DEFAULT_MIN_TANK_LEVEL)

        # TODO set stream rate
        rospy.Service('set_tank_level', SetTankLevel, self.handle_set_tank_level)
        self.setup_subscribers()
        self.setup_publishers()

    def setup_subscribers(self):
        rospy.Subscriber("mavros/local_position/pose",
                PoseStamped,
                self.position_callback)

        rospy.Subscriber("mavros/state",
                State,
                self.state_callback)

        rospy.Subscriber("mavros/missions/waypoints",
                        WaypointList,
                         self.mission_callback)

    def setup_publishers(self):
        self.location_setpoint_publisher = \
                rospy.Publisher("mavros/setpoint_position/local",
                        PoseStamped,
                        queue_size=10)

        self.velocity_setpoint_publisher = \
                rospy.Publisher("mavros/setpoint_velocity/cmd_vel",
                        TwistStamped,
                        queue_size=10)

    def handle_set_tank_level(self, data):
        if data.level <= 100 and data.level >= 0:
            self.tank_level = data.level
            result = True
        else:
            rospy.logerr("Tank level should be between 0 and 100")
            result = False
        return result


    def set_local_setpoint(self, setpoint):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with
        position x, y, z and yaw.
        """
        x, y, z, yaw_degrees = setpoint

        msg = PoseStamped()
        msg.header = Header()
        msg.header.frame_id = "setpoint_frame"
        msg.header.stamp = rospy.get_rostime()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = Quaternion(*quaternion)

        self.location_setpoint_publisher.publish(msg)

    def set_velocity_setpoint(self, velocity):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with linear
        velocities vx, vy, vz and angular velocity yaw_rate.
        """
        vx, vy, vz, yaw_rate = velocity

        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate

        self.velocity_setpoint_publisher.publish(msg)

    def mission_callback(self, data):
        self.mission_list = data.waypoints

    def position_callback(self, data):
        """
        Handle mavros/local_position/local messages.
        """
        pose = data.pose
        self.position = pose.position
        self.orientation = pose.orientation

    def state_callback(self, data):
        self.fcu_mode = data.mode
        if self.fcu_mode in self.offb_modes:
            self.is_offboard = True
        else:
            self.is_offboard = False

        if self.is_armed is not data.armed:
            if data.armed:
                self.run_on_armed()
            elif not data.armed:
                self.run_on_disarm()
        self.is_armed = data.armed

    def run_on_armed(self):
        """
        Function that runs when the vehicle arms
        """
        pass

    def run_on_disarm(self):
        """
        Function that runs when the vehicle disarms
        """
        pass

    def set_mode(self, mode):
        """
        Ask the FCU to transition to the specified custom flight mode.
        """
        set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
        set_mode(custom_mode=mode)

    def set_armed_state(self, state):
        set_armed_state = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        set_armed_state(value=state)

    def set_arm(self):
        self.set_armed_state(True)

    def set_disarm(self):
        self.set_armed_state(False)

    def set_offboard(self):
        """
        Attempt to transition into offboard mode
        """
        control_loop_rate = rospy.get_param("~control_loop_rate", DEFAULT_CONTROL_LOOP_RATE)
        rate = rospy.Rate(control_loop_rate)
        offb_mode = self.offb_modes[0] if self.fcu_type is "PX4" else self.offb_modes[1]
        # first send setpoints, else offb mode will be rejected
        for i in range(100):
            if self.is_offboard:
                break
            setpoint = [self.position.x, self.position.y, self.position.z, 0]
            self.set_local_setpoint(setpoint)
            rate.sleep()
        self.set_mode(offb_mode)


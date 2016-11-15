#!/usr/bin/env python

import rospy

import geometry_msgs.msg

from mavros.srv import CommandBool, SetMode
from mavros.msg import State


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

        # TODO set stream rate
        self.setup_subscribers()
        self.setup_publishers()

    def setup_subscribers(self):
        rospy.Subscriber("/mavros/local_position/local",
                geometry_msgs.msg.PoseStamped,
                self.position_callback)

        rospy.Subscriber("/mavros/state",
                State,
                self.state_callback)

    def setup_publishers(self):
        self.location_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint_position/local_position",
                        geometry_msgs.msg.PoseStamped,
                        queue_size=10)

        self.velocity_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",
                        geometry_msgs.msg.TwistStamped,
                        queue_size=10)

    def set_local_setpoint(self, setpoint):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with
        position x, y, z and yaw.
        """
        x, y, z, yaw = setpoint

        msg = geometry_msgs.msg.PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = yaw

        self.location_setpoint_publisher.publish(msg)

    def set_velocity_setpoint(self, velocity):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with linear
        velocities vx, vy, vz and angular velocity yaw_rate.
        """
        vx, vy, vz, yaw_rate = velocity

        msg = geometry_msgs.msg.TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate

        self.velocity_setpoint_publisher.publish(msg)

    def position_callback(self, data):
        """
        Handle mavros/local_position/local messages.
        """
        pose = data.pose
        self.position = pose.position
        self.orientation = pose.orientation

    def state_callback(self, data):
        self.fcu_mode = data.mode
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
        set_armed_state = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        set_armed_state(value=state)

    def set_arm(self):
        self.set_armed_state(True)

    def set_disarm(self):
        self.set_armed_state(False)


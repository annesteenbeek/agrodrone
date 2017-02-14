#!/usr/bin/env python

import rospy

from numpy import array, linalg
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler
import math
from mavros_agrodrone.msg import TankLevel
from std_msgs.msg import Header
from mavros import command
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, StreamRate
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode, State, ExtendedState
from sensor_msgs.msg import NavSatFix

DEFAULT_FCU_TYPE = "Ardupilot"
DEFAULT_CONTROL_LOOP_RATE = 100
DEFAULT_MIN_TANK_LEVEL = 10
DEFAULT_FULL_TANK_LEVEL = 90
DEFAULT_MISSION_START_ALTITUDE = 10
DEFAULT_STREAM_RATE = 10
DEFAULT_ACC_RAD = 1

class Vehicle(object):
    """
    Class that stores all vehicle information and handles the control
    """
    # TODO create helper class to setup subscribers, publishers etc...
    def __init__(self):
        self.position = None
        self.orientation = None
        self.global_position = None
        self.is_armed = None
        self.is_offboard = None
        self.firmware = None
        self.landed_state = None
        self.fcu_mode = None    # mode the FCU is currently in
        self.tank_level = None
        self.mission_list = None
        self.last_set_mode = None
        self.state_mutated = False
        self.offb_modes = {'PX4': 'OFFBOARD', 'Ardupilot': 'GUIDED'}
        self.RTL_modes = {'PX4': 'AUTO.RTL', 'Ardupilot': 'RTL'}
        self.mission_modes = {'PX4': 'AUTO.MISSION', 'Ardupilot': 'AUTO'}
        self.default_modes = {'PX4': 'MANUAL', 'Ardupilot': 'STABILIZE'}
        # TODO find better method to detect fcu type
        self.fcu_type = rospy.get_param("~fcu_type", DEFAULT_FCU_TYPE)
        self.min_tank_level = rospy.get_param("~min_tank_level", DEFAULT_MIN_TANK_LEVEL)
        self.full_tank_level = rospy.get_param("~full_tank_level", DEFAULT_FULL_TANK_LEVEL)
        self.mission_start_altitude = rospy.get_param("~mission_start_altitude", DEFAULT_MISSION_START_ALTITUDE)
        self.acc_rad = rospy.get_param("~acc_rad", DEFAULT_ACC_RAD)
        stream_rate = rospy.get_param("~stream_rate", DEFAULT_STREAM_RATE)

        rospy.wait_for_service("mavros/set_stream_rate")
        set_stream_rate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)
        set_stream_rate(stream_id=0,
                        message_rate=stream_rate,
                        on_off=1) 

        self.setup_subscribers()
        self.setup_publishers()

    def setup_subscribers(self):
        rospy.Subscriber("mavros/local_position/pose",
                PoseStamped,
                self.position_callback)

        rospy.Subscriber("mavros/global_position/global",
                         NavSatFix,
                         self.global_callback)

        rospy.Subscriber("mavros/extended_state",
                         ExtendedState,
                         self.extended_callback)

        rospy.Subscriber("mavros/state",
                State,
                self.state_callback)

        rospy.Subscriber("mavros/mission/waypoints",
                        WaypointList,
                         self.mission_callback)

        rospy.Subscriber("mavros/tank_level",
                       TankLevel,
                       self.tank_level_callback)

    def setup_publishers(self):
        self.location_setpoint_publisher = \
                rospy.Publisher("mavros/setpoint_position/local",
                        PoseStamped,
                        queue_size=10)

        self.velocity_setpoint_publisher = \
                rospy.Publisher("mavros/setpoint_velocity/cmd_vel",
                        TwistStamped,
                        queue_size=10)

    def tank_level_callback(self, msg):
        self.tank_level = msg.percentage

    def extended_callback(self, data):
        if data.landed_state == 0:
            self.landed_state = None
        elif data.landed_state == 1:
            self.landed_state = True
        elif data.landed_state == 2:
            self.landed_state = False

    def global_callback(self, data):
        self.global_position = data

    def has_gps(self):
        """
        This method is used to check if the vehicle currently has GPS
        Connection, this can also be used to check if mavros is connected
        correctly or if SITL is ready to be used.
        """
        return self.global_position is not None

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
        # track for state changes, this is used to fix the state change delay issue
        # for tracking a manual state change, state_mutated is reset after check
        if self.fcu_mode != data.mode:
            self.state_mutated = True

        self.fcu_mode = data.mode

        if self.fcu_mode in self.offb_modes.values():
            self.is_offboard = True
        else:
            self.is_offboard = False

        if self.is_armed is not data.armed:
            if data.armed:
                self.run_on_armed()
            elif not data.armed:
                self.run_on_disarm()
        self.is_armed = data.armed

    def get_manual_mode_change(self, reset=False, ign_guided=True):
        """
        This method checks if the mode has been changed either by
        the user or by using ROS.
        :param reset: If this is True, the check resets again after a
                        change has been detected.
        :param ign_guided: Guided mode changes are ignored.
        :return: Returns True/False if a manual mode change has been detected.
        """
        result = False
        if self.last_set_mode is not None and self.state_mutated:
            self.state_mutated = False # reset state mutation after check
            if self.fcu_mode != self.last_set_mode:
                if not (ign_guided and self.fcu_mode.upper() in self.offb_modes.values()):
                    result = True
                    rospy.loginfo("Manual mode change! mode set in ros: %s, last state received: %s" %
                                (self.last_set_mode, self.fcu_mode))
        if reset and result:
            self.last_set_mode = None
        return result

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

    def set_armed_state(self, state):
        set_armed_state = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        set_armed_state(value=state)

    def set_arm(self):
        self.set_armed_state(True)

    def set_disarm(self):
        self.set_armed_state(False)

    def set_mode(self, mode):
        """
        Ask the FCU to transition to the specified custom flight mode.
        """
        set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.last_set_mode = mode
        res = set_mode(custom_mode=mode)
        if not res.success:
            rospy.logerr("Failed mode change: to %s" % mode)
        return res.success

    def set_mode_autoland(self):
        """
        Transition to the RTL mode
        """
        RTL_string = self.RTL_modes[self.fcu_type]
        return self.set_mode(RTL_string)

    def set_mode_default(self):
        """
        Set the mode to the default mode, for pre arming etc..
        """
        mission_string = self.default_modes[self.fcu_type]
        return self.set_mode(mission_string)

    def set_mode_mission(self):
        """
        Set the FCU in follow waypoint mission mode
        """
        mission_string = self.mission_modes[self.fcu_type]
        return self.set_mode(mission_string)

    def set_mode_offboard(self):
        """
        Attempt to transition into offboard mode
        """
        control_loop_rate = rospy.get_param("~control_loop_rate", DEFAULT_CONTROL_LOOP_RATE)
        rate = rospy.Rate(control_loop_rate)
        offb_string = self.offb_modes[self.fcu_type]
        # first send setpoints, else offb mode will be rejected
        for i in range(100):
            if self.is_offboard:
                break
            setpoint = [self.position.x, self.position.y, self.position.z, 0]
            self.set_local_setpoint(setpoint)
            rate.sleep()
        return self.set_mode(offb_string)

    def command_mission_start(self):
        send_command_long = rospy.ServiceProxy("mavros/cmd/command", CommandLong)
        send_command_long(
            broadcast=False,
            command=CommandCode.CMD_MISSION_START,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

    def command_takeoff_cur(self, alt):
        """
        Takeoff at the current GPS position
        """
        if self.global_position is not None:
            ret = command.takeoff(min_pitch=0,
                yaw=0,
                latitude=self.global_position.latitude,
                longitude=self.global_position.longitude,
                altitude=alt)
            return ret.success
        else:
            rospy.logerr("Cannot do takeoff, no GPS data.")
        return False

    def do_takeoff(self, alt):
        """
        Do a takeoff at the current gps position
        """
        self.set_mode_offboard()
        self.do_takeoff(alt)


    def get_distance(self, lat, lon, alt=0, relative=True):
        """
        This method can be used to get the distance from the vehicle to a
        certain postion, in both 2d and 3d.
        """
        v_gps = self.global_position
        if alt == 0: # calculate 2d distance
            point = array((lat, lon))
            veh_point = array((v_gps.latitude, v_gps.longitude))
        else:
            if relative:
                v_alt = self.position.z
            else:
                v_alt = v_gps.altitude
            point = array((lat, lon, alt))
            veh_point = array((v_gps.latitude, v_gps.longitude, v_alt))
        return linalg.norm(point - veh_point)

#!/user/bin/env python

import rospy
from src.lib.state import FlightState
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, \
                            WaypointSetCurrent


class InterruptSpray(FlightState):
    """
    This state is called when the tank or battery is empty and the copter
    should return to dock
    """

    def enter(self, event_data):
        """
        Create a modified mission to resume later on, then set to RTL
        """
        super(FlightState, self).enter(event_data)
        # TODO store the current position,
        #  turn it into a waypoint,
        #  modify current set of waypoints, store it
        new_waypoint_list = self.modify_waypoint_list()
        push_service = rospy.ServiceProxy("mavros/mission/push",
                                WaypointPush, persistent=True)
        self.vehicle.set_mode_autoland()
        push_service(new_waypoint_list)

    def exit(self, event_data):
        """
        This function is called when the mode exits this state
        """
        set_current_service = rospy.ServiceProxy("mavros/mission/set_current",
                                                WaypointSetCurrent)
        set_current_service(self.count)
        super(FlightState, self).exit(event_data)


    def is_state_complete(self):
        """
        This should return true once the copter is safely on it's docking
        station/home position.
        Since ardupilot does not send landed state, this returns true when the
        copter is disarmed.
        """
        # TODO Improve correctness, check geo date of home etc..
        return not self.vehicle.is_armed

    def modify_waypoint_list(self):
        """
        This method is used to create a modified waypoint list that
        is uploaded to the copter so it can resume it's mission
        after refuelling.
        """
        # Create a new waypoint
        backup_point = Waypoint()
        backup_point.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        backup_point.command = CommandCode.NAV_WAYPOINT
        backup_point.is_current = True
        backup_point.autocontinue = True
        backup_point.x_lat = self.vehicle.global_position.latitude
        backup_point.y_long = self.vehicle.global_position.longitude
        new_waypoint_list = self.vehicle.mission_list

        # Add a takeoff DO command 
        takeoff_point = backup_point
        takeoff_point.command = CommandCode.NAV_TAKEOFF
        takeoff_point.is_current = False
        takeoff_point.z_alt = 15 # TODO allow manual setting control takeoff altitude

        count = 0
        for waypoint in new_waypoint_list:
            if waypoint.is_current:
                waypoint.is_current = False
                # set backup waypoint at same height as next waypoint
                backup_point.z_alt = waypoint.z_alt
                new_waypoint_list.insert(count, backup_point)
                new_waypoint_list.insert(count + 1, takeoff_point)
                break
            count += 1
        return new_waypoint_list


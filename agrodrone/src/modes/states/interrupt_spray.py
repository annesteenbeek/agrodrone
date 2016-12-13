#!/user/bin/env python

import rospy
from src.lib.state import FlightState
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, \
                            WaypointSetCurrent


class InterruptSpray(FlightState):
    """
    This state is called when the tank or battery is empty and the copter should return to dock
    """


    def enter(self, event_data):
        """
        Create a modified mission to resume later on, then set to RTL
        """
        super(FlightState, self).enter(event_data)
        # TODO store the current position,
        #  turn it into a waypoint,
        #  modify current set of waypoints, store it
        backupPoint = Waypoint()
        backupPoint.command = CommandCode.NAV_WAYPOINT
        backupPoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        backupPoint.autocontinue = True
        backupPoint.x_lat = self.vehicle.global_position.latitude
        backupPoint.y_long = self.vehicle.global_position.longitude
        newWaypointList = self.vehicle.mission_list

        self.count = 0
        for waypoint in newWaypointList:
            if waypoint.is_current:
                waypoint.is_current = False
                # set backup waypoint at same height as next waypoint
                backupPoint.z_alt = waypoint.z_alt
                newWaypointList.insert(self.count, backupPoint)
                break
            self.count += 1
        push_service = rospy.ServiceProxy("mavros/mission/push", WaypointPush, persistent=True)
        self.vehicle.set_mode_autoland()
        push_service(newWaypointList)

    def exit(self, event_data):
        """
        This function is called when the mode exits this state
        """
        set_current_service = rospy.ServiceProxy("mavros/mission/set_current", WaypointSetCurrent)
        set_current_service(self.count)
        super(FlightState, self).exit(event_data)


    def is_state_complete(self):
        """
        This function returns True when the state has completed its task
        """
        # TODO check for signal that should be send using service
        return self.vehicle.tank_level > 90


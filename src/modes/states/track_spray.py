#!/user/bin/env python

from src.lib.state import FlightState
from mavros_msgs.msg import CommandCode


class TrackSpray(FlightState):
    """
    This state is used to monitor the process of spraying
    """

    def enter(self, event_data):
        """
        This function is called when the mode enters this state
        """
        super(FlightState, self).enter(event_data)
        # TODO check if tank is connected and do pre spray checks...
        self.vehicle.set_mode("AUTO.MISSION")
        self.final_waypoint_flag = False


    def is_state_complete(self):
        """
        This method checks if the final waypoint has been passed and returns True if so
        """
        # Walk backwards from array, get first item that has CMD_ID 16 (waypoint)
        # Set final waypoint flag if it is set as current waypoint
        # If it is no longer current waypoint, it means mission is finished
        result = False
        if self.vehicle.mission_list is not None and not self.final_waypoint_flag:
            for waypoint in reversed(self.vehicle.mission_list):
                if waypoint.command == CommandCode.NAV_WAYPOINT:
                    if waypoint.is_current:
                        self.final_waypoint_flag = True
                    if not self.final_waypoint_flag and not waypoint.is_current:
                        # final waypoint has been reached
                        result = True
                    break # final waypoint has been found, exit loop
        return result


    def run(self):
        """
        This function is called on every loop iteration of the main control loop when this function is active
        """
        pass

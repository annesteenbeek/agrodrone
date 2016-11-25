#!/user/bin/env python

from src.lib.state import FlightState
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
        self.vehicle.set_mode("AUTO.RTL")



    def exit(self, event_data):
        """
        This function is called when the mode exits this state
        """
        super(FlightState, self).exit(event_data)


    def is_state_complete(self):
        """
        This function returns True when the state has completed its task
        """
        # TODO check for signal that should be send using service
        return True


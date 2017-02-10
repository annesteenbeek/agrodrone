#!/user/bin/env python

from src.lib.state import FlightState


class Docked(FlightState):
    """
    This state handles everything when the tank is on the dock
    """

    def enter(self, event_data):
        """
        Set the vehicle in default mode so it can be re-armed
        """
        super(FlightState, self).enter(event_data)
        self.vehicle.set_mode_default()

    def is_state_complete(self):
        """
        This method checks if the copter is ready for launch
        """
        if not self.vehicle.has_gps():
            return False

        full = self.vehicle.tank_level >= self.vehicle.full_tank_level
        # check if vehicle is at acceptable altitude
        # TODO maybe include altitude takeoff check in seperate state
        airborne = self.vehicle.position.z >= self.vehicle.mission_start_altitude
        return full and self.vehicle.is_armed and airborne

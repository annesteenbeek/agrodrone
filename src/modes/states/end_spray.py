#!/user/bin/env python

from src.lib.state import FlightState


class EndSpray(FlightState):
    """
    This state is used for controlling the vehicle during descend above the dock
    """


    def enter(self, event_data):
        """
        This function is called when the mode enters this state
        """
        super(FlightState, self).enter(event_data)
        self.vehicle.set_mode_autoland()


    def exit(self, event_data):
        """
        This function is called when the mode exits this state
        """
        super(FlightState, self).exit(event_data)


    def is_state_complete(self):
        """
        This function returns True when the state has completed its task
        """
        return True


    def run(self):
        """
        This function is called on every loop itteration of the main controll loop when this function is active
        """
        pass
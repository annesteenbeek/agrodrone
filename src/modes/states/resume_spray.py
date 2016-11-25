#!/user/bin/env python

from src.lib.state import FlightState


class ResumeSpray(FlightState):
    """
    This state is used for controlling the vehicle during descend above the dock
    """


    def enter(self, event_data):
        """
        Upload the altered misison to the FCU
        """
        super(FlightState, self).enter(event_data)
        #TODO upload modified mission

    def is_state_complete(self):
        """
        This method checks if the new mission has been uploaded to the FCU
        """
        # TODO return true when the new mission has been uploaded
        return True


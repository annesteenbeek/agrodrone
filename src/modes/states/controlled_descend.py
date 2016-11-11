#!/user/bin/env python

from src.lib.state import FlightState


class ControlledDescend(FlightState):
    """
    This state is used for controlling the vehicle during descend above the dock
    """


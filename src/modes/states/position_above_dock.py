#!/user/bin/env python

from src.lib.state import FlightState


class PositionAboveDock(FlightState):
    """
    This state is used to place the vehicle in the correct orientation above the dock
    """


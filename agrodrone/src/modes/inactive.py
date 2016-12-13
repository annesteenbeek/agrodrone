#!/usr/bin/env python

from src.lib.mode import Mode
from src.modes.states.pending import Pending

class Inactive(Mode):
    """
    This is the initial mode of the vehicle, it's doing nothing
    """

    states = [Pending] # first state is automatically the initial state
    transitions = []

    def __init__(self, vehicle):
        self.vehicle = vehicle
        Mode.__init__(self, self.vehicle, self.states, self.transitions)

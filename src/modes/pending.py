#!/usr/bin/env python

from agrodrone.lib import Mode
from agrodrone.modes.states import Pending

class Pending(Mode):
    """
    This is the initial mode of the vehicle, it's doing nothing
    """

    states = [Pending]
    transitions = []
    initialState = 'Pending'

    def __init__(self, vehicle):
        self.vehicle = vehicle
        Mode.__init__(self, self.vehicle, self.states, self.transitions, self.initalState)

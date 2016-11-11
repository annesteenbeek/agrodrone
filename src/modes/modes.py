#!/usr/bin/env python
from transitions import Machine
from src.modes import Inactive, RTD#, autospray


class Modes(Machine):
    """
    This class holds all the modes and also functions as a state machine
    Transitions to a new mode are triggered by self.to_'mode name'()
    """

    def set_new_mode(self):
        self.current_mode = self.states[self.state]

    def __init__(self, vehicle):
        self.current_mode = None
        self.vehicle = vehicle
        modes = [
                Inactive(self.vehicle),
                RTD(self.vehicle),
                # autospray(self.vehicle)
                ]
        self.initialState = modes[0].name
        Machine.__init__(self, states=modes, initial=self.initialState, after_state_change='set_new_mode')
        self.set_new_mode()


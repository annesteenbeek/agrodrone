#!/usr/bin/env python
from transitions import Machine
import Pending#, RTD, autospray


class Modes(Machine):
    """
    This class holds all the modes and also functions as a state machine
    Transitions to a new mode are triggered by self.to_'mode name'()
    """

    def set_cur_mode(self): 
        self.current_mode = self.get_state()

    def __init__(self, vehicle):
        self.vehicle = vehicle
        modes = [
                Pending(self.vehicle)
                # RTD(self.vehicle),
                # autospray(self.vehicle)
                ]
        Machine.__init__(self, states=modes, initial='Pending', after_state_change='set_cur_mode')
        set_cur_mode() 

#!/usr/bin/env python

from src.lib.mode import Mode
from src.modes.states import MoveAboveDock,\
    PositionAboveDock,\
    ControlledDescend,\
    LandOnDock,\
    Pending

class RTD(Mode):
    """
    This is the Return To Dock mode, where the vehicle rerturns to the docking station
    """


    def __init__(self, vehicle):
        self.vehicle = vehicle

        states = [MoveAboveDock,
                  PositionAboveDock,
                  ControlledDescend,
                  LandOnDock,
                  Pending]  # first state is automatically the initial state

        transitions = [['MoveAboveDock', 'PositionAboveDock', 'move_to_position'],
                       ['PositionAboveDock', 'ControlledDescend', 'position_to_controlled'],
                       ['ControlledDescend', 'LandOnDock', 'controlled_to_land'],
                       ['LandOnDock', 'Pending', 'land_to_pending']]

        Mode.__init__(self, self.vehicle, states, transitions)


    def move_to_position(self):
        return True

    def position_to_controlled(self):
        return True

    def controlled_to_land(self):
        return True

    def land_to_pending(self):
        return True

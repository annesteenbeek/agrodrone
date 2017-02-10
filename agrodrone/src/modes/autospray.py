#!/usr/bin/env python

from src.lib.mode import Mode
from src.modes.states import TrackSpray,\
    InterruptSpray,\
    Docked,\
    EndSpray

class Autospray(Mode):
    """
    In this mode the copter follows a given mission, is ableto return to the dock, recharge and resume the mission from where it was interrupted.
    """

    def __init__(self, vehicle):
        self.vehicle = vehicle

        # TODO currently enters at Docked, should add additional check to handle entering
        #       Autospray mode while already in air
        states = [Docked,
                  TrackSpray,
                  InterruptSpray,
                  EndSpray
                  ]  # first state is automatically the initial state

        transitions = [['TrackSpray', 'InterruptSpray', 'track_to_interrupt'],
                       ['InterruptSpray', 'Docked', 'interrupt_to_docked'],
                       ['Docked', 'TrackSpray', 'docked_to_track'],
                       ['TrackSpray', 'EndSpray', 'track_to_end']
                       ['EndSpray', 'Docked', 'end_to_dock']]

        Mode.__init__(self, self.vehicle, states, transitions)


    def track_to_interrupt(self):
        """
        Checks if the spraying should be interrupted due to low battery
        or empty tank
        :return: True/False
        """
        # TODO check if battery is empty or tank is empty
        result = False
        if self.vehicle.tank_level is not None:
            if self.vehicle.tank_level < self.vehicle.min_tank_level:
                result = True
        return result

    def interrupt_to_docked(self):
        """
        Waits for a docking complete signal to resume the spraying
        :return: True/False
        """
        # Checks the interrupt state for complete
        return self.cur_state.is_state_complete()

    def docked_to_track(self):
        """
        Checks for ready signal from resume state to make sure the new misison has been uploaded
        :return: True/False
        """
        # Checks if resume state is complete
        return self.cur_state.is_state_complete()

    def track_to_end(self):
        """
        Checks if the spray mission has been completed
        :return:
        """
        # Checks if Tracking state is complete
        return self.cur_state.is_state_complete()

    def end_to_dock(self):
        """
        Waits for a docking complete signal to resume the spraying
        :return: True/False
        """
        return self.cur_state.is_state_complete()

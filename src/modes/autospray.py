#!/usr/bin/env python

from src.lib.mode import Mode
from src.modes.states import TrackSpray,\
    InterruptSpray,\
    ResumeSpray,\
    EndSpray

class Autospray(Mode):
    """
    This is the Return To Dock mode, where the vehicle returns to the docking station
    """

    def __init__(self, vehicle):
        self.vehicle = vehicle

        states = [TrackSpray,
                  InterruptSpray,
                  ResumeSpray,
                  EndSpray
                  ]  # first state is automatically the initial state

        transitions = [['TrackSpray', 'InterruptSpray', 'track_to_interrupt'],
                       ['InterruptSpray', 'ResumeSpray', 'interrupt_to_resume'],
                       ['ResumeSpray', 'TrackSpray', 'resume_to_track'],
                       ['TrackSpray', 'EndSpray', 'track_to_end']]

        Mode.__init__(self, self.vehicle, states, transitions)


    def track_to_interrupt(self):
        """
        Checks if the spraying should be interrupted due to low battery or empty tank
        :return: True/False
        """
        # TODO check if battery is empty or tank is empty
        result = False
        if self.vehicle.tank_level is not None:
            if self.vehicle.tank_level < self.vehicle.min_tank_level:
                result = True
        return result

    def interrupt_to_resume(self):
        """
        Waits for a docking complete signal to resume the spraying
        :return: True/False
        """
        # Checks the interrupt state for complete
        return self.cur_state.is_state_complete()

    def resume_to_track(self):
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

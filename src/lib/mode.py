#!/usr/bin/env python

from transitions import Machine, State


class Mode(State, Machine):
    """
    This is the helper class for the modes.
    Each mode takes care of a feature of the copter.
    A mode consists of multiple flight states and takes care of state transitions 
    """

    def __init__(self, vehicle,
            states, # an array of State classes
            transitions, # a double list of state transitions, [[source, dest, conditions, unless],..]
            initialState
            ):
        self.name = self.__class__.__name__

        self.vehicle = vehicle

        # initialize the states
        for state in states:
            state(vehicle)

        # set a default trigger name for each transition
        for transition in transitions:
            transition.insert(0, 'attempt_transition')

        Machine.__init__(self, states, transitions, initial)
        super(Mode, self).__init__(self.name)   # initialize the State class

    def run(self):
        self.attempt_transition()
        self.state.run()


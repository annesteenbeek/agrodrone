#!/usr/bin/env python

from transitions import Machine, State


class Mode(State, Machine):
    """
    This is the helper class for the modes.
    Each mode takes care of a feature of the copter.
    A mode consists of multiple flight states and takes care of state transitions 
    """

    def set_current_state(self):
        self.current_state = self.states[self.state]

    def check_transition(self):
        """
        Handlte an attempted state transition
        """
        result = hasattr(self, 'attempt_transition')
        if result:
            self.attempt_transition()
        else:
            print("attempt_transition does not exist")

    def __init__(self, vehicle,
            states,
            transitions
            ):
        """

        :param vehicle: The vehicle object
        :param states: an array of State classes
        :param transitions: a double list of state transitions, [[source, dest, conditions, unless],..]
        """
        self.name = self.__class__.__name__
        self.current_state = None

        self.vehicle = vehicle

        # initialize the states
        activeStates = []
        for x in states:
             activeStates.append(x(vehicle))

        self.initialState = activeStates[0].name

        # set a default trigger name for each transition
        if transitions is not None:
            modified_transitions = []
            for transition in transitions:
                transition.insert(0, 'attempt_transition')
                modified_transitions.append(transition)

        Machine.__init__(self,
                         states=activeStates,
                         transitions=modified_transitions,
                         initial=self.initialState,
                         after_state_change='set_current_state')
        self.set_current_state()
        super(Mode, self).__init__(self.name)   # initialize the State class

    def run(self):
        self.check_transition()
        self.current_state.run()


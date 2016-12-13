#!/usr/bin/env python

from transitions import Machine, State


class Mode(State, Machine):
    """
    This is the helper class for the modes.
    Each mode takes care of a feature of the copter.
    A mode consists of multiple flight states and takes care of state transitions 
    """

    def set_current_state(self):
        self.cur_state = self.states[self.state]

    def check_transition(self):
        """
        Handle an attempted state transition
        """
        # TODO handle looping of states better
        exists = hasattr(self, 'attempt_transition')

        if exists and self.cur_state.name in self.events['attempt_transition'].transitions:
            self.attempt_transition()
        else:
            pass

    def enter(self, event_data):
        print("Entered mode: " + self.name)
        super(Mode, self).enter(event_data)
        # Make sure to also initialize the initial state
        self.cur_state.enter(event_data)


    def __init__(self, vehicle,
            states,
            transitions
            ):
        """
        This class is used to create a state machine for modes
        :param vehicle: The vehicle object
        :param states: an array of State classes
        :param transitions: a double list of state transitions, [[source, dest, conditions, unless],..]
        """
        self.name = self.__class__.__name__
        self.cur_state = None
        self.transitions = transitions
        self.vehicle = vehicle

        # initialize the states
        activeStates = []
        for x in states:
             activeStates.append(x(vehicle))

        self.initialState = activeStates[0].name

        # set a default trigger name for each transition
        if transitions is not None:
            for transition in transitions:
                transition.insert(0, 'attempt_transition')

        Machine.__init__(self,
                         states=activeStates,
                         transitions=self.transitions,
                         initial=self.initialState,
                         after_state_change='set_current_state')
        self.set_current_state()
        super(Mode, self).__init__(self.name)   # initialize the State class

    def run(self):
        self.check_transition()
        self.cur_state.run()


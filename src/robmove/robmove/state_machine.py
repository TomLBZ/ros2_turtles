""" Provided class: StateMachine\n
StateMachine can be inherited to perform complicated state transitions by
adding states and setting the current state.
Note that each state should set the next state to transition to as the last
action in the state function, except for the idle state provided by default.
"""

from typing import Dict, Callable, Self

class StateMachine:
    """
    `StateMachine` class provides a simple state machine implementation that:
    - allows adding states
    - allows setting the current state
    - allows running the current state
    - provides a default `idle` state
    """

    def __init__(self, name: str) -> Self:
        '''Initialize the `StateMachine` of a given `name` with an `idle` state'''
        self.states: Dict[str, Callable] = {'idle': self.idle} # state functions
        self.current: str = 'idle' # default state
        self.name: str = name # name of the state machine

    def add_state(self, name: str, state: callable) -> None:
        '''Add a new state to the `StateMachine`'''
        self.states[name] = state

    def reset_states(self) -> None:
        '''Reset the states list to only contain the idle state'''
        self.states = {'idle': self.idle}

    def set_state(self, name: str) -> None:
        '''Set the current state of the `StateMachine`'''
        if name not in self.states:
            print('[%s] State not found: %s' % (self.name, name))
            return
        self.current = name
        print('[%s] State set to %s' % (self.name, name))

    def run_state(self, **args) -> None:
        '''Run the current state function with the given arguments'''
        if self.current not in self.states:
            print('[%s] State not found: %s' % (self.name, self.current))
            return
        self.states[self.current](**args)

    def idle(self) -> None:
        '''Idle state function, doing nothing'''
        return

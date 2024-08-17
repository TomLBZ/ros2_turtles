# StateMachine class provides a simple state machine implementation that:
# - allows adding states
# - allows setting the current state
# - allows running the current state
# - provides a default idle state
class StateMachine:

    # name: string, the name of the state machine
    def __init__(self, name):
        self.states = {'idle': self.idle}
        self.current = 'idle'
        self.name = name

    # add a new state
    def add_state(self, name, state):
        self.states[name] = state

    # reset the states list
    def reset_states(self):
        self.states = {'idle': self.idle}

    # set the current state
    def set_state(self, name):
        if name not in self.states:
            print('[%s] State not found: %s' % (self.name, name))
            return
        self.current = name
        print('[%s] State set to %s' % (self.name, name))

    # run the current state function
    def run_state(self, **args):
        if self.current not in self.states:
            print('[%s] State not found: %s' % (self.name, self.current))
            return
        self.states[self.current](**args)

    # default idle state
    def idle(self):
        return

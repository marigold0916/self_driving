class statemachine:
    def __init__(self):
        self.handlers = {}
        self.startState =None
        self.endState = []
    def add_state(self, name, handler, end_state = 0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endState.append(name)
    def set_start(self,name):
        self.startState = name.upper()
    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise InitializationError("muset call .set_start() before run()")
        if not self.endState:
            raise InitializationError("at least one state must be an end_state")
        while True:
            (newState, cargo) = handler(cargo)
            if newState.upper() in self.endState:
                print("reached", newState)
                break
            else:
                handler = self.handlers[newState.upper()]
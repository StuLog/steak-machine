class XState():
    def __init__(self,outcomes = ["default"]):
        self.outcomes = outcomes
    def execute(self):
        return outcomes[0]
    def getOutcomes(self):
        return self.outcomes
    
class XStateMachine():
    def __init__(self, outcomes:list = ["default"], debugToggle:bool = False):
        self.machineOutcomes = outcomes
        self.statesDict = {}
        self.mapDict = {}
        self.startState = ""
        self.currentState = ""
        self.debug = debugToggle
    
    def addState(self, name:str, state:XState, transitions:dict):
        if name == "" or name in self.machineOutcomes or name in self.statesDict.keys():
            return False
        if self.startState == "":
            self.startState = name
            self.currentState = name
        self.statesDict[name] = state
        self.mapDict[name] = transitions
        if self.debug :
            print("added State")
        return True
    
    def __call__(self):
        if self.currentState in self.statesDict.keys():
            prevstate = self.currentState
            outcome = self.statesDict[self.currentState].execute()
            if outcome != "self":
                if outcome in self.mapDict[self.currentState].keys():
                    self.currentState = self.mapDict[self.currentState][outcome]
                else:
                    if self.debug :
                        print("Outcome Fail - not in state map", end = ' ') 
            if self.debug :
                print("prev: ",prevstate," cur: ",self.currentState," outc: ",outcome)
        else:
            if self.debug :
                return "States Stopped"
        return self.currentState, prevstate, outcome
    
    def setStartState(self,state:str):
        if state in self.statesDict.keys():
            self.startState = state
            return True
        return False
    
    def toStart(self):
        self.currentState = self.startState
        return
    
    def __str__(self):
        return self.mapDict.__str__()
    
    def getCurrentState(self):
        return self.currentState

class testA(XState):
    def __init__(self):
        super().__init__(["outcomeA"])
    def execute(self):
        return "outcomeA"
class testB(XState):
    def __init__(self):
        super().__init__(["outcomeB"])
    def execute(self):
        return "outcomeB"

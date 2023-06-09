#Import typing hint
from typing import Callable

#XState class - To be overwritten
class XState():
    def __init__(self,outcomes = ["default"]):
        self.outcomes = outcomes
    def execute(self):
        return outcomes[0]
    def getOutcomes(self):
        return self.outcomes

#VState class - To be instantiated
class VState(XState):
    def VStateEmp():
        pass
    def __init__(self,outcomes = ["default"], executeCallback:Callable = VStateEmp, debugEnable:bool = True):
        self.call = executeCallback
        self.outcomes = outcomes
    def execute(self):
        toRet = self.call()
        if type(toRet) != str:
            if debugEnable:
                print("VState error - no string returned")
            toRet = "self"
        return toRet
    def getOutcomes(self):
        return self.outcomes
    
#XStateMachine class
class XStateMachine():
    
    #Default stateChange method, does nothing
    def stateChange():
        return
    
    #Initialize XStateMachine
    #@param outcomes the list of outcomes possible for the machine as a whole
    #@param debugToggle *optional*, default False. True allows debug printing
    #@param onStateChange *optional* function to call on state change
    def __init__(self, outcomes:list = ["default"], debugToggle:bool = False, onStateChange: Callable = stateChange):
        self.machineOutcomes = outcomes
        self.statesDict = {}
        self.mapDict = {}
        self.startState = ""
        self.currentState = ""
        self.debug = debugToggle
        self.stateChange = onStateChange
    
    #Add a state to the machine
    #@param name String name of the state (MUST BE CONSISTENT WITH REDIRECTS)
    #@param state the XState subclass that will be used
    #@param transitions the dictionary redirecting outcomes from the XState to XState names (MUST BE CONSISTENT) (String:String)
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
    
    #Runs the state machine once, updating states and potentially printing debug
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
            if prevstate != self.currentState:
                self.stateChange()
        return self.currentState, prevstate, outcome
    
    #Sets the start state for the machine
    def setStartState(self,state:str):
        if state in self.statesDict.keys():
            self.startState = state
            return True
        return False
    
    #Sends the state machine to the start state
    def toStart(self):
        self.currentState = self.startState
        return
    
    #Returns a string of the mapDict 
    def __str__(self):
        return self.mapDict.__str__()
    
    #Returns the current state of the machine
    def getCurrentState(self):
        return self.currentState

#---testing classes---
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

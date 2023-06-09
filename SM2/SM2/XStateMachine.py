import matplotlib.pyplot as plt
import networkx as nx

class XState():
    def __init__(self,outcomes = ["default"]):
        self.outcomes = outcomes
    def execute(self):
        return outcomes[0]
    def getOutcomes(self):
        return self.outcomes
    
class XStateMachine():
    def __init__(self, outcomes:list = ["default"]):
        self.machineOutcomes = outcomes
        self.statesDict = {}
        self.mapDict = {}
        self.startState = ""
        self.currentState = ""
    
    def addState(self, name:str, state:XState, transitions:dict):
        if name == "" or name in self.machineOutcomes or name in self.statesDict.keys():
            return False
        if self.startState == "":
            self.startState = name
            self.currentState = name
        self.statesDict[name] = state
        self.mapDict[name] = transitions
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
                    print("Outcome Fail - not in state map", end = ' ') 
            
            print("prev: ",prevstate," cur: ",self.currentState," outc: ",outcome)
        else:
            return "States Stopped"
        return self.currentState, prevstate, outcome
    
    def setStartState(self,state:str):
        if state in statesDict.keys():
            self.startState = state
            return True
        return False
    
#     def getCurrentState()
#         return 
    def toStart(self):
        self.currentState = self.startState
        return
    
    def visual(self, size:int = 10):
        #Super broken, do not use. As such, safety thing below:
        try:
            if size < 1:
                return False

            visual = nx.MultiDiGraph()
            nums = {}
            count = 1
            for index in self.mapDict.keys():
                nums[index] = count
                count += 1
            for source in nums.keys():
                for destination in self.mapDict[source]:
                    if destination in nums.keys():
                        visual.add_edges_from([(nums[source],nums[destination])])
            plt.figure(figsize = (size,size))
            nx.draw(visual, connectionstyle='arc3, rad = 0.1')
            plt.show()
            print(nums)
        except:
            print("visualization fail, non critical")
        return
    
    def getCurrentState():
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
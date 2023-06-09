import rclpy
import ics
import cantools
import can
import threading
from rclpy.node import Node
from .XStateMachine import *
from .canFunctions import *
from custom_interfaces.msg import StanleyInfo, CANmsgs, ControlCommands



active_loop = False
# HS_LS_tx_rx_dev = open_device(1)
SC_CE_tx_rx_dev = open_device(0)

ics.override_library_name("/home/autodrive/CANbus/libicsneo/libicsneolegacy.so")
CE_db = cantools.database.load_file("/home/autodrive/CANbus/ADC_CE.dbc")
SC_db = cantools.database.load_file("/home/autodrive/CANbus/ADC_SC.dbc")
HS_db = cantools.database.load_file("/home/autodrive/CANbus/ADC_HS.dbc")
LS_db = cantools.database.load_file("/home/autodrive/CANbus/ADC_LS.dbc")


#for transmit state machine
message11 = SMessage(SC_db, 'AVState', '0x11', [0,0,0,0]) #AV.State message sent to CAN, this is the startup state in the flowchart
message337 = SMessage(CE_db, 'Park_Assist_Parallel_CE', '0x337', [0,0,0,0])
message315 = SMessage(CE_db, 'Adaptive_Cruise_Command_Ext_CE', '0x315', [0,0,0])
message2cb = SMessage(HS_db, 'PPEI_Adaptive_Cruise_Axl_Trq_Req', '0x2cb', [0,0,0,0,0])
message1e1 = SMessage(HS_db, 'PPEI_Cruise_Control_Sw_Status', '0x1e1', [0,0,0,0,0]) #Cruise State Switch

#for receive state machine
message184 = SMessage(HS_db, 'LKA_Steering_Trq_Overlay_Stat_HS', '0x184', [0,0,0,0,0,0,0,0,0])
message3e9 = SMessage(HS_db, 'PPEI_Vehicle_Speed_and_Distance', '0x3e9', [0,0,0,0,0,0,0,0,0,0,0,0])
message170 = SMessage(CE_db, 'Automatic_Braking_Status_CE', '0x170', [0,0,0,0,0,0])
messagec9 = SMessage(HS_db, 'PPEI_Engine_General_Status_1', '0xc9', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
messagef1 = SMessage(HS_db, 'PPEI_Brake_Apply_Status', '0xf1', [0,0,0,0,0,0,0,0,0])
message230 = SMessage(HS_db, 'Electric_Park_Brake_HS', '0x230', [0,0,0,0,0,0,0,0,0,0,0,0,0])
message12a = SMessage(HS_db, 'Body_Information_HS', '0x12a',  [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
message183 = SMessage(HS_db, 'Str_Ang_Ctrl_Auth_Stat_HS', '0x183', [0,0,0,0])
messagec1hs = SMessage(HS_db, 'PPEI_Driven_Whl_Rotational_Stat', '0xc1', [0,0,0,0,0,0,0,0,0,0])
messagec1ce = SMessage(HS_db, 'Driven_Whl_Rotational_Stat_CE', '0xc1', [0,0,0,0,0,0,0,0,0,0]) #TODO CHECK NUMBER OF ITEMS
message1c4 = SMessage(HS_db, 'PPEI_Torque_Request_Status', '0x1c4', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
messaged3 = SMessage(HS_db, 'PPEI_Hybrid_General_Status_3_HS', '0xd3', [0,0])
message1e5 = SMessage(HS_db, 'PPEI_Steering_Wheel_Angle', '0x1e5', [0,0,0,0,0,0,0,0,0]) #Steering wheel angle for changin - #NOTE NOT USED IN STARTUP
message7e0 = SMessage(HS_db, '7e0', '0x7e0', [7,35,64,2,35,38,0,1]) #TODONE SEND MESSAGE TO RECEIVE 7e0, e.i. in receive function = 7e8
message7e8 = SMessage(HS_db, '7e8', '0x7e8', [0])
message335 = SMessage(CE_db, 'Electric_Power_Steering_CE', '0x335', [0,0,0])



"""ROS node"""
# transmit can node
class SteakMachineNode(Node):
    def __init__(self):
        super().__init__('steak_machine')
        self.rate = self.create_rate(100)
        self.statemachine = initStateMachine(self.checkForActive,self.get_ros_time,debugToggle=True)

        

        #ControlCommands is for float32 torque              DOUBLE CHECK THIS
        #                       int16 steering_wheel_angle
        #                       float32 braking_input
        if(active_loop):
            self.control_commands_sub = self.create_subscription(ControlCommands, "/control_commands", self.control_callback, 10)  

    def control_callback(self,msg):
        #Stuart change global variables
        message337.update_data([1,1,0,msg.steering_wheel_angle])
        message315.update_data([1,2,msg.brake])
        message2cb.update_data([1,1,0,0,msg.torque])

    def run(self):
        ros_time = self.get_ros_time()
        counter = 0
       
        while rclpy.ok():
            self.statemachine()
            if(counter%3 == 0):
                message1e1.update_data(message1e1.Data,True)
                transmit_can(SC_CE_tx_rx_dev, message1e1,message1e1.Data) #Hs
                # self.get_logger().info('1e1')
            if(counter%4 == 0):
                message315.update_data(message315.Data,True)
                message2cb.update_data(message2cb.Data,True)
                transmit_can(SC_CE_tx_rx_dev, message315,message315.Data) #Hs
                transmit_can(SC_CE_tx_rx_dev, message2cb,message2cb.Data) #Hs
                # self.get_logger().info('315,2cb')
            if(counter%10 == 0):
                message11.update_data(message11.Data,True)
                transmit_can(SC_CE_tx_rx_dev, message11,message11.Data)
                # self.get_logger().info('11')

            message337.update_data(message337.Data,True)
            transmit_can(SC_CE_tx_rx_dev, message337, message337.Data)

            # self.get_logger().info('337')
            counter += 1
            self.rate.sleep()

    def get_ros_time(self):
        ros_time = (self.get_clock().now().nanoseconds)/10**6
        return ros_time
    
    def checkForActive(self):
        active_loop =  self.statemachine.getCurrentState() == "ActiveModeLoop"
    
def main(args=None):
    rclpy.init(args=args)

    steakMachineNode = SteakMachineNode()
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(steakMachineNode,), daemon=True)
    thread.start()

    try:
        steakMachineNode.run()
    except KeyboardInterrupt:
        pass

    steakMachineNode.destroy_node()
    rclpy.shutdown()
    thread.join()    


if __name__ == '__main__':
    main()



#------State Machine and States-------
# exec by Stuart - Done
class Startup(XState):
    def __init__(self):
        super().__init__(["startup_to_passive"])
    def execute(self):
        message337.update_data([0,0,0,0],True)
        message315.update_data([0,1,0],True)
        message1e1.update_data([0,0,0,0,0,0,0],True)
        message2cb.update_data([0,0,0,0,0],True)
        message11.update_data([1,0,0,0],True)
        return "startup_to_passive"

# stuart!
class Passive(XState):
    def __init__(self):
        super().__init__(["passive_to_activation_condition_check"])
        self.runOnce = False
    def execute(self):
        if not self.runOnce:
            self.lock(10000)
            self.runOnce = True
            return "self"
        return "passive_to_activation_condition_check"

# exec by Ryan
class ActivationFailure(XState):
    def __init__(self):
        super().__init__(["activation_failure_to_passive"])
    def execute(self):
        # Steering.active = False
        message337.update_data([0,0,0,0])

        # Brake.mode = No Braking
        message315.update_data([0,1,0])

        # Accel.active = False
        message1e1.update_data([0,0,0,0,0,0,0])

        # AVState.FrictionBrakeCtrlActive = False
        # AVState.SteeringCtrlActive = False
        # AVState.PropulsionCtrlActive = False
        # AVState.GlobalAutonomyStatus = Inactive
        message11.update_data([1,0,0,0])
        # never returns to self since no checks
        # return "self"

        return "activation_failure_to_passive"

# exec by DJ
class ActivationConditionCheck(XState):
    def __init__(self):
        super().__init__(["activation_condition_check_to_activation_failure","activation_condition_check_to_brake_activation"])
    def execute(self):
        checksPassed = True

        # Check Engine Run Active == True
        if messagec9.Data['EngRunAtv'] != "true":
            print("EngRunAtv not True: ",  messagec9.Data['EngRunAtv'])
            checksPassed = False

        # Check abs(SteerAngle) < 10 
        elif abs(float( message1e5.Data['StrWhAng'])) >= 10:
            print("Steering wheel is over 10 degrees: ",  message1e5.Data['StrWhAng'])
            checksPassed = False

        # Check abs(LKADrvAppldTrq) < 0.2
        elif abs(float( message184.Data['LKADrvAppldTrq'])) >= 0.2:
            print("Steering torque above 0.05 Nm: ",  message184.Data['LKADrvAppldTrq'])
            checksPassed = False

        # Check 184 Message Age < 300ms
        elif  message184.age >= 0.3:
            print("184 aged out - please wait: ",  message184.age)
            checksPassed = False

        # Check VehSpdAvgDrvn == 0
        elif float( message3e9.Data['VehSpdAvgDrvn']) != 0:
            print("Vehicle is not stopped: ",  message3e9.Data['VehSpdAvgDrvn'])
            checksPassed = False

        # Check VehSpdAvgNDrvn == 0
        elif float( message3e9.Data['VehSpdAvgNDrvn']) != 0:
            print("Vehicle is not stopped(NDrvn): ",  message3e9.Data['VehSpdAvgNDrvn'])
            checksPassed = False

        # Check 3E9 Message Age < 300ms
        elif  message3e9.age >= 0.3:
            print("3e9 aged out - please wait: ",  message3e9.age)
            checksPassed = False

        # Check BrkPedTrvlAchvd == True
        elif  messagef1.Data['BrkPedTrvlAchvd'] != "true":
            print("Brake Pedal Not Depressed: ",  messagef1.Data['BrkPedTrvlAchvd'])
            checksPassed = False

        # Check ElecPrkBrkStat == 'Released'
        elif  message230.Data['ElecPrkBrkStat'] != "Released":
            print("Parking Brake Applied: ",  message230.Data['ElecPrkBrkStat'])
            checksPassed = False

        # Check 170 Message Age is < 30ms
        elif  message170.age >= 0.03:
            print("170  message aged out - please wait: ",  message170.age)
            checksPassed = False

        # Check Driver Seatbelt Attached == True
        elif  message12a.Data['DrSbltAtc'] != "true":
            print("Please Fasten Your Seatbelt Silly: ",  message12a.Data['DrSbltAtc'])
            checksPassed = False

        # Check All Door Ajar Switches == False
        elif  message12a.Data['PDAjrSwAtv'] != "false":
            print("Passenger Door Open: ",  message12a.Data['PDAjrSwAtv'])
            checksPassed = False

        elif  message12a.Data['DDAjrSwAtv'] != "false":
            print("Driver Door Open: ",  message12a.Data['DDAjrSwAtv'])
            checksPassed = False

        elif  message12a.Data['RLDoorAjarSwAct'] != "false":
            print("Rear Left Door Open: ",  message12a.Data['RLDoorAjarSwAct'])
            checksPassed = False
            
        elif  message12a.Data['RRDoorAjarSwAct'] != "false": 
            print("Rear Right Door Open: ",  message12a.Data['RRDoorAjarSwAct'])
            checksPassed = False

        if checksPassed:
            print("All Activation Checks passed! Vehicle should proceed to activation states.") 
            return "activation_condition_check_to_brake_activation"
        else:
            return "activation_condition_check_to_activation_failure"


        

# Ryan
class BrakeActivation(XState):
    def __init__(self):
        super().__init__(["brake_activation_to_activation_failure","brake_activation_to_wait_for_brake_release"])
        self.runOnce = False
    def execute(self):
        
        if not self.runOnce:
            # AVState.GlobalAutonomyStatus = Active
            # AVState.FrictionBrakeCtrlActive = True
            message11.update_data([2,0,1,0])
            # Brake.mode = Hold
            message315.update_data([0,5,0])
            
            #lock for 1 second
            self.lock(1000)
            self.runOnce = True
            return "self"
        
        self.runOnce = False
        
        # Check $170_CE Message Age < 50ms
        if (message170.age >= 0.05):
            print("170_CE OUT OF SCOPE AGE", message170.age)

        # Check BrkSysAutBrkStat == Active [$170_CE]
        elif (message170.Data['BrkSysAutBrkStat'] != "Active"):
            print("BrkSysAut not active: ", message170.Data['BrkSysAutBrkStat'])

        # Check BrkSysExtHldCpbltyFld == False [$170_CE]
        elif (message170.Data['BrkSysExtHldCpbltyFld'] != "false"):
            print("BrkSysExtHldCpbltyFld is true: ", message170.Data['BrkSysExtHldCpbltyFld'])

        # Check FSRACCBrkngReqDenied == False [$170_CE]
        elif (message170.Data['FSRACCBrkngReqDenied'] != "false"):
            print("FRACCBrkingReqDenied is true: ", message170.Data['FSRACCBrkngReqDenied'])

        else: # everything passes so we move to Wait for Brake Release
            return "brake_activation_to_wait_for_brake_release"

        return "brake_activation_to_activation_failure"


class WaitForBrakeRelease(XState):
    def __init__(self):
        super().__init__(["wait_for_brake_release_to_activation_failure","wait_for_brake_release_to_steering_activation"])
        self.hasReleased = False
    def execute(self):
        # Wait for BrkPedTrvlAchvd == False [$F1_HS]
        if (messagef1.Data['BrkPedTrvlAchvd'] != "false" and not self.hasReleased):
            return "self"

        if not self.hasReleased:
            self.hasReleased = True
            self.lock(1000)
            return "self"
        self.hasReleased = False

        # Check VehSpdAvgDrvn == 0 [$3E9_HS]
        if (message3e9.Data['VehSpdAvgDrvn'] != 0):
            return "wait_for_brake_release_to_activation_failure"
        # else we good
        return "wait_for_brake_release_to_steering_activation"

        # need to check if time elapsed in mode is > 10s
        # for now we will ignore the 10 second timeout

class SteeringActivation(XState):
    def __init__(self):
        super().__init__(["steering_activation_to_propulsion_activation","steering_activation_to_activation_failure"])
        self.stage = 0
    def execute(self):
        
        if(self.stage == 0):
            checksPassed = True
            # Check abs(SteerAngle) < 10
            if abs(float(self.message1e5.Data['StrWhAng'])) > 10:
                print("Steering wheel is over 10 degrees: ",
                      self.message1e5.Data['StrWhAng'])
                checksPassed = False

            # Check abs(LKADrvAppldTrq) < 0.2
            elif abs(float(self.message184.Data['LKADrvAppldTrq'])) > 0.2:
                print("Steering torque above 0.05 Nm: ",
                      self.message184.Data['LKADrvAppldTrq'])
                checksPassed = False

            # Check ElecPwrStrAvalStat == Available for control
            elif self.message335.Data['ElecPwrStrAvalStat'] != "Available for control":
                print("Steering wheel is not in correct Auth State: ",
                      self.message335.Data['ElecPwrStrAvalStat'])
                return "steering_activation_to_activation_failure"

            if checksPassed:
                message337.update_data([message337.Data['StrWhlTctlFdbkReqAct'], 1, message337.Data['SWAR_ReqActV'], 0, message337.Data['StrWhlAngReqARC']])

                message11.update_data(
                    [message11.Data['GlobalAutonomyStatus'], 1, message11.Data['FrictionBrakeCtrlActive'], message11.Data['PropulsionCtrlActive']])
                self.stage = 1
                self.lock(0.5)
                return "self"
            else:
                return "steering_activation_to_activation_failure"
        else :
            self.stage = 0
            # Check ElecPwrStrAvalStat == Active
            if self.message335.Data['ElecPwrStrAvalStat'] != "Active":
                print("Steering wheel is not in correct Auth State: ",
                      self.message335.Data['ElecPwrStrAvalStat'])
                return "steering_activation_to_activation_failure"

            return "steering_activation_to_propulsion_activation"
        
    
class PropulsionActivation(XState):
    def __init__(self):
        super().__init__(["propulsion_activation_to_activation_failure","propulsion_activation_to_active_mode_loop"])
        self.stage = 0
    def execute(self):
        if self.stage == 0:
            message2cb.update_data([0,0,0,0,0])
            message1e1.update_data([1,0,0,0,3])
            self.stage = 1
            self.lock(0.1)
            return "self"
        elif self.stage == 1:
            message1e1.update_data([1,0,0,0,1])
            self.stage = 2
            self.lock(0.5)
            return "self"
        elif self.stage == 2:
            self.stage = 0
            if(message7e8.Data == 1):
                message2cb.update_data([0,1,0,0,0], True)
                message11.update_data([2,1,1,1], True)
                return "propulsion_activation_to_active_mode_loop"
        return "propulsion_activation_to_activation_failure"
        

class ActiveModeLoop(XState):
    def __init__(self):
        super().__init__(["active_mode_loop_to_deactivation"])
        self.manualTakeover = 0
    def execute(self):
        condition = np.ones(6)
        # all numbers come from the state machine
        if (self.manualTakeover):
            if (message184.Data['LKADrvAppldTrq'] > 4.5*10**(-9)):
                condition[0] = 0
            if (message183.Data['ElcPwrStrAngAuthStat'] != "Active"):
                condition[1] = 0
            if (messagef1.Data['BrkPedTrvlAchvd'] != False):
                condition[2] = 0
            if (messagec9.Data['AccActPos'] >= 0.01):
                condition[3] = 0
            if (message230.Data['ElecPrkBrkStat'] != "Released"):
                condition[4] = 0
        else:
            if (messagec1hs.age > 0.03):
                condition[0] = 0
            if (messagec1ce.age > 0.03):
                condition[1] = 0
            if (messagec9.age > 0.036):
                condition[2] = 0
            if (messagef1.age > 0.03):
                condition[3] = 0
            if (message183.age > 0.03):
                condition[4] = 0
            if (message184.age > 0.3):
                condition[5] = 0

        self.manualTakeover = not self.manualTakeover

        if (np.prod(condition) == 0):
            return "active_mode_loop_to_deactivation"
        else:
            return "self"

class Deactivation(XState):
    def __init__(self):
        super().__init__(["deactivation_to_passive"])
        self.runOnce = False
    def execute(self):
        # !!! complete? not for us
        complete = False
        if (complete and message3e9.Data['VehSpdAvgDrvn'] == 0 and not self.runOnce):
            message315.update_data([message315.Data['ACCBSCE_ACCAct'],
                                    4, message315.Data['ACCBSCE_ACCAccl']])
            #wait 2 seconds
            self.lock(2000)
            self.runOnce = True
            return "self"
        
        message337.update_data(
            [False, message337.Data['SWAR_ReqAct'], message337.Data['SWAR_ReqActV'], message337.Data['SWAR_TrgtAng'], message337.Data['StrWhlAngReqARC']])
        message315.update_data([message315.Data['ACCBSCE_ACCAct'],
                                1, message315.Data['ACCBSCE_ACCAccl']])
        message2cb.update_data(
            [False, message2cb.Data['ACCATC_ACCTyp'], message2cb.Data['ACCATC_DrvAstdGoSt'], message2cb.Data['ACCATC_SplREngInpR'], message2cb.Data['ACCATC_AxlTrqRq']])
        message11.update_data([False, False, False, False])
        
        self.runOnce = False
  
        return "deactivation_to_passive"
    
#---initStateMachine---
#Returns an initialized state machine 
def initStateMachine(onStateChange:Callable, getTime:Callable, debugToggle:bool = True):
    #Initialize state machine and add an output message
    machine = XStateMachine(["should_not_appear"], onStateChange=onStateChange, getTime=getTime,debugToggle=debugToggle) 

    #Add each state and connections to the machine
    #Startup, goes to passive
    machine.addState("Startup", Startup(), 
                transitions = {"startup_to_passive":"Passive"})
    
    #Passive, goes to ActivationConditionCheck
    machine.addState("Passive", Passive(), 
                transitions = {"passive_to_activation_condition_check":"ActivationConditionCheck"})
    
    #ActivationConditionCheck, goes to ActivationFailure and BrakeActivation
    machine.addState("ActivationConditionCheck", ActivationConditionCheck(),
                transitions = {"activation_condition_check_to_activation_failure":"ActivationFailure",
                                "activation_condition_check_to_brake_activation":"BrakeActivation"})
    
    #ActivationFailure, goes to Passive
    machine.addState("ActivationFailure", ActivationFailure(), 
                transitions = {"activation_failure_to_passive":"Passive"})
    
    #Activation, goes to BrakeActivation or WaitForBrakeRelease
    machine.addState("BrakeActivation", BrakeActivation(),
                transitions = {"brake_activation_to_activation_failure":"ActivationFailure",
                              "brake_activation_to_wait_for_brake_release":"WaitForBrakeRelease"})
    
    #WaitForBrakeRelease, goes to ActivationFailure
    machine.addState("WaitForBrakeRelease", WaitForBrakeRelease(), 
                transitions = {"wait_for_brake_release_to_activation_failure":"ActivationFailure",
                              "wait_for_brake_release_to_steering_activation":"SteeringActivation"})
    
    #SteeringActivation, goes to PropulsionActivation or ActivationFailer
    machine.addState("SteeringActivation", SteeringActivation(), 
                transitions = {"steering_activation_to_propulsion_activation":"PropulsionActivation",
                             "steering_activation_to_activation_failure":"ActivationFailure"})
    
    #PropulsionActivation, goes to ActivationFailure or ActiveModeLoop
    machine.addState("PropulsionActivation", PropulsionActivation(),
                transitions = {"propulsion_activation_to_activation_failure":"ActivationFailure",
                              "propulsion_activation_to_active_mode_loop":"ActiveModeLoop"})
    
    #ActiveModeLoop, goes to Deactivation
    machine.addState("ActiveModeLoop", ActiveModeLoop(), 
                transitions = {"active_mode_loop_to_deactivation":"Deactivation"})
    
    #Deactivation, goes to Passive
    machine.addState("Deactivation", Deactivation(), 
                transitions = {"deactivation_to_passive":"Passive"})
    
    #ensure start state is correct
    machine.setStartState("Startup")
    return machine

import rclpy
import ics
import cantools
import can
from rclpy.node import Node
from XStateMachine import XState

from canFunctions import SMessage
from canFunctions import open_device
from canFunctions import get_protect_value
from canFunctions import transmit_can

HS_LS_tx_rx_dev = open_device(1)
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

# Stuart
class Startup(XState):
    def __init__(self):
        super().__init__(["startup_to_passive"])
    def execute(self):
        #Startup code here!
        return "self"
        return "startup_to_passive"

# Michael  
class Passive(XState):
    def __init__(self):
        super().__init__(["passive_to_activation_condition_check"])
    def execute(self):
        return "self"
        return "passive_to_activation_condition_check"

# Ryan
class ActivationFailure(XState):
    def __init__(self):
        super().__init__(["activation_failure_to_passive"])
    def execute(self):
        return "self"
        return "activation_failure_to_passive"

# DJ
class ActivationConditionCheck(XState):
    def __init__(self):
        super().__init__(["activation_condition_check_to_activation_failure","activation_condition_check_to_brake_activation"])
    def execute(self):
        return "self"
        return "activation_condition_check_to_activation_failure"
        return "activation_condition_check_to_brake_activation"

class BrakeActivation(XState):
    def __init__(self):
        super().__init__(["brake_activation_to_activation_failure","brake_activation_to_wait_for_brake_release"])
    def execute(self):
        return "self"
        return "brake_activation_to_activation_failure"
        return "brake_activation_to_wait_for_brake_release"

class WaitForBrakeRelease(XState):
    def __init__(self):
        super().__init__(["wait_for_brake_release_to_activation_failure","wait_for_brake_release_to_steering_activation"])
    def execute(self):
        return "self"
        return "wait_for_brake_release_to_activation_failure"
        return "wait_for_brake_release_to_steering_activation"

class SteeringActivation(XState):
    def __init__(self):
        super().__init__(["steering_activation_to_propulsion_activation","steering_activation_to_activation_failure"])
    def execute(self):
        return "self"
        return "steering_activation_to_propulsion_activation"
        return "steering_activation_to_activation_failure"
    
class PropulsionActivation(XState):
    def __init__(self):
        super().__init__(["propulsion_activation_to_activation_failure","propulsion_activation_to_active_mode_loop"])
    def execute(self):
        return "self"
        return "propulsion_activation_to_activation_failure"
        return "propulsion_activation_to_active_mode_loop"

class ActiveModeLoop(XState):
    def __init__(self):
        super().__init__(["active_mode_loop_to_deactivation"])
    def execute(self):
        return "self"
        return "active_mode_loop_to_deactivation"

class Deactivation(XState):
    def __init__(self):
        super().__init__(["deactivation_to_passive"])
    def execute(self):
        return "self"
        return "deactivation_to_passive"
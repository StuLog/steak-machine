import ics
import cantools
import can

enable_use_server = True

class SMessage: #Class to send message, needs DBC to know which file to go from, name of the message, message id, followed by data
    def __init__(self, dbc: cantools.database.can.database.Database, message_name: string, message_id: string, Data: list):
        self.DBC = dbc
        self.MessageName = message_name
        self.ID = message_id
        self.Data = Data
        self.RollingCount = 2 #rolling count is on all messages
        self.age = 0
    def update_data(self, Data: list, update_RC: bool): 
        self.Data = Data                    #grab data
        if(update_RC):   #if message updates RC, set true and ad 1 to RC
            self.RollingCount += 1
        if(self.RollingCount > 3): #RC is only two bits (xx) so if rolling count is >3 (11) 4 = 0, (00)
            self.RollingCount = 0

def dev_name(device):
    # Return a friendly name of the device (ie. neoVI FIRE2 CY1234)
    if int("AA0000", 36) <= device.SerialNumber <= int("ZZZZZZ", 36):
        return device.Name + " " + ics.base36enc(device.SerialNumber)
    else:
        return device.Name + " " + str(device.SerialNumber)
    
def open_device(index=0):
    device = None
    if enable_use_server:
        # ics.open_device() won't open a device if we have handles open already
        # so we need to find them and specify which ones to connect to.
        devices = ics.find_devices()
        
        print("Opening Device {} (Open Client handles: {})...".format(dev_name(devices[index]), devices[index].NumberOfClients))
        ics.open_device(devices[index])
        device = devices[index]
        
    else:
        print("Opening Device...")
        device = ics.open_device()
    print("Opened Device %s." % dev_name(device))
    return device

def get_protect_value(encoded_data: list, rolling_count: int, length: int, special: string):
    Ptr = ""
    data_list = encoded_data
    #print(data_list)
    for i in range(len(data_list)-1):
        if((i ==1 or i ==2) and (special == "2cb" and len(str(bin(data_list[i])).split("b")[1]) < 2)):
            Ptr += "0"
        Ptr += str(bin(data_list[i])).split("b")[1]
    if(data_list[len(data_list) -1] < 0):
        temp = numpy.binary_repr(data_list[len(data_list) -1],length)
        Ptr += temp[(len(Ptr)):]
    else:
        for i in range(length - len(Ptr) - len(str(bin(data_list[len(data_list) -1])).split("b")[1])):
            Ptr = Ptr  + "0"
        Ptr += str(bin(data_list[len(data_list) -1])).split("b")[1]
    PrtVal = int(Ptr , 2)
    PrtVal += rolling_count
    Ptr = str(bin(PrtVal)).split("b")[1]
    if(len(Ptr) > length):
        Ptr = Ptr[1:]
    for i in range(length - len(Ptr)):
        Ptr = "0" + Ptr
    temp = list(Ptr)
    all_one = True
    for i in range(len(Ptr)):
        if(str(Ptr)[i] == "0"):
            temp[i] = "1"
        else:
            temp[i] = "0"
            all_one = False
    Ptr = "".join(temp) 
    if(all_one):
        PrtVal = 0
    else:
        PrtVal = int(Ptr , 2) + 1
    #print(Ptr)
    return(PrtVal)

def transmit_can(device: ics.ics.NeoDevice, Message: SMessage): #device: ics.ics.NeoDevice
    # print ("BEFORE, ", Message.Data)
    message = ics.SpyMessage()
    #print(type(Message))
    if(Message.ID == "0x337"):
        Message.Data[3] = int(Message.Data[3]/0.0625)
        ProtVal = get_protect_value(Message.Data[0:1],Message.RollingCount,2, "no")
        PrtVal = get_protect_value(Message.Data[1:], Message.RollingCount,18, "no")
        #print(PrtVal)
        name_message = Message.DBC.get_message_by_name(Message.MessageName)
        RC = Message.RollingCount    
        # print("Rolling Count: " + str(RC))                                                                                                                                              #[0,0,0,-1]
        data = name_message.encode({'StrWhlTctlFdbkReqActProtVal': ProtVal, 'StrWhlTctlFdbkReqAct': Message.Data[0], 'StrWhlTctlFdbkReqActRC': RC, 'SWAR_ReqAct': Message.Data[1], 'SWAR_ReqActV': Message.Data[2], 'SWAR_TrgtAng': Message.Data[3]*0.0625, 'StrWhlAngReqARC': RC , 'StrWhlAngReqPrtVal': PrtVal})
        data_string = str(data)
        data_list = data_string[2:-1].split('\\')
        # print(data_string)
        # print(data_list)
        new_data_list = []
        if len(data_list[0]) == 0:
            data_list = data_list[1:]
        Ptr = str(bin(PrtVal)).split("b")[1]
        for i in range(18 - len(Ptr)):
            Ptr = "0" + Ptr
        x = 0
        # print(data_list)
        data = []
        for i in data_list:
            if(Message.Data[3] < 0 and x == 6):
                pt = str(bin(PrtVal)).split("b")[1]
                for i in range(18 - len(pt)):
                    pt = "0" + pt
                i = hex(int(pt[10:],2))
                new_data_list.append(i.split("x")[1][0:2])
                data.append(int(new_data_list[x],16))
                x += 1
            elif(x == 4):
                i = hex(int('0000' + str(bin(RC)).split("b")[1] + Ptr[0:2] , 2))
                new_data_list.append(i.split("x")[1][0:2])
                data.append(int(new_data_list[x],16))
                x += 1
            elif(x == 2 and len(data_list[x]) == 2):
                SWAR = (Message.Data[3]*0.0625*0.0625) 
                num = str(SWAR).split('.')[0]
                num = int(num)
                attwo = str(i)
                num = str(bin(num)).split("b")[1]
                for i in range(8 - len(num)):
                    num =  "0" + num
                data.append(int(hex(int(num,2)),16))
                new_data_list.append(i)
                x += 1
                new_data_list.append(hex(ord(attwo[1:])))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(x == 2 and len(data_list[x]) == 1):
                SWAR = (Message.Data[3]*0.0625*0.0625) 
                num = str(SWAR).split('.')[0]
                num = int(num)
                num = str(bin(num)).split("b")[1]
                for i in range(8 - len(num)):
                    num =  "0" + num
                data.append(int(hex(int(num,2)),16))
                new_data_list.append(i)
                x+=1
            elif(len(i) > 3):
                new_data_list.append(i.split("x")[1][0:2])
                data.append(int(new_data_list[x],16))
                x += 1
                new_data_list.append(hex(ord(i[3:])))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(x == 5 and len(i) == 2):
                new_data_list.append(hex(int(Ptr[2:10],2)))
                data.append(int(new_data_list[x],16))
                x += 1
                new_data_list.append(hex(int(Ptr[10:],2)))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(x == 5 and len(i) == 1):
                new_data_list.append(hex(int(Ptr[2:10],2)))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) <= 1 and len(i) > 0):
                new_data_list.append(hex(ord(i)))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) > 0):
                new_data_list.append(i.split("x")[1])
                data.append(int(new_data_list[x],16))
                x += 1
        # print(data)
        # print(RC)
        data = tuple(data)
        message.NetworkID = ics.NETID_HSCAN
        Message.Data[3]=Message.Data[3]/16
        
        
        nmessage = CE_db.get_message_by_name("Park_Assist_Parallel_CE")
        nmessage = can.Message(arbitration_id=0x337, data= data)
        # print(CE_db.decode_message(nmessage.arbitration_id, nmessage.data))


    elif(Message.ID == "0x11"):
        name_message = Message.DBC.get_message_by_name(Message.MessageName)
        data = name_message.encode({'Rolling_Count': Message.RollingCount, 'GlobalAutonomyStatus': Message.Data[0], 'SteeringCtrlActive': Message.Data[1], 'FrictionBrakeCtrlActive': Message.Data[2], 'PropulsionCtrlActive': Message.Data[3]})
        data_string = str(data)
        #print(data_string)
        data_list = data_string[2:-1].split('\\')
        #print(len(data_list))
        #print(Message.RollingCount)
        if(len(data_list) <= 1):     
            data = hex(ord(data_list[0]))
        else:
            data = data_list[1].split("x")[1]
        data = tuple(int(data,16).to_bytes(1, byteorder ='big'))
        message.NetworkID = ics.NETID_HSCAN2

    elif(Message.ID == "0x315"):
        Message.Data[2] = int(Message.Data[2])
        ProtVal = get_protect_value(Message.Data,Message.RollingCount,16, "no")
        name_message = Message.DBC.get_message_by_name(Message.MessageName)
        data = name_message.encode({'ACCBSCE_ACCAct': Message.Data[0], 'ACCBSCE_AutBrkTp': Message.Data[1], 'ACCBSCE_ACCAccl': Message.Data[2], 'ACCBrkSysCmmndExtProtVal': ProtVal, 'ACCBrkSysCmmndExtRC': Message.RollingCount})
        data_string = str(data)
        data_list = data_string[2:-1].split('\\')
        # print(data_string)
        # print(data_list)
        new_data_list = []
        if len(data_list) > 7:
            data_list.pop(0)
        x = 0
        data = []
        for i in data_list:
            if(len(i) <= 1 and len(i) > 0):
                new_data_list.append(hex(ord(i)))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) > 3):
                new_data_list.append(i.split("x")[1][0:2])
                data.append(int(new_data_list[x],16))
                x += 1
                new_data_list.append(hex(ord(i[3:])))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) > 0):
                new_data_list.append(i.split("x")[1])
                data.append(int(new_data_list[x],16))
                x += 1
        data = tuple(data)
        message.NetworkID = ics.NETID_HSCAN
    elif(Message.ID == "0x7e0"):
        data = tuple(Message.Data)
        message.NetworkID = ics.NETID_HSCAN
    elif(Message.ID == "0x1e1"):
        ProtVal = get_protect_value(Message.Data[0:1],Message.RollingCount,4,"no")
        PrtVal = get_protect_value(Message.Data[4:5], Message.RollingCount,4, "no")
        #print(PrtVal)
        name_message = Message.DBC.get_message_by_name(Message.MessageName)
        RC = Message.RollingCount    
        #print("Rolling Count: " + str(RC))                                                                                                                                              #[0,0,0,-1]
        data = name_message.encode({"CrsSecSwStat": Message.Data[0], "DrvSelMdSw1Sta": Message.Data[1], "DrvSelMdSw2Sta": Message.Data[2], "DrvSelMdSw3Sta": Message.Data[3] , "DrvSelMdSwStaARC": Message.RollingCount, "CrsSpdLmtrSwStat": Message.Data[4], "CrsSpdLmtrSwStatARC": Message.RollingCount,"CrsSecSwStatARC": Message.RollingCount,"CrsSpdLmtrSwStatPVal": PrtVal,"CrsSecSwStatPVal": ProtVal})
        data_string = str(data)
        data_list = data_string[2:-1].split('\\')
        #print(data_string)
        #print(data_list)
        if len(data_list[0]) == 0:
            data_list = data_list[1:]
        x = 0
        #print(data_list)
        new_data_list = []
        data = []
        for i in data_list:
            if(len(i) > 3):
                new_data_list.append(i.split("x")[1][0:2])
                data.append(int(new_data_list[x],16))
                x += 1
                new_data_list.append(hex(ord(i[3:])))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) <= 1 and len(i) > 0):
                new_data_list.append(hex(ord(i)))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) > 0):
                new_data_list.append(i.split("x")[1])
                data.append(int(new_data_list[x],16))
                x += 1
        # print(data)
        data = tuple(data)
        message.NetworkID = ics.NETID_HSCAN
    elif(Message.ID == "0x2cb"): #1,1,0,0,torque
        # pdb.set_trace()
        # print("AFTER: ", Message.Data)
        Message.Data[4] = int(Message.Data[4]/0.125) #dividing by scale factor
        #NOTE undetermined if scale factor is applied at this stage, scaling -3 to 5k nm to the -22 - 43k CAN value
        #alternative is after encoding it is applied to shift by three decimal spaces
        # print(Message.Data)
        ProtVal = get_protect_value(Message.Data, Message.RollingCount, 25, "2cb")
        #print(PrtVal)
        name_message = Message.DBC.get_message_by_name(Message.MessageName)
        RC = Message.RollingCount

        #print("Rolling Count: " + str(RC))                                                                                                                                              #[0,0,0,-1]
        data = name_message.encode({"ACCCmndAlvRlgCnt": Message.RollingCount, "ACCATC_ACCAct": Message.Data[0], "ACCATC_ACCTyp": Message.Data[1], "ACCATC_DrvAstdGoSt": Message.Data[2] , "ACCATC_SplREngInpR": Message.Data[3], "ACCATC_AxlTrqRq": (Message.Data[4]*0.125), "ACCAxlTrqCmdProt": ProtVal})
        data_string = str(data)
        data_list = data_string[2:-1].split('\\')
        #print(data_string)
        #print(data_list)
        if len(data_list[0]) == 0:
            data_list = data_list[1:]
        x = 0
        if(Message.Data[4] > 0):
            hell0 = 0
        new_data_list = []
        data = []
        
        encoded_torque = bin(int((Message.Data[4]/8) + 22534)/0.125).split("b")
        encoded_torque1 = encoded_torque[0:2]
        encoded_torque2 = encoded_torque[3:]

        for i in data_list:
            if(len(i) > 3):
                if(x == 2):
                    torque_real = str(bin(int(Message.Data[4]/8)+22534)).split("b")[1]
                    for n in range(16 - len(torque_real)):
                        torque_real = "0" + torque_real
                    bitTwo = torque_real[3:11]
                    bitThree = torque_real[11:] + "000"
                    new_data_list.append(hex(int(bitTwo,2)))
                    data.append(int(new_data_list[x],16))
                    x += 1
                    new_data_list.append(hex(int(bitThree,2)))
                    data.append(int(new_data_list[x],16))
                    x += 1
                else:   
                    new_data_list.append(i.split("x")[1][0:2])
                    data.append(int(new_data_list[x],16))
                    x += 1
                if(x == 1):
                    torque_real = str(bin(Message.Data[4]+22534)).split("b")[1]
                    for i in range(16 - len(torque_real)):
                        torque_real = "0" + torque_real
                    bitOne = "01000" + torque_real[0:3] 
                    new_data_list.append(hex(int(bitOne,2)))
                    data.append(int(new_data_list[x],16))
                    x += 1
                elif(x!=2 and x != 4):
                    new_data_list.append(hex(ord(i[3:])))
                    data.append(int(new_data_list[x],16))
                    x += 1
            elif(len(i) <= 1 and len(i) > 0):
                new_data_list.append(hex(ord(i)))
                data.append(int(new_data_list[x],16))
                x += 1
            elif(len(i) == 2):
                new_data_list.append(hex(ord(i[0:1])))
                data.append(int(new_data_list[x],16))
                x += 1
                if(x == 1):
                    torque_real = str(bin(Message.Data[4]+22534)).split("b")[1]
                    for i in range(16 - len(torque_real)):
                        torque_real = "0" + torque_real
                    bitOne = "01000" + torque_real[0:3]
                    new_data_list.append(hex(int(bitOne,2)))
                    data.append(int(new_data_list[x],16))
                    x += 1
                else:
                    new_data_list.append(hex(ord(i[1:])))
                    if(int(new_data_list[x],16) == 66):
                        data.append(64)
                    else:
                        data.append(int(new_data_list[x],16))
                    x += 1
            elif(len(i) > 0):
                new_data_list.append(i.split("x")[1])
                data.append(int(new_data_list[x],16))
                x += 1

        # data = tuple(data)
        # print(data)
        data = data[0:4]
        # print("HERE", data)
        # data_bin = data[0:4]
        # for i in range(len(data)):
        #     data[i] = bin(data[i]).split("b")[1]
       
        # data = data[0] + data[1] + data[2] + data[3]     
        # #print(data)

        # data = int(data, 2)
        # Message.Data = data

        temp = int((Message.Data[4]/8+22534)/0.125)
        # print([Message.Data[0:3],temp])
        PrtVal = get_protect_value([Message.Data[0],Message.Data[1],Message.Data[2],Message.Data[3],temp], Message.RollingCount, 25, "2cb")
        print(PrtVal, Message.Data[4], temp)
        PrtVal = bin(PrtVal).split("b")[1]
        PrtVal = PrtVal.zfill(32)
        PrtValBin = [PrtVal[0:8], PrtVal[8:16], PrtVal[16:24], PrtVal[24:]]

      
        for i in range(4):
            data.append(int(PrtValBin[i],2))
        


        # print(PrtVal)
    
        # data.append(PrtValBin)
        # print(data)
        print("CHECK PROTECTION HERE: RC:", RC, " DATA: ",data)
        data = tuple(data)
        # print(data)
        # print("HERE", data)
        # Message.Data = tuple(data[0:4])

        message.NetworkID = ics.NETID_HSCAN
        nmessage = HS_db.get_message_by_name("PPEI_Adaptive_Cruise_Axl_Trq_Req")
        nmessage = can.Message(arbitration_id=0x2cb, data= data)
        Message.Data[4] = Message.Data[4]/8
        # print(HS_db.decode_message(nmessage.arbitration_id, nmessage.data))

    #print("Data 2 = " + str(data))
    message.ArbIDOrHeader = int(Message.ID,16) # CAN Arbitration ID
    message.Data = data
    # First channel of CAN on the device
    # msg parameter here can also be a tuple of messages
    # print(message.NetworkID)
    
    # nmessage = SC_db.get_message_by_name("AVState")
    # nmessage = can.Message(arbitration_id=0x11, data= data)(i[1:],16) for i in data_list[1:]]
    # print(SC_db.decode_message(nmessage.arbitration_id, nmessage.data))
    if Message.ID == "0x2cb":
        print("BEFORE", time.time())
    ics.transmit_messages(device, message)
    if Message.ID == "0x2cb":
        print("AFTER", time.time())
    #print("sucess")
    # StrWhlTctlFdbkReqActRC +=1
        # q-=1
        # print(q)        # print(q)
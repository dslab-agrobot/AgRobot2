
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaR
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
import rospy
from modbus_jxiot.modbus_wrapper_client import ModbusWrapperClient
from std_msgs.msg import Int32MultiArray as HoldingRegister
from copy import deepcopy
from enum import Enum

class operateDecorator (object):
    def __init__(self,registers_per_axis,ADDRESS_READ_START,ADDRESS_WRITE_START,oper) -> None:
        """Static parameters 

        Args:
            registers_per_axis (uint16): 每轴x个寄存器
            
            ADDRESS_READ_START (uint16): 读取的寄存器的基地址
            
            ADDRESS_WRITE_START (uint16): 写入的寄存器的基地址
            
            oper (uint8): 0:只读 1:只写 2:读写
        """
        
        self.registers_per_axis = registers_per_axis  
        self.ADDRESS_READ_START = ADDRESS_READ_START  
        self.ADDRESS_WRITE_START = ADDRESS_WRITE_START  
        self.oper = oper  

class operationTable(Enum):
    move_abs = operateDecorator(2,12160,12160,2)
    read = operateDecorator(6,12000,None,0)
    stop = operateDecorator(2,None,12096,1)
    

# MD-9DI6SM-TCP
class D9ModbusClient(ModbusWrapperClient):
    def __init__(self,host,port=502,rate=50,reset_registers=True):
        """
            :param host: Contains the IP adress of the modbus server
            :type host: string
            :param rate: How often the registers on the modbusserver should be read per second
            :type rate: float
            :param reset_registers: Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
            :type reset_registers: bool
        """
#         print("Use the appropriate Step7 Project to enable the Modbus Server on your Siemens S1200 PLC")

        # init wrappered ModbusWrapperClient object
        modbusWrapperClient = ModbusWrapperClient()
        modbusWrapperClient.__init__(self,host,port,rate,ADDRESS_READ_START = 0,ADDRESS_WRITE_START = 0,NUM_REGISTERS = 0,reset_registers = reset_registers,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
        self._modbusWrapperClient = modbusWrapperClient
        self.pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=50)
        # start listening whether reading values isn't equaled with writing values
        # modbusWrapperClient.startListening()
        #self.startListening()

    def getModbusWrapperClient(self):
        return self._modbusWrapperClient

    def _pub_to(self,msg):
        # 设置循环的频率
        rate = rospy.Rate(20)
        # 定义循环次数
        roate = 1

        rospy.loginfo("Sending arrays to the modbus server")
        while not rospy.is_shutdown():
            self.pub.publish(msg)
            rospy.loginfo('read_holding_registers %s',str(msg.data))
            rate.sleep()
            roate -= 1
            if roate < 1:
                #client.write_registers(address=12160,values=[0])
                break 

    def _norm_xyz(values):
        values = deepcopy(values)
        # tramsform list like [x,y,z] into [0,x,0,x,0,y,0,z]
        # DEBUG, change here if you tring on few axis
        values = [values[0]] + values
        for v in range (0,len(values)):
            values.insert(v*2,0)
        return values


    # 多轴状态读取
    def multiAxisStateRead(self,address_read_start,num_registers):
        rospy.loginfo("Reading status")
        result = self._modbusWrapperClient.readRegisters(address_read_start,num_registers)
        msg = HoldingRegister()
        self._pub_to(msg)
           

    # 多轴绝对位置移动
    def multiAxisAbsoluteMove(self,values:list):
        """Absolute moving

        Args:
            values (list): x, y and z in a list, e.g. [200,10,546]
        """
        rospy.loginfo("Absolute move")
        
        num_registers = len(values) # 3
        address_read_start = operationTable.move_abs.value.ADDRESS_READ_START
        address_write_start = operationTable.move_abs.value.ADDRESS_WRITE_START
        modbusWrapperClient = self.getModbusWrapperClient()

        
        values = self._norm_xyz(values)
            
        # 写入寄存器数据
        modbusWrapperClient.writeRegisters(address=address_write_start,values=values)

        # 读取寄存器数据
        result = modbusWrapperClient.readRegisters(address_read_start,num_registers)
        while result != values:
            rospy.sleep(1)
            result = modbusWrapperClient.readRegisters(address_read_start,num_registers)
        msg = HoldingRegister()
        msg.data = result
        
        self._pub_to(msg)
    
    # 多轴停止控制
    def multiAxisStopControl(self,values:list):
        """Stop, 0 for stop 

        Args:
            values (list): x, y and z in a list, e.g. [0,1,0]
        """
        rospy.loginfo("STOP")
        address_write_start = operationTable.stop.value.ADDRESS_WRITE_START
        
        modbusWrapperClient = self.getModbusWrapperClient()
        values = self._norm_xyz(values)
        
        # 停止的轴写入0
        modbusWrapperClient.writeRegisters(address=address_write_start,values=values)
        print("client.write_registers(address=12096,values=values):",self.writeRegisters(address=address_write_start,values=values))
        

    

    
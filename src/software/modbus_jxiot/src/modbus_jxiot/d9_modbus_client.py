
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
        modbusWrapperClient.__init__(self,host,port,rate,ADDRESS_READ_START = 0,ADDRESS_WRITE_START = 0,NUM_REGISTERS = 0,reset_registers = True,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
        self._modbusWrapperClient = modbusWrapperClient
        # start listening whether reading values isn't equaled with writing values
        # modbusWrapperClient.startListening()
        #self.startListening()

    def getModbusWrapperClient(self):
        return self._modbusWrapperClient

    # 多轴状态读取
    def multiAxisStateRead(self,pub,address_read_start,num_registers):
        rospy.loginfo("多轴状态读取")

        # 读取寄存器数据
        modbusWrapperClient = self.getModbusWrapperClient()
        result = modbusWrapperClient.readRegisters(address_read_start,num_registers)
        msg = HoldingRegister()
        msg.data = result

        # 设置循环的频率
        rate = rospy.Rate(20)
        # 定义循环次数
        roate = 1

        rospy.loginfo("Sending arrays to the modbus server")
        while not rospy.is_shutdown():
            pub.publish(msg)
            rospy.loginfo('read_holding_registers %s',str(msg.data))
            rate.sleep()
            roate += 1
            if roate > 50:
                #client.write_registers(address=12160,values=[0])
                break    

    # 多轴绝对位置移动
    def multiAxisAbsoluteMove(self,pub,address_read_start,num_registers,address_write_start,values):
        rospy.loginfo("多轴绝对位置移动")

        modbusWrapperClient = self.getModbusWrapperClient()

        # 写入寄存器数据
        modbusWrapperClient.writeRegisters(address=address_write_start,values=values)
        #print("client.write_registers(address=12160,values=values):",modclient.writeRegisters(address=12160,values=values))

        # 读取寄存器数据
        result = modbusWrapperClient.readRegisters(address_read_start,num_registers)
        while result != values:
            rospy.sleep(1)
            result = modbusWrapperClient.readRegisters(address_read_start,num_registers)
        msg = HoldingRegister()
        msg.data = result
        # output2 = HoldingRegister()
        # output2.data = range(40,20,-1)
        
        # 设置循环的频率
        rate = rospy.Rate(20)
        # 定义循环次数
        roate = 1

        rospy.loginfo("Sending arrays to the modbus server")
        while not rospy.is_shutdown():
            pub.publish(msg)
            rospy.loginfo('read_holding_registers %s',str(msg.data))
            rate.sleep()
            roate += 1
            if roate > 50:
                #client.write_registers(address=12160,values=[0])
                break
    
    # 多轴绝对位置移动
    # def multiAxisAbsoluteMoveWithDefinedAxis():

    # 多轴停止控制
    def multiAxisStopControl(self,address_write_start,values):
       rospy.loginfo("多轴停止控制")
       
       modbusWrapperClient = self.getModbusWrapperClient()

       # 停止的轴写入0
       modbusWrapperClient.writeRegisters(address=address_write_start,values=values)
       print("client.write_registers(address=12096,values=values):",self.writeRegisters(address=address_write_start,values=values))
        

    

    
#!/usr/bin/env python3
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
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.

import rospy
# from modbus.modbus_wrapper_client import ModbusWrapperClient 
from modbus_jxiot.modbus_wrapper_client import ModbusWrapperClient 
# from pyModbusTCP.client import ModbusClient
from std_msgs.msg import Int32MultiArray as HoldingRegister
from pymodbus.register_read_message import ReadInputRegistersResponse
from modbus_jxiot.msg import absolute_mov 
from modbus_jxiot.msg import readable_address
from modbus_jxiot.msg import stop_controlling

if __name__=="__main__":
    rospy.init_node("modbus_client")
    rospy.loginfo("""
    This file shows the usage of the Modbus Wrapper Client.
    To see the read registers of the modbus server use: rostopic echo /modbus_wrapper/input
    To see sent something to the modbus use a publisher on the topic /modbus_wrapper/output
    This file contains a sample publisher.
    """)

    # 默认参数
    host = "192.168.1.222"
    port = 502
    position = 15000
    com = 3
    operation = "absolute_mov"
    oper = 2

    # 获取初始化参数
    if rospy.has_param("~ip"):
        host = rospy.get_param("~ip")
    else:
        rospy.loginfo("For not using the default IP %s, add an arg e.g.: '_ip:=\"192.168.1.222\"'",host)
    if rospy.has_param("~port"):
        port = rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=502'",port)
    if rospy.has_param("~position"):
        position = rospy.get_param("~position")
    else:
        rospy.loginfo("For not using the default position %d, add an arg e.g.: '_position:=15000'",position)
    if rospy.has_param("~com"):
        com = rospy.get_param("~com")
    else:
        rospy.loginfo("For not using the default com %d, add an arg e.g.: '_com:=3'",com)
    if rospy.has_param("~operation"):
        operation = rospy.get_param("~operation")
    else:
        rospy.loginfo("For not using the default operation %d, add an arg e.g.: '_operation:=readable_address'",operation)
    if rospy.has_param("~oper"):
        oper = rospy.get_param("~oper")
    else:
        rospy.loginfo("For not using the default oper %d, add an arg e.g.: '_oper:=2'",oper)

    # 根据字符串参数获取消息类
    # getClass = globals()[operation]  oper_class = getClass()
    # oper_class = globals()[operation]
    if operation == "absolute_mov":
        # print("here")
        num_registers = absolute_mov.registers_per_axis*com
        address_read_start = absolute_mov.ADDRESS_READ_START  
        address_write_start = address_read_start
    elif operation == "readable_address":
        num_registers = readable_address.registers_per_axis*com
        address_read_start = readable_address.ADDRESS_READ_START  
        address_write_start = address_read_start
    elif operation == "stop_controlling":
        num_registers = stop_controlling.registers_per_axis*com
        address_read_start = stop_controlling.ADDRESS_READ_START  
        address_write_start = address_read_start

    # setup modbus client：启动modbus客户端    
    modclient = ModbusWrapperClient(host,port=port,rate=50,ADDRESS_READ_START = address_read_start,
                                    ADDRESS_WRITE_START=address_write_start,NUM_REGISTERS=num_registers,
                                    reset_registers=True,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    # 设置应读取的寄存器的起始地址及寄存器数量；
    modclient.setReadingRegisters(address_read_start,num_registers)
    print("modclient.getReadingRegisters():",modclient.getReadingRegisters())
    # 设置可写寄存器的起始地址及寄存器数量;
    modclient.setWritingRegisters(address_write_start,num_registers)
    print("modclient.getWritingRegisters():",modclient.getWritingRegisters())
    rospy.loginfo("Setup complete")
    
    # start listening to modbus and publish changes to the rostopic：开始监听modbus并发布对rostopic的更改；
    modclient.startListening(oper)
    rospy.loginfo("Listener started")
    
    #################
    # Example 2
    # Create a listener that show us a message if anything on the readable modbus registers change
    # 创建一个监听器，如果可读modbus寄存器上的任何内容发生更改，则向我们显示一条消息 
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")
    def showUpdatedRegisters(msg):
        rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))
    sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=50)
    #################
    
    #################
    # Example 3
    # writing to modbus registers using a standard ros publisher
    # 使用标准ros发布者写入modbus寄存器
    pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=50)

    # 多轴状态读取
    if oper == 0:   
        print("readable_address")
        # 读取寄存器数据
        result = modclient.readRegisters(address_read_start,num_registers)
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
    # 多轴停止控制
    elif oper == 1:
        print("stop_controller")
        # 停止的轴写入0
        values = [0,0]*com
        modclient.writeRegisters(address=address_write_start,values=values)
        print("client.write_registers(address=12096,values=values):",modclient.writeRegisters(address=address_write_start,values=values))
    # 多轴绝对位置移动
    elif oper == 2:
        # 对position进行合法判断
        try: 
            if 0 < position < 20200:        
                rospy.loginfo("position判断合法")
        except Exception as e:
            rospy.logwarn("position不合法")
            raise e
        
        print("absolute_mov")
        # 写入寄存器数据
        values = [0,position]*com
        modclient.writeRegisters(address=address_write_start,values=values)
        #print("client.write_registers(address=12160,values=values):",modclient.writeRegisters(address=12160,values=values))

        # 读取寄存器数据
        result = modclient.readRegisters(address_read_start,num_registers)
        while result != values:
            rospy.sleep(1)
            result = modclient.readRegisters(address_read_start,num_registers)
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
    
    modclient.stopListening(oper)
        
        
    
        
        

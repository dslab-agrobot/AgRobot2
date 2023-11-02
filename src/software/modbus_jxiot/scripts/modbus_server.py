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
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,****** OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.

import rospy
from modbus.modbus_wrapper_server import ModbusWrapperServer
from std_msgs.msg import Int32MultiArray as HoldingRegister
from modbus.msg import readable_address
from modbus.msg import relative_mov
from modbus.msg import absolute_mov
from modbus.msg import stop_controlling

NUM_REGISTERS = absolute_mov.registers_per_axis*4    # COM3的寄存器编号
ADDRESS_READ_START = absolute_mov.ADDRESS_READ_START    # 寄存器的基地址
# 计算出寄存器的写入数据的地址并赋予自定义消息属性
absolute_mov.ADDRESS_WRITE_START = ADDRESS_READ_START+NUM_REGISTERS     
ADDRESS_WRITE_START = absolute_mov.ADDRESS_WRITE_START
    
if __name__=="__main__":
    rospy.init_node("modbus_server")
    #port = 1234 # custom modbus port without requirement of sudo rights：自定义modbus端口，无需sudo权限
    port = 502 # default modbus port：默认modbus端口
    if rospy.has_param("~port"):
        port =  rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=502'",port)
    # Init modbus server with specific port：初始化具有特定端口的modbus服务器
    mws = ModbusWrapperServer(port)
    print("mws = ModbusWrapperServer(port)")
    # Stop the server if ros is shutdown. This should show that the server is stoppable：如果ros关闭，请停止服务器。这应该表明服务器可以停止 
    rospy.on_shutdown(mws.stopServer)
    # Starts the server in a non blocking call：在非阻塞调用中启动服务器
    mws.startServer()
    print("Server started")
    
    ###############
    # Example 1
    # write to the Discrete Input：写入离散输入
    mws.setDigitalInput(ADDRESS_WRITE_START,1) # args: address , value. sets address to value
    # Example 2
    # read from clients coil output：从客户端读取线圈输出
    print("waiting for line 0 to be set to True") 
    result = mws.waitForCoilOutput(ADDRESS_READ_START,5) # args: address,timeout in sec. timeout of 0 is infinite. waits until address is true
    print("result:",result)
    if result:
        print("got line 0 is True from baxter")
    else:
        print("timeout waiting for signal on line 0")
    
    
    ###############
    # Example 3
    # Listen for the writeable modbus registers in any node：监听任何节点中的可写modbus寄存器:回调函数
    def callback(msg):
        rospy.loginfo("Modbus register have been written: %s",str(msg.data))
        rospy.sleep(2)
    sub = rospy.Subscriber("modbus_server/read_from_registers",HoldingRegister,callback,queue_size=100) 
    ###############
    
    
    ###############
    # Example 4
    # Publisher to write first NUM_REGISTERS modbus registers from any node：Publisher从任何节点写入前NUM_REGISTERS个modbus寄存器
    pub = rospy.Publisher("modbus_server/write_to_registers",HoldingRegister,queue_size=100)
    rospy.sleep(1)
    msg = HoldingRegister()
    msg.data = range(NUM_REGISTERS)
    msg2 = HoldingRegister()
    msg2.data = range(NUM_REGISTERS,0,-1)
 
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.sleep(1)
        pub.publish(msg2)
        rospy.sleep(1)
    ################
    rospy.spin()
        
    mws.stopServer()

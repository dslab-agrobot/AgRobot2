#!/usr/bin/env python3

import rospy
from wrapper_client.d9_modbus_client import D9ModbusClient, OperationTable
from pymodbus.register_read_message import ReadInputRegistersResponse
from std_msgs.msg import Int32MultiArray as HoldingRegister
from modbus_tcp.msg import slider as msg_slider



if __name__=="__main__":
    rospy.init_node("modbus_subscriber_client")
     # 默认参数
    host = "192.168.1.222"
    port = 502
    rospy.loginfo("启动！")

    # 初始化需要订阅的消息变量参数
    operation = None
    p1 = 0
    p2 = 0
    p3 = 0
    rospy.loginfo("Received subscriber message: operation:%s p1:%d p2:%d p3:%d", operation,p1,p2,p3) 

    def callback_subscriber(msg):
        global operation,p1,p2,p3 
        operation = msg.operation
        p1 = msg.p1
        p2 = msg.p2
        p3 = msg.p3
        rospy.loginfo("Received subscriber message: operation:%s p1:%d p2:%d p3:%d", operation,p1,p2,p3)     
        
    rospy.Subscriber("subscriber/input",msg_slider,callback_subscriber,queue_size=50)
    
    # 循环监听订阅到的消息
    rospy.loginfo("运行！")
    # 启动modbus客户端     
    client = D9ModbusClient(host=host,port=port)
    # rospy.loginfo("运行！")

    # 创建一个监听器，如果可读modbus寄存器上的任何内容发生更改，则向我们显示一条消息 
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")
    def showUpdatedRegisters(msg):
        rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))
    sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=50) 

    if operation == "multiAxisAbsoluteMove":
        values = [p1,p2,p3] # [x,y,z]
        # print(type(values),values)
        for v in values:
            if v<0 or v > 20200:
                raise Exception("Wrong move pos as ", values)
        # rospy.loginfo("yunxing !")
        client.multiAxisAbsoluteMove(values=values)
        rospy.loginfo("yunxing !")


    elif operation == "multiAxisStateRead":
        address_read_start = p1 
        num_registers = p2
        client.multiAxisStateRead(address_read_start, num_registers)
    elif operation == "multiAxisStopControl":
        values = [p1,p2,p3] # [x,y,z]
        client.multiAxisStopControl(values)

    rospy.spin()
    
   
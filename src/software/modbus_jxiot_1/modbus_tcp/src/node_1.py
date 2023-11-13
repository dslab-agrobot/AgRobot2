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

    # 初始化需要订阅的消息变量参数
    operation = None
    
    # 启动modbus客户端  
    client = D9ModbusClient(host=host,port=port)
    print("启动modbus客户端")
   
    def callback_subscriber(msg):
        operation = msg.operation
        p1 = msg.p1
        p2 = msg.p2
        p3 = msg.p3
        rospy.loginfo("Received subscriber message: operation:%s p1:%d p2:%d p3:%d", operation,p1,p2,p3)   

        # multiAxisAbsoluteMove
        if operation == 1:
            values = [p1,p2,p3] # [x,y,z]
            # print(type(values),values)
            for v in values:
                if v<0 or v > 20200:
                    raise Exception("Wrong move pos as ", values)

            client.multiAxisAbsoluteMove(values=values)
        
        # multiAxisStateRead
        elif operation == 2:
            address_read_start = p1 
            num_registers = p2
            client.multiAxisStateRead(address_read_start, num_registers)
        
        # multiAxisStopControl
        elif operation == 3:
            values = [p1,p2,p3] # [x,y,z]
            client.multiAxisStopControl(values) 

    sub_input = rospy.Subscriber("subscriber/input",msg_slider,callback_subscriber,queue_size=50)
      
    rospy.spin()
    


    
   
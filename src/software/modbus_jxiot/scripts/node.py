import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from modbus_jxiot.msg import slider

if __name__ == '__main__':
    rospy.init_node('modbus_subscriber_client')

    host = '192.168.1.222'
    port = 502

    client = BaseModbusClient(host)
    print("启动modbus客户端")
   
    def callback_subscriber(msg):
        operation = msg.operation
        p1 = msg.p1
        p2 = msg.p2
        p3 = msg.p3
        rospy.loginfo("Received subscriber message: operation:%s p1:%d p2:%d p3:%d", operation,p1,p2,p3)   

        # multiAxis_AbsoluteMove
        if operation == 1:
            values = [p1,p2,p3] # [x,y,z]
            for v in values:
                if v<0 or v > 20200:
                    raise Exception("Wrong move pos as ", values)

            client.multiAxis_AbsoluteMove(values=values)
        
        # multiAxis_StateRead
        elif operation == 2:
            client.multiAxis_StateRead()

        # multiAxis_EMERGENCYSTOP  所有轴的急停
        elif operation == 3:
            client.multiAxis_EMERGENCYSTOP()
        
        # multiAxis Stop
        elif operation == 4:
            values = [p1,p2,p3] # [x,y,z]
            client.multiAxis_Stop(values=values)
        
        # multiAxis_Origin_all
        elif operation == 5:
            client.multiAxis_Origin_all()

        # multiAxis_Origin
        elif operation == 6:
            values = [p1,p2,p3] # [x,y,z]
            client.multiAxis_Stop(values=values)

    sub_input = rospy.Subscriber("sub_input",slider,callback_subscriber,queue_size=50)
      
    rospy.spin()






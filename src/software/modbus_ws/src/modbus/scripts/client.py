import rospy
import pymodbus
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from pymodbus.register_read_message import ReadInputRegistersResponse
# from pyModbusTCP.client import ModbusClient
from std_msgs.msg import Int32MultiArray as HoldingRegister
from std_msgs.msg import String as Operation # 1、multiAxisAbsoluteMove  2、multiAxisStateRead  3、multiAxisStopControl
from std_msgs.msg import Int32 as Position # 0~20200
from std_msgs.msg import Int32 as Axis   # 0~5
from modbus.msg import absolute_mov 
from modbus.msg import readable_address
from modbus.msg import stop_controlling

if __name__=="__main__":
    rospy.init_node("modbus_subscriber_client")
    
     # 默认参数
    host = "192.168.1.222"
    port = 502

    # 初始化需要订阅的消息变量参数
    position = 0
    operation = None
    axis = 0

    def callback_position(msg):
        global position
        position = msg.data
        rospy.loginfo("Received position message: %d", msg.data)     

    def callback_operation(msg):
        global operation
        operation = msg.data
        rospy.loginfo("Received operation message: %s", msg.data)

    def callback_axis(msg):
        global axis
        axis = msg.data
        rospy.loginfo("Received axis message: %d", msg.data)     
        

    rospy.Subscriber("position/input",Position,callback_position,queue_size=50)
    rospy.Subscriber("operation/input",Operation,callback_operation,queue_size=50)
    rospy.Subscriber("axis/input",Axis,callback_axis,queue_size=50)

    # 循环监听订阅到的消息
    rospy.spin()

    if operation == "multiAxisAbsoluteMove":
        num_registers = absolute_mov.registers_per_axis*axis
        address_read_start = absolute_mov.ADDRESS_READ_START  
        address_write_start = address_read_start
        oper = absolute_mov.oper

    elif operation == "multiAxisStateRead":
        num_registers = readable_address.registers_per_axis*axis
        address_read_start = readable_address.ADDRESS_READ_START  
        address_write_start = address_read_start
        oper = readable_address.oper
    elif operation == "multiAxisStopControl":
        num_registers = stop_controlling.registers_per_axis*axis
        address_read_start = stop_controlling.ADDRESS_READ_START  
        address_write_start = address_read_start
        oper = stop_controlling.oper

    # 启动modbus客户端    
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
    
    # 开始监听modbus并发布对rostopic的更改；
    modclient.startListening(oper)
    rospy.loginfo("Listener started")

    # 创建一个监听器，如果可读modbus寄存器上的任何内容发生更改，则向我们显示一条消息 
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")
    def showUpdatedRegisters(msg):
        rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))
    sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=50)
    
    # 使用标准ros发布者写入modbus寄存器
    pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=50)

    if oper == 0:
        modclient.multiAxisStateRead(address_read_start,num_registers)
    elif oper == 1:
        values = [0,0]*axis
        modclient.multiAxisStopControl(address_write_start,values)
    elif oper == 2:
         # 对position进行合法判断
        try: 
            if 0 < position < 20200:        
                rospy.loginfo("position判断合法")
        except Exception as e:
            rospy.logwarn("position不合法")
            raise e
        values = [0,position]*axis 
        # values = [0,p] for p in [] #pos0-pos3

        modclient.multiAxisAbsoluteMove(pub,address_read_start,num_registers,address_write_start,values)

    # 停止监听
    modclient.stopListening(oper)      
   
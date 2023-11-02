#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray as HoldingRegister
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.bit_read_message import ReadCoilsResponse
from pymodbus.register_read_message import ReadInputRegistersResponse
from pymodbus.exceptions import ConnectionException      # 连接失败，用于异常处理

host = "192.168.1.222"
port = 502
client = ModbusTcpClient(host,port)

rospy.init_node("modbus_client")

# 读写保持寄存器
#client.write_register(address=12160,value=0) # 写单个寄存器
values = [0,15000]*3
client.write_registers(address=12160,values=values)
print("client.write_registers(address=12160,values=values):",client.write_registers(address=12160,values=values))

result:ReadInputRegistersResponse = client.read_holding_registers(address=12160,count=6)
while result.registers != values:
    rospy.sleep(1)
    result:ReadInputRegistersResponse = client.read_holding_registers(address=12160,count=6)

pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=50)
msg = HoldingRegister()
msg.data = result.registers
#print('read_holding_registers ')
#print(result.registers)

# 设置循环的频率
rate = rospy.Rate(20)
# 定义循环次数
#roate = 1

while not rospy.is_shutdown():
    pub.publish(msg)
    rospy.loginfo('read_holding_registers %s',str(msg.data))
    rate.sleep()
    #roate += 1
    #if roate > 15:
        #client.write_registers(address=12160,values=[0])
        #break

# 关闭连接
client.close()








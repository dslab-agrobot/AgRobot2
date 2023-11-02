
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the BSD License on
# github: https://github.com/Humarobotics/modbus_wrapper
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 
#
# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#
# This wrapper package is based on the pymodbus library developed by:
# Galen Collins
# github: https://github.com/bashwork/pymodbus
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
import rospy,time
from .post_threading import Post
from std_msgs.msg import Int32MultiArray as HoldingRegister
"""
    from std_msgs.msg import Int32MultiArray as HoldingRegister
    import rospy

    # 创建一个包含整数数组的 HoldingRegister 消息
    msg = HoldingRegister()
    msg.data = [1, 2, 3, 4, 5]

    # 发布消息
    rospy.init_node('publisher_node')
    pub = rospy.Publisher('my_topic', HoldingRegister, queue_size=10)
    pub.publish(msg)
"""
from modbus.msg import readable_address
from modbus.msg import relative_mov
from modbus.msg import absolute_mov
from modbus.msg import stop_controlling

try:
    from pymodbus.server.sync import ModbusSocketFramer, ModbusTcpServer
    from pymodbus.device import ModbusDeviceIdentification
    from pymodbus.datastore import ModbusSequentialDataBlock
    from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
except Exception as e:
    print("-- INSTALL DEPENDENCIES -- ")
    print("sudo apt-get install python-pymodbus")   
    print("or from source:")
    print("git clone git@github.com:bashwork/pymodbus.git")
    print("cd pymodbus")
    print("sudo python setup.py install")
    print("and")
    print("sudo apt-get install python-pyasn1 python-twisted-conch")
    print(e)
    exit()

NUM_REGISTERS = absolute_mov.registers_per_axis*4    # COM3的寄存器编号
ADDRESS_READ_START = absolute_mov.ADDRESS_READ_START    # 寄存器的基地址
# 计算出寄存器的写入数据的地址并赋予自定义消息属性
absolute_mov.ADDRESS_WRITE_START = ADDRESS_READ_START+NUM_REGISTERS     
ADDRESS_WRITE_START = absolute_mov.ADDRESS_WRITE_START

class CustomHoldingRegister(ModbusSequentialDataBlock):# ModbusSequentialDataBlock:创建顺序 Modbus 数据存储的类         
    def __init__(self,address,value,sub_topic,pub_topic):
        """
            Creates a custom holding register to add a publisher and subscriber to the modbus server：创建一个自定义持有寄存器，将发布者和订阅者添加到modbus服务器
             
            :param address: The starting address of the holding register：保持寄存器的起始地址
            :type address: int
            :param values: The initial values for the holding registers：保持寄存器的初始值
            :type values: list[int]
            :param sub_topic: ROS topic name for the subscriber that updates the modbus registers：更新modbus寄存器的订阅者的ROS主题名称
            :type sub_topic: string
            :param pub_topic: ROS topic name for the publisher that publishes a message, once there is something written to the writeable modbus registers
            # 发布消息的发布者的ROS主题名称，一旦有内容写入可写modbus寄存器
            :type pub_topic: string
        """
        
        super(CustomHoldingRegister,self).__init__(address,value)
        self.reading_start = ADDRESS_READ_START
        self.writing_start = ADDRESS_WRITE_START
        self.sub = rospy.Subscriber(sub_topic,HoldingRegister,self.__updateWriteableRegisters,queue_size=500)
        self.pub = rospy.Publisher(pub_topic,HoldingRegister,queue_size=500)
        
        
    def setValues(self, address, value):
        """ 
            Sets the requested values to the holding registers
            and publishes they new values on a rostopic
            将请求的值设置为保持寄存器,并将它们的新值发布在rostopic上

            :param address: The starting address
            :type address: int
            :param values: The new values to be set
            :type values: list[int]
        """
        print("address:",address,"value:",value)
        
        # ModbusSequentialDataBlock:创建顺序 Modbus 数据存储的类
        ModbusSequentialDataBlock.setValues(self, address, value)
        if address >= self.reading_start:
            msg = HoldingRegister()
            msg.data = value
            self.pub.publish(msg)
        
    def __updateWriteableRegisters(self,msg):
        if len(msg.data) > self.reading_start+1:
            rospy.logwarn("Message to long. Shorten it or it will be ignored")
        self.setValues(self.writing_start, list(msg.data))
        


class ModbusWrapperServer():
    def __init__(self,port=502,sub_topic="modbus_server/write_to_registers",pub_topic="modbus_server/read_from_registers"):
        """
            Creates a Modbus TCP Server object:创建Modbus TCP服务器对象
            .. note:: The default port for modbus is 502. This modbus server uses port 1234 by default, otherwise superuser rights are required.
            modbus的默认端口是502。此modbus服务器默认使用端口1234，否则需要超级用户权限
            .. note:: Use "startServer" to start the listener.
            使用“startServer”启动侦听器。
            :param port: Port for the modbus TCP server
            :type port: int
            :param sub_topic: ROS topic name for the subscriber that updates the modbus registers:
            # 更新 Modbus 寄存器的订阅服务器的 ROS 主题名称:更新 Modbus 寄存器的订阅服务器的 ROS 主题名称
            :type sub_topic: string
            :param pub_topic: ROS topic name for the publisher that publishes a message, once there is something written to the writeable modbus registers:
            # 发布消息的发布者的 ROS 主题名称，一旦有内容写入可写的 Modbus 寄存器
            :type pub_topic: string 
            
        """
        chr = CustomHoldingRegister(ADDRESS_WRITE_START, [17]*100,sub_topic,pub_topic)
        self.store = ModbusSlaveContext(
            di = ModbusSequentialDataBlock(ADDRESS_WRITE_START, [17]*100),
            co = ModbusSequentialDataBlock(ADDRESS_WRITE_START, [17]*100),
            hr = chr, 
            ir = ModbusSequentialDataBlock(ADDRESS_WRITE_START, [17]*100))
        self.context = ModbusServerContext(slaves=self.store, single=True)

        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName  = 'Pymodbus'
        self.identity.ProductCode = 'PM'
        self.identity.VendorUrl   = 'http://github.com/bashwork/pymodbus/'
        self.identity.ProductName = 'Pymodbus Server'
        self.identity.ModelName   = 'Pymodbus Server'
        self.identity.MajorMinorRevision = '1.0'

        self.store.setValues(2,0,[0]*1)
        self.post = Post(self)
        framer = ModbusSocketFramer
        self.server = ModbusTcpServer(self.context, framer, self.identity, address=("192.168.1.223", port))
        
    def startServer(self):
        """
            Non blocking call to start the server:启动服务器的非阻塞调用
        """
        self.post.__startServer()
        rospy.loginfo("Modbus server started")
        
    def __startServer(self):
        self.server.serve_forever()
        
    def stopServer(self):
        """
            Closes the server:关闭服务
        """
        self.server.server_close()
        self.server.shutdown()
        
    def waitForCoilOutput(self,address,timeout=2):
        """
            Blocks for the timeout in seconds (or forever) until the specified address becomes true. Adapt this to your needs：
            在指定地址变为真之前，以秒为单位（或永远）阻止超时。根据您的需要进行调整
            :param address: Address of the register that wanted to be read.
            :type address: int
            :param timeout: The time in seconds until the function should return latest.
            :type: float/int
        """
        now = time.time()
        while True:
            values = self.store.getValues(1,address, 1)
            print("values:",values)
            if values[0]:
                return True
            else:
                if timeout <=0 or now + timeout > time.time():
                    time.sleep(1/50)
                else:
                    return False
    
    def setDigitalInput(self,address,values):
        """
            Writes to the digital input of the modbus server：用于向 Modbus 服务器的数字输入（Digital Input）寄存器写入数据;
            在Modbus通信中，数字输入通常使用离散输入寄存器（Discrete Input Registers）来表示，它们是一种只读的寄存器类型，用于读取外部设备的状态信息。
            :param address: Starting address of the values to write
            :type: int
            :param values: List of values to write
            :type: list/boolean/int
        """
        # 如果values不是列表类型（或者说不是一个列表），则将其转换为包含values值的列表。
        if not isinstance(values, list):
            values = [values]          
        """
           向 Modbus 服务器的数字输入寄存器写入数据。具体来说，它传递了以下参数：
            1、2：这个参数可能是指 Modbus 寄存器类型或标识，通常是一个数字，用于指示要写入的寄存器类型；
            2、address：起始地址，指示从哪个地址开始写入数据；
            3、values：要写入的数据，这是一个包含一个或多个值的列表；
        """
        self.store.setValues(2,address,values)     
        
        

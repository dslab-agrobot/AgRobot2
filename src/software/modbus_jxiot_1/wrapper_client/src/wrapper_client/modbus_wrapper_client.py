#! /usr/env/ python

import rospy
from pymodbus.register_read_message import ReadInputRegistersResponse

try:
    from pymodbus.client import ModbusTcpClient
except Exception as e:
    print("pymodbus does not seem to be installed.\nInstall it by:\nsudo apt-get install python-pymodbus")
    print(e)
    exit()
from std_msgs.msg import Int32MultiArray as HoldingRegister
from .post_threading import Post
from threading import Lock

class ModbusWrapperClient():
    """
        将python modbus集成到标准化ros消息中的包装器。该包装器能够从标准modbus tcp/ip服务器中读取和写入。
        Wrapper that integrates python modbus into standardized ros msgs.
        The wrapper is able to read from and write to a standard modbus tcp/ip server.
    """
    def __init__(self,host,port=502,rate=50,ADDRESS_READ_START = 0,ADDRESS_WRITE_START = 0,NUM_REGISTERS = 0,reset_registers=True,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input"):
        """
            使用订阅者和发布者与modbus服务器进行通信。查看脚本中的示例代码。
            Use subscribers and publisher to communicate with the modbus server. Check the scripts for example code.
            :param host: Contains the IP adress of the modbus server
            :type host: string
            :param port: The port number, where the modbus server is runnning
            :type port: integer
            :param rate: How often the registers on the modbusserver should be read per second
            :type rate: float
            # 定义在读取寄存器后是否应将其重置为0,只有当它们是可写的时才有可能。
            :param reset_registers: Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
            :type reset_registers: bool
        """
        try:
            self.client = ModbusTcpClient(host,port)
        except Exception as e:
            rospy.logwarn("Could not get a modbus connection to the host modbus. %s", str(e))
            raise e
            
        self.__rate = rate
        self.__reading_delay = 1/rate
        self.post = Post(self)
        
        self.__reset_registers = reset_registers
        self.__reading_register_start = ADDRESS_READ_START
        self.__num_reading_registers = NUM_REGISTERS
#         self.input_size = 16
        self.__input = HoldingRegister()
        self.__input.data = [0 for i in range(self.__num_reading_registers )]

        self.__writing_registers_start = ADDRESS_WRITE_START
        self.__num_writing_registers = NUM_REGISTERS
#         self.output_size = 16 
        self.__output = [None for i in range(self.__num_writing_registers)]
        
        self.__last_output_time = rospy.get_time()
        self.__mutex = Lock()  # 定义互斥锁
        
        self.__sub = rospy.Subscriber(sub_topic,HoldingRegister,self.__updateModbusOutput,queue_size=50)
        self.__pub = rospy.Publisher(pub_topic,HoldingRegister,queue_size=50, latch=True)
    
        rospy.on_shutdown(self.closeConnection)
        #print("here")
    
    def startListening(self,oper):
        """
            用于启动可读modbus服务器寄存器的侦听器的非阻塞调用
            Non blocking call for starting the listener for the readable modbus server registers 
        """
        #start reading the modbus
        self.post.__updateModbusInput(oper)
        
    def stopListening(self,oper):
        """
            停止侦听器循环
            Stops the listener loop
        """
        if oper == 1:
            return
        self.stop_listener = True
        while not rospy.is_shutdown() and self.listener_stopped is False:
            rospy.sleep(0.01)
        
    def setReadingRegisters(self,start,num_registers):
        """
            设置应读取的寄存器的起始地址及其数量
            Sets the start address of the registers which should be read and their number
            :param start: First register that is readable
            :type start: int
            :param num_registers: Amount of readable registers
            :type num_registers: int
        """
        self.__reading_register_start = start
        self.__num_reading_registers = num_registers

    def setWritingRegisters(self,start,num_registers):
        """
            设置可写寄存器的起始地址及其数量
            Sets the start address of the registers which are writeable and their number
            :param start: First register that is writeable
            :type start: int
            :param num_registers: Amount of writeable registers
            :type num_registers: int
        """
        self.__writing_registers_start = start
        self.__num_writing_registers = num_registers
    
    def getReadingRegisters(self):
        """
            返回可读寄存器的第一个地址和寄存器数
            :return: Returns the first address of the readable registers and the number of registers
            :rtype: int,int
        """
        return self.__reading_register_start,self.__num_reading_registers
        
    def getWritingRegisters(self):
        """
            返回可写寄存器的第一个地址和寄存器数
            :return: Returns the first address of the writeable registers and the number of registers
            :rtype: int,int
        """
        return self.__writing_registers_start,self.__num_writing_registers
        
    def __updateModbusInput(self,oper,delay=0):
        """ 
            正在侦听可读modbus的循环注册并在主题上发布它               
            Loop that is listening to the readable modbus registers and publishes it on a topic
            :param delay: The delay time until the loop starts
            :type delay: float 
        """
        if oper == 1:
            return
        
        rospy.sleep(delay)
        self.listener_stopped = False
        self.stop_listener = False
        update = True
        while not rospy.is_shutdown() and self.stop_listener is False:
            try: 
                if not rospy.is_shutdown() :
                    tmp =  self.readRegisters(None,None) 
                    # print("__updateModbusInput tmp:",tmp)
                    if tmp is None:
                        rospy.sleep(2)
                        continue
                    rospy.logwarn("__updateModbusInput tmp is %s ", str(tmp))
                    rospy.logwarn("__updateModbusInput self.__input.data is %s ", str(self.__input.data))

                    if tmp != self.__input.data:
                        update = True
                        self.__input.data = tmp
                    else:
                        update = False 
            except Exception as e:
                rospy.logwarn("Could not read holding register. %s", str(e))
                raise e
                rospy.sleep(2)
        
            if update:
                if self.__pub.get_num_connections() > 0:
                    try:
                        self.__pub.publish(self.__input)
                    except Exception as e:
                        rospy.logwarn("Could not publish message. Exception: %s",str(e))
                        raise e
            rospy.Rate(self.__rate).sleep()
        self.listener_stopped = True
    
    def __updateModbusOutput(self,msg):
        """
            从订阅者回调以更新可写modbus寄存器
            Callback from the subscriber to update the writeable modbus registers
            :param msg: value of the new registers
            :type msg: std_msgs.Int32MultiArray
        """
        output_changed = False
        for index in range(self.__num_writing_registers):
            if self.__output[index] != msg.data[index]:
                output_changed = True
                break
        if not output_changed:
            return
        self.__writeRegisters(self.__writing_registers_start,msg.data)
        
    def __writeRegisters(self,address,values):
        """
            # 写入modbus寄存器
            Writes modbus registers
            :param address: First address of the values to write
            :type address: int
            :param values: Values to write
            :type values: list
        """
        with self.__mutex:
            try:
                if not rospy.is_shutdown() :
                    
                    self.output = values
                    self.client.write_registers(address, values)

                    print("writing address:",address,",values:",values)

            except Exception as e:
                rospy.logwarn("Could not write values %s to address %d. Exception %s",str(values),address, str(e))
                raise e

    def writeRegisters(self,address,values):
        self.__writeRegisters(address,values)
    
    def readRegisters(self,address,num_registers):
        """
            读取modbus寄存器
            Reads modbus registers
            :param address: First address of the registers to read
            :type address: int
            :param num_registers: Amount of registers to read
            :type num_registers: int
        """
        if address is None:
            address = self.__reading_register_start
        if num_registers is None:
            num_registers = self.__num_reading_registers

        tmp = None
        with self.__mutex:            
            try:
                print("address:",address,"num_registers",num_registers)
                result:ReadInputRegistersResponse = self.client.read_holding_registers(address,num_registers)
                #tmp = result.registers
                tmp = result.registers
                print("readRegisters' tmp:",tmp)
            except Exception as e:
                rospy.logwarn("Could not read on address %d. Exception: %s",address,str(e))
                raise e
 
        return tmp

    def setOutput(self,address,value,timeout=0):
        """
            直接写入一个寄存器
            Directly write one register
            :param address: The exact register address to write
            :type address: int
            :param value: What is written in the register
            :type value: int
            :param timeout: If set, the register is set after this time to 0
            :type timeout: float
        """
        if not type(value) is list:
            value = [int(value)]
        self.__writeRegisters(address, value)
        if timeout > 0:
            self.post.__reset(address,timeout)
            
    def __reset(self,address,timeout):
        """
            在特定时间后将寄存器重置为0
            Resets a register to 0 after a specific amount of time
            :param address: The register address to reset
            :type address: int
            :param timeout: The delay after the register is reset
            :type timeout: float
        """
        rospy.sleep(timeout)
        self.__writeRegisters(address,[0])
    
    def closeConnection(self):
        """
            断开modbus连接
            Closes the connection to the modbus
        """
        self.client.close()


uint16 NUM_REGISTERS  # 寄存器编号：不同编号寄存器的功能对应不同通道的操作
uint16 registers_per_axis = 2  # 每轴2个寄存器
uint16 ADDRESS_READ_START = 12160  # 读取的寄存器的基地址
uint16 ADDRESS_WRITE_START  # 写入不同通道/轴的寄存器地址
uint8 oper = 2 # 0:只读 1：只写 2：读写
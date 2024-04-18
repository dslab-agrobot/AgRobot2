# 履带车底盘CAN通讯协议 V2.0

注意：上电等待8秒左右再控制

## 一、通讯格式

- 波特率	 帧格式	帧类型
- 500K	标准帧	数据帧

## 二、上位机下发控制（CAN-ID:0X01）

- 1、轮速设置
   - 字节	说明   	数据类型	        备注
   - Byt0	0x01	Unsigned int8	底盘 Node-ID
   - Byt1	0x01	Unsigned int8	命令
   - Byt2	左轮速度设置低八位	Int16	单位：mm/s
   - Byt3	左轮速度设置高八位		
   - Byt4	右轮速度设置低八位	Int16	单位：mm/s
   - Byt5	右轮速度设置高八位		
   - Byt6	急停设置	Unsigned int8	1：急停  0：取消急停
   - Byt7	无		

    示例	备注
    01 01 C8 00 C8 00 00 00	左轮速度：200； 右轮速度：200 ；急停：0
    01 01 00 00 00 00 01 00	左轮速度：0； 右轮速度：0；急停：1

- 2、线速设置
   - 字节	说明	数据类型	备注
   - Byt0	0x01	Unsigned int8	底盘 Node-ID
   - Byt1	0x02	Unsigned int8	命令
   - Byt2	Vx设置低八位	Int16	单位：mm/s
   - Byt3	Vx设置高八位		
   - Byt4	Vz设置低八位	Int16	单位：0.001rad/s
   - Byt5	Vz设置高八位		
   - Byt6	急停设置	Unsigned int8	1：急停  0：取消急停
   - Byt7	无		

    示例	备注
    01 02 C8 00 64 00 00 00	Vx：200； Vz：100；急停：0
    01 02 00 00 00 00 01 00	Vx：0； Vz：0；急停：0

- 3、升降平台控制
   - 字节	说明	数据类型	备注
   - Byt0	0x01	Unsigned int8	底盘 Node-ID
   - Byt1	0x03	Unsigned int8	命令
   - Byt2	控制模式	Unsigned int8	1：位置模式 2：速度模式
   - Byt3	低八位	Int16	        速度模式：单位：rmp 位置模式：单位：0.1mm
   - Byt4	高八位		
   - Byt5	位置清零	Unsigned int8	1：有效
   - Byt6	保留	    Unsigned int8	
   - Byt7	保留  	Unsigned int8	
   - Byt1	保留	    Unsigned int8	

    示例	备注
    01 03 01 64 00 00 00 00	位置模式、100mm
    01 03 02 0A 00 00 00 00	速度模式、10rpm （速度范围：0~25）

- 4、清除错误
   - 字节	说明	    数据类型	        备注
   - Byt0	0x01	Unsigned int8	底盘 Node-ID
   - Byt1	0x04	Unsigned int8	命令
   - Byt2	驱动器故障清除	Unsigned int8	1：有效
   - Byt3	保留	    Unsigned int8	
   - Byt4	保留	    Unsigned int8	
   - Byt5	保留  	Unsigned int8	
   - Byt6	保留	    Unsigned int8	
   - Byt7	保留 	Unsigned int8	
   - Byt1	保留	    Unsigned int8	

## 三、状态信息自动反馈

- 1、线速反馈  (CAN-ID:0X50)
   - 字节	说明	        数据类型	          备注
   - Byt0	0x02	  Unsigned int8	  底盘 Node-ID
   - Byt1	0x01	  Unsigned int8	     命令
   - Byt2	Vx读取低八位	Int16	     单位：mm/s
   - Byt3	Vx读取高八位		
   - Byt4	Vz读取低八位	Int16	     单位：0.001rad/s
   - Byt5	Vz读取高八位		
   - Byt6	底盘电压读取低八位	Int16	单位：0.1V
   - Byt7	底盘电压读取高八位		

    示例	备注
    02 01 C8 00 64 00 E6 01	Vx：200； Vz：100；电压：486
    02 01 37 FF 00 00 ED 01	Vx：-200； Vz：0； 电压：493

- 2、底盘状态反馈(CAN-ID:0X50)
   - 字节	说明  	数据类型	        备注
   - Byt0	0x02	Unsigned int8	底盘 Node-ID
   - Byt1	0x02	Unsigned int8	命令
   - Byt2	        Unsigned int8	0:取消急停、离线线、正常 1：急停、在线、异常
      - Bit0	硬件急停
      - Bit1	遥控急停
      - Bit2	软件急停
      - Bit3	遥控器在线/离线
      - Bit4	驱动器在线/离线
      - Bit5	左驱动器正常/异常
      - Bit6	右驱动器正常/异常
      - Bit7	
   - Byt3	控制状态	Unsigned int8	0x00：空闲状态
                                    0x01：遥控器模式
                                    0x02：上位机模式
                                    0x05:  升降台模式
                                    0xFF:  错误状态
   - Byt4	读取驱动器温度低八位	Int16	单位：0.1°
   - Byt5	读取驱动器温度高八位		
   - Byt6			
   - Byt7			

    示例	备注
    02 02 05 01 27 01 29 01	Byt2:遥控器在线、硬件急停打开
    Byte3:控制模式是遥控模式
    左驱动器温度：295
    右驱动器温度：297

- 3、升降平台状态（CAN-ID:0X50)
   - 字节	说明	    数据类型	      备注
   - Byt0	0x02	Unsigned int8	底盘 Node-ID
   - Byt1	0x03	Unsigned int8	命令
   - Byt2	控制模式	Unsigned int8	1：位置模式 2：速度模式
   - Byt3	速度低八位	Int16	速度模式：单位：rmp
   - Byt4	速度高八位		
   - Byt5	位置低八位	Int16	位置模式：单位：mm
   - Byt6	位置高八位		
   - Byt7	状态	Unsigned int8	bit0	下限位	1：表示触碰
                                Bit1	上限位	1：表示触碰
                                Bit2	驱动器异常	1：表示异常

- 4、驱动器状态（CAN-ID:0X55)
   - 字节	说明             	数据类型	          备注
   - Byt0	左驱动器状态低八位	Unsigned int16	Bit0	电源欠压
   - Byt1	左驱动器状态高八位                    Bit1	位置异常
   - Byte2	右驱动器状态低八位	Unsigned int16  Bit2	霍尔错误
   - Byt3	右驱动器状态高八位                    Bit3	过流
                                                Bit4	超载
                                                Bit5	EEPROM 故障
                                                Bit6	IGBT 故障
                                                Bit7	驱动器过热
                                                Bit8	电机缺陷
                                                Bit9	电源超差
                                                Bit10	速度超差
                                                Bit11	电机过热
                                                Bit12	电源过压
                                                Bit13	飞车故障
                                                Bit14	驱动器过热报警
                                                Bit15	保留
		
   - Byt4	左驱动器电流低八位	Int16	单位：0.1A
   - Byt5	左驱动器电流高八位		
   - Byt6	右驱动器电流低八位	Int16	单位：0.1A
   - Byt7	右驱动器电流高八位		

- 5、电机状态（CAN-ID:0X56)
   - 字节	说明	            数据类型	               备注
   - Byt0	左电机状态低八位	Unsigned int16	Bit0	伺服启动
   - Byt1	左电机状态高八位                        Bit1	伺服运行
   - Byte2	右电机状态低八位                        Bit2	零速运行
   - Byt3	右电机状态高八位                        Bit3	目标速度到达
                                                  Bit4	目标位置到达
                                                  Bit5	转矩限制中
                                                  Bit6	警告
                                                  Bit7	制动输出
                                                  Bit8	原点回复完成
                                                  Bit9	超过载门槛
                                                  Bit10	错误警告
                                                  Bit11	命令完成
                                                  Bit12	反向堵转
                                                  Bit13	正向堵转
                                                  Bit14	反向指示
                                                  Bit15	正向输出
  - Byt4	记录电流最大值低八位（左）	Int16	单位：0.1A
  - Byt5	记录电流最大值高八位（左）		
  - Byt6	记录电流最大值低八位（右）	Int16	单位：0.1A
  - Byt7	记录电流最大值高八位（右）		



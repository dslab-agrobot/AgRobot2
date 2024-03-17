# ros_ht_msg - 基于ROS和CAN网络的机器人底盘控制程序



## 1. 概述

`ros_ht_msg`是用于控制四轮四转机器人底盘的功能包，基于ROS[ROS介绍](1.01ROS介绍.md)和CAN[CAN介绍](1.02CAN介绍)网络通讯；

`ros_ht_msg`功能包提供了以ROS话题通讯[ROS话题通信介绍](1.03ros话题通信介绍.md)的消息控制机器人底盘的功能。

`ros_ht_msg`功能包主要由四轮四转底盘供应商提供，功能包中包括了必要的文档、示例、消息类型等文件夹；

## 依赖项[依赖项介绍](1.04依赖项介绍.md)

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  message_generation
  geometry_msgs
)
```

## 安装  install with Github

- 本功能包基于python3；

- 本功能包基于ROS noetic 1.16.0

- 本版本地址为：[[hmxf/ros_ht_msg: AgRobot ROS control node for HT-03. (github.com)](https://github.com/hmxf/ros_ht_msg)](https://github.com/hmxf/ros_modbus_msg/tree/release-v1.0)

  ```
  cd ~/catkin_ws
  mkdir src
  cd src
  git clone 功能包.git
  ```

## 编译

```
cd ~/catkin_ws
catkin build --pre-clean
```

**注**：

- 编译时可以使用`catkin_make`，但推荐为`catkin build`，此工具更加灵活高效。

- 参数`--pre-clean`表示开始构建之前先执行清理操作，以避免可能存在的旧文件对新构建的影响：删除先前生成的`build` 和 `devel` 目录。



## 2. 使用方法

## 使用前准备

1. **确认编译的系统架构，选择合适的动态链接库文件`libcontrolcan.so`；**[动静态链接库介绍](1.05动静态链接库介绍.md)

   - 在功能包`lib`文件夹下已经设置好四种系统架构下的相应的文件，同时还有相应的静态链接库文件，因为四种动态链接库文件均为同名文件`libcontrolcan.so`，本功能包采用软链接[软链接介绍](1.06软链接介绍.md)的方式调用该文件；

     ```
     lib文件夹tree:
     .
     ├── libcontrolcan.so -> linux/x86_64/libcontrolcan.so
     └── linux
         ├── aarch64
         │   ├── libcontrolcan.a
         │   └── libcontrolcan.so
         ├── arm32
         │   ├── libcontrolcan.a
         │   └── libcontrolcan.so
         ├── controlcan.h
         ├── README.md
         ├── x86
         │   ├── libcontrolcan.a
         │   └── libcontrolcan.so
         └── x86_64
             ├── libcontrolcan.a
             └── libcontrolcan.so
     ```

     **注：**当前展示的文件树为软链接连接在`X86_64`系统架构下的动态链接库文件上；

   - 编译运行时，需要确认动态链接库文件是否和自己的系统架构适配，否则会编译失败：
     - 查看自己系统架构：

       ```
       uname -m
       ```

     - 查看当前软连接连接的文件：
     
       ```
       cd ~/catkin_ws/src/ros_ht_msg/lib
       ll
       ```
     
       **注：**`libcontrolcan.so -> linux/x86_64/libcontrolcan.so`中`x86_64`表示目前连接在X86的64位系统的动态链接库文件上。
     
     - 修改方式：
     
       ```
       ln -s linux/相应架构/libcontrolcan.so libcontrolcan.so
       ```

   - 在底盘的工控机上编译运行本功能包时，需要将软连接修改到`aarch64`中的动态链接文件上；

     在个人PC上编译编译运行本功能包时，需要将软连接修改到`x86_64`中的动态链接文件上。

   

2. **连接CAN网络设备**：

   - 本项目使用 创芯科技 提供的 **CANalyst-Ⅱ分析仪**[CANalyst-Ⅱ分析仪介绍](1.07CANalyst-II分析仪介绍.md)作为CAN网络通讯设备，设备自身接线方式有对应手册，位于功能包`doc`文件夹中，文件名为：`USBCAN_Test_Process_V1.70.pdf`；
   - CAN网络通讯设备通过TCP协议[TCP介绍](1.08TCP介绍.md)和主机通讯[主机通讯介绍](1.09主机通讯介绍.md)；
   - 使用该设备时，涉及到USB底层驱动协议[USB底层驱动协议介绍](1.10USB底层驱动协议介绍.md)，在运行时需要加`sudo`获取权限，否则 USB设备没有权限操作；也可以增加新的udev规则[udev规则介绍](1.11udev规则介绍.md)，使得后续调用不用频繁获取权限，具体方式可参考`doc`文件夹中的`HT01_ROS_Comm_Package_Instructions.pdf`文件。



## 使用

- 方式一：单一文件启动功能包（可用于前期基础测试）：

  1. 刷新环境变量：

     ```
     source ./devel/setup.bash 
     ```

  2. 运行节点：

     ```
     rosrun ros_ht_msg ros_ht_msg 
     ```

  3. 发布命令：

     ```
     rostopic pub /HT_Control ros_ht_msg/ht_control "mode: 0
     x: 0
     y: 0
     z: 0" 
     ```

     **注：**命令参数含义可以参考`doc`文件夹中的`HT01_CAN_Communication_Protocol_20210722 .pdf`文件。

  4. 此时可以通过如下命令查看底盘各系统状态：

     ```
     rostopic echo /HT_Drive_Motor   # 驱动电机状态
     rostopic echo /HT_Motion        # 运动状态
     rostopic echo /HT_Steer_Motor   # 转向电机状态
     rostopic echo /HT_System        # 系统状态
     ```

- 方式二：roslaunch启动多个节点：方式基本同上，

  1. 启动节点时，需启动：

     ```
     roslaunch ros_ht_msg ros_ht_start.launch
     ```

     当编写其余命令发布节点后，可以将节点添加在launch文件中，在启动launch文件时一次全部启动；

- 示例：

  1. 在`examples`文件夹中包含多个测试样例：

     - example_1：实验一：同一指令间隔 1s 以上发送两次；
     - example_2：实验二：同一指令间隔 50ms，发送两次；
     - example_3：实验三：第一指令发送后间隔 50ms，发送相反指令；
     - example_4：将三个实验整合在一个测试文件中；

  2. 在`scripts`文件夹中保存目前编写的两个简单机器人运行demo：

     分别使用了四轮四转底盘的两种运动方式

     - 实验一：前进 20s 后，左平移 2s，后退 20s；
     - 实验二：前进 20s 后，原地旋转，继续前进20s；



## 3. 节点信息

## 节点名称：`/publish_ht_msg`

该节点接受到工控机发布的命令，转换为CAN帧转发给底盘电机，并收集地盘电机相关状态帧[状态帧介绍](1.12状态帧介绍.md)，转换为设定好的信息展示；

## 订阅的话题：

- /HT_Control
  - 消息类型：`ros_ht_msg/ht_control`；具体消息格式可参看功能包中`msg`文件夹下`ht_control.msg`文件。
  - 作用：接受**工控机**发布的**运动指令**；
- /HT_Drive_Motor
  - 消息类型：ros_ht_msg/ht_drive_motor；具体消息格式可参看功能包中`msg`文件夹下`ht_drive_motor.msg`文件。
  - 作用：接受**驱动电机**发布的**运动状态信息**，通过回调函数设置可以配合`rostopic echo`命令展示在屏幕上
- /HT_Motion
  - 消息类型：ros_ht_msg/ht_motion；具体消息格式可参看功能包中`msg`文件夹下`ht_motion.msg`文件。
  - 作用：接受**运动状态信息**，通过回调函数设置可以配合`rostopic echo`命令展示在屏幕上
- /HT_Steering_Motor
  - 消息类型：ros_ht_msg/ht_steering_motor；具体消息格式可参看功能包中`msg`文件夹下`ht_steering_motor.msg`文件。
  - 作用：接受**转向电机**[转向电机介绍](1.13转向电机介绍.md)发布的**状态信息**，通过回调函数[回调函数介绍](1.14回调函数介绍.md)设置可以配合`rostopic echo`命令展示在屏幕上
- /HT_System
  - 消息类型：ros_ht_msg/ht_system；具体消息格式可参看功能包中`msg`文件夹下`ht_system.msg`文件。
  - 作用：接受**底盘系统**发布的**状态信息**，通过回调函数设置可以配合`rostopic echo`命令展示在屏幕上

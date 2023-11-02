#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include <iostream>

#include "ros/ros.h"
#include <sstream>
#include <geometry_msgs/Twist.h>

#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ctime>
#include <stdint.h>

#include "ros_ht_msg/ht_drive_motor.h"  //自定义消息类型
#include "ros_ht_msg/ht_motion.h"  //自定义消息类型
#include "ros_ht_msg/ht_steering_motor.h"  //自定义消息类型
#include "ros_ht_msg/ht_system.h"  //自定义消息类型
#include "ros_ht_msg/ht_control.h"//自定义消息类型
#include "ros_ht_msg.h"

#define PI 3.141592653

VCI_BOARD_INFO pInfo;//用来获取设备信息。
using namespace std;

s8 mode = 2;
s16 angular_velocity = 0.0;
s16 angle = 0.0;
s16 linear_velocity = 0.0;

//send can data to ht
void SendSpeedToHt(u8 mode,s16 x,s16 y,s16 z)
{
     //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;
            
    //要写入的数据(mode 0x00为阿克曼/自转 ,0x01为FTFD 模式)
    send[0].Data[0] = 0x00000001;
    send[0].Data[1] = mode;
    send[0].Data[2] = x & 0xFF;
    send[0].Data[3] = (x>>8) & 0xFF;
    send[0].Data[4] = y & 0xFF;
    send[0].Data[5] = (y>>8) & 0xFF;
    send[0].Data[6] = z & 0xFF;
    send[0].Data[7] = (z>>8) & 0xFF;
    //写入数据
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {
        printf("TX data successful!\n");
    }                  
}

void cmd_velCallback(const ros_ht_msg::ht_control::ConstPtr &ht_control_msg)//速度控制回调
{
    SendSpeedToHt(ht_control_msg->mode,ht_control_msg->x,ht_control_msg->y,ht_control_msg->z);

}

int main(int argc, char **argv)
{
    int i=0,re_size = 0;
    ros_ht_msg::ht_motion ht_motion_msg;
    ros_ht_msg::ht_drive_motor ht_drive_motor_msg;
    ros_ht_msg::ht_steering_motor ht_steering_motor_msg;
    ros_ht_msg::ht_system  ht_system_msg;
    ros_ht_msg::ht_control ht_control_msg;

    printf(">>this is hello !\r\n");//指示程序已运行
    int num = VCI_OpenDevice(VCI_USBCAN2,0,0);
    if(num == 0 || num == 1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error %d!\n",num);
        exit(1);
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;//FFFFFFFF全部接收
    config.Filter=2;//接收所有帧  2-只接受标准帧  3-只接受扩展帧
    config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
    config.Timing1=0x1C;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }
   
    //需要读取的帧，结构体设置
    VCI_CAN_OBJ rev[2500];
    rev[0].SendType=0;
    rev[0].RemoteFlag=0;
    rev[0].ExternFlag=0;
    rev[0].DataLen=8;

    ros::init(argc, argv, "publish_ht_msg");
    ros::NodeHandle n;
    ros::Subscriber ht_control_sub = n.subscribe<ros_ht_msg::ht_control>("/HT_Control", 100, cmd_velCallback);//速度回调
    if (ht_control_sub != NULL) {  
    printf("Successfully subscribed to '/HT_Control'\n");  
    } 
    else {  
    printf("Failed to subscribe to '/HT_Control'\n");  
    }

    ros::Publisher ht_motion_pub = n.advertise<ros_ht_msg::ht_motion>("/HT_Motion", 100);//运动状态消息发布
    ros::Publisher ht_system_pub = n.advertise<ros_ht_msg::ht_system>("/HT_System", 100);//系统状态消息发布
    ros::Publisher ht_drive_motor_pub = n.advertise<ros_ht_msg::ht_drive_motor>("/HT_Drive_Motor", 100);//驱动电机状态消息发布
    ros::Publisher ht_steering_motor_pub = n.advertise<ros_ht_msg::ht_steering_motor>("/HT_Steering_Motor", 100);//转向电机消息发布
    ros::Rate loop_rate(50);
    
    while (ros::ok())
    {
        //读取数据
        re_size = VCI_Receive(VCI_USBCAN2, 0, 0,rev, 2500, 0);
        if(re_size > 0)
        {
            for(int i=0;i< re_size;i++)
            {
                //运动状态反馈
                if((rev[i].ID == 0x10)&&(rev[i].Data[0] == 01) && (rev[i].Data[1] == 00))//can发送00代表阿克曼模式
                {
                    mode = 0;                    
                    ht_motion_msg.mode = mode;
                    ht_motion_msg.VX = (rev[i].Data[3] << 8) | rev[i].Data[2];
                    int16_t angle = (rev[i].Data[5] << 8) | rev[i].Data[4];
                    ht_motion_msg.angle = angle;
                    ht_motion_msg.VZ = (rev[i].Data[7] << 8) | rev[i].Data[6];
                    ht_motion_msg.vx = 0.0;
                    ht_motion_msg.vy = 0.0;
                    ht_motion_msg.vz = 0.0;
                    ht_motion_pub.publish(ht_motion_msg);
                }
                else if((rev[i].ID == 0x10)&&(rev[i].Data[0] == 01) && (rev[i].Data[1] == 01))//can发送01代表FTFD模式
                {
                    mode = 1 ;
                    ht_motion_msg.mode = 1;
                    ht_motion_msg.VX = 0.0;
                    ht_motion_msg.angle = 0.0;
                    ht_motion_msg.VZ = 0.0;
                    ht_motion_msg.vx = (rev[i].Data[3] << 8) | rev[i].Data[2];
                    ht_motion_msg.vy = (rev[i].Data[5] << 8) | rev[i].Data[4];
                    ht_motion_msg.vz = (rev[i].Data[7] << 8) | rev[i].Data[6];
                    ht_motion_pub.publish(ht_motion_msg);
                }
                //系统状态反馈
                else if(rev[i].ID == 0x11)
                {
                    uint16_t voltage = (rev[i].Data[1]<<8) | rev[i].Data[0];
                    uint8_t control_mode = rev[i].Data[2];
                    uint8_t status = rev[i].Data[3];
                    uint8_t drive_motor_error = rev[i].Data[4];
                    uint8_t  encode_error = rev[i].Data[5];
                    ht_system_msg.voltage = voltage ;
                    ht_system_msg.control_mode = control_mode;
                    ht_system_msg.status = status;
                    ht_system_msg.drive_motor_error = drive_motor_error;
                    ht_system_msg.encode_error = encode_error;
                    ht_system_pub.publish(ht_system_msg);
                }
                //驱动电机状态反馈
                else if((rev[i].ID == 0x12)&&(rev[i].Data[0] == 01))
                {
                    uint8_t drive_motor_id = rev[i].Data[1];
                    int16_t  drive_motor_speed = (rev[i].Data[3]<<8) | rev[i].Data[2];
                    uint32_t encoding_count = (rev[i].Data[7]<<24) | (rev[i].Data[6]<<16) | (rev[i].Data[5]<<8) | rev[i].Data[4] ;
                    ht_drive_motor_msg.drive_motor_id = drive_motor_id;
                    ht_drive_motor_msg.drive_motor_speed = drive_motor_speed;
                    ht_drive_motor_msg.encoding_count = encoding_count;
                    ht_drive_motor_pub.publish(ht_drive_motor_msg);
                }
                //转向电机状态反馈
                else if((rev[i].ID == 0x13)&&(rev[i].Data[0] == 01))
                {
                    uint8_t steering_motor_id = rev[i].Data[1];
                    int16_t steering_motor_angle = (rev[i].Data[3]<<8) | rev[i].Data[2];
                    ht_steering_motor_msg.steering_motor_id =steering_motor_id;
                    ht_steering_motor_msg.steering_motor_angle = steering_motor_angle;
                    ht_steering_motor_pub.publish(ht_steering_motor_msg);
                }
            }
        }        
        ros::spinOnce();
        loop_rate.sleep();
    }
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    return 0;
}

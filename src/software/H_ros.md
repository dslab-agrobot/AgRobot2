# H_ros

打开home文件，创建工作空间

将src文件和.catkin_workspace和rtk.rviz文件放置在所创建的工作空间下。创建工作空间“ robotcar ”。

src[src](H_ros/src)            .catkin_workspace[.catkin_workspace](H_ros/.catkin_workspace)

![image-20250106095210995](C:\Users\ayx\AppData\Roaming\Typora\typora-user-images\image-20250106095210995.png)

```
mkdir -p robotcar
将src文件复制到robotcar文件目录下
cd robotcar
catkin_make
```

编译运行，根据错误提示，一步一步安装所缺少的包（以下已列出缺少的包）

```
sudo apt update
sudo apt upgrade
ubuntu-drivers devices
lshw -c display
ubuntu-drivers devices
lshw -c display
ubuntu-drivers devices
sudo apt install nvidia-driver-535 
nvidia-smi 
pwd 
cd ~/robotcar/devel/
ls
roscore 
./rtk_mqtt.exe 
./rtk_robot 
sudo apt-get install ros-noetic-serial
sudo apt-get install libsdl2-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install ros-noetic-tf2-sensor-msgs
sudo apt-get install ros-noetic-geographic-msgs 
sudo apt-get install ros-noetic-geographic-info 
sudo apt-get install libgeographic-dev 
sudo apt-get install ros-noetic-costmap-converter
sudo apt-get install ros-noetic-mbf-costmap-core 
sudo apt-get install ros-noetic-mbf-msgs
sudo apt-get install libsuitesparse-dev
sudo apt-get install 
sudo apt-get install ros-noetic-libg2o
sudo apt-get install libpcap-dev
sudo apt-get install libsente
sudo apt-get install ros-noetic-nmea-msgs 
reboot
catkin_make 
```

安装emqx服务

```
./rtk_mqtt.exe 
sudo apt install ./emqx-5.0.17-ubuntu18.04-amd64.deb 
sudo systemctl start emqx
```

安装myusb

本步需要在etc文件夹下找到gdm3文件夹，更改gdm3文件夹下custom.conf文件的内容

custom.conf[custom.conf](H_ros/custom.conf)

在etc文件夹下找到udev文件夹，在rules.d文件夹内写入70-snap.snapd.rules和70-snap.snap-store.rules文件

70-snap.snapd.rules[70-snap.snapd.rules](H_ros/rules.d/70-snap.snapd.rules)                    70-snap.snap-store.rules[70-snap.snap-store.rules](H_ros/rules.d/70-snap.snap-store.rules)

![image-20250106105436511](C:\Users\ayx\AppData\Roaming\Typora\typora-user-images\image-20250106105436511.png)

```
sudo gedit ~/.bashrc 
roscore
rosdepc update
wget http://fishros.com/install -O fishros &&  . fishros
pwd
cd catkin_rtk/
ls
reboot
sudo gedit /etc/gdm3/custom.conf
reboot
sudo cp myusb.rules /etc/udev/rules.d/
reboot
```

```
custom.conf文件内容
	# GDM configuration storage
# See /usr/share/gdm/gdm.schemas for a list of available options.

[daemon]
# Uncomment the line below to force the login screen to use Xorg
#WaylandEnable=false

# Enabling automatic login
#  AutomaticLoginEnable = true
#  AutomaticLogin = user1

# Enabling timed login
  TimedLoginEnable = true
  TimedLogin = robotcar
  TimedLoginDelay = 10

[security]

[xdmcp]

[chooser]

[debug]
# Uncomment the line below to turn on debugging
# More verbose logs
# Additionally lets the X server dump core if it crashes
#Enable=true
```

```
70-snap.snapd.rules和70-snap.snap-store.rules文件内容见附件
```

设置开机自启文件，在home目录下找到自己的用户目录例如：“ ayx ”，在ayx目录下找到.config文件，然后创建一个autostart文件夹，在autostart文件夹下创建robot_start.sh.desktop文件，在robot_start.sh.desktop文件中设置开机自启。

robot_start.sh.desktop[robot_start.sh.desktop](H_ros/robot_start.sh.desktop)

![image-20250106095316898](C:\Users\ayx\AppData\Roaming\Typora\typora-user-images\image-20250106095316898.png)

```
[Desktop Entry]
Type=Application
Exec=/home/ayx/mysh/rtk_start.sh
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
Name[zh_CN]=rtk_start
Name=robot_start
Comment[zh_CN]=
Comment=
```



设置mysh文件，在home/ayx/mysh文件中写入四个文件：robot_control.sh和robot_start.sh和rtk_start.sh和updata.sh文件。

robot_control.sh[robot_control.sh](H_ros/mysh/robot_control.sh)     robot_start.sh[robot_start.sh](H_ros/mysh/robot_start.sh)      rtk_start.sh[rtk_start.sh](H_ros/mysh/rtk_start.sh)     updata.sh[updata.sh](H_ros/mysh/updata.sh)

![image-20250106103044954](C:\Users\ayx\AppData\Roaming\Typora\typora-user-images\image-20250106103044954.png)

```
robot_control.sh文件内容
#! /bin/bash
### BEGIN INIT INFO
# Provides:          xxxx.com
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mylaunch service
# Description:       mylaunch service test
### END INIT INFO
sleep 10 #等待8秒
#必须加网络这个，不然多机通信失败
#export ROS_MASTER_URI=http://192.168.101.116:11311
#export ROS_IP=192.168.101.116
#source devel/setup.bash;
gnome-terminal -- bash -c "source ~/.bashrc;
			   cd ~/robotcar/;
                           source ~/robotcar/devel/setup.bash;
			   rosrun ros_robot_control_mqtt ros_robot_control_mqtt" #新建终端启动节点


robot_start文件内容
#! /bin/bash
### BEGIN INIT INFO
# Provides:          xxxx.com
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mylaunch service
# Description:       mylaunch service test
### END INIT INFO
#必须加网络这个，不然多机通信失败
#export ROS_MASTER_URI=http://10.42.0.1:11311
#export ROS_IP=10.42.0.1

gnome-terminal --tab -- bash -c "source ~/.bashrc;
                           source ~/robotcar/devel/setup.bash;
			   roscore;
			   exec bash" #roscore" #新建终端启动节点

gnome-terminal --tab -- bash -c "sleep 10;
			   source ~/.bashrc;
			   cd ~/catkin_ws/;
                           source ~/robotcar/devel/setup.bash;
			   rosrun ros_robot_control_mqtt ros_robot_control_mqtt;
                           exec bash" #新建终端启动节点

gnome-terminal --tab -- bash -c "sleep 10;
			   source ~/.bashrc;
			   cd ~/robotcar/;
                           source ~/robotcar/devel/setup.bash;
			   rosrun ros_mqtt_modbus ros_mqtt_modbus;
                           exec bash" #新建终端启动节点
#wait
#exit 0


updata.sh
#! /bin/bash
### BEGIN INIT INFO
# Provides:          xxxx.com
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mylaunch service
# Description:       mylaunch service test
### END INIT INFO
#必须加网络这个，不然多机通信失败
#export ROS_MASTER_URI=http://10.42.0.1:11311
#export ROS_IP=10.42.0.1

gnome-terminal --tab -- bash -c "sleep 10;
                           source ~/.bashrc;
                           cd ~/robotcar/;
                           source ~/robotcar/devel/setup.bash;
                           roslaunch ros_mqtt_control ros_mqtt_control.launch;
			   exec bash" #roscore" #新建终端启动节点

#wait
#exit 0
```



将rtk_mqtt_ubuntu文件夹放在桌面，rtk_mqtt_ubuntu文件中的AppRun是控制界面。

注意：

如果使用的是虚拟机，需要注意虚拟机的串口配置

AppRun界面的权限问题



遇见问题请参考.bash_history[.bash_history](H_ros/.bash_history)

注意:

本参考环境配置用户名为“ayx”，工作空间名为“robotcar”，使用者需要根据实际情况修改。

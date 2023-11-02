import rospy
from ros_ht_msg.msg import *



def controlfunc(mode,x,y,z):
    """
    指令转换
    """
    control = ht_control()
    control.mode = mode
    control.x = x
    control.y = y
    control.z = z
    return control


def dopub(pub, time, control1,control2):    
    du_x = rospy.Duration(time)
    rospy.loginfo("第一次指令发送\n")
    pub.publish(control1)
    rospy.loginfo("休眠%.2fs---------\n", time)
    rospy.sleep(du_x)
    rospy.loginfo("第二次指令发送")
    pub.publish(control2)
    rospy.sleep(rospy.Duration(20))

    

if __name__ == "__main__":
    
    rospy.init_node("pub_command")

    pub =  rospy.Publisher("/HT_Control", ht_control, queue_size=100)
    control_1 = controlfunc(1, -100, 0, 0)

    rospy.sleep(rospy.Duration(5))

    rospy.loginfo("实验一：同一指令间隔1s以上发送两次: \n")
    dopub(pub, 1.00, control_1,control_1)


    rospy.loginfo("实验二：同一指令间隔50ms，发送两次：\n")
    dopub(pub, 0.05, control_1, control_1)

    control_2 = controlfunc(1, 100, 0, 0)
    rospy.loginfo("实验三：第一指令发送后间隔50ms，发送相反指令：\n")
    dopub(pub, 0.05, control_1, control_2)


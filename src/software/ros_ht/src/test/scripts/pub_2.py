import rospy
from ros_ht_msg.msg import *



def controlfunc(mode,x,y,z):
    control = ht_control()
    control.mode = mode
    control.x = x
    control.y = y
    control.z = z
    return control
    

if __name__ == "__main__":
    
    rospy.init_node("pub_command")

    pub =  rospy.Publisher("/HT_Control", ht_control, queue_size=100)
    control_1 = controlfunc(1, -100, 0, 0)

    rospy.loginfo("实验二：同一指令间隔50ms以上发送两次: \n")
    rospy.sleep(rospy.Duration(5))
    
    time = 0.05
    du_x = rospy.Duration(time)
    
    rospy.loginfo("第一次指令发送\n")
    pub.publish(control_1)
    
    rospy.sleep(du_x)
    rospy.loginfo("休眠%.2fs---------\n", time)
    
    rospy.loginfo("第二次指令发送")
    pub.publish(control_1)
    
    rospy.sleep(rospy.Duration(20))
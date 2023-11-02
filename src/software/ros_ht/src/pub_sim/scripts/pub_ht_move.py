import rospy
from ros_ht_msg.msg import *
from pub_sim.msg import Slipway_control


def HTcontrolfunc(mode = 1,x = 0,y = 0,z = 0):
    control = ht_control()
    control.mode = mode
    control.x = x
    control.y = y
    control.z = z
    return control


def dopub(pub, control): 
    """
    指令发布，
    pub,发布端
    control,命令指令
    一次控制指令发布后会休眠1s
    """   
    rospy.loginfo("指令发送\n")
    pub.publish(control)
    du_x = rospy.Duration(1)
    rospy.sleep(du_x)

 
def pubMove(time, pub, control):
    """
    控制地盘行进方式：包括前进、后退、向左平移、向右平移；
    
    time 移动时间， 指令需要按照每1s一次的频率发布
    pub 发布实例
    control  移动指令， 建议仅使用x和y
    """
    while time > 0:
        dopub(pub, control)
        mode = "FTFD(四轮四转)"
        if control.x != 0:
            if (control.x > 0):
                towards = "前"
            elif (control.x < 0):
                towards = "后"
            rospy.loginfo("向%s行进: mode 为%s 速度 =%d mm/s ", towards, mode, control.x)
        
        if control.y != 0:
            if (control.y > 0):
                towards = "左"
            elif (control.y < 0):
                towards = "右"
            rospy.loginfo("向%s行进: mode 为%s 速度 =%d mm/s ", towards, mode, control.y)
        
        time -= 1


def pubTurnAround(time, pub, control):
    """
    原地旋转的移动方式，参数同上
    """
    while time > 0:
        dopub(pub, control)
        mode = "阿克曼/自转"
        rospy.loginfo("原地旋转180度: mode 为%s 速度 =%d 度/min ", mode, control.z)
        time -= 1


if __name__ == "__main__":
    
    rospy.init_node("pub_command")
    pub =  rospy.Publisher("/HT_Control", ht_control, queue_size=100)
    rospy.sleep(rospy.Duration(5))

    # 前进指令：
    control_forwards = HTcontrolfunc(1, 100)

    # 左平移指令：
    control_left = HTcontrolfunc(1, y=100)

    # 后退指令：
    control_backwards = HTcontrolfunc(1, -100)   

    # 原地旋转指令：(转180度)
    control_revolve = HTcontrolfunc(0,0,4500,1080)

    rospy.loginfo("实验一： 前进20s后， 左平移2s， 后退20s \n")
    pubMove(20, pub, control_forwards)
    pubMove(2, pub, control_left)
    pubMove(22, pub, control_backwards)


    rospy.loginfo("实验二： 前进20s后， 原地旋转， 继续前进20s \n")
    pubMove(20, pub, control_forwards)
    pubTurnAround(2, pub, control_revolve)
    pubMove(20, pub, control_backwards)




#!/usr/bin/env python
# -*- coding: utf-8 -*
 
import  os
import  sys
import  tty, termios
import roslib
import rospy
from Keyboard import Keyboard
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)		#3
grasp_pub = rospy.Publisher('/grasp', String, queue_size=0)

global can_grasp
global can_release

def grasp_status_cp(msg):
    global can_release,can_grasp
    # 物体抓取成功,让机器人回起始点
    if msg.data=='1':
        can_release=True
    if msg.data=='0' or msg.data=='-1':
        can_grasp=True
grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=1)

def keyboardLoop():
    rospy.init_node('teleop')
    #初始化监听键盘按钮时间间隔
    rate = rospy.Rate(rospy.get_param('~hz', 100)) 	#由于后续将sleep注释掉了,因此无间隔
 
    #速度变量
    # 慢速
    walk_vel_ = rospy.get_param('walk_vel', 0.1)
    # 快速
    run_vel_ = rospy.get_param('run_vel', 0.5)	#1.0
    # 慢转
    yaw_rate_ = rospy.get_param('yaw_rate', 0.3)	#1
    # 快转
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 3)	#1.5#1
    # max_tv*speed是最终的移速,max_rv*turn是最终的转速
    max_tv = walk_vel_
    max_rv = yaw_rate_
    # 参数初始化
    global can_release,can_grasp
    can_grasp=True
    can_release=False

    keyboard = Keyboard()

    print "使用[WASD]控制机器人"
    print "使用[jikl]控制机器人微小转动"
    print "按[g]抓取物体，按[f]放下物体"
    print "按[h]扔物体，按[b]机械臂复位" 
    print "按[c]进入左扫物块模式，按[v]进入右扫物块模式"
    print "按[=]退出"
 
    count = 0 
    speed = 0
    turn = 0
    max_tv = 0
    max_rv = 0
    #读取按键循环
    while not rospy.is_shutdown():

        if keyboard.isPressAndWaitForRelease('g'):
            if can_grasp:
                msg=String()
                msg.data='1'
                grasp_pub.publish(msg)
                can_grasp=False
        # 放下物体,对应data为0
        elif keyboard.isPressAndWaitForRelease('f'):
            if can_release:
                msg=String()
                msg.data='0'
                grasp_pub.publish(msg)
                can_release=False

        # 丢下物体,对应data为2
        elif keyboard.isPressAndWaitForRelease('h'):
            if can_release:
                msg=String()
                msg.data='2'
                grasp_pub.publish(msg)
                can_release=False
        # 注意：此后我们添加的功能都没有是否能release的判断，也就是无论是否抓到物块都执行
        # 机械臂复位,对应data为3
        elif keyboard.isPressAndWaitForRelease('b'):
            msg=String()
            msg.data='3'
            #对抓取的的布尔变量重置
            can_release=False
            can_grasp=True 
            grasp_pub.publish(msg)
        #左扫物块模式,对应data为4
        elif keyboard.isPressAndWaitForRelease('c'):
            msg=String()
            msg.data='4'
            grasp_pub.publish(msg)
        #右扫物块模式,对应data为5
        elif keyboard.isPressAndWaitForRelease('v'):
            msg=String()
            msg.data='5'
            grasp_pub.publish(msg)
        # 位移
        
        if keyboard.isPress('w'):
            max_tv = run_vel_
            speed = 1
            #print "w"

        elif keyboard.isPress('s'):
            max_tv = run_vel_
            speed = -1
            #print "s"

        else:
            speed = 0 

        if keyboard.isPress('a'):
            max_rv = yaw_rate_run_
            turn = 1
            #print "a"

        elif keyboard.isPress('d'):
            max_rv = yaw_rate_run_
            turn = -1
            #print "d"

        else:
            turn = 0 

        #shift 慢速模式
        if keyboard.isPress(keyboard.Key.shift):
            max_tv = walk_vel_
            max_rv = yaw_rate_
 
        #发送消息
        cmd.linear.x = speed * max_tv
        #print count
        #count = count+1
        cmd.angular.z = turn * max_rv
        #print "v:"+str(speed * max_tv) + "  a:"+str(turn * max_rv)
        pub.publish(cmd)
        #rate.sleep()
		#停止机器人
        #stop_robot()
        rate.sleep()
 
def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
 
if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass

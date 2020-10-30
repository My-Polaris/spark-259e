#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import random
import ctypes
import roslib
import rospy
import smach
import smach_ros
import threading
import thread
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *
import threading
from swiftpro.msg import status as SwiftProStatus
class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''

        global xc, yc, xc_prev, yc_prev, found_count, detect_color_blue
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        detect_color_blue=False
        self.is_found_object = False
        # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
        thread1 = threading.Thread(target=self.image_cb,)
        thread1.setDaemon(True)
        thread1.start()
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        r1 = rospy.Rate(1)
        r1.sleep()
        pos = position()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        self.pub1.publish(pos)

    def grasp_cp(self, msg):
        global detect_color_blue
        rospy.loginfo("recvice grasp command:%s",msg.data)
        if msg.data == '1':
            # 订阅摄像头话题,对图像信息进行处理
            #self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
            detect_color_blue=True
            self.is_found_object = False
            rate = rospy.Rate(10)
            times=0
            steps=0
            while not self.is_found_object:
                rate.sleep()
                times+=1
                # 转一圈没有发现可抓取物体,退出抓取
                if steps>=1:
                    #self.sub.unregister()
                    print("stop grasp\n")
                    detect_color_blue=False
                    status=String()
                    status.data='-1'
                    self.grasp_status_pub.publish(status)
                    return
                # 旋转一定角度扫描是否有可供抓取的物体
                if times>=30:
                    times=0
                    steps+=1
                    self.turn_body()
                    print("not found\n")
            print("unregisting sub\n")
            # 抓取检测到的物体 
            detect_color_blue=False   
            self.grasp()
            status=String()
            status.data='1'
            self.grasp_status_pub.publish(status) 
        if msg.data=='0':
            # 放下物体
            self.is_found_object = False
            detect_color_blue=False
            self.release_object()
            status=String()
            status.data='0'
            
            self.grasp_status_pub.publish(status)
        # 丢下物块
        if msg.data=='2':
            # 放下物体
            self.is_found_object = False
            detect_color_blue=False
            self.release_object_2()
            status=String()
            status.data='0'
            
            self.grasp_status_pub.publish(status)
        # 机械臂复位
        if msg.data=='3':
            # 机械臂恢复
            status = SwiftProStatus()
            status_pub = rospy.Publisher('/swiftpro_status_topic',SwiftProStatus,queue_size=1)
            rate = rospy.Rate(5)
            status.status = 0
            status_pub.publish(status)
            rate.sleep()
            status.status = 1
            status_pub.publish(status)
            rate.sleep()
            pos = position()
            pos.x = 120
            pos.y = 0
            pos.z = 35
            self.pub1.publish(pos)
        # 扫物块模式,x是远近(越远数值越大),y是左右(左正右负),z是高度(越高数值越大)
        if msg.data=='4':
            #左扫
            pos = position()
            pos.x = 160
            pos.y = 208
            pos.z = -118
            self.pub1.publish(pos)
        if msg.data=='5':
            #右扫
            pos = position()
            pos.x = 160
            pos.y = -208
            pos.z = -118
            self.pub1.publish(pos)

    # 执行抓取
    def grasp(self):
        rospy.loginfo("start to grasp\n")
        global xc, yc, found_count
        # stop function

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        print(s)
        arr=s.split()
        a1=arr[0]
        a2=arr[1]
        a3=arr[2]
        a4=arr[3]
        a = [0]*2
        b = [0]*2
        a[0]=float(a1)
        a[1]=float(a2)
        b[0]=float(a3)
        b[1]=float(a4)
        print('k and b value:',a[0],a[1],b[0],b[1])
        r1 = rospy.Rate(10)
        r2 = rospy.Rate(10)
        pos = position()
        # 物体所在坐标+标定误差
        pos.x = a[0] * yc + a[1]
        pos.y = b[0] * xc + b[1]
        pos.z = 20
         # pos.z = 20
        print("z = 20\n")
        self.pub1.publish(pos)
        r2.sleep()
        # go down -100
        pos.z = -50
        self.pub1.publish(pos)
        print("z = -83\n")
        r2.sleep()

        # 开始吸取物体
        self.pub2.publish(1)
        r2.sleep()

        # 提起物体
        pos.x = 200  #160
        pos.y = 0
        pos.z = 95  #55
        self.pub1.publish(pos)
        r1.sleep()

    def hsv_value(self):
        try:
            filename = os.environ['HOME'] + "/color_block_HSV.txt"
            with open(filename, "r") as f:
                for line in f:
                    split = line.split(':')[1].split(' ')
                    lower = split[0].split(',')
                    upper = split[1].split(',')
                    for i in range(3):
                        lower[i] = int(lower[i])
                        upper[i] = int(upper[i])

            lower = np.array(lower)
            upper = np.array(upper)
        except:
            raise IOError('could not find hsv_value file : {},please execute #13 command automatically '
                          'generate this file'.format(filename))

        return lower, upper

    # 使用CV检测物体       
    def image_cb(self):
        global xc, yc, xc_prev, yc_prev, found_count, detect_color_blue	
        capture = cv2.VideoCapture(0)
        LowerBlue, UpperBlue = self.hsv_value()

        while True:
            # time.sleep(0.08)
            ret,frame = capture.read()
            cv_image2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)	    #第一个参数是原图 第二个是低于lower_blue的值 第三个是高于upper_blue的值  该函数将之间的所有颜色都去掉只识别在该范围类的颜色	
            mask = cv2.erode(mask, None, iterations=5)              #使图像加上高斯模糊使图像色彩更加突出，色彩追终更加准确
            #mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.GaussianBlur(mask, (9,9), 0)                 #该函数用于图像的减噪过程
            # detect contour
            cv2.imshow("win2", mask)                                #将去掉颜色的图像显示在win2这个窗口上
            cv2.waitKey(1)                                          #功能是不断的刷新win2这个窗口，频率为delay
            #if detect_color_blue :                                  #这个布尔变量表示 是否探测到符合的蓝色    
            _, contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)      #第一个参数是带有参数的图像 第二个参数cv2.RETR_TREE是提取轮廓信息 第三个参数是指定轮廓的近似方法 
            if len(contours) > 0:    #len计算contour里面的元素个数                                    #第一个返回值 _是二值图（这里没用到） 第二个值counters是一个列表每个元素是（x,1,2）x是每个元素的边缘像素点的多少                    
                size = []                                                                           # 1 不知道什么意思 ， 2文档上说的是代表每个点的横纵坐标                        
                size_max = 0
                distance_list = []
                max_distance = 450      #定义一个最远距离 
                min_distance = 450      #定义一个最远距离   

                for i, c in enumerate(contours):    #enumerate枚举的意思 将contours里的各个元素都提取出来
                    rect = cv2.minAreaRect(c)       #生成最小的外接矩阵里面包含该矩阵的中心点坐标，高度，宽度等信息
                    box = cv2.boxPoints(rect)       #获取生成的最小矩阵的四个顶点坐标
                    box = np.int0(box)              
                    x_mid, y_mid = rect[0]

                    w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)      #因为矩形的边并不是平行或者垂直于屏幕的于是要通过一系列的计算算出
                    h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)      #该外接矩形的长和宽
                    if int(w*h) < 10000 :
                        continue 
                    
                    size.append(w * h)        #将外接矩阵的面积放入这个列表之中                        
                    # 所有点到spark的距离
                    distance_list.append(math.sqrt((320 - x_mid) ** 2 + (300 - y_mid) ** 2))

                    #信息
                    str_x = "x_mid: "+str(int(x_mid))
                    str_y = "y_mid: "+str(int(y_mid))
                    str_A = "Area: "+str(int(w*h))
                    str_D = "dist: "+str(int(math.sqrt((320 - x_mid) ** 2 + (300 - y_mid) ** 2)))
                    #画图                    
                    frame = cv2.drawContours(frame,[contours[i]],0,(0,255,0),3)
                    frame = cv2.putText(frame,str_x, (box[3][0],box[3][1]), cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 255), 1)
                    frame = cv2.putText(frame,str_y, (box[3][0],box[3][1]+10), cv2.FONT_HERSHEY_COMPLEX,0.3, (0, 255, 255), 1)
                    frame = cv2.putText(frame,str_A, (box[1][0],box[1][1]+20), cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 255), 1)
                    frame = cv2.putText(frame,str_D, (box[3][0],box[3][1]+30), cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 255), 1)

                    #画四个点
                    #cv2.circle(frame, (np.int32(box[0][0]), np.int32(box[0][1])), 2, (255, 233, 0), 2, 8, 0)
                    #cv2.circle(frame, (np.int32(box[1][0]), np.int32(box[1][1])), 2, (255, 233, 0), 2, 8, 0)
                    #cv2.circle(frame, (np.int32(box[2][0]), np.int32(box[2][1])), 2, (255, 233, 0), 2, 8, 0)
                    #cv2.circle(frame, (np.int32(box[3][0]), np.int32(box[3][1])), 2, (255, 233, 0), 2, 8, 0)                    
                    
                    if distance_list[-1] < max_distance and distance_list[-1]<min_distance:
                        #修改微调值
                        x_re = 0 
                        y_re = 0 
                        if size[-1]>20000 :
                            x_re = int(w/8)
                            y_re = int(h/16)
                        size_max = size[-1]
                        min_distance = distance_list[-1]
                        index = len(distance_list) -1 
                        xc = x_mid+x_re
                        yc = y_mid+y_re
                        cv2.circle(frame, (np.int32(xc), np.int32(yc)), 2, (255, 0, 0), 2, 8, 0)

                    
                if found_count >= 15 and min_distance < 190:
                    self.is_found_object = True
                    #cmd_vel = Twist()
                    cv2.circle(frame, (np.int32(xc), np.int32(yc)), 4, (0,0,255), -1)      #将目标点显示出来
                    #self.cmd_vel_pub.publish(cmd_vel)
                else:
                    # if box is not moving
                    if abs(xc - xc_prev) <= 50 and abs(yc - yc_prev) <= 50:
                        found_count = found_count + 1
                    else:
                        found_count = 0


            else:
                found_count = 0

            xc_prev = xc
            yc_prev = yc
            cv2.imshow("win1", frame)
            cv2.waitKey(1)
            #key = cv2.waitKey(10)
 
    # 释放物体
    def release_object(self):
        r1 = rospy.Rate(1)  # 5s
        r2 = rospy.Rate(1)     # 1s
        pos = position()
        # go forward
        pos.x = 200
        pos.y = 0
        pos.z = 65  #-80
        self.pub1.publish(pos)
        r1.sleep()

        # stop pump
        self.pub2.publish(0)
        r2.sleep()
        r1.sleep()
        pos.x = 120
        pos.y = 0
        pos.z = 50
        #self.pub1.publish(pos)
        r1.sleep()
        return 'succeeded'

    # 释放物体
    def release_object_2(self):
        r1 = rospy.Rate(1)  # 5s
        r2 = rospy.Rate(1)     # 1s
        pos = position()
        # go forward
        #pos.x = 200
        #pos.y = 0
        #pos.z = -40  #-80
        # stop pump
        self.pub2.publish(0)
        #self.pub1.publish(pos)
        r1.sleep()

        #r2.sleep()
        #r1.sleep()
        pos.x = 120
        pos.y = 0
        pos.z = 50
        self.pub1.publish(pos)
        r1.sleep()
        return 'succeeded'

    # 转动机器人到一定角度       
    def turn_body(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 1.2
        #rate = rospy.Rate(10)
        for i in range(1):            
            self.cmd_vel_pub.publish(cmd_vel)            
            #rate.sleep()
       

        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")


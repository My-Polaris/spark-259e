#!/usr/bin/env python
# -*- coding: UTF-8 -*-
 
import rospy  
import actionlib  
from actionlib_msgs.msg import *
from std_msgs.msg import String  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from tf.transformations import quaternion_from_euler  
from visualization_msgs.msg import Marker  
from math import radians, pi  
 
class Move2Grasp():  
    def __init__(self):  
        rospy.init_node('move2grasp', anonymous=False)  
 
        rospy.on_shutdown(self.shutdown)
        #订阅RVIZ上的点击事件
	rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        #订阅机械臂抓取状态
        rospy.Subscriber('/grasp_status', String, self.grasp_status_cp, queue_size=1)
        # Publisher to manually control the robot (e.g. to stop it)  
        # 发布TWist消息控制机器人  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 发布机械臂抓取指令 
        self.grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)
 
        # Subscribe to the move_base action server  
        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
 
        rospy.loginfo("Waiting for move_base action server...")  
 
        # Wait 60 seconds for the action server to become available  
        # 等待move_base服务器建立  
        self.move_base.wait_for_server(rospy.Duration(60))  
 
        rospy.loginfo("Connected to move base server")  
        rospy.loginfo("Starting navigation test")  
  
 
    def cp_callback(self, msg):
            rospy.loginfo("POINT:%f,%f,%f", msg.point.x, msg.point.y, msg.point.z)

            # Intialize the waypoint goal  
            # 初始化goal为MoveBaseGoal类型  
            goal = MoveBaseGoal()  
 
            # Use the map frame to define goal poses  
            # 使用map的frame定义goal的frame id  
            goal.target_pose.header.frame_id = 'map'  
 
            # Set the time stamp to "now"  
            # 设置时间戳  
            goal.target_pose.header.stamp = rospy.Time.now()  
 
            # Set the goal pose 
            # 设置目标点  
            #pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
            pose = Pose(Point(msg.point.x,msg.point.y,msg.point.z), Quaternion(0.0, 0.0, 0.0, 1.0))
            goal.target_pose.pose=pose  
 
            # Start the robot moving toward the goal  
            # 机器人移动  
            status=self.move(goal)
            # 如果到达指定地点,就发送抓取指令
            if status==True:
                print('goal reached and start grasp')
                msg=String()
                msg.data='1'
                self.grasp_pub.publish(msg)

    # 使用CV检测物体       
    def box_cb(self):
        if found_count >= 15 and min_distance < 190:
            self.is_found_object = True
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

            self.pub2.publish(0)
            r2.sleep()
            r1.sleep()
            pos.x = 120
            pos.y = 0
            pos.z = 50
            #self.pub1.publish(pos)
            r1.sleep()
        return 'succeeded'
        else:
            pass



    def grasp_status_cp(self, msg):
            # 物体抓取成功,让机器人回起始点
            if msg.data=='1': 
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map' 
                goal.target_pose.header.stamp = rospy.Time.now() 
                pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0, 0.0, 0.0, 5.0))
                goal.target_pose.pose=pose  
                status=self.move(goal)
                #status=self.move(goal)
                # 到达起始点,放下物体
                if status==True:
                    if self.box_cb():
                        pass
                    else:
                        msg=String()
                        msg.data='0'
                        self.grasp_pub.publish(msg)
                


    def move(self, goal):  
            # Send the goal pose to the MoveBaseAction server  
            # 把目标位置发送给MoveBaseAction的服务器  
            self.move_base.send_goal(goal)  
 
            # Allow 1 minute to get there  
            # 设定1分钟的时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))   
 
            # If we don't get there in time, abort the goal  
            # 如果一分钟之内没有到达，放弃目标  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")
                return False  
            else:  
                # We made it!  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!!!!!!!")
                    return True  
 
 
 
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        # Cancel any active goals  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        # Stop the robot  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
 
if __name__ == '__main__':  
    try:  
        Move2Grasp()
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Move2grasp finished.")

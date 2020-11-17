import rospy
import tf
import tf2_ros
import math
from math import radians, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped

class RT():
	def __init__(self):
        #监听者
		self.tf_listener = tf.TransformListener()
        #这个发布者用来实时发布自身相对于“世界”的位置
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        #这个发布者用来发布目标点相对于“世界”的位置
		self.tf_staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
	
	# return negative mean right
	def angelFromXY(self, x, y):
		if x == 0:
			if y > 0:
				yaw = pi / 2
			elif y < 0:
				yaw = -pi / 2
			elif y == 0:
				yaw = 0
		else:
			yaw = math.atan(y / x)

		if x < 0:
			yaw += pi

		return self.standardRadian(-yaw)

	# return negative mean right
	def yawBetweenPoints(self, from_point, to_point):
		delta_x = to_point.x - from_point.x
		delta_y = to_point.y - from_point.y
		return self.angelFromXY(delta_x, delta_y)

	def yawBetweenPoseAndPoint(self, from_pose, to_point):
		yaw_now = self.quaternion2Euler(from_pose.orientation)[2]
		yaw_between = self.yawBetweenPoints(from_pose.position, to_point)
		return self.standardRadian(yaw_now + yaw_between)

	def standardRadian(self, radian):
		radian += pi
		radian %= 2 * pi
		radian -= pi
		return radian

	def radian2Angel(self, radian):
		return (radian / pi * 180)

	def angel2Radian(self, angel):
		return (angel * pi / 180)

    #将点位置信息变成 Point对象
	def list2Point(self, d):
		return Point(d[0], d[1], d[2])

    #将位置信息变成 一个三元组
	def Point2List(self, d):
		return (d.x, d.y, d.z)

    #将姿态四元组变成 Quaternion 的格式
	def list2Quaternion(self, d):
		return Quaternion(d[0], d[1], d[2], d[3])

    #将姿态四元组变成一个 一个四元组
	def quaternion2List(self, d):
		return (d.x, d.y, d.z, d.w)

    #将姿态信息转换成 quaternion2Euler 的格式
	def quaternion2Euler(self, d):
		return euler_from_quaternion(self.quaternion2List(d))
		
    #将姿态信息 i j k 打包成一个Quaternion格式
	def eulerValue2Quaternion(self, i, j, k):
		return self.list2Quaternion(quaternion_from_euler(i, j, k))
		
    #将姿态信息 i j k 打包成一个quaternion_from_euler格式
	def eulerValue2QuaternionList(self, i, j, k):
		return quaternion_from_euler(i, j, k)

    #将位置 p q 打包成pose
	def pq2Pose(self, p, q):
		return Pose(p, q)

    #监听坐标变化
	def get_ll(self, from_frame, to_frame):
		rate = rospy.Rate(10)
		while True:
			rate.sleep()
			try:
				return self.tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
    
    #该函数用来获取点的坐标 与 自身姿态
	def get_pq(self, from_frame, to_frame):
		(point, quaternion) = self.get_ll(from_frame, to_frame)
		return self.list2Point(point), self.list2Quaternion(quaternion)
		
    #获取位置
	def get_pose(self, from_frame, to_frame):
		(point, quaternion) = self.get_pq(from_frame, to_frame)
		return self.pq2Pose(point, quaternion)

	#用于迭代位置  将自己新的位置和新的姿态进行封装加上时间返回
    def generate(self, from_frame, to_frame, point, quaternion):
		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = from_frame
		t.child_frame_id = to_frame
		t.transform.translation = point
		t.transform.rotation = quaternion
		return t

    #该函数用来发布 自身 位置与姿态 的变化给“世界”
	def publish_pq(self, from_frame, to_frame, point, quaternion):
		self.tf_broadcaster.sendTransform(self.generate(from_frame, to_frame, point, quaternion))
		
    #对上一个函数的封装 在其他文件引用时代码可读性增加    
	def publish_pose(self, from_frame, to_frame, pose):
		self.publish_pq(from_frame, to_frame, pose.position, pose.orientation)

    #发布机器人的位置及姿态的变化
	def publish_ll(self, from_frame, to_frame, point, quaternion):
		self.publish_pq(from_frame, to_frame, self.list2Point(point), self.list2Quaternion(quaternion))

    #设定目标点
	def static_pq(self, from_frame, to_frame, point, quaternion):
		self.tf_staticBroadcaster.sendTransform(self.generate(from_frame, to_frame, point, quaternion))
    
    #对上一个函数的封装同上
	def static_pose(self, from_frame, to_frame, pose):
		self.static_pq(from_frame, to_frame, pose.position, pose.orientation)

    #定义一个目标点封装的函数
	def static_ll(self, from_frame, to_frame, point, quaternion):
		self.static_pq(from_frame, to_frame, self.list2Point(point), self.list2Quaternion(quaternion))
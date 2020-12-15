#!/usr/bin/python

import rospy as rp
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class SensorInput:
	def __init__(self):
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.odom_sub = rp.Subscriber('/odom', Odometry, self.odometry_callback)


	def odometry_callback(self, data):
		"""
			Callback function for odometry data. Quaternion pose is converted
			to Euler angles for use in the controller.
		"""
		orientation_quat = data.pose.pose.orientation
		self.roll, self.pitch, self.yaw = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])

		# rp.loginfo('RPY: {}, {}, {}'.format(self.roll, self.pitch, self.yaw))

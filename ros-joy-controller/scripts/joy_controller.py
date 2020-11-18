#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class JoyController:
	def __init__(self):

		# Parameters
		self.param_topic_joy = "/controller/xbox"
		self.param_topic_twist = "/frobit/twist"
		
		self.param_publish_rate = 10

		self.param_lin_speed_max =  1.0 # m/s
		self.param_rot_speed_max =  0.04 # rad/s

		# Create node
		rospy.init_node("Frobit_driver")

		# Subscribers 
		rospy.Subscriber(self.param_topic_joy, Joy, self.cb_joy)

		# Publishers
		self.pub_twist = rospy.Publisher(self.param_topic_twist, TwistStamped,queue_size = 1)

		# Internal 
		self.joy_data = None

	def cb_joy(self, data):
		self.joy_data = data

	def spin(self):
		r = rospy.Rate(self.param_publish_rate)
		msg_twist_out = TwistStamped()
		while not rospy.is_shutdown():
			if self.joy_data == None:
					rospy.loginfo("Waiting for joy message to be received ...")
					rospy.sleep(2)
					continue
			
			lin_vel_factor = self.joy_data.axes[1]
			lin_rot_factor = self.joy_data.axes[0]

			lin_vel_scaled = lin_vel_factor * self.param_lin_speed_max
			rot_vel_scaled = lin_rot_factor * self.param_rot_speed_max

			msg_twist_out.twist.linear.x  = lin_vel_scaled
			msg_twist_out.twist.angular.z = rot_vel_scaled

			msg_twist_out.header.stamp = rospy.Time.now()

			self.pub_twist.publish(msg_twist_out)

			r.sleep()
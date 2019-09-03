#!/usr/bin/env python

import rospy
from math import *

import actionlib
from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import Empty
from contrail.msg import TrajectoryAction, TrajectoryGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

class NavigationInterface():
	def __init__(self):
		self.current_x = 0.0
		self.current_y = 0.0
		self.current_z = 0.0

		self.nom_lvel = 0.1
		self.p_start = Vector3(x=0.0,y=0.0,z=1.0)
		self.yaw_start = 0.0
		self.p_end = Vector3(x=3.0,y=0.0,z=1.0)
		self.yaw_end = 0.0

		rospy.loginfo("Waiting for contrail")
		action_topic = rospy.get_param("~action_topic", '~contrail')
		rospy.loginfo(action_topic)
		self.client_base = actionlib.SimpleActionClient(action_topic, TrajectoryAction)
		self.client_base.wait_for_server()

		self.sub_trig = rospy.Subscriber('~imagery_trigger', Empty, self.callback_trigger)
		self.sub_pose = rospy.Subscriber('~pose', PoseStamped, self.callback_pose)

		self.send_wp_path(self.p_start, self.yaw_start, self.p_end, self.yaw_end, self.nom_lvel)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_trig.unregister()
		self.sub_pose.unregister()

	def send_wp_path(self, ps, ys, pe, ye, lv):
		# Function to send a new path directly to contrail
		goal_base = TrajectoryGoal()

		goal_base.positions.append(ps)
		goal_base.positions.append(pe)
		goal_base.yaws.append(ys)
		goal_base.yaws.append(ye)

		dx = pe.x - ps.x
		dy = pe.y - ps.y
		dz = pe.z - ps.z
		dt = sqrt((dx*dx)+(dy*dy)+(dz*dz)) / lv
		goal_base.duration = rospy.Duration.from_sec(dt)

		goal_base.start = rospy.Time(0)
		self.client_base.send_goal(goal_base)

	def callback_pose(self, msg_in):
		# Store the current position at all times so it can be accessed later
		self.current_pos = Vector3(msg_in.pose.position.x, msg_in.pose.position.y, msg_in.pose.position.z)

	# Callback to handle an alert from image processing that the target is found
	def callback_trigger(self, msg_in):
		rospy.loginfo("[NAV] Got imagery trigger!")
		rospy.loginfo("[NAV] Stopping for 5 seconds")

		self.client_base.cancel_goal()
		rospy.sleep(rospy.Duration(5.0))

		rospy.loginfo("[NAV] Continuing to end point...")

		self.send_wp_path(self.current_pos, self.yaw_start, self.p_end, self.yaw_end, self.nom_lvel)

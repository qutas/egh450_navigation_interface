#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from contrail_msgs.msg import DiscreteProgress
from contrail_msgs.srv import SetTracking, SetTrackingRequest

class NavigationInterface():
	def __init__(self):
		# Needs to be connected to contrail
		self.sub_trig = rospy.Subscriber('~imagery_trigger', Empty, self.callback_trigger)
		self.sub_prog = rospy.Subscriber('~discrete_progress', DiscreteProgress, self.callback_progress)

		# Prepare diversion logic
		self.on_diversion = False
		self.pub_divert = rospy.Publisher('~pose', PoseStamped, queue_size=10)

		# Wait for the contrail interface to start up
		# then prepare a Service Client
		rospy.loginfo("[NAV] Waiting to connect with Contrail")
		rospy.wait_for_service('~set_tracking')
		self.srv_track = rospy.ServiceProxy('~set_tracking', SetTracking)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_trig.unregister()
		self.sub_prog.unregister()

	# Callback to handle an alert from image processing that the target is found
	def callback_trigger(self, msg_in):
		rospy.loginfo("[NAV] Got imagery trigger, setting diversion...")

		msg_out = PoseStamped()
		msg_out.header.stamp = rospy.Time.now()
		msg_out.header.frame_id = "map"
		msg_out.pose.position.x = 1.0
		msg_out.pose.position.y = 1.0
		msg_out.pose.position.z = 1.5
		msg_out.pose.orientation.w = 1.0
		msg_out.pose.orientation.x = 0.0
		msg_out.pose.orientation.y = 0.0
		msg_out.pose.orientation.z = 0.0

		# Publish the diversion and set the flag for the progress callback
		self.pub_divert.publish(msg_out)
		self.on_diversion = True

	# Callback to handle progress updates from Contrail
	def callback_progress(self, msg_in):
		#If we were on a diversion, and have reached the drop point
		if self.on_diversion and (msg_in.progress == 1.0):
			rospy.loginfo("[NAV] Rached drop point, requesting path continue...")

			# Disable further tracking changes (unless re-triggered)
			self.on_diversion = False

			# Send a request to contrail to resume path tracking
			req = SetTrackingRequest()
			req.tracking = req.TRACKING_PATH
			res = self.srv_track(req)

			if res.success:
				rospy.loginfo("[NAV] Diversion complete!")
			else:
				rospy.logerr("[NAV] Could not return to path tracking!")
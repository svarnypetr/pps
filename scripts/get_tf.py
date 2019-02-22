#!/usr/bin/env python  
import rospy
import math
import tf
# std_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import csv

if __name__ == '__main__':
	# row = ["timestamp", "x", "y", "z", "rx", "ry", "rz", "rw"]
	rospy.init_node("get_tfs")
	listener = tf.TransformListener()

	# f = open("tf_data.csv", "a")

	publisher = rospy.Publisher('ee_position', PoseStamped, queue_size=10)
	position = PoseStamped()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/r1_link_0', '/r1_ee', rospy.Time(0))
			position.header.stamp = rospy.Time(0)
			position.pose.position = trans
			position.pose.orientation = rot
			publisher.publish(position)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue



		# f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(
		# 	rospy.Time.now(), trans[0], trans[1], trans[2], rot[0], rot[1],
		# 	rot[2], rot[3]
		# ))
		
		rate.sleep()

	f.close()

#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg
import csv

if __name__ == '__main__':
	row = ["timestamp", "x", "y", "z", "rx", "ry", "rz", "rw"]
	rospy.init_node("get_tfs")
	listener = tf.TransformListener()

	f = open("tf_data.csv", "a")

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/r1_link_0', '/r1_ee', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(
			rospy.Time.now(), trans[0], trans[1], trans[2], rot[0], rot[1],
			rot[2], rot[3]
		))
		
		rate.sleep()

	f.close()

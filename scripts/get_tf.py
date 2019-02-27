#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
	rospy.init_node("get_tfs")
	listener = tf.TransformListener()

	# publisher = rospy.Publisher('ee_position', PoseStamped, queue_size=10)
	# position = PoseStamped()

	f = open("ee_positions.csv", "a")
	# f.write(",".join(row)+"\n")

	rate = rospy.Rate(100.0)
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/r1_link_0', '/r1_ee', rospy.Time(0))
			# position.header.stamp = rospy.Time.now()
			# position.pose.position.x = trans[0]
			# position.pose.position.y = trans[1]
			# position.pose.position.z = trans[2]
			# position.pose.orientation.x = rot
			# publisher.publish(position)
			f.write("{}, {}, {}, {}\n".format(
				rospy.Time.now(), trans[0], trans[1], trans[2]
			))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		rate.sleep()

f.close()

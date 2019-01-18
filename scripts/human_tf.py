#!/usr/bin/python

import rospy
import tf
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)


    while not rospy.is_shutdown() and keypoint.any():
        br.sendTransform(tuple(keypoint),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "neck",
                         "camera")
        rate.sleep()
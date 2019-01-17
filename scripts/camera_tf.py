#!/usr/bin/python

import rospy
import tf
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((1.0, -1.0, 1.0),
                         quaternion_from_euler(0, 1.5707, 2*1.5707),
                         rospy.Time.now(),
                         "camera",
                         "r2_link_0")
        rate.sleep()

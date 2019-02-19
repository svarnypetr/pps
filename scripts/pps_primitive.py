#!/usr/bin/env python

import json

import numpy as np

import rospy
import tf
from std_msgs.msg import Int8


if __name__ == "__main__":
    node_name = 'pps'
    rospy.init_node(node_name)

    publisher = rospy.Publisher("pps_status", Int8, queue_size=100)
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (transform, rotation) = listener.lookupTransform('/7', '/r1_ee', rospy.Time(0))
            print('found transform')
            if np.linalg.norm(transform) < 0.6:
                # STOP
                publisher.publish(2)
            else:
                # ALL OK
                publisher.publish(0)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            publisher.publish(1)

        rate.sleep()

    rospy.spin()

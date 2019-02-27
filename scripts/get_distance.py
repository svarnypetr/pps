#!/usr/bin/env python
import rospy
import math
import tf
import geometry_msgs.msg
import csv
import numpy as np

if __name__ == '__main__':
    # row = ["timestamp", "x", "y", "z", "rx", "ry", "rz", "rw"]
    row = ["timestamp", "left_ee_distance", "nose_ee_distance", "left_3_distance",
           "nose_3_distance", "left_0_distance", "nose_0_distance"]
    rospy.init_node("get_distances")
    listener = tf.TransformListener()

    f = open("distances.csv", "a")
    # f.write(",".join(row)+"\n")

    pairs = [('/7', '/r1_ee'), ('/0', '/r1_ee'), ('/7', '/r1_link_3'),
             ('/0', '/r1_link_3'), ('/7', '/r1_link_0'), ('/0', '/r1_link_0')]

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pair_distances = []
        for pair in pairs:
            try:
                (trans, rot) = listener.lookupTransform(pair[0], pair[1], rospy.Time(0))
                pair_distances.append(np.linalg.norm(trans))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pair_distances.append(0)
                continue

        if not (0 in pair_distances):
            f.write("{}, {}, {}, {}, {}, {}, {}\n".format(
                rospy.Time.now(), pair_distances[0], pair_distances[1], pair_distances[2], pair_distances[3],
                pair_distances[4], pair_distances[5]
            ))
        rate.sleep()

    f.close()

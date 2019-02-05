#!/usr/bin/env python

import json

import numpy as np

import rospy
import tf
from coefficient_generator import CoefficientGenerator
from rospy_message_converter import message_converter
from std_msgs.msg import String, Int8


class PeriPersonalSpaceChecker(object):
    """
    Checks if robots PPS was interfered
    """
    def __init__(self,
                 pair_array=np.nan,
                 keypoints=[],
                 topic_alert='pps_alert',
                 topic_status='pps_status',
                 stop_threshold=1.0,
                 warning_threshold=2.0,
                 ):
        self.keypoints = keypoints
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher(topic_alert, String, queue_size=10)
        self.publisher_status = rospy.Publisher(topic_status, Int8, queue_size=10)
        self.current_positions = np.nan
        self.distance_mat = np.nan

        self.stop_threshold = stop_threshold
        self.warning_threshold = warning_threshold

    @staticmethod
    def make_combinations(keypoints):
        """
        Makes a combination of all keypoints with all keypoints.
        :param keypoints: np.array with column of robot and obstacle keypoints
        :return: np.array of keypoint pairs with combination all with all
        """
        pairs = np.array([["", ""]])
        for robot_keypoint in keypoints[0]:
            for obstacle_keypoint in keypoints[1]:
                pairs = np.append(pairs, [[robot_keypoint, obstacle_keypoint]], axis=0)
        return pairs[1:, :]

    def check_pps(self):

        pairs = self.make_combinations(self.keypoints)

        pair_states = []
        for pair in pairs:
            try:
                (transform, rotation) = self.listener.lookupTransform(pair[0], pair[1], rospy.Time(0))
                print('found transform')
                if np.linalg.norm(transform) < 0.6:
                    # STOP
                    pair_states.append(2)
                else:
                    # ALL OK
                    pair_states.append(0)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pair_states.append(1)

        print('pair and their statuses: {} {}'.format(pairs, pair_states))
        self.publisher_status.publish(max(pair_states))


if __name__ == "__main__":
    node_name = 'pps'
    rospy.init_node(node_name)

    keypoints = [['/r1_ee', '/r1_link_0'], ['/7']]
    pps = PeriPersonalSpaceChecker(keypoints=keypoints)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        pps.check_pps()

        rate.sleep()

    rospy.spin()

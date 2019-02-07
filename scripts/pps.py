#!/usr/bin/env python

import json

import numpy as np

import rospy
import tf

from std_msgs.msg import String, Int8
from pps_setup import CoefficientGenerator


class PeriPersonalSpaceChecker(object):
    """
    Checks if robots PPS was interfered
    """
    def __init__(self,
                 pairs=np.nan,
                 keypoints=[],
                 topic_alert='pps_message',
                 topic_status='pps_status',
                 stop_threshold=1.0,
                 warning_threshold=2.0,
                 ):
        self.keypoints = keypoints
        self.pairs = pairs
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

    def construct_pps_message(self, pairs, pair_states):
        max_status = max(pair_states)
        max_pairs = [x[0] for x in zip(pairs, pair_states) if x[1] == max_status]
        verbose = {0: 'OK',
                   1: 'WARNING',
                   2: 'STOP'
                   }
        pps_message_output = {'status': max_status,
                              'pairs': max_pairs,
                              'status_verbose': verbose[max_status]
                              }

        return pps_message_output

    def check_pps(self):
        pairs = self.make_combinations(self.keypoints)

        pair_states = []
        for pair in pairs:
            try:
                (transform, rotation) = self.listener.lookupTransform(pair[0], pair[1], rospy.Time(0))
                print('found transform')  # WIP
                if np.linalg.norm(transform) < 0.6:
                    # STOP
                    pair_states.append(2)
                else:
                    # ALL OK
                    pair_states.append(0)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pair_states.append(1)

        print('pair and their statuses: {} {}'.format(pairs, pair_states))  # WIP
        self.publisher_status.publish(max(pair_states))
        pps_message = self.construct_pps_message(pairs, pair_states)
        self.publisher.publish(str(pps_message))


if __name__ == "__main__":
    node_name = 'pps'
    rospy.init_node(node_name)

    keypoints = [['/r1_ee', '/r1_link_0'], ['/7', '/0']]
    pps = PeriPersonalSpaceChecker(keypoints=keypoints)
    coeffgen = CoefficientGenerator(pps.listener, keypoints[0])

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        pps.check_pps()

        rate.sleep()

    rospy.spin()

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
                 config,
                 topic_alert='pps_message',
                 topic_status='pps_status',
                 ):
        self.keypoints = config['keypoints']
        self.pairs = self.make_combinations(self.keypoints)
        self.listener = tf.TransformListener()
        # self.publisher = rospy.Publisher(topic_alert, String, queue_size=10)
        self.publisher_status = rospy.Publisher(topic_status, Int8, queue_size=10)
        self.pps_status = ''

        self.stop_threshold = config['stop_threshold']
        self.slow_threshold = config['slow_threshold']

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

    def construct_pps_message(self, pair_states):
        max_status = max(pair_states)
        max_pairs = [x[0] for x in zip(self.pairs, pair_states) if x[1] == max_status]
        verbose = {0: 'OK',
                   1: 'SlOW',
                   2: 'STOP',
                   8: 'WARNING',
                   }
        pps_message_output = {'status': max_status,
                              'pairs': max_pairs,
                              'status_verbose': verbose[max_status]
                              }
        return pps_message_output

    def check_pps(self):

        pair_states = []
        pair_distances = []
        for pair in self.pairs:
            try:
                (transform, rotation) = self.listener.lookupTransform(pair[0], pair[1], rospy.Time(0))
                stop_threshold = max(self.stop_threshold[pair[0]], self.stop_threshold[pair[1]])
                slow_threshold = max(self.slow_threshold[pair[0]], self.slow_threshold[pair[1]])
                distance = np.linalg.norm(transform)
                pair_distances.append(distance)
                if 0 < distance < stop_threshold:
                    # STOP
                    pair_states.append(3)
                elif 0 < distance < slow_threshold:
                    # SLOW
                    pair_states.append(2)
                else:
                    # ALL OK
                    pair_states.append(1)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pair_states.append(8)

        if pair_distances:
            print(min(pair_distances))
        new_status = max(pair_states)
        if self.pps_status != new_status:
            self.publisher_status.publish(new_status)
            self.pps_status = new_status
        # pps_message = self.construct_pps_message(pair_states)
        # self.publisher.publish(str(pps_message))


def generate_uni_thresholds(keypoints, thr):
    """
    For a set of keypoints generates uniform threshold values.
    keypoints: {list} of str
    thr: {float}
    :return: {dict}
    """
    uniform_thresholds = {}
    for kpt in keypoints:
        uniform_thresholds[kpt] = thr
    return uniform_thresholds


if __name__ == "__main__":
    node_name = 'pps'
    rospy.init_node(node_name)

    all_human_keypoints = ['/'+str(x) for x in range(8)] + ['/14', '/15', '/16', '/17']
    robot_base = ['/r1_link_0']
    moving_robot = ['/r1_link_'+str(x) for x in range(3, 8)] + ['/r1_ee']
    human_hands = ['/4', '/7']

    various_thr_03 = generate_uni_thresholds(all_human_keypoints + moving_robot, 0.2)
    various_thr_03.update({'/r1_ee': 0.4, '/r1_link_8': 0.4, '/r1_link_7': 0.4, '/r1_link_6': 0.4,
                           '/r1_link_5': 0.4, '/r1_link_0': 0.4, '/r1_link_0': 0.4})

    various_thr_06 = generate_uni_thresholds(all_human_keypoints + moving_robot, 0.2)
    various_thr_06.update({'/r1_ee': 0.6, '/r1_link_8': 0.6, '/r1_link_7': 0.6, '/r1_link_6': 0.6,
                           '/r1_link_5': 0.6, '/r1_link_0': 0.6, '/r1_link_0': 0.6})

    head_thr = generate_uni_thresholds(all_human_keypoints + moving_robot, 0)
    head_thr.update({'/0': 0.6, '/1': 0.6, '/14': 0.6, '/15': 0.6, '/16': 0.6, '/17': 0.6})
    # print('head thr {}'.format(head_thr))  # WIP

    scenarios = [
                    {'keypoints': [robot_base, all_human_keypoints],
                     'stop_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, 1),
                     'slow_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, 0),
                     'name': 'scenario 0 stop zone',
                     },
                    {'keypoints': [robot_base, all_human_keypoints],
                     'stop_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, 0.6),
                     'slow_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, 1),
                     'name': 'scenario 1 warning and stop zone',
                     },
                    {'keypoints': [moving_robot, all_human_keypoints],
                     'stop_threshold': various_thr_06,
                     'slow_threshold': generate_uni_thresholds(
                                        all_human_keypoints + moving_robot, 0),
                     'name': 'scenario 2 keypoint stop',
                     },
                    {'keypoints': [moving_robot, all_human_keypoints],
                     'stop_threshold': various_thr_06,
                     'slow_threshold': various_thr_03,
                     'name': 'scenario 3 keypoint warning and stop',
                     },
                    {'keypoints': [moving_robot, all_human_keypoints],
                     'stop_threshold': head_thr,
                     'slow_threshold': various_thr_06,
                     'name': 'scenario 4 keypoint warning and stop',
                     },
                ]

    config = scenarios[0]
    '''
    Experiment scenarios 
    0 - distance only from base, stopping
    1 - distance only from base slowing down 
    2 - all keypoints taken into account, only stopping
    3 - all keypoints taken into account, slowing down
    4 - all keypoints taken into account, stopping only on head
    '''

    pps = PeriPersonalSpaceChecker(config)
    # coeffgen = CoefficientGenerator(pps.listener, config['keypoints'][0]) #DO NOT CHANGE

    RATE = rospy.Rate(10.0)
    f = open('py_data.csv', 'a')
    while not rospy.is_shutdown():

        pps.check_pps()
        #
        # Speed calculation
        # try:
        #     twist = pps.listener.lookupTwist('r1_ee', 'r1_link_0', rospy.Time(), rospy.Duration.from_sec(0.1))
        #     print(np.linalg.norm(twist[0]))
        #     # print(twist[0])
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print('none')

        RATE.sleep()

    f.close()
    rospy.spin()
#!/usr/bin/env python

import json

import numpy as np

import rospy
import tf

from std_msgs.msg import String, Int8


class PeriPersonalSpaceChecker(object):
    """
    Checks if robots PPS was interfered
    """
    def __init__(self,
                 config,
                 rate,
                 topic_status='pps_status',
                 ):
        self.keypoints = config['keypoints']
        self.pairs = self.make_combinations(self.keypoints)
        self.listener = tf.TransformListener()
        self.publisher_status = rospy.Publisher(topic_status, Int8, queue_size=10)
        self.pps_status = ''

        self.stop_threshold = config['stop_threshold']
        self.slow_threshold = config['slow_threshold']
        self.status_buffer = [1]*rate

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

        new_state = max(pair_states)
        # print('pre buffer {}'.format(self.status_buffer))
        self.status_buffer.append(new_state)
        self.status_buffer = self.status_buffer[1:]
        # print('post buffer {}'.format(self.status_buffer))
        new_status = max(self.status_buffer)
        if new_status != self.pps_status:
            self.pps_status = new_status
            self.publisher_status.publish(self.pps_status)


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

    thr_sml = 0.3
    various_thr_sml = generate_uni_thresholds(all_human_keypoints + moving_robot, 0.2)
    various_thr_sml.update({'/r1_ee': thr_sml, '/r1_link_8': thr_sml, '/r1_link_7': thr_sml, '/r1_link_6': thr_sml,
                           '/r1_link_5': thr_sml, '/r1_link_0': thr_sml, '/r1_link_0': thr_sml})

    thr_mid = 0.45
    various_thr_mid = generate_uni_thresholds(all_human_keypoints + moving_robot, 0.2)
    various_thr_mid.update({'/r1_ee': thr_mid, '/r1_link_8': thr_mid, '/r1_link_7': thr_mid, '/r1_link_6': thr_mid,
                           '/r1_link_5': thr_mid, '/r1_link_0': thr_mid, '/r1_link_0': thr_mid})

    thr_far = 0.75
    various_thr_far = generate_uni_thresholds(all_human_keypoints + moving_robot, 0.2)
    various_thr_far.update({'/r1_ee': thr_far, '/r1_link_8': thr_far, '/r1_link_7': thr_far, '/r1_link_6': thr_far,
                          '/r1_link_5': thr_far, '/r1_link_0': thr_far, '/r1_link_0': thr_far})

    head_thr = generate_uni_thresholds(all_human_keypoints + moving_robot, 0)
    head_thr.update({'/0': 0.45, '/1': 0.45, '/14': 0.45, '/15': 0.45, '/16': 0.45, '/17': 0.45})
    # print('head thr {}'.format(head_thr))  # WIP

    scenarios = [
                    {'keypoints': [robot_base, all_human_keypoints],
                     'stop_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, thr_far),
                     'slow_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, 0),
                     'name': 'scenario 0 stop zone',
                     },
                    {'keypoints': [robot_base, all_human_keypoints],
                     'stop_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, thr_mid),
                     'slow_threshold': generate_uni_thresholds(
                                        all_human_keypoints + robot_base, thr_far),
                     'name': 'scenario 1 slow and stop zone',
                     },
                    {'keypoints': [moving_robot, all_human_keypoints],
                     'stop_threshold': various_thr_far,
                     'slow_threshold': generate_uni_thresholds(
                                        all_human_keypoints + moving_robot, 0),
                     'name': 'scenario 2 keypoint stop',
                     },
                    {'keypoints': [moving_robot, all_human_keypoints],
                     'stop_threshold': various_thr_mid,
                     'slow_threshold': various_thr_far,
                     'name': 'scenario 3 keypoint slow and stop',
                     },
                    {'keypoints': [moving_robot, all_human_keypoints],
                     'stop_threshold': head_thr,
                     'slow_threshold': various_thr_far,
                     'name': 'scenario 4 keypoint slow and head stop',
                     },
                ]

    config = scenarios[4]
    RATE = 100
    rate = rospy.Rate(RATE)

    '''
    Experiment scenarios 
    0 - distance only from base, stopping
    1 - distance only from base slowing down 
    2 - all keypoints taken into account, only stopping
    3 - all keypoints taken into account, slowing down
    4 - all keypoints taken into account, stopping only on head
    '''

    pps = PeriPersonalSpaceChecker(config, rate=RATE)
    # coeffgen = CoefficientGenerator(pps.listener, config['keypoints'][0]) #DO NOT CHANGE

    # f = open('py_data.csv', 'a')
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

        rate.sleep()

    # f.close()
    rospy.spin()

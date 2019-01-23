#!/usr/bin/env python

import json

import numpy as np

import rospy
import tf
from coefficient_generator import CoefficientGenerator
from rospy_message_converter import message_converter
from std_msgs.msg import String, Int8


# if __name__ == "__main__":
#     node_name = 'pps'
#     rospy.init_node(node_name)
#     # robot_coef = [0, 0]
#     # obst_coef = [0, 0]
#
#     publisher = rospy.Publisher("pps_status", Int8, queue_size=100)
#     listener = tf.TransformListener()
#
#     while True:
#         try:
#             (transform, rotation) = listener.lookupTransform('/camera', '/r2_ee', rospy.Time(0))
#             if np.linalg.norm(transform) < 0.6:
#                 publisher.publish(2)
#             else:
#                 publisher.publish(0)
#
#         except:
#             publisher.publish(1)
#             continue
#
#
class PeriPersonalSpaceChecker(object):
    """
    Checks if robots PPS was interfered
    """
    def __init__(self,
                 listener_instance,
                 pair_array=np.nan,
                 keypoints=[],
                 topic_alert='pps_alert',
                 topic_status='pps_status',
                 stop_threshold=1.6,
                 warning_threshold=1.9,
                 robot_coef=[],
                 obstacle_coef=[],
                 ):

        self.publisher = rospy.Publisher(topic_alert, String, queue_size=10)
        self.publisher_status = rospy.Publisher(topic_status, Int8, queue_size=10)
        self.current_positions = np.nan
        self.distance_mat = np.nan

        self.stop_threshold = stop_threshold
        self.warning_threshold = warning_threshold

        if pair_array is np.nan and keypoints is np.nan:
            raise NameError('No input, need keypoints or pairs.')
        if pair_array is np.nan:
            if type(keypoints) is list:
                self.keypoints = keypoints
                self.pairs = self.make_combinations(keypoints)
            else:
                raise NameError('Not proper keypoints type!')
        if keypoints is []:
            if type(pairs) is np.ndarray:
                self.pairs = pair_array
                self.keypoints = self.get_keypoints(pair_array)
            else:
                raise NameError('Not proper pairs type!')

        def no_set_coeffs(robot_coef, obstacle_coef):
            zero_robot_coefs = [0] * len(self.keypoints[0])
            zero_obstacle_coefs = [0] * len(self.keypoints[1])

            if robot_coef == {} or obstacle_coef == {}:
                return True
            if robot_coef == [] or obstacle_coef == []:
                return True
            return False

        while no_set_coeffs(robot_coef, obstacle_coef):
            robot_coeffgen = CoefficientGenerator(listener_instance, self.keypoints[0])
            robot_coef = robot_coeffgen.calculate_coefficients()

            obstacle_coeffgen = CoefficientGenerator(listener_instance, self.keypoints[1])
            obstacle_coef = obstacle_coeffgen.calculate_coefficients()

        print ('COEFFICIENTS GENERATION')  # WIP
        print robot_coef  # WIP
        print obstacle_coef  # WIP
        self.robot_coef = robot_coef
        self.obstacle_coef = obstacle_coef

        self.stop_threshold_mat = self.get_threshold_mat(self.stop_threshold)
        self.warning_threshold_mat = self.get_threshold_mat(self.warning_threshold)


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

    @staticmethod
    def get_keypoints(pairs):
        """
        Collects unique keypoints from the pairs.
        :param pairs: np.array with pairs of (robot, obstacle) keypoints
        :return: list of robot_keypoints list and obstacle keypoints list
        """
        robot_keypoints = list(np.unique(pairs[:, 0]))
        obstacle_keypoints = list(np.unique(pairs[:, 1]))
        return list([robot_keypoints, obstacle_keypoints])

    @staticmethod
    def calculate_vector_dist(vect1, vect2):
        """
        Distance between vect1 and vect2
        :param vect1: list
        :param vect2: list
        :return: int
        """
        a = np.array(vect1)
        b = np.array(vect2)
        return np.linalg.norm(a - b)

    def calculate_keypoint_dist(self, point1, point2, positions):
        """
        Calculate distance between two keypoints with current position
        :param point1: list
        :param point2: list
        :param positions: dict
        :return: int
        """
        position1 = positions[point1]
        position2 = positions[point2]
        return self.calculate_vector_dist(position1, position2)

    @staticmethod
    def get_matrix_from_pairs(pairs):
        """
        Creates a matrix with nan as default value for pairs of of nodes.
        :param pairs: np.array
        :return: np.array
        """
        robot_keypoints = set(pairs[:, 0])
        obstacles = set(pairs[:, 1])
        matrix = np.full((len(robot_keypoints), len(obstacles)), np.nan)
        return matrix

    @staticmethod
    def get_matrix_from_lists(list1, list2):
        """
        Creates a matrix with nan as default value for two keypoint sets.
        :param pairs: np.array
        :return: np.array
        """
        matrix = np.full((len(list1), len(list2)), np.nan)
        return matrix

    def compute_distance_matrix(self, positions):
        """
        Compute distances between robot keypoints and obstacle keypoints
        :param pairs: np.array of coordinates
        :param positions: np.array of coordinates
        :return: np.array of distances
        """
        robot_keypoints = keypoints[0]
        obstacle_keypoints = keypoints[1]
        distance_mat = self.get_matrix_from_lists(robot_keypoints, obstacle_keypoints)
        for id_robot, robot_keypoint in enumerate(robot_keypoints):
            for id_obstacle, obstacle in enumerate(obstacle_keypoints):
                distance_mat[id_robot][id_obstacle] = self.calculate_keypoint_dist(robot_keypoint, obstacle, positions)
        return distance_mat

    @staticmethod
    def flatten(list_of_lists):
        return [item for sublist in list_of_lists for item in sublist]

    def get_current_positions(self):
        """
        Get current position of the given keypoint pairs
        :return: dict keypoint(str):transform or np.nan
        """
        current_positions = {}
        all_keypoints = self.flatten(self.keypoints)
        for keypoint in all_keypoints:
            try:
                (transform, rotation) = listener.lookupTransform('/world', keypoint, rospy.Time(0))
                current_positions[keypoint] = transform
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                current_positions[keypoint] = np.nan
                continue
        return current_positions

    def get_threshold_mat(self, basic_threshold):
        """
        Get the threshold matrix for PPS given as a combination of properties of the two interacting objects.
        The matrix is generated for the pairs that of the PPS instance.
        :return: numpy.array
        """
        threshold_mat = self.get_matrix_from_pairs(self.pairs)
        for rpc_idx, robot_point_coef in enumerate(self.robot_coef):
            for opc_idx, obstacle_point_coef in enumerate(self.obstacle_coef):
                threshold_mat[rpc_idx][opc_idx] = basic_threshold + robot_point_coef + obstacle_point_coef
        return threshold_mat

    @staticmethod
    def is_pps_incursion(diff_mat):
        """
        Return True if some part of a skeleton is bellow threshold distance to a body part.
        return also indexes of these parts
        """
        min_idx = np.unravel_index(diff_mat.argmin(), diff_mat.shape)

        if diff_mat[min_idx[0]][min_idx[1]] < 0:
            return True
        else:
            return False

    @staticmethod
    def create_diff(distance_mat, threshold_mat):
        """
        Return difference matrix between matrices.
        """
        return distance_mat - threshold_mat

    def get_incursion_points(self, difference_matrix, state):
        """
        Creates a dict of incursion points based on difference matrix.
        :param difference_matrix: {numpy.array}
        :param state: {str}
        :return: {dict}
        """
        robot_keypoints = self.keypoints[0]
        obstacle_keypoints = self.keypoints[1]
        incursion_points = {}
        for robot_index, row in enumerate(difference_matrix):
            obstacle_idx_array = np.where(row < 0)
            obstacle_dict = {}
            for obstacle_idx in obstacle_idx_array[0]:
                obstacle_dict.update(
                    {obstacle_keypoints[obstacle_idx]: {
                                                        'difference': difference_matrix[robot_index][obstacle_idx],
                                                        'state': state,
                                                        'distance': self.distance_mat[robot_index][obstacle_idx],
                                                        'stop threshold': self.stop_threshold_mat[robot_index][obstacle_idx],
                                                        'warning threshold': self.warning_threshold_mat[robot_index][obstacle_idx],
                                                        }})
            incursion_points.update({robot_keypoints[robot_index]: obstacle_dict})
        return incursion_points

    def construct_alarm_dict(self, warning_diff_matrix, stop_diff_matrix):
        """
        Constructs the alarm message that captures the demanded state of the robot - stop/warning and presents the nodes
        that are in violation of PPS.
        :param warning_diff_matrix:
        :param stop_diff_matrix:
        :return: {dict}
        """

        robot_coef = np.array(self.robot_coef)
        obstacle_coef = np.array(self.obstacle_coef)

        alarm_dict_output = {'state': 'SAFE',
                             'keypoints': {},
                             'reason': 'Default state.',
                             'context':
                                       {
                                        'pairs': np.ndarray.tolist(self.pairs),
                                        'keypoints': self.keypoints,
                                        # 'warning_threshold': self.warning_threshold,
                                        # 'stop_threshold': self.stop_threshold,
                                        'robot_stop': dict(zip(self.keypoints[0],
                                                               (self.stop_threshold / 2) + robot_coef)),
                                        'robot_warning': dict(zip(self.keypoints[0],
                                                                  (self.warning_threshold / 2) + robot_coef)),
                                        'obstacle_stop': dict(zip(self.keypoints[1],
                                                                  (self.stop_threshold / 2) + obstacle_coef)),
                                        'obstacle_warning': dict(zip(self.keypoints[1],
                                                                     (self.warning_threshold / 2) + obstacle_coef)),
                                       },
                            }
        if self.is_pps_incursion(warning_diff_matrix):
            alarm_dict_output['state'] = 'WARNING'
            alarm_dict_output['keypoints'].update(self.get_incursion_points(warning_diff_matrix, 'warning'))
            alarm_dict_output['reason'] = 'Warning threshold exceeded.'
            # print('Warning')
            # print(alarm_dict_output)
        if self.is_pps_incursion(stop_diff_matrix):
            alarm_dict_output['state'] = 'STOP'
            alarm_dict_output['keypoints'].update(self.get_incursion_points(stop_diff_matrix, 'stop'))
            alarm_dict_output['reason'] = 'Stopping threshold exceeded.'
            # print('STOP')
            # print(alarm_dict_output)
        return alarm_dict_output

    @staticmethod
    def is_missing_threshold_mat(thr_mat):
        if np.any(np.isnan(thr_mat)):
            return True

    def check_pps(self):
        """
        Checks whether for the given keypoint pairs a PPS violation happened
        :return:
        """
        import ipdb; ipdb.set_trace()
        current_positions = self.get_current_positions()
        self.distance_mat = self.compute_distance_matrix(current_positions)
        stop_activation = self.create_diff(self.distance_mat, self.stop_threshold_mat)
        warning_activation = self.create_diff(self.distance_mat, self.warning_threshold_mat)
        print (self.distance_mat)  # WIP
        # print (self.warning_threshold_mat)  # WIP
        # print (self.stop_threshold_mat)  # WIP
        # import ipdb; ipdb.set_trace()  # WIP
        alarm_msg = self.construct_alarm_dict(warning_activation, stop_activation)
        if self.is_missing_threshold_mat(self.stop_threshold_mat):
            alarm_dict['state'] = 'STOP'
            alarm_dict['reason'] = 'Missing Threshold matrix.'
        return alarm_msg

    @staticmethod
    def transform_dict_to_message(dict_data):
        json_data = json.dumps(dict_data)
        dict_wrapper = {'data': json_data}
        return message_converter.convert_dictionary_to_ros_message('std_msgs/String', dict_wrapper)


if __name__ == "__main__":
    node_name = 'pps'
    rospy.init_node(node_name)
    # robot_coef = [0, 0]
    # obst_coef = [0, 0]

    try:
        listener = tf.TransformListener()
        #
        # # pairs = np.array([['/r1_ee', '/r2_ee'],
        # #                   ['/r1_link_7', '/r2_link_7'],
        # #                   ['/r1_link_6', '/r2_link_6']
        # #                  ]
        # #                 )
        # #
        # # pps = PeriPersonalSpaceChecker(listener, pair_array=pairs)
        #
        # keypoints = [['/camera'], ['/r2_ee', '/r2_link_0']]
        # pps = PeriPersonalSpaceChecker(listener,
        #                                keypoints=keypoints,)
        #                                # robot_coef=robot_coef,
        #                                # obstacle_coef=obst_coef)
        # #
        # coeffgen = CoefficientGenerator(listener, keypoints[0])


        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                print(listener.lookupTwist('world', '0', rospy.Time(), rospy.Duration.from_sec(1)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('none')
            # alarm_dict = pps.check_pps()
            # ros_message = pps.transform_dict_to_message(alarm_dict)
            pps_status = 0
            # pps.publisher.publish(ros_message)
            # pps.publisher_status.publish(pps_status)

            rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("TF marker failed.")

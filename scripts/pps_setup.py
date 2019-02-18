#!/usr/bin/env python

import math
import re

import numpy as np

import rospy
import tf


class CoefficientGenerator:
    """
    Generates thresholds for a robot representation
    Coeff for a frame is generated based on the distance to related frames. The longer distance is taken, halved and
    multiplied by sqrt(2) in order to create at least partly overlapping spheres.
    """
    def __init__(self,
                 listener,
                 keypoints_list=[],
                 ):
        self.coeffs = np.nan
        self.listener = listener
        self.keypoints = self.clean_slash(keypoints_list)
        self.all_keypoints_dict = {}
        self.all_keypoints_list = list(set(self.keypoints))

    @staticmethod
    def clean_slash(kp_list):
        """
        Eliminates possible starting slash on keypoints.
        :param kp_list:
        :return: {list} of {str}
        """
        kp_list_cleaned = []
        for kp in kp_list:
            if kp[0] == '/':
                kp_list_cleaned.append(kp[1:])
        return kp_list_cleaned

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

    def get_current_positions(self, keypoints):
        """
        Get current position of the given keypoint pairs
        :return: dict keypoint(str):transform or np.nan
        """
        current_positions = {}
        for keypoint in keypoints:
            try:
                (transform, rotation) = self.listener.lookupTransform('world', keypoint, rospy.Time())
                current_positions[keypoint] = transform
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                current_positions[keypoint] = np.nan
                continue
        return current_positions

    def collect_all_keypoints(self, keypoints):
        """
        For given keypoints of interest gets the related keypoints based on the TF map.
        :return: dict keypoint{str}: list of keypoints that are connected to it
        """
        frames = self.listener.allFramesAsString()
        all_keypoints_dict = {}

        def get_parents(frames, keypoint):
            parent_str = keypoint + ' exists with parent '
            return re.findall('(?<='+parent_str+').*(?=\.)', frames)

        def get_children(frames, keypoint):
            child_str = ' exists with parent ' + keypoint
            return re.findall('(?<=Frame ).*(?='+child_str+')', frames)

        def transform_zero(kp1, kp2):
            try:
                transform, rotation = self.listener.lookupTransform(kp1, kp2, rospy.Time())
                if transform == [0, 0, 0]:
                    return True
            finally:
                return False
            return False

        def calculate_earlier_parent(frames, kp, parent):
            temp_parent = parent[0]
            while True:
                if not transform_zero(kp, temp_parent):
                    break
                if temp_parent == 'base_link':
                    break
                temp_parent = get_parents(frames, temp_parent)[0]
            return [temp_parent]

        for keypoint in keypoints:

            parent = get_parents(frames, keypoint)

            children = get_children(frames, keypoint)

            if children + parent and transform_zero(keypoint, parent[0]):
                parent = calculate_earlier_parent(frames, keypoint, parent)

            all_keypoints_dict[keypoint] = parent + children

        flat_values = [item for sublist in all_keypoints_dict.values() for item in sublist]
        all_keypoints_list = list(set(all_keypoints_dict.keys() + flat_values))
        return all_keypoints_dict, all_keypoints_list

    def calculate_coefficients(self):
        """
        Calculates coefficients for given keypoints according to TF distances.
        :return: dict keypoint{str}: coefficient{float} for given keypoint
        """
        output_coeffs = []
        coeffs = {}
        all_keypoints_dict, all_keypoints_set = self.collect_all_keypoints(self.keypoints)
        current_positions = self.get_current_positions(all_keypoints_set)
        for keypoint in self.keypoints:
            distances = []
            if all_keypoints_dict[keypoint]:
                for i_keypoint in all_keypoints_dict[keypoint]:
                    distances.append(self.calculate_keypoint_dist(keypoint, i_keypoint, current_positions))

                coeffs[keypoint] = max(distances)*math.sqrt(2)
        if set(coeffs.keys()) == set(self.keypoints):
            output_coeffs = [coeffs[x] for x in self.keypoints]
        return output_coeffs


if __name__ == "__main__":
    node_name = 'coeffs'
    rospy.init_node(node_name)

    try:
        listener = tf.TransformListener()

        # pairs = np.array([['/r1_ee', '/r2_ee'],
        #                   ['/r1_link_7', '/r2_link_7'],
        #                   ['/r1_link_6', '/r2_link_6']
        #                  ]
        #                 )
        #
        # pps = PeriPersonalSpaceChecker(listener, pair_array=pairs)

        keypoints1 = ['/r1_ee', '/r1_link_6']
        coeffs_generator1 = CoefficientGenerator(listener, keypoints_list=keypoints1)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            print(coeffs_generator1.collect_all_keypoints(coeffs_generator1.keypoints))
            print(coeffs_generator1.calculate_coefficients())

            rate.sleep()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("TF marker failed.")

#!/usr/bin/env python

import json
import rospy
import tf
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rospy_message_converter import message_converter


class DistanceMarkers(object):

    def __init__(self, pair_array=np.nan, keypoints=[]):
        self.publisher = {}
        self.marker_array = MarkerArray()
        self.pair_array = pair_array
        self.keypoints = keypoints
        self.pps_data = {}
        self.tf_listener = {}

    def update_markers(self):
        self.update_line_markers()
        self.update_sphere_markers()

    def update_sphere_markers(self):

        def flatten(list_of_lists):
            return [item for sublist in list_of_lists for item in sublist]

        kp_data = self.pps_data['keypoints']
        kp_obstacles = [kp_data[item].keys() for item in kp_data]
        colliding_keypoints = kp_data.keys() + flatten(kp_obstacles)

        for marker in colliding_keypoints:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/world', marker, rospy.Time(0))

                is_robot_marker = marker in self.pps_data['context']['keypoints'][0]
                is_obstacle_marker = marker in self.pps_data['context']['keypoints'][1]

                color_setting = self.determine_color(marker, is_robot_marker, is_obstacle_marker)

                if is_robot_marker:
                    sphere_size = [self.pps_data['context']['robot_warning'][marker] * 2] * 3
                    sphere_marker = self.create_sphere_marker(marker,
                                                              trans,
                                                              size=sphere_size,
                                                              color=color_setting)
                elif is_obstacle_marker:
                    sphere_size = [self.pps_data['context']['obstacle_warning'][marker] * 2] * 3
                    sphere_marker = self.create_sphere_marker(marker,
                                                              trans,
                                                              size=sphere_size,
                                                              color=color_setting)
                else:
                    sphere_marker = self.create_sphere_marker(marker, trans)

                self.marker_array.markers.append(sphere_marker)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def determine_color(self, marker, is_robot, is_obstacle):
        colors = [1, 1, 0]
        if is_robot:
            for collision in self.pps_data['keypoints'][marker]:
                if self.pps_data['keypoints'][marker][collision]['state'] == 'stop':
                    colors = [1, 0, 0]
        if is_obstacle:
            for robot_kp in self.pps_data['keypoints']:
                if marker not in self.pps_data['keypoints'][robot_kp]:
                    continue
                if self.pps_data['keypoints'][robot_kp][marker]['state'] == 'stop':
                    colors = [1, 0, 0]
        return colors

    def update_line_markers(self):
        for marker in self.pps_data['context']['pairs']:
            try:
                (trans1, rot1) = self.tf_listener.lookupTransform('/world', marker[0], rospy.Time(0))
                (trans2, rot2) = self.tf_listener.lookupTransform('/world', marker[1], rospy.Time(0))

                line_marker = self.create_line_marker(marker, trans1, trans2)

                self.marker_array.markers.append(line_marker)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rospy.Time()

    def update_human_skeleton(self):
        human_keypoint_pairs = [(0, 1), (1, 5), (1, 2), (2, 3), (3, 4), (5, 6), (6, 7)]
        for marker in human_keypoint_pairs:
            try:
                (trans1, rot1) = self.tf_listener.lookupTransform('/world', str(marker[0]), rospy.Time(0))
                (trans2, rot2) = self.tf_listener.lookupTransform('/world', str(marker[1]), rospy.Time(0))
                print('found transform')
                line_marker = self.create_line_marker(str(marker), trans1, trans2)

                self.marker_array.markers.append(line_marker)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('error with lookup')
                continue

            rospy.Time()
        self.publisher.publish(markers.marker_array)

    @staticmethod
    def create_line_marker(marker, trans1, trans2, color=[0.0, 1.0, 0.0]):
        line_marker = Marker()
        line_marker.ns = marker[0] + marker[1]
        line_marker.id = 0
        line_marker.header.frame_id = '/world'
        line_marker.header.stamp = rospy.Time(0)
        line_marker.type = line_marker.LINE_STRIP
        line_marker.action = line_marker.ADD
        line_marker.scale.x = 0.05
        line_marker.scale.y = 0.05
        line_marker.scale.z = 0.05
        line_marker.color.a = 1.0
        line_marker.color.r = color[0]
        line_marker.color.g = color[1]
        line_marker.color.b = color[2]
        line_marker.pose.orientation.w = 1.0

        line_marker.points = []

        for trans in [trans1, trans2]:
            p = Point()
            p.x = trans[0]
            p.y = trans[1]
            p.z = trans[2]
            line_marker.points.append(p)
        return line_marker

    @staticmethod
    def create_sphere_marker(marker, trans, size=[1.0, 1.0, 1.0], color=[0.0, 1.0, 0.0]):
        sphere_marker = Marker()
        sphere_marker.ns = marker
        sphere_marker.id = 0
        sphere_marker.header.frame_id = '/world'
        sphere_marker.header.stamp = rospy.Time(0)
        sphere_marker.type = sphere_marker.SPHERE
        sphere_marker.action = sphere_marker.MODIFY
        sphere_marker.scale.x = size[0]
        sphere_marker.scale.y = size[1]
        sphere_marker.scale.z = size[2]
        sphere_marker.color.a = 0.3
        sphere_marker.color.r = color[0]
        sphere_marker.color.g = color[1]
        sphere_marker.color.b = color[2]
        sphere_marker.pose.orientation.w = 1.0
        sphere_marker.lifetime = 2

        sphere_marker.lifetime = rospy.Duration(0)

        p = Point()
        p.x = trans[0]
        p.y = trans[1]
        p.z = trans[2]

        sphere_marker.pose.position = p
        return sphere_marker

    @staticmethod
    def delete_markers(marker, trans, size=[1.0, 1.0, 1.0], color=[0.0, 1.0, 0.0]):
        return 0

    def callback(self, data):
        self.pps_data = message_converter.convert_ros_message_to_dictionary(data)['data']

    def callback_update_markers(self, data):
        # print(message_converter.convert_ros_message_to_dictionary(data)['data'])
        self.pps_data = json.loads(message_converter.convert_ros_message_to_dictionary(data)['data'])
        self.update_line_markers()
        # self.update_sphere_markers()
        self.publisher.publish(markers.marker_array)

    def marker_node(self):
        rospy.init_node('tf_markers')
        self.tf_listener = tf.TransformListener()
        topic = 'visualization_marker_array'
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

        try:
            rate = rospy.Rate(10.0)

            while not rospy.is_shutdown():
                rospy.Subscriber('pps_alert', String, self.callback_update_markers)
                self.update_human_skeleton()
                rate.sleep()

            rospy.spin()

        except rospy.ROSInterruptException:
            rospy.logerr("TF markers failed.")


if __name__ == '__main__':
    pairs = np.array([['/r1_ee', '/r2_ee'],
                      ['/r1_link_6', '/r2_link_6'],
                      ]
                     )
    keypoints = [['/r1_ee', '/r1_link_6'], ['/r2_ee', '/r2_link_6']]

    markers = DistanceMarkers(pairs, keypoints)
    markers.marker_node()

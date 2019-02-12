#!/usr/bin/env python

import time

import cv2
import numpy as np
import pyrealsense2 as rs

import rospy
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int8

from camera_setup import camera_setup


def get_distances(coord_tuples, depth_img, scale):
    def in_bound(coord):
        return int(coord[0]) < 848 and int(coord[1]) < 480

    # Needs to be switched for Openpose/RS have different coord systems
    return [depth_img[int(x[1]), int(x[0])] * scale
            if in_bound(x) else 0 for x in coord_tuples]


def get_int_coords(keypoints):
    return [x[:2].astype(int).tolist() for x in keypoints]


show = False
align, depth_scale, openpose, pc, points, pipeline, profile = camera_setup(show)

# Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

rospy.init_node('keypoint_tf_broadcaster')
br = tf.TransformBroadcaster()
rate = rospy.Rate(10.0)
listener = tf.TransformListener()
publisher = rospy.Publisher("pps_status", Int8, queue_size=1000)

try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # Tell pointcloud object to map to this color frame
        pc.map_to(color_frame)

        # Generate the pointcloud and texture mappings
        points = pc.calculate(depth_frame)

        # Validate that both frames are valid
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        keypoints, output_image = openpose.forward(color_image, True)

        # OpenPose Keypoint indexes
        head = [0, 14, 15, 16, 17]
        upper_body = [1, 2, 3, 4, 5, 6, 7]
        lower_body = [8, 9, 10, 11, 12, 13]

        interesting = head + upper_body
        kpt_names = [str(x) for x in interesting]

        if keypoints.any() and depth_image.any():
            interest_kpts = keypoints[0][interesting, :]
            # print(interest_kpts[0])

            distances_to_camera = get_distances(interest_kpts, depth_image, depth_scale)

            int_coords = get_int_coords(interest_kpts)
            distances = get_distances(interest_kpts, depth_image, depth_scale)
            point_lst = [rs.rs2_deproject_pixel_to_point(depth_intrin, x, y) for x, y in zip(int_coords, distances)]

            # print("distance to camera for mid: {}".format(depth_image[424, 240] * depth_scale))
            for idx, pt in enumerate(point_lst):
                br.sendTransform(tuple(pt),
                                 quaternion_from_euler(0, 0, 0),
                                 rospy.Time.now(),
                                 kpt_names[idx],
                                 'camera_link')

        if show:
            # cv2.circle(depth_colormap, tuple(int_coords[0]), 5, (0, 0, 255), -1) #  WIP: Debug code for eval point
            images = np.hstack((output_image, depth_colormap))
            cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Align Example', images)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

finally:
    pipeline.stop()


# Original for video export
# fourcc = cv2.VideoWriter_fourcc('X', '2', '6', '4')
# frame_width = 640  # int(color_data.get(3))
# frame_height = 480   # int(color_data.get(4))
# out = cv2.VideoWriter('outputD.avi', fourcc, 20.0, (frame_width, frame_height), True)

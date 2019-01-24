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
    def in_bound(x):
        return int(x[0]+1) < 480 and int(x[1]+1) < 640

    return [depth_img[int(x[0]+1), int(x[1])+1] * scale
            if in_bound(x) else 1000 for x in coord_tuples]


show = True
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
        depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
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

        # fourcc = cv2.VideoWriter_fourcc('X', '2', '6', '4')
        # frame_width = 640  # int(color_data.get(3))
        # frame_height = 480   # int(color_data.get(4))
        # out = cv2.VideoWriter('outputD.avi', fourcc, 20.0, (frame_width, frame_height), True)

        keypoints, output_image = openpose.forward(color_image, True)

        # interesting keypoints based on Openpose homunculus
        upper_body_idxs = [1, 2, 3, 4, 5, 6, 7]
        head_idxs = [0, 14, 15, 16, 17]

        interest_groups = [upper_body_idxs, head_idxs]

        if keypoints.any() and depth_image.any():
            for group in interest_groups:
                kpts = keypoints[0][group, :]
                # print(interest_kpts[0])
                print('Number of kpt groups: {}'.format(len(keypoints)))

                import ipdb; ipdb.set_trace()
                distances = get_distances(kpts, depth_image, depth_scale)
                min_distance_to_camera = min(distances)
                if min_distance_to_camera > 0:
                    min_dist_keypoint = kpts[distances.index(min_distance_to_camera)]
                    print(min_distance_to_camera)
                    #
                    # if min_dist_keypoint.any():
                    # use non zero minimum
                    int_coords = min_dist_keypoint[:2].astype(int).tolist()
                    print(int_coords)

                    # point = rs.rs2_deproject_pixel_to_point(depth_intrin, int_coords, min_distance_to_camera)
                    # print("deproject: {} at {}").format(point, int_coords)
                    # # print(distances_to_camera)
                    # # brat minimum z hlavy - head, arm, body
                    #
                    # br.sendTransform(tuple(point),
                    #                  quaternion_from_euler(0, 0, 0),
                    #                  rospy.Time.now(),
                    #                  "neck",
                    #                  "camera")
                    #
                    # try:
                    #     (trans, rot) = listener.lookupTransform('/camera', '/neck', rospy.Time(0))
                    #     dist = np.linalg.norm(trans)
                    #
                    #     if 0 < point[2] < 0.8:
                    #         publisher.publish(2)
                    #     elif point[2] > 4:
                    #         publisher.publish(3)
                    #     # print("Distance between the hands is = {0:f}".format(np.linalg.norm(trans)))
                    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #     continue

        if show:
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

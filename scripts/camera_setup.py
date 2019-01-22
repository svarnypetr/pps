#!/usr/bin/env python

import os
import sys

import pyrealsense2 as rs


def camera_setup(show):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    # Append to syspath the path to openpose python build
    sys.path.append('/home/naoskin/openposeKuka/openpose/build/python')

    # Parameters for OpenPose.
    # Take a look at C++ OpenPose example for meaning of components.
    try:
        from openpose import *
    except:
        raise Exception('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    params = dict()
    params["logging_level"] = 3
    params["output_resolution"] = "-1x-1"
    params["net_resolution"] = "-1x368"
    params["model_pose"] = "COCO"
    params["alpha_pose"] = 0.6
    params["scale_gap"] = 0.3
    params["scale_number"] = 1
    params["render_threshold"] = 0.05
    params["num_gpu_start"] = 0
    params["disable_blending"] = False
    params["render_pose"] = True
    params["number_people_max"] = 1
    params["download_heatmaps"] = False
    params["heatmaps_add_parts"] = True
    params["heatmaps_add_PAFs"] = True
    # Ensure you point to the correct path where models are located
    params["default_model_folder"] = "/home/naoskin/openposeKuka/openpose/models/"
    if not show:
        params["display"] = 0
    # Construct OpenPose object allocates GPU memory
    openpose = OpenPose(params)

    depth_fps = 90

    safety_coefficient = 0

    # experiment_timestamp = int(time.time())
    # experiment_server_matrixes_file_name = 'output/' + str(experiment_timestamp) + '_server_matrixes'
    # with open(experiment_server_matrixes_file_name, 'w') as output_file:
    #     output_file.write('SERVER MATRIXES\n')

    # Declare pointcloud object, for calculating pointclouds and texture mappings
    pc = rs.pointcloud()
    # We want the points object to be persistent so we can display the last cloud when a frame drops
    points = rs.points()

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.depth
    align = rs.align(align_to)

    print("Camera setup ready.")

    return align, depth_scale, openpose, pc, points, pipeline, profile

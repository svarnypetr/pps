#!/usr/bin/env python

import os
import sys

import pyrealsense2 as rs
import time
import json


DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]


def find_device_that_supports_advanced_mode():
    ctx = rs.context()
    ds5_dev = rs.device()
    devices = ctx.query_devices();
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            if dev.supports(rs.camera_info.name):
                print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
            return dev
    raise Exception("No device that supports advanced mode was found")


def setup_advanced_camera():
    try:
        dev = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

        # Loop until we successfully enable advanced mode
        while not advnc_mode.is_enabled():
            print("Trying to enable advanced mode...")
            advnc_mode.toggle_advanced_mode(True)
            # At this point the device will disconnect and re-connect.
            print("Sleeping for 5 seconds...")
            time.sleep(5)
            # The 'dev' object will become invalid and we need to initialize it again
            dev = find_device_that_supports_advanced_mode()
            advnc_mode = rs.rs400_advanced_mode(dev)
            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

        with open('/home/naoskin/michael_pycommander/src/pps_robot/scripts/short_range.json') as f:
            as_json_object = json.load(f)

            # We can also load controls from a json string
            # For Python 2, the values in 'as_json_object' dict need to be converted from unicode object to utf-8
            if type(next(iter(as_json_object))) != str:
                as_json_object = {k.encode('utf-8'): v for k, v in as_json_object.items()}
            # The C++ JSON parser requires double-quotes for the json object so we need
            # to replace the single quote of the pythonic json to double-quotes
            json_string = str(as_json_object).replace("'", '\"')
            advnc_mode.load_json(json_string)

    except Exception as e:
        print(e)
        pass


def camera_setup():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    # Append to syspath the path to openpose python build
    sys.path.append('/home/naoskin/openpose/build/python/')
    try:
        from openpose import pyopenpose as op
    except:
        raise Exception(
            'Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')

    # Parameters for OpenPose.
    # Take a look at C++ OpenPose example for meaning of components.
    params = dict()
    params["logging_level"] = 4
    params["output_resolution"] = "-1x-1"
    params["net_resolution"] = "-1x480"
    params["model_pose"] = "COCO"
    params["alpha_pose"] = 0.6
    params["scale_gap"] = 0.3
    params["scale_number"] = 1
    params["render_threshold"] = 0.05
    # params["render_pose"] = True
    params["num_gpu_start"] = 0
    params["disable_blending"] = False
    params["number_people_max"] = 1
    params["heatmaps_add_parts"] = True
    params["heatmaps_add_PAFs"] = True
    # params["tracking"] = 5
    # params["number_people_max"] = 1
    # Ensure you point to the correct path where models are located
    params["model_folder"] = "/home/naoskin/openpose/models/"
    params["display"] = 0
    # Construct OpenPose object allocates GPU memory
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    depth_fps = 90

    # setup_advanced_camera()

    # Declare pointcloud object, for calculating pointclouds and texture mappings
    pc = rs.pointcloud()
    # We want the points object to be persistent so we can display the last cloud when a frame drops
    points = rs.points()

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    # This is the minimal recommended resolution
    config.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    # align_to = rs.stream.depth
    align_to = rs.stream.color

    align = rs.align(align_to)

    print("Camera setup ready.")

    return align, depth_scale, op, opWrapper, pc, points, pipeline, profile


if __name__ == '__main__':
    camera_setup(True)

#!/usr/bin/python

import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def create_camera_marker(trans, size=(0.005, 0.005, 0.005), color=(1.0, 1.0, 1.0)):
    camera_marker = Marker()
    camera_marker.id = 0
    camera_marker.header.frame_id = '/world'
    camera_marker.header.stamp = rospy.Time(0)
    camera_marker.type = camera_marker.MESH_RESOURCE
    camera_marker.mesh_resource = "package://pps_robot/meshes/camera/d435.stl"
    camera_marker.action = camera_marker.MODIFY
    camera_marker.scale.x = size[0]
    camera_marker.scale.y = size[1]
    camera_marker.scale.z = size[2]
    camera_marker.color.a = 1
    camera_marker.color.r = color[0]
    camera_marker.color.g = color[1]
    camera_marker.color.b = color[2]
<<<<<<< HEAD
    camera_marker.pose.orientation.x = 1.0
    camera_marker.pose.orientation.y = 0.0
    camera_marker.pose.orientation.z = 0.0
    camera_marker.pose.orientation.w = 0.0
=======

    quaternion = quaternion_from_euler(-1.5707, 0, -1.5707)

    camera_marker.pose.orientation.x = quaternion[0]
    camera_marker.pose.orientation.y = quaternion[1]
    camera_marker.pose.orientation.z = quaternion[2]
    camera_marker.pose.orientation.w = quaternion[3]
>>>>>>> 3c9d43aa433676efe824c7a66b095d7ee6e93d0c

    p = Point()
    p.x = trans[0]
    p.y = trans[1]
    p.z = trans[2]

    camera_marker.pose.position = p
    return camera_marker


def update_camera_marker(tf_listener):
    try:
        (trans, rot) = tf_listener.lookupTransform('/world', 'camera_link', rospy.Time(0))
        camera_marker = create_camera_marker(trans)
        publisher.publish(camera_marker)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('No camera was found.')


if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    # br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    publisher = rospy.Publisher('camera_link', Marker, queue_size=100)

    while not rospy.is_shutdown():
        # br.sendTransform((-1.0, -1.0, 1.0),
        #                  quaternion_from_euler(-1.5707, 0, -1.5707),
        #                  rospy.Time.now(),
        #                  "camera",
        #                  "r2_link_0")
        update_camera_marker(listener)

        rate.sleep()

#!/usr/bin/python

import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def create_camera_marker(trans, size=[1.0, 1.0, 1.0], color=[0.0, 1.0, 0.0]):
    camera_marker = Marker()
    camera_marker.id = 0
    camera_marker.header.frame_id = '/camera'
    camera_marker.header.stamp = rospy.Time(0)
    camera_marker.header.stamp = rospy.Time(0)
    camera_marker.type = camera_marker.MESH_RESOURCE
    camera_marker.mesh_resource = "package://pps_robot/meshes/camera/d435.stl"
    camera_marker.action = camera_marker.MODIFY
    camera_marker.scale.x = size[0]
    camera_marker.scale.y = size[1]
    camera_marker.scale.z = size[2]
    camera_marker.color.a = 0.3
    camera_marker.color.r = color[0]
    camera_marker.color.g = color[1]
    camera_marker.color.b = color[2]
    camera_marker.pose.orientation.w = 1.0

    p = Point()
    p.x = trans[0]
    p.y = trans[1]
    p.z = trans[2]

    camera_marker.pose.position = p
    return camera_marker


if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    cam_marker = create_camera_marker((-1.0, -1.0, 1.0))
    publisher = rospy.Publisher('camera', Marker, queue_size=100)
    publisher.publish(cam_marker)

    while not rospy.is_shutdown():
        br.sendTransform((-1.0, -1.0, 1.0),
                         quaternion_from_euler(-1.5707, 0, -1.5707),
                         rospy.Time.now(),
                         "camera",
                         "r2_link_0")

        rate.sleep()

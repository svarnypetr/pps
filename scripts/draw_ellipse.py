#!/usr/bin/env python
import rospy
import numpy
from math import sin, cos, pi
from copy import deepcopy
from geometry_msgs.msg import Pose
from capek_pycommander.capek_robot import CapekRobotCommander
from std_msgs.msg import Int8


# Initialize node and robot
rospy.init_node("draw_ellipse")
crc = CapekRobotCommander("r1")

# Go to the names position L
crc.move_l_position()


class Ellipse(object):
    
    def __init__(self, size_x=0.2, size_y=0.4, points=50, repeat=3):
        self.coordinates = [(cos(2*pi/points*x)*size_x, sin(2*pi/points*x)*size_y) for x in range(0,points+1)]

        p = Pose()
        
        self.poses = list()

        for point in self.coordinates:
            pp = deepcopy(p)
            pp.position.x = point[0]
            pp.position.y = point[1] + 0.1
            pp.position.z = 0.6
            pp.orientation.x = 0.0
            pp.orientation.y = 1.0
            pp.orientation.z = 0.0
            pp.orientation.w = 0.0
            self.poses.append(pp)
    
        pp = deepcopy(p)
        pp.position.x = self.coordinates[0][0]
        pp.position.y = self.coordinates[0][1]
        pp.position.z = 0.6
        pp.orientation.x = 0.0
        pp.orientation.y = 1.0
        pp.orientation.z = 0.0
        pp.orientation.w = 0.0
        self.poses.append(pp)
        
        """ self.poses[-1].position.x = self.poses[0].position.x
        self.poses[-1].position.y = self.poses[0].position.y
        """
        self.poses.extend(self.poses)
        self.poses.extend(self.poses)
        #self.poses.extend(self.poses)
        #self.poses.extend(self.poses)

e = Ellipse(size_x=0.1, size_y=0.05, points=10, repeat=2)

crc.group.set_pose_reference_frame("robot_frame")

# Go to the first point first
start_point, fraction = crc.group.compute_cartesian_path([e.poses[0]], 0.1, 0.0, True)
rospy.loginfo("Attaching to the first point of ellipse")
crc.display_trajectory(start_point)
crc.execute_plan(start_point)

# Run the one round of ellipse
cartesian_plan, fraction = crc.group.compute_cartesian_path(e.poses[1:], 0.1, 0.0, True)
rospy.loginfo("Run round one")
crc.display_trajectory(cartesian_plan)
crc.execute_plan(cartesian_plan)

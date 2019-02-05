#!/usr/bin/python

import rospy
import numpy
from capek_pycommander.capek_robot import CapekRobotCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from copy import deepcopy

# Initialize node and robot
rospy.init_node("joint_space_move_example")
crc = CapekRobotCommander("r1")
crc.group.set_planner_id("RRTConnectkConfigDefault")
crc.group.set_pose_reference_frame("r1_link_0")

# Go to the names position L
crc.move_l_position()

# Set start state
crc.group.set_start_state(RobotState())
crc.group.clear_pose_targets()
crc.group.allow_replanning(True)

rospy.sleep(0.5)

# Move to blue mark
blue_mark = Pose()
blue_mark.position.x = 0.45
blue_mark.position.y = -0.3
blue_mark.position.z = 0.3
blue_mark.orientation.y = 1.0

blue_down = deepcopy(blue_mark)
blue_down.position.z = 0.1

green_mark = Pose()
green_mark.position.x = 0.65
green_mark.position.y = 0.3
green_mark.position.z = 0.3
green_mark.orientation.y = 1.0

green_down = deepcopy(green_mark)
green_down.position.z = 0.1

l_position = Pose()
l_position.position.x = 0.5
l_position.position.z = 0.5
l_position.orientation.y = 1.0

trajectories = list()
template = [blue_mark, blue_down, blue_mark, green_mark, green_down, green_mark]
for repetition in range(15):
	if not repetition % 5:
		trajectories.extend([l_position])
	else:
		trajectories.extend(template)

cartesian_plan, fraction = crc.group.compute_cartesian_path(trajectories, 0.01, 0.0, True)
crc.display_trajectory(cartesian_plan)
crc.execute_plan(cartesian_plan)

# Go to the names position L
crc.move_l_position()

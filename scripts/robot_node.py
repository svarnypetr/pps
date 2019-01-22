#!/usr/bin/python

import rospy
import numpy
from capek_pycommander.capek_robot import CapekRobotCommander
from std_msgs.msg import Int8


# Initialize node and robot
rospy.init_node("joint_space_move_example")
crc = CapekRobotCommander("r2")

# Go to the names position L
crc.move_l_position()

# Prepare a square coordinates
length = 0.2
trajectory_line = list()
for i in range(10):
    if i % 2 == 0:
        trajectory_line.append([-length, 0.0, 0.0, 0.0, 0.0, 0.0])
    else:
        trajectory_line.append([+length, 0.0, 0.0, 0.0, 0.0, 0.0])

# Pre compute a cartesian plan
cartesian_plan, fraction = crc.plan_cartesian_path(trajectory_line)


# Define function during execution
def printout():
    pose = crc.group.get_current_joint_values()
    pose_value = numpy.sum(pose)
    plan_value = list()
    for configuration in crc._plan:
        plan_value.append(numpy.sum(configuration))
    last_index = (numpy.abs(plan_value - pose_value)).argmin()
    #print(last_index)


# Define a PPS logic in here
def pps_callback(msg):
    status = msg.data
    if status == 0:
        rospy.loginfo("Safe: Executing trajectory")
        crc.execute_plan2(cartesian_plan, True, [printout])
    if status == 1:
        rospy.loginfo("Warning: Adjusting robot speed down")
    if status == 2:
        rospy.loginfo("Stop: Possible collision with human")
        crc.stop()
    if status == 3:
        rospy.loginfo("Start: Executing plan from beginning")
        crc.move_l_position()
        crc.execute_plan2(cartesian_plan, True, [printout])
    if status == 4:
        rospy.loginfo("Debugging: Executing plan from beginning at 20% of speed")
        crc.move_l_position()
        retimed_cartesian_plan = crc.retime_trajectory(cartesian_plan, 0.2)
        crc.execute_plan2(retimed_cartesian_plan, True, [printout])


# It only works during async execution
pps_subscriber = rospy.Subscriber("pps_status", Int8, pps_callback)

# Spin forever
rospy.spin()

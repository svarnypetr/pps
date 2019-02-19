#!/usr/bin/python
import rospy
from math import radians, pi
from capek_pycommander.capek_robot import CapekRobotCommander
from std_msgs.msg import Int8

template = [
	[-radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
	[-radians(30), radians(55), 0.0, -radians(90+20), 0.0, radians(15), 0.0],
	[-radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
	[+radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
	[+radians(30), radians(55), 0.0, -radians(90+20), 0.0, radians(15), 0.0],
	[+radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
]

N = 2
REPETITION = 0
FULL_SPEED = 1.0
SLOW_SPEED = 0.2
INDEX = 0
STATUS = 0

def pps_callback(msg):
	global STATUS
	global INDEX
	global REPETITION
	global FULL_SPEED
	global SLOW_SPEED
	global N
	
	STATUS = msg.data
	rate = rospy.Rate(10)
	rate.sleep()

if __name__ == "__main__":									
	joints = list()
	for _ in range(N):
		joints.extend(template)

	rospy.init_node("move")
	crc = CapekRobotCommander("r1")
	crc.move_l_position()

	pps_subscriber = rospy.Subscriber("pps_status", Int8, pps_callback)
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		for index, coordinate in enumerate(joints[INDEX:]):
			if not STATUS == 2:
				if STATUS == 1:
					crc.set_speed(SLOW_SPEED)
				if STATUS == 3:
					crc.set_speed(FULL_SPEED)
				crc.group.set_start_state_to_current_state()
				joint_plan = crc.plan_joint_goal(coordinate)
				crc.display_trajectory(joint_plan)
				crc.group.execute(joint_plan, wait=False)			
				while True:
					c1 = sum(crc.group.get_current_joint_values())
					rate.sleep()
					c2 = sum(crc.group.get_current_joint_values())
					if round(c1, 6) == round(c2, 6):
						INDEX = index % 6
						break


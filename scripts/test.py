#!/usr/bin/python
import rospy
import math
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.srv import SetPathParameters
from std_msgs.msg import Time
from math import radians

REACHED_DESTINATION = False
CURRENT_JOINT_VALUES = None
N = 10
TRAJECTORY = list()

template = [
	[-radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
	[-radians(30), radians(55), 0.0, -radians(90+20), 0.0, radians(15), 0.0],
	[-radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
	[+radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
	[+radians(30), radians(55), 0.0, -radians(90+20), 0.0, radians(15), 0.0],
	[+radians(30), 0.0, 0.0, -radians(90), 0.0, radians(90), 0.0],
]

for _ in range(N):
    TRAJECTORY.extend(template)

def move_joint_space(a1, a2, a3, a4, a5, a6, a7):
    pub = rospy.Publisher("/r1/command/JointPosition", JointPosition, queue_size=10)
    rate = rospy.Rate(10)
    message = JointPosition()
    message.position.a1 = a1
    message.position.a2 = a2
    message.position.a3 = a3
    message.position.a4 = a4
    message.position.a5 = a5
    message.position.a6 = a6
    message.position.a7 = a7
    pub.publish(message)
    rate.sleep()

def move_home():
    move_joint_space(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

def move_l_position():
    move_joint_space(0.0, 0.174225583673, 0.0, -1.39891409874, 0.0, 1.57086062431, 0.0)

def reached_destination(msg):
    global REACHED_DESTINATION
    REACHED_DESTINATION = True
    rospy.loginfo(msg)

def set_path_parameters():
    # It only works in different possition than it is when calling script
        client = rospy.ServiceProxy(
            "/r1/configuration/pathParameters",
            SetPathParameters)
        # FIXME We set both parameters 0.1, a velocity exeption occurs 
        joint_relative_velocity = 0.2
        joint_relative_acceleration = 0.1
        # override_joint_acceleration = 0.1
        print(client(joint_relative_velocity=joint_relative_velocity,
               joint_relative_acceleration=joint_relative_acceleration,
               ))
        print('Path parameter set.')

def get_current_joint_values(msg):
    global CURRENT_JOINT_VALUES
    CURRENT_JOINT_VALUES = msg.position
    rospy.loginfo("Current joint values {}".format(CURRENT_JOINT_VALUES))

if __name__ == '__main__':
    rospy.init_node("move_robot", anonymous=True)
    rospy.Subscriber("/r1/state/DestinationReached", Time, reached_destination)
    rospy.Subscriber("/iiwa/state/JointPosition", JointPosition, get_current_joint_values)

    REACHED_DESTINATION = False
    while not rospy.is_shutdown() and not REACHED_DESTINATION:
        move_home()

    for index, position in enumerate(TRAJECTORY):
        REACHED_DESTINATION = False
        while not rospy.is_shutdown() and not REACHED_DESTINATION:
            move_joint_space(
                position[0], 
                position[1], 
                position[2], 
                position[3], 
                position[4],
                position[5],
                position[6]
            )

#!/usr/bin/python
import rospy
import math
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.srv import SetPathParameters
from std_msgs.msg import Time, Int8
from math import radians

REACHED_DESTINATION = False
CURRENT_JOINT_VALUES = None
STATUS = 0
N = 100
REPETITION = 0
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
    rate = rospy.Rate(50)
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

def set_path_parameters(robot_speed):
    # It only works in different possition than it is when calling script
        client = rospy.ServiceProxy(
            "/r1/configuration/pathParameters",
            SetPathParameters)
        # FIXME We set both parameters 0.1, a velocity exeption occurs 
        joint_relative_velocity = robot_speed
        joint_relative_acceleration = 0.1
        client(joint_relative_velocity=joint_relative_velocity,
               joint_relative_acceleration=joint_relative_acceleration)

def get_current_joint_values(msg):
    global CURRENT_JOINT_VALUES
    CURRENT_JOINT_VALUES = msg.position

def pps_callback(msg):
    global STATUS
    STATUS = msg.data
    if STATUS == 2:
        REACHED_DESTINATION = True
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node("move_robot", anonymous=True)
    rospy.Subscriber("/r1/state/DestinationReached", Time, reached_destination)
    rospy.Subscriber("/r1/state/JointPosition", JointPosition, get_current_joint_values)
    pps_subscriber = rospy.Subscriber("pps_status", Int8, pps_callback)

    REACHED_DESTINATION = False
    while not rospy.is_shutdown() and not REACHED_DESTINATION:
        move_l_position()
    
    set_path_parameters(robot_speed=1.0)
    rate = rospy.Rate(100)
    
    index = 0
    while index < len(TRAJECTORY):

        rospy.loginfo("Index: {}".format(index))

        # STOP PROCEDURE
        if STATUS == 2:
            rospy.loginfo("Stopping robot")
            """ REACHED_DESTINATION = False
            move_joint_space(
                CURRENT_JOINT_VALUES.a1,
                CURRENT_JOINT_VALUES.a2,
                CURRENT_JOINT_VALUES.a3,
                CURRENT_JOINT_VALUES.a4,
                CURRENT_JOINT_VALUES.a5,
                CURRENT_JOINT_VALUES.a6,
                CURRENT_JOINT_VALUES.a7,
            ) """
            while (not rospy.is_shutdown()) and (not REACHED_DESTINATION):
                rate.sleep()
            
            while STATUS == 2:
                rate.sleep()
        else:            
            if STATUS == 1 or STATUS == 3:
                if STATUS == 1:
                    set_path_parameters(robot_speed=0.2)
                elif STATUS == 3:
                    set_path_parameters(robot_speed=1.0)
                REACHED_DESTINATION = False
                move_joint_space(
                    TRAJECTORY[index][0], 
                    TRAJECTORY[index][1], 
                    TRAJECTORY[index][2], 
                    TRAJECTORY[index][3], 
                    TRAJECTORY[index][4],
                    TRAJECTORY[index][5],
                    TRAJECTORY[index][6]
                )
                while (not rospy.is_shutdown()) and (not REACHED_DESTINATION):
                    rate.sleep()
                index += 1
            else:
                rospy.loginfo("Use STATUS 1, 2 or 3")

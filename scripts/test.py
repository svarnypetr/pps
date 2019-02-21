#!/usr/bin/python
import rospy
import math
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.srv import SetPathParameters
from std_msgs.msg import Time, Int8
from math import radians
from scipy.interpolate import interp1d
import numpy as np

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
    rate = rospy.Rate(100)
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
    rate = rospy.Rate(100)
    CURRENT_JOINT_VALUES = msg.position
    rate.sleep()

def pps_callback(msg):
    global STATUS
    rate = rospy.Rate(100)
    STATUS = msg.data
    rate.sleep()

def interpolate(array, n_points):
    indexes = np.arange(1, n_points+1)
    interpolated = list()
    for i in range(len(template)-1):
        linfit = interp1d([1, n_points], np.vstack([array[i], array[i+1]]), axis=0)
        interpolated.extend(linfit(indexes))
    return interpolated

def find_closest(current_state, array, offset=0):   
    distances = list()
    for interpolation in array:
        dist = np.linalg.norm(current_state-interpolation)
        distances.append(dist)

    closest_point = distances.index(min(distances))
    if closest_point+offset < len(array):
        return array[closest_point+offset]
    elif offset == 0:
        return array[closest_point]
    else:
        return None

if __name__ == '__main__':
    rospy.init_node("move_robot", anonymous=True)
    rospy.Subscriber("/r1/state/DestinationReached", Time, reached_destination)
    rospy.Subscriber("/r1/state/JointPosition", JointPosition, get_current_joint_values)
    pps_subscriber = rospy.Subscriber("pps_status", Int8, pps_callback)

    INTERPOLATED_TEMPLATE = interpolate(TRAJECTORY, 100)

    NEXT_FULL_INDEX = 20    
    NEXT_SLOW_INDEX = 25

    REACHED_DESTINATION = False
    while not rospy.is_shutdown() and not REACHED_DESTINATION:
        move_l_position()
    
    SPEED = "full"
    set_path_parameters(robot_speed=1.0)

    rate = rospy.Rate(100)

    index = 0
    while index < len(TRAJECTORY):

        rospy.loginfo("Index: {}".format(index))

        # STOP PROCEDURE
        if STATUS == 2:
            current_state = [
                CURRENT_JOINT_VALUES.a1,
                CURRENT_JOINT_VALUES.a2,
                CURRENT_JOINT_VALUES.a3,
                CURRENT_JOINT_VALUES.a4,
                CURRENT_JOINT_VALUES.a5,
                CURRENT_JOINT_VALUES.a6,
                CURRENT_JOINT_VALUES.a7
            ]

            if SPEED == "full":
                next_stop = find_closest(current_state, INTERPOLATED_TEMPLATE, offset=NEXT_FULL_INDEX)
            elif SPEED == "slow":
                next_stop = find_closest(current_state, INTERPOLATED_TEMPLATE, offset=NEXT_SLOW_INDEX)
            else:
                rospy.logerr("We dont know our speed status!")

            if next_stop.any():
                """ rospy.loginfo("Current state: {}".format(current_state))
                rospy.loginfo("Next state   : {}".format(next_stop))"""

                REACHED_DESTINATION = False
                move_joint_space(
                    next_stop[0],
                    next_stop[1],
                    next_stop[2],
                    next_stop[3],
                    next_stop[4],
                    next_stop[5],
                    next_stop[6],
                )
                """ move_joint_space(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                ) """
            else:
                rospy.loginfo("Next stop index is out of reach")
            
            while STATUS == 2:
                rate.sleep()
            index -= 1
        else:            
            if STATUS == 1 or STATUS == 3:
                if STATUS == 1:
                    if SPEED == "full":
                        # STOP first
                        current_state = [
                            CURRENT_JOINT_VALUES.a1,
                            CURRENT_JOINT_VALUES.a2,
                            CURRENT_JOINT_VALUES.a3,
                            CURRENT_JOINT_VALUES.a4,
                            CURRENT_JOINT_VALUES.a5,
                            CURRENT_JOINT_VALUES.a6,
                            CURRENT_JOINT_VALUES.a7
                        ]

                        if SPEED == "full":
                            next_stop = find_closest(current_state, INTERPOLATED_TEMPLATE, offset=NEXT_FULL_INDEX)
                        elif SPEED == "slow":
                            next_stop = find_closest(current_state, INTERPOLATED_TEMPLATE, offset=NEXT_SLOW_INDEX)
                        else:
                            rospy.logerr("We dont know our speed status!")
                        
                        rospy.loginfo(next_stop)
                        
                        try:
                            if next_stop.any():
                                """ rospy.loginfo("Current state: {}".format(current_state))
                                rospy.loginfo("Next state   : {}".format(next_stop))"""

                                REACHED_DESTINATION = False
                                move_joint_space(
                                    next_stop[0],
                                    next_stop[1],
                                    next_stop[2],
                                    next_stop[3],
                                    next_stop[4],
                                    next_stop[5],
                                    next_stop[6],
                                )
                                """ move_joint_space(
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                ) """
                                while (not rospy.is_shutdown()) and (not REACHED_DESTINATION):
                                    rate.sleep()
                            else:
                                rospy.loginfo("Next stop index is out of reach")
                        except:
                            pass
                    set_path_parameters(robot_speed=0.2)
                    SPEED = "slow"
                elif STATUS == 3:
                    set_path_parameters(robot_speed=1.0)
                    SPEED = "full"

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
                while (not rospy.is_shutdown()) and (not REACHED_DESTINATION) and (not STATUS == 2):
                    rate.sleep()
                index += 1
            else:
                rospy.loginfo("Use STATUS 1, 2 or 3")

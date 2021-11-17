#! /usr/bin/env python

"""
.. module:: go_to_point
   :platform: Unix
   :synopsis: Node implementing an algorithm to simulate the motion of the robot
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com
This node simulate the the motion of the robot from a place to another of the Cluedo game

Service:
 	/move_point

This node provides a trivial simulation of the motion of the robot in the environment: it receives from the state_machine node the actual 
position of the robot and the target position to reach via Move custom message, then it evaluates the euclidean distance between these two
points and waits for a time proportionally to the distance. Since the simulation is quite long the time is "just" 1/10 of the evaluated
distance.

"""

import rospy
import math
import time
from geometry_msgs.msg import Point
from experimental_assignment1.srv import Move, MoveResponse



def move(dist):
    """
    Function to emulate the motion of the robot: it receives the distance between the two places and waits for 1/10 nanoseconds of the 
    distance.

        Args: 
            dist: is the euclidean distance between the actual position and the position to reach
        Returns:
            dist(bool): the message to notify that the target has been reached

    """

    print("\nDistance to the next position: " + str(dist))
    time.sleep(0.1*dist) 
    print("\nTarget reached!")
    
    return True ## Notify the target has been reached


def get_target(pos):
    """
    Callback to execute when state_machine node asks to perform a motion from a position to another. It evaluates the euclidean distance 
    between the actual position expressed in x and y coordinates and the target position (also expressed in x and y coordinates).
    After having evaluated it, it calls the function move() to simulate the motion.

        Args: 
            pos(Move): is the custom service message conaining the actual position and the target position to reach
        Returns:
            msg(MoveResponse): the message to notify the state_machine that the target has been reached

    """

    actual_pos=Point()
    target=Point()

    actual_pos.x=pos.x_start
    actual_pos.y=pos.y_start

    target.x=pos.x_end
    target.y=pos.y_end

    # Compute Euclidean distance
    dist = math.sqrt(pow(target.x-actual_pos.x, 2)+pow(target.y-actual_pos.y, 2))

    # Simulate the motion by calling the move() function
    res=move(dist)
    msg=MoveResponse()
    msg.reached=res

    return msg


    

def main():
    """
    Main function of the go_to_point node it initializes the node itself, then creates a service on topic /move_point.

    """

    rospy.init_node('go_to_point') 
    rospy.Service('/move_point', Move, get_target)

    rospy.spin()



if __name__ == '__main__':
    main()
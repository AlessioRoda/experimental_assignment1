#! /usr/bin/env python3

"""
.. module:: go_to_point
   :platform: Unix
   :synopsis: Node implementing an algorithm to move the robot
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com

This node drive the robot to the desired position received

Subscriber:
    /odom 
Publisher:
	/cmd_vel 

ActionServer:
    go_to_point_action: action server to receive the coordinates of the place to reach 
 
 It's performed as the node that permits the robot to move: it contains all the functions to set the velocity and the orientation of the robot
 based on the target it has to reach via the ActionGoal that was sent from the gotoWaypoint node. In order to do this, when a new target is 
 generated the robot aligns its orientation, then moves to the target. Once the target is reached, the node sets the target action as succeeded, then waits until a new 
 target is sent.

"""

import rospy
import actionlib
from erl2.msg import MoveAction, MoveActionResult
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math



# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

ang_coef=2
''' int: coefficient to increase manage the angular velocity of the robot during the motion

'''
lin_coef=1.7
''' int: coefficient to increase manage the linear velocity of the robot during the motion

'''

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

server=None


## Callback to get the current odom position of the robot 
def clbk_odom(msg):
    '''
    Callback to get the odometry of the robot 

    Args: 
        msg(Odom): odometry of the robot with its x, y, z coordinates and orientation
            
    '''
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    '''
    Function to change the state of th FSM in the go_to_point function

    Args: 
        state(int): new state of the FSM
            
    '''
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    '''
    Function to normalize the angle of the rotation that the robot must perform

    Args: 
        angle(float): angle of the rotation to perform

    '''
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    '''
    Function to direct the robot towards the target rotation

    Args: 
        des_pos(float): desired position to reach

    '''
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    twist_msg.angular.z=twist_msg.angular.z*ang_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    '''
    Function to move the robot straight ahead to a certain destination respect to its actual position

    Args: 
        des_pos(Pose): object containing the x and y coordinates of the position to reach

    '''
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
           twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw

        twist_msg.angular.z=twist_msg.angular.z*ang_coef
        twist_msg.linear.x=twist_msg.linear.x*lin_coef
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    '''
    Function to fix the orientation of the robot on the basis of the desired orientation

    Args: 
        msg(MoveGoal): action goal to get the coordinates of the position to reach

    '''

    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    twist_msg.angular.z=twist_msg.angular.z*ang_coef
    twist_msg.linear.x=twist_msg.linear.x*lin_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    '''
    Function to stop the robot by setting its linear and angular velocity to zero

    '''

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


def go_to_point(goal):
    """
    Function that gets the goal position to reach and, based on that, calls the functions to perform the motion of the robot.
    It checks if the goal has't been cancelled; in this case it sets the state_ variable to 3 in order to terminate the process to move 
    to reach a certain position

    Args: 
        msg(MoveGoal): action goal to get the coordinates of the position to reach
 
    """
    global server, state_
    result=MoveActionResult()
    desired_position = Point()
    desired_position.x = goal.x_pos
    desired_position.y = goal.y_pos
    des_yaw = goal.theta

    change_state(0)
    while True:
        
        if server.is_preempt_requested():
            state_=3

        if state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
    result.result.reached=True
    server.set_succeeded(result.result)
    


## In the main there are the initialization of the publisher, th subscriber and the SimpleActionServer
def main():
    '''
    Main function that performs the initialization of the node itself and initializes all the services and subscribers.

    '''
    global pub_, server
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    server = actionlib.SimpleActionServer('go_to_point_action', MoveAction, execute_cb = go_to_point, auto_start=False)
    server.start()
    
    rospy.spin()


if __name__ == '__main__':
    main()
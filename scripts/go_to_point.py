#! /usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Point
from experimental_assignment1.srv import Move, MoveResponse

pub_move=None
actual_pos=Point()
target=Point()


## Wait time proportionally to the distance
def move(dist):
    global pub_move

    print("\nDistance to the next position: " + str(dist))
    time.sleep(0.1*dist) 
    print("\nTarget reached!")
    
    return True ## Notify the target has been reached


def get_target(pos):
    global target, actual_pos

    actual_pos.x=pos.x_start
    actual_pos.y=pos.y_start
    actual_pos.z=0 ## don't care

    target.x=pos.x_end
    target.y=pos.y_end
    target.z=0 ## don't care

    ## Compute Euclidean distance
    dist = math.sqrt(pow(target.x-actual_pos.x, 2)+pow(target.y-actual_pos.y, 2))

    res=move(dist)
    msg=MoveResponse()
    msg.reached=res

    return msg


    

def main():
    global pub_move

    rospy.init_node('go_to_point') 
    pub_move=rospy.Service('/move_point', Move, get_target)
    #pub2=rospy.ServiceProxy('/move_point', Move)

    rospy.spin()



if __name__ == '__main__':
    main()
#! /usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import smach
import smach_ros
from classes.place import Place
import actionlib
from erl2.msg import MoveAction


pub_move_base=None
movearm_client=None

places=[]
''' Array with the places of the scene
'''
actual_pos=None
''' Actual position of the robot in the environment
'''
oracle=False
''' Boolean to know if the robot must go to the Oracle_Room
'''
response_complete=None
''' Response of the query for a complete hypothesis
'''
oracle_room=None
''' Initialize the Oracle_Room
'''



def init_scene():

    global places, actual_pos, oracle_room
    places.append(Place('ROOM1', -4, -3))
    places.append(Place('ROOM2', -4, 2))
    places.append(Place('ROOM3', -4, 7))
    places.append(Place('ROOM4', 5, -7))
    places.append(Place('ROOM5', 5, -3))
    places.append(Place('ROOM6', 5, 1))

    oracle_room=Place('Oracle_Room', 0, 0)
    actual_pos=Place('Oracle_Room', 0, 0) #Starts in the oracle room



class Explore(smach.State):
    
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['check_consistency', 'solution'])
        
    def execute(self, userdata):
        global pub_move_base, actual_pos, places, oracle, movearm_client

        if oracle==False:
            destination=places.pop()
            
        else:
            destination=oracle_room

        ## Initialize a MoveBaseActionGoal target to move my robot
        move_goal = MoveBaseActionGoal()
        move_goal.goal.target_pose.header.frame_id="map"
        move_goal.goal.target_pose.pose.orientation.w=1

        move_goal.goal.target_pose.pose.position.x = destination.x
        move_goal.goal.target_pose.pose.position.y = destination.y

        pub_move_base.wait_for_server()
        print("\nMoving from " + actual_pos.name + " to " + destination.name +"\n")
        pub_move_base.send_goal(move_goal)
        pub_move_base.wait_for_result()
        
        actual_pos.x=destination.x
        actual_pos.y=destination.y
        actual_pos.name=destination.name


        # If it is going to the oracle room it sets oracle to False for the future iterations of the state_machine, then returns 'solution'
        if oracle == True:
            oracle=False
            return 'solution'
        
        else:
            movearm_client.send_goal()
            # Move the arm to explore the room

            return 'enter_room'


def main():
    global pub_move_base, movearm_client

    # Initialize the node
    rospy.init_node('state_machine')
    #pub_move_base=rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
    pub_move_base=actionlib.SimpleActionClient('move_base', MoveBaseAction)
    movearm_client=actionlib.SimpleActionClient('movearm_action', MoveAction)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['state_machine'])
    sm.userdata.sm_counter = 0


     # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Explore(), 
                               transitions={'check_consistency':'CHECK_CONSISTENCY',
                                            'solution':'SOLUTION' })

        smach.StateMachine.add('ENTER_ROOM', Check_Consistency(), 
                               transitions={'explore':'MOVE'})
        
        smach.StateMachine.add('SOLUTION', Try_Solution(),
                                transitions={'explore':'MOVE',
                                             'correct':'state_machine'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()




if __name__ == '__main__':
    main()
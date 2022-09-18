#! /usr/bin/env python3

from time import sleep
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros
from classes.place import Place
import actionlib
from std_msgs.msg import Int32
from erl2.msg import MoveAction, MoveActionGoal, ErlOracle
from erl2.srv import MarkerRequest, Marker, Hint, HintRequest, TrySolution, TrySolutionRequest, Update


pub_move_base=None
movearm_client=None
sub_ID=None
client_ID_msg=None
client_add_hint=None
client_update_ontology=None
client_try_solution=None

IDs=[]
tried_IDs=[]

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
stop=False


def IDs_callback(id):
    global IDs
    if id.data not in IDs and stop==False and id.data<=40:
        IDs.append(id.data)


def init_scene():

    global places, actual_pos, oracle_room
    places.append(Place('ROOM1', -4, -3))
    places.append(Place('ROOM2', -4, 2))
    places.append(Place('ROOM3', -4, 7))
    places.append(Place('ROOM4', 5, -7))
    places.append(Place('ROOM5', 5, -3))
    places.append(Place('ROOM6', 5, 1))

    oracle_room=Place('Oracle_Room', 0, -1)
    actual_pos=Place('Oracle_Room', 0, -1) #Starts in the oracle room



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
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id="map"
        move_goal.target_pose.pose.orientation.w=1
        move_goal.target_pose.pose.orientation.z=0
        move_goal.target_pose.pose.orientation.x=0
        move_goal.target_pose.pose.orientation.y=0

        move_goal.target_pose.pose.position.x = destination.x
        move_goal.target_pose.pose.position.y = destination.y
        move_goal.target_pose.pose.position.z = 0

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
            sleep(5)

            return 'check_consistency'

class Check_Consistency(smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore'])

    def execute(self, userdata):
        global IDs, tried_IDs, client_ID_msg, oracle, client_update_ontology, stop

        #Stop adding elements in the list
        stop=True

        for id in IDs:
            if id not in tried_IDs:
                print("State machine 1")
                req=MarkerRequest()
                req.markerId=id
                rospy.wait_for_service('/oracle_hint')
                res=client_ID_msg(req)
                print("Risposta: "+str(res))
                print("State machine 2")
                oracle_hint=ErlOracle()
                oracle_hint.ID=res.oracle_hint.ID
                oracle_hint.key=res.oracle_hint.key
                oracle_hint.value=res.oracle_hint.value
                req=HintRequest()
                req.oracle_hint=oracle_hint
                client_add_hint(req)
                print("State machine 3")
            
                tried_IDs.append(id)

        #After having added all the hints to the ontology, perform "reason" operation
        rospy.wait_for_service('/ontology_interface/update_request')
        client_update_ontology()

        print("State machine 4")
        #Clear ID list and update oracle flag to send robot in the oracle room to try a solution
        IDs.clear()
        oracle=True
        stop=False

        return 'explore'


class Try_Solution(smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore', 'correct'])

    def execute(self, userdata):
        res=client_try_solution(TrySolutionRequest())

        if res.solution_found==True:
            print("Solution found!")
            return 'correct'
        else:
            print("Solution not correct")
            return 'explore'


def main():
    global pub_move_base, movearm_client, sub_ID, client_ID_msg, client_add_hint, client_update_ontology, client_try_solution

    # Initialize the node
    rospy.init_node('state_machine')
    #pub_move_base=rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
    pub_move_base=actionlib.SimpleActionClient('move_base', MoveBaseAction)
    movearm_client=actionlib.SimpleActionClient('movearm_action', MoveAction)
    sub_ID=rospy.Subscriber('/marker_id', Int32, IDs_callback)
    client_ID_msg=rospy.ServiceProxy('/oracle_hint', Marker)
    client_add_hint=rospy.ServiceProxy('/ontology_interface/add_hint', Hint)
    client_update_ontology=rospy.ServiceProxy('/ontology_interface/update_request', Update)
    client_try_solution=rospy.ServiceProxy('/ontology_interface/try_solution', TrySolution)

    init_scene()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['state_machine'])
    sm.userdata.sm_counter = 0


     # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Explore(), 
                               transitions={'check_consistency':'CHECK_CONSISTENCY',
                                            'solution':'SOLUTION' })

        smach.StateMachine.add('CHECK_CONSISTENCY', Check_Consistency(), 
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
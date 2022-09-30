#! /usr/bin/env python3

'''
.. module:: state_machine
   :platform: Unix
   :synopsis: Node implementing a finite state machine that represents the steps of the Cluedo game
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com

This node allows to simulate the Cluedo game 

Subscriber:
    /marker_id custom service to get the ID of a marker when it's detected by the camera

Client: 
    /oracle_hint custom service to ask the hint (ID, type and message) associated to a specific marker ID 

    /ontology_interface/add_hint service to send a new hint to the oracle_interface node 

    /ontology_interface/update_request service to send the request to the oracle_interface node for updating the ontology by adding the hints to the ontology and by performing the "REASON" command
    
    /ontology_interface/check_consistency service to send a request to the oracle_interface to find the complete and consistent hypothesis in the ontology
    
    /oracle_solution service asks to the final_module node to send the solution ID of the Cluedo game
    
    /ontology_interface/ask_solution to query the place, the person and the weapon corresponding to a certain ID

ActionClient:
    movebase to move to a specific position  

 It's performed as the node that permits to simulate the entire Cluedo game, it's the main of the whole architecture and with the smach 
 StateMachine can simulate the game by switching from three states:
 
 -MOVE: It's defined by the Explore class and performs the motion of the robot from a room to another by using the move_base action service. Each time the MOVE state is executed, 
        it checks if the robot must go to the Oracle Room (oracle==True) or in one of the other rooms: in this case all the rooms (that are defined as
        Place objects, with their name and coordinates) are stored in a list and, each time the robot moves, it reaches the last element in the list.
 
 -CHECK_CONSISTENCY: It's defined by the Check_Consistency class, in this state all the aruco IDs that are received from the topic /marker_id and haven't already 
                    been checked are sent via the /oracle_hint to receive the corresponding hint; then the new hints are stored in the ontology_interface node via /ontology_interface/add_hint service. 
                    After having stored all the hints, the ontology is updated with the new hints, by calling the service /ontology_interface/update_request; then it also asks for all the complete 
                    and consistent hypothesis in the ontology calling /ontology_interface/check_consistency service and removes inconsistent hypothesis. If there are complete and consistent hypothesis the robot moves to the Oracle Room 
                    (oracle set to True), otherwise it will move to another room (oracle is kept to False value).
 
 -SOLUTION: the state that represents the moment in which the robot is trying to generate a solution for the Cluedo game. It's defined with
            the Try_Solution class and during its execution checks if one of the complete and consistent IDs corresponds to the solution of the game, that is obtained via /oracle_solution service.
            If the solution is found, it asks the place, the person and the weapon corresponding to the solution ID and the simulation ends, otherwise the robot returns in the MOVE state.           

'''

from time import sleep
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros
from classes.place import Place
import actionlib
from std_msgs.msg import Int32
from erl2.msg import ErlOracle
from erl2.srv import MarkerRequest, Marker, Hint, HintRequest, Consistent, ConsistentRequest, Update, Oracle, OracleRequest, Solution, SolutionRequest


pub_move_base=None
''' Initialize the action client for performing the motion of the robot via move_base action 

'''
sub_ID=None
''' Subscriber to /marker_id topic

'''
client_ID_msg=None
''' Initialize the /oracle_hint service client

'''
client_add_hint=None
''' Initialize the /ontology_interface/add_hint service client

'''
client_update_ontology=None
''' Initialize the /ontology_interface/update_request service client

'''
client_try_solution=None
''' Initialize the /ontology_interface/check_consistency service client

'''
ask_solution=None
''' Initialize the /oracle_solution service client

'''
armor_solution=None
''' Initialize the /ontology_interface/ask_solution service client

'''

IDs=[]
''' list: List with the new IDs that are received

'''
tried_IDs=[]
''' list: List with the IDs that have already been included in the ontology

'''
consistent_ids=[]
''' list: List with the consistent_ids that have been found

'''

places=[]
''' list: List with the places of the scene

'''
actual_pose=None
''' Actual position of the robot in the environment

'''
oracle=False
''' bool: Boolean to know if the robot must go to the Oracle_Room

'''
oracle_room=None
''' Initialize the Oracle_Room

'''
stop=False
''' bool: Flag to stop storing new IDs in the IDs list

'''

def IDs_callback(id):
    '''
        Callback to get the IDs of the marker that have been detected by the aruco_ros node

    '''
    global IDs
    if id.data not in IDs and stop==False and id.data<=40:
        IDs.append(id.data)


def init_scene():
    '''
        Function to initialize the scenario with the rooms of the simulation

    '''
    global places, actual_pose, oracle_room
    places.append(Place('ROOM1', -4, -3))
    places.append(Place('ROOM2', -4, 2))
    places.append(Place('ROOM3', -4, 7))
    places.append(Place('ROOM4', 5, -7))
    places.append(Place('ROOM5', 5, -3))
    places.append(Place('ROOM6', 5, 1))

    oracle_room=Place('Oracle_Room', 0, -1)
    actual_pose=Place('Oracle_Room', 0, -1) #Starts in the oracle room



class Explore(smach.State):
    '''
        A class to represent the behavior of the robot when it moves from a place to another

    '''
    
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['check_consistency', 'solution'])
        
    def execute(self, userdata):
        '''
        Description of the execute method:
        It generates a move_base action goal with the coordinates of the last room in the list or, 
        in case a possible solution is found (oracle==True), the coordinates of the Oracle Room, then sends it and waits until 
        the target is reached.

            Args:
                userdata to store the variables between the states
            Returns:
                'solution'(string): if it must go to the SOLUTION state
                'enter_room'(string): if it must go to the ENTER_ROOM state

        '''

        global pub_move_base, actual_pose, places, oracle

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
        print("\nMoving from " + actual_pose.name + " to " + destination.name +"\n")
        pub_move_base.send_goal(move_goal)
        pub_move_base.wait_for_result()
        
        actual_pose.x=destination.x
        actual_pose.y=destination.y
        actual_pose.name=destination.name


        # If it is going to the oracle room it sets oracle to False for the future iterations of the state_machine, then returns 'solution'
        if oracle == True:
            oracle=False
            return 'solution'
        
        else:
            sleep(5)

            return 'check_consistency'

class Check_Consistency(smach.State):
    '''
        A class to interact with the ARMOR ontology, asking for complete and consistent hypothesis as possible solutions

    '''

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore'])

    def execute(self, userdata):
        '''
        Description of the execute method:
        It checks if there are new IDs to try as a possible solution and in that case it sends them to the ontology_interface node, updates the ontology and then
        asks for complete and consistent hypothesis to use as a possible solution, then removes the inconsistent hypotheses (if any).
           
            Args:
                userdata to store the variables between the states 
            
            Returns:
                'explore'(string): it must go to the MOVE state

        '''

        global IDs, tried_IDs, client_ID_msg, oracle, client_update_ontology, stop, consistent_ids

        #Stop adding elements in the list
        stop=True

        for id in IDs:
            if id not in tried_IDs:
                req=MarkerRequest()
                req.markerId=id
                rospy.wait_for_service('/oracle_hint')
                res=client_ID_msg(req)
                print("Reply: "+str(res))
                oracle_hint=ErlOracle()
                oracle_hint.ID=res.oracle_hint.ID
                oracle_hint.key=res.oracle_hint.key
                oracle_hint.value=res.oracle_hint.value
                req=HintRequest()
                req.oracle_hint=oracle_hint
                client_add_hint(req)
            
                tried_IDs.append(id)

        #After having added all the hints to the ontology_interface node, update the ontology performing "reason" operation
        rospy.wait_for_service('/ontology_interface/update_request')
        client_update_ontology()

        #Clear ID list and update oracle flag to send robot in the oracle room to try a solution
        IDs.clear()

        #Ask for complete and consistent hypothesis as potential solutions
        res=client_try_solution(ConsistentRequest())

        if len(res.consistent)>0:
            consistent_ids=res.consistent
            oracle=True
        
        stop=False
        return 'explore'


class Try_Solution(smach.State):
    '''
        A class to represent the behavior of the robot when it tries to generate a solution

    '''

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore', 'correct'])

    def execute(self, userdata):
        '''
        Description of the execute method:
        It calls the /oracle_solution service to get the solution of the game, then it checks if one of the complete and consistent hypothesis 
        corresponds to the solution. If it's correct it prints the solution associated to the ID, then the game ends, otherwise it
        returns to the MOVE state.
           
            Args:
                userdata to store the variables between the states (not utilized)
            
            Returns:
                'explore'(string): it must go to the MOVE state
                'correct'(string): the solution is correct and the state machine is stopped

        '''

        global consistent_ids

        solution=ask_solution(OracleRequest())
        for id in consistent_ids:
            if str(solution.ID) == id:
                print("\nSolution found!: "+ str(solution.ID))

                req=SolutionRequest()
                req.ID=id
                res=armor_solution(req)
                print("Person: "+res.person+" Weapon: "+res.weapon+" Place: "+res.place)

                return 'correct'
            else:
                print("Solution ID "+id +" is not correct")

        consistent_ids.clear()
        return 'explore'

        


def main():
    '''
    It's the main of the state_machine node, it performs the initialization of the node itself and initializes all the services and subscribers.
    It also defines the SMACH state machine and runs it to start the simulation

    '''

    global pub_move_base, sub_ID, client_ID_msg, client_add_hint, client_update_ontology, client_try_solution, ask_solution
    global armor_solution

    # Initialize the node
    rospy.init_node('state_machine')
 
    pub_move_base=actionlib.SimpleActionClient('move_base', MoveBaseAction)
    sub_ID=rospy.Subscriber('/marker_id', Int32, IDs_callback)
    client_ID_msg=rospy.ServiceProxy('/oracle_hint', Marker)
    client_add_hint=rospy.ServiceProxy('/ontology_interface/add_hint', Hint)
    client_update_ontology=rospy.ServiceProxy('/ontology_interface/update_request', Update)
    client_try_solution=rospy.ServiceProxy('/ontology_interface/check_consistency', Consistent)
    ask_solution=rospy.ServiceProxy('/oracle_solution', Oracle)
    armor_solution=rospy.ServiceProxy('/ontology_interface/ask_solution', Solution)

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
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()




if __name__ == '__main__':
    main()
#! /usr/bin/env python

'''
.. module:: state_machine
   :platform: Unix
   :synopsis: Node implementing a finite state machine that represents the steps of the Cluedo game
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com
This node permits to simulate the cluedo game 

Client: 
     /move_point: custom service to send the position to reach to the go_to_point node
     /hint_request: custom service to ask a hint message to oracle node
     /solution: custom service to send the possible solution to the oracle node

 It's performed as the node that permits to simulate the entire Cluedo game, it's the main of the whole architecture and with the smach 
 StateMachine can simulate the game by switching from three states:
 
 -MOVE: to simulate the motion of the robot from a room to another, it's referred to the Explore class. The game begins with the robot in the Oracle_Room 
        and in order to simulate the motion a Move custom service message with the actual 
        position of the robot and the target to reach is sent to the go_to_point node. Once the motion has been performed to the go_to_point node
        it can switch to two possible steps: if the robot wants to try a solution (and so in this case the variable oracle==True) it switches
        to the SOLUTION state, otherwise it switches to the ENTER_ROOM, that means that it wants to explore the rooms to collect hints

 -ENTER_ROOM: the state represented with the Enter_Room class that represents the moment in which the robot enters in a room to acquire hints.
              In order to get hints it sends a request to the oracle node, then it gets a hint defined by an ID
              and three arrays (what, where and who) that represent some information about weapons, places and people.
              After having obtained the hint it adds it to the ontology by using the MyArmor class methods, if the hypothesis uploaded is 
              completed and consistent, then the variable oracle=True, that means that the robot must go to the Oracle_Room to ask to the
              oracle if the hypothesis is the possible solution. 

 -SOLUTION: the state that represents the moment in which the robot is trying to generate a solution for the Cluedo game. It's defined with
            the Try_Solution class and gets the person, the place and the weapon associated with the complete and consistent hypothesis,
            then it sends them to the oracle node with the Solution custom service message. If the oracle confirms that the solution is exact
            the game ends, otherwise it continue by switching to the MOVE state.          

'''


from posixpath import dirname, realpath
import rospy
from rospy.impl.tcpros_service import ServiceProxy, wait_for_service
import smach
import smach_ros
import random
from datetime import datetime
import time
from geometry_msgs.msg import Point
from experimental_assignment1.srv import Move, MoveRequest, AskHint, Solution, SolutionRequest
from classes.myArmor import MyArmor
from classes.place import Place
from armor_msgs.srv import *
from armor_msgs.msg import *


pub_move=None
''' Initialize the publisher to /move_point

'''
pub_ask_hint=None
''' Initialize the publisher to /hint_request

'''
pub_solution=None
''' Initialize the publisher to /solution

'''
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

people_ontology=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
''' Define all the people of the scene

'''
places_ontology=['Ballroom', 'Billiard_Room', 'Conservatory']
''' Define all the places of the scene

'''
weapons_ontology=['Candlestick', 'Dagger','LeadPipe']
''' Define all the weapons of the scene

'''
oracle_room=None
''' Initialize the Oracle_Room

'''


def init_scene():
    '''
    Function to initialize the scene, it defines three rooms as Place objects, then defines the starting position in the Oracle_Room (x=0, y=0)
    then it adds all the information about the scene to the armor ontology, by loading all the people, places and weapons.
    When a new element is loaded in the ontology it must be disjointed respect to the others of the same class, in order to notify the ontology
    that they are different. Finally it preforms 'REASON' command to update the ontology.
    
    '''

    global places, actual_pos, oracle_room
    places.append(Place(places_ontology[0], 5, 5))
    places.append(Place(places_ontology[1], -5, -5))
    places.append(Place(places_ontology[2], 5, -5))

    oracle_room=Place('Oracle_Room', 0, 0)
    actual_pos=Place('Oracle_Room', 0, 0) #Starts in the oracle room

    ## Add people, weapons and places o the ontology

    j=0
    while j!=len(people_ontology):
        res=MyArmor.add_item(people_ontology[j], 'PERSON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(people_ontology[j], people_ontology[count-1])
                count = count -1
        j=j+1

    j=0
    while j!=len(places_ontology):
        res=MyArmor.add_item(places_ontology[j], 'PLACE')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(places_ontology[j], places_ontology[count-1])
                count = count -1
        j=j+1

    j=0
    while j!=len(weapons_ontology):
        res=MyArmor.add_item(weapons_ontology[j], 'WEAPON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(weapons_ontology[j], weapons_ontology[count-1])
                count = count -1
        j=j+1

    res=MyArmor.reason()
    
        

class Explore(smach.State):
    '''
        A class used to represent the behavior of the robot when it moves from a place to another
        ...
        Methods
        -------
        __init__(outcomes=['enter_room', 'solution'])
            initialize the state
        execute(userdata)
            implement the behavior
    '''

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['enter_room', 'solution'])
        
    def execute(self, userdata):
        '''
        Description of the execute method:
        First it checks if the robot must go to the Oracle_Room or have to move randomly in the environment, by default it moves randomly
        in the places of the environment except in the Oracle_Room, since there aren't any hints to collect. Each time it has to move
        it checks that the new place to reach does't correspond to the actual robot position, once the target is reached it updates the actual
        position. Finally it returns 'solution' or 'enter_room' on the basis of the state it has to reach.

            Args:
                userdata to store the variables between the states

            Returns:
                'solution'(string): if it must go to the SOLUTION state
                'enter_room'(string): if it must go to the ENTER_ROOM state

        '''
        global pub_move, actual_pos, places, oracle

        if oracle==False:
            random.seed(datetime.now())
            index=random.randint(0, len(places)-1)

            ## Can't be in the same place in which it is
            while places[index].name==actual_pos.name:
                index=random.randint(0, len(places)-1)
            destination=places[index]

        else:
            destination=oracle_room

        # Generate the Move custom message to perform the motion
        msg=MoveRequest()
        msg.x_start=actual_pos.x
        msg.y_start=actual_pos.y
        msg.x_end=destination.x
        msg.y_end=destination.y

        res=pub_move(msg)
        print("\nMoving from " + actual_pos.name + " to " + destination.name +"\n")

        # Wait for a response from the go_to_point node
        rospy.wait_for_service('move_point')

        if(res.reached==True):
            # When reached update the current position
            actual_pos.x=destination.x
            actual_pos.y=destination.y
            actual_pos.name=destination.name
        else:
            print("\nPosition " + destination.name + " not reached")

        # If it is going to the oracle room it sets oracle to False for the future iterations of the state_machine, then returns 'solution'
        if oracle == True:
            oracle=False
            return 'solution'
        
        else:
            return 'enter_room'


class Enter_Room(smach.State):
    '''
        A class used to represent the behavior of the robot when it enters in a room and asks for hints
        ...
        Methods
        -------
        __init__(outcomes=['explore'])
            initialize the state
        execute(userdata)
            implement the behavior
    '''

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore'])
        
    def execute(self, userdata):
        '''
        Description of the execute method:
        It asks for a hint from the oracle node, then adds to the ontology all the informations that it received about the PERSON (who),
        the PLACE (where) and the WEAPON (what), then it checks if there are complete and consistent hypothesis in the ontology.
        If there exists oracle=True, that means that the robot must go to the Oracle_Room, otherwise the hypothesis is removed from the
        ontology in order to mantain it as simple as possible.

            Args:
                userdata to store the variables between the states 
            
            Returns:
                'explore'(string): it must go to the MOVE state
        '''

        global pub_ask_hint, oracle, response_complete

        res=pub_ask_hint()  
        # ID of the hint received
        ID=res.ID

        # Add hints to the ontology 
        count=0
        while(count!=len(res.what)):
            request=MyArmor.add_hypothesis('what', ID, res.what[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.what[count])
            count = count +1

        count=0
        while(count!=len(res.where)):
            request=MyArmor.add_hypothesis('where', ID, res.where[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.where[count])
            count = count +1

        count=0
        while(count!=len(res.who)):
            request=MyArmor.add_hypothesis('who', ID, res.who[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.who[count])
            count = count +1

        # Reason
        reason=MyArmor.reason()

        if reason.armor_response.success==False:
                print("\nError, cannot perform reasoning")


        # First asks for all consistent queries
        response_complete=MyArmor.ask_complete()
        if response_complete.armor_response.success==False:
            print("\nError in asking query")        

        if len(response_complete.armor_response.queried_objects)!=0:
            response_inconsistent=MyArmor.ask_inconsistent()
        
            # Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:
                str_inconsistent=response_inconsistent.armor_response.queried_objects[0]
                str_inconsistent=str_inconsistent[40:]
                id_inconsistent=str_inconsistent[:-1]
                print("ID_inconsistent "+str(id_inconsistent) +"\n")
                res=MyArmor.remove(id_inconsistent)
                
                if res.armor_response.success==False:
                    print("Error in removing\n")
            
            ## If the hypothesis are completed and not inconsistent, then let's go to the oracle
            else:
                oracle=True
        # If the response isn't complete it removes it from the ontology
        else:
            res=MyArmor.remove(ID)
            if res.armor_response.success==False:
                print("Error in removing\n")

        return 'explore'
            



class Try_Solution(smach.State):
    '''
            A class used to represent the behavior of the robot when it tries to generate a solution
            ...
            Methods
            -------
            __init__(outcomes=['explore', 'correct'])
                initialize the state
            execute(userdata)
               implement the behaviour
    '''

    def __init__(self):
        # initialization function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['explore', 'correct'])
        
    def execute(self, userdata):
        '''
        Description of the execute method:
        It gets all the informations about the person, the place and the weapon from the consistent hypothesis, then sends these infomation
        to the oracle node via Solution custom message. If the oracle node confirms that it's correct the game ends and it saves the final
        ontology in the solution_cluedo_ontology.owl file, otherwise it removes the incorrect hypothesis from the ontology, 
        then returns to the MOVE state.

            Args:
                userdata to store the variables between the states (not utilized)
            
            Returns:
                'explore'(string): it must go to the MOVE state
                'correct'(string): the solution is correct and the state machine is stopped
        '''

        global  pub_solution

        #In order to make the view simulation more comfortable wait 1 second before the execution
        time.sleep(1) 
        
        # Get the id of the hypothesis
        id_consistent=response_complete.armor_response.queried_objects[0]
        id_consistent=id_consistent[40:]
        id_consistent=id_consistent[:-1]

        print("\n\nID consistent: " + id_consistent + "\n")

        # Get the weapon
        res_what=MyArmor.ask_item('what', id_consistent)
        what=res_what.armor_response.queried_objects[0]
        what=what[40:]
        what=what[:-1]

        # Get the place
        res_where=MyArmor.ask_item('where', id_consistent)
        where=res_where.armor_response.queried_objects[0]
        where=where[40:]
        where=where[:-1]

        # Get the person
        res_who=MyArmor.ask_item('who', id_consistent)
        who=res_who.armor_response.queried_objects[0]
        who=who[40:]
        who=who[:-1]

        # Formalize the possible solution
        print("\nThe killer is: " + who + " in the " + where + " with " + what + "\n")

        # Send the solution to oracle
        sol=SolutionRequest()
        sol.what=what
        sol.where=where
        sol.who=who
        res=pub_solution(sol)
        rospy.wait_for_service('solution')
        if res.correct==True:
            print("\nSolution is correct, the game is finished!\n")
            MyArmor.save()
            return 'correct'

        elif res.correct==False:
            print("\nIt's not the correct solution\n")
            res=MyArmor.remove(id_consistent)

            return 'explore'
    


def main():
    '''
    It's the main of the state_machine node, it performs the initialization of the node itself and initializes the services to the /move_point,
    /hint_request and /solution topics. Then it loads the cluedo_ontology.owl ontology with the "load" method of the MyArmor class, calls 
    the init_scene() method to initialize the scene and finally defines all the state of the FSM with smach.
    '''

    global  pub_move, pub_ask_hint, pub_solution

    # Initialize the node
    rospy.init_node('state_machine')

    pub_move=ServiceProxy('/move_point', Move)
    pub_ask_hint=ServiceProxy('/hint_request', AskHint)
    pub_solution=ServiceProxy('/solution', Solution)

    # Gets the actual path of the state_machine.py file
    path = dirname(realpath(__file__))
    path = path[:-7] + "cluedo_ontology.owl"
    
    # Loads the ontology
    response=MyArmor.load(path)

    if response.armor_response.success==True:
        print("\nOntology loaded successfully")
    else:
        print("\nERROR: Ontology not loaded correctly")

    # Initialize the scene
    init_scene()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['state_machine'])
    sm.userdata.sm_counter = 0


     # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Explore(), 
                               transitions={'enter_room':'ENTER_ROOM',
                                            'solution':'SOLUTION' })

        smach.StateMachine.add('ENTER_ROOM', Enter_Room(), 
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

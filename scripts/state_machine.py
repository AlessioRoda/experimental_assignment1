#! /usr/bin/env python

from posixpath import dirname, realpath
import rospy
from rospy.impl.tcpros_service import ServiceProxy, wait_for_service
import smach
import smach_ros
import random
from datetime import datetime, time
from geometry_msgs.msg import Point
from experimental_assignment1.srv import Move, MoveRequest, AskHint, Solution, SolutionRequest
from classes.myArmor import MyArmor
from classes.place import Place
from armor_msgs.srv import *
from armor_msgs.msg import *


armor_interface=None
pub_move=None
pub_ask_hint=None
pub_solution=None
places=[]
actual_pos=None
pos_sent=Point()
reached=False
hint_received=False
oracle=False
response_complete=None


people_ontology=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
places_ontology=['Ballroom', 'Biliard_Room', 'Conservatory']
weapons_onotology=['Candlestick', 'Dagger','LeadPipe']
oracle_room=None


def init_scene():
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
    while j!=len(weapons_onotology):
        res=MyArmor.add_item(weapons_onotology[j], 'WEAPON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(weapons_onotology[j], weapons_onotology[count-1])
                count = count -1
        j=j+1

    res=MyArmor.reason()
    
        

class Explore(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['enter_room', 'solution'])
        
    def execute(self, userdata):
        global pub_move, actual_pos, places, oracle

        if oracle==False:
            random.seed(datetime.now())
            index=random.randint(0, len(places)-1)

            ## Can't be neither in the same place in wich it is, nor in the oracle
            while places[index].name==actual_pos.name:
                index=random.randint(0, len(places)-1)
            destination=places[index]

        else:
            destination=oracle_room

        msg=MoveRequest()
        msg.x_start=actual_pos.x
        msg.y_start=actual_pos.y
        msg.x_end=destination.x
        msg.y_end=destination.y

        res=pub_move(msg)
        print("\nMoving from " + actual_pos.name + " to " + destination.name +"\n")

        rospy.wait_for_service('move_point')

        if(res.reached==True):
            actual_pos.x=destination.x
            actual_pos.y=destination.y
            actual_pos.name=destination.name
        else:
            print("\nPosition " + destination.name + " not reached")

        if oracle == True:
            oracle=False
            return 'solution'
        
        else:
            return 'enter_room'


class Enter_Room(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['explore'])
        
    def execute(self, userdata):
        global pub_ask_hint, armor_interface, oracle, response_complete

        #rospy.wait_for_service('hint_request')
        res=pub_ask_hint()  
        ID=res.ID

        ## Add hints to the ontology 
        count=0
        while(count!=len(res.what)):
            request=MyArmor.add_hipotesis('what', ID, res.what[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.what[count])
            count = count +1

        count=0
        while(count!=len(res.where)):
            request=MyArmor.add_hipotesis('where', ID, res.where[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.where[count])
            count = count +1

        count=0
        while(count!=len(res.who)):
            request=MyArmor.add_hipotesis('who', ID, res.who[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.who[count])
            count = count +1

        ## Reason
        reason=MyArmor.reason()

        if reason.armor_response.success==False:
                print("\nError, cannot perform reasoning")


        ## First asks for all consistent queries
        response_complete=MyArmor.ask_complete()
        if response_complete.armor_response.success==False:
            print("\nError in asking query")        

        if len(response_complete.armor_response.queried_objects)!=0:
            response_inconsistent=MyArmor.ask_inconsistent()
        
            ## Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:
                str_inconsitent=response_inconsistent.armor_response.queried_objects[0]
                str_inconsitent=str_inconsitent[40:]
                id_inconsistent=str_inconsitent[:-1]
                print("ID_inconsistent "+str(id_inconsistent) +"\n")
                res=MyArmor.remove(id_inconsistent)
                
                if res.armor_response.success==False:
                    print("Error in removing\n")
            
            ## If the hypotesis are completed and not inconsistent, then let's go to the oracle
            else:
                oracle=True
        else:
            res=MyArmor.remove(ID)
            if res.armor_response.success==False:
                print("Error in removing\n")

        return 'explore'
            



class Try_Solution(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['explore', 'correct'])
        
    def execute(self, userdata):
        global armor_interface, pub_solution

        time.sleep(2) #Wait to make the smach simulation easier to follow on the visor
             
        id_consistent=response_complete.armor_response.queried_objects[0]
        id_consistent=id_consistent[40:]
        id_consistent=id_consistent[:-1]

        print("\n\nID consistent: " + id_consistent + "\n")

        res_what=MyArmor.ask_item('what', id_consistent)
        what=res_what.armor_response.queried_objects[0]
        what=what[40:]
        what=what[:-1]

        res_where=MyArmor.ask_item('where', id_consistent)
        where=res_where.armor_response.queried_objects[0]
        where=where[40:]
        where=where[:-1]

        res_who=MyArmor.ask_item('who', id_consistent)
        who=res_who.armor_response.queried_objects[0]
        who=who[40:]
        who=who[:-1]

        print("\nThe killer is: " + who + " in the " + where + " with " + who + "\n")

        ## Send to oracle the solution
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
    global armor_interface, pub_move, pub_ask_hint, pub_solution
    rospy.init_node('state_machine')

    pub_move=ServiceProxy('/move_point', Move)
    pub_ask_hint=ServiceProxy('/hint_request', AskHint)
    pub_solution=ServiceProxy('/solution', Solution)

    path = dirname(realpath(__file__))
    path = path[:-7] + "cluedo_ontology.owl"
    
    response=MyArmor.load(path)

    if response.armor_response.success==True:
        print("\nOntology loaded succesfully")
    else:
        print("\nERROR: Ontology not loaded correctly")

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

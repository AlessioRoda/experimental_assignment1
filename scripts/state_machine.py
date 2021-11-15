#! /usr/bin/env python

from ctypes import sizeof
import sys
from os.path import dirname, realpath

sys_path= dirname(realpath(__file__))
sys.path.append(sys_path)

import roslib
import rospy
from rospy.impl.tcpros_service import Service, ServiceProxy, wait_for_service
import smach
import smach_ros
import time
import random
from datetime import datetime
from geometry_msgs.msg import Point
from experimental_assignment1.srv import Move, MoveRequest, AskHint, Solution
from classes.myArmor import MyArmor
from classes.place import Place
from armor_msgs.srv import *
from armor_msgs.msg import *


armor_interface=None
pub_move=None
pub_ask_hint=None
pub_solution=None
places=[]
actual_pos=Place(None, 0, 0)
pos_sent=Point()
reached=False
hint_received=False


people_ontology=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
places_ontology=['Ballroom', 'Biliard_Room', 'Oracle_Room', 'Conservatory', 'Dining_Room']
weapons_onotology=['Candlestick', 'Dagger','LeadPipe']


def init_scene():
    global places, actual_pos
    places.append(Place(places_ontology[0], 5, 5))
    places.append(Place(places_ontology[1], -5, -5))
    places.append(Place(places_ontology[2], 0, 0))
    places.append(Place(places_ontology[3], 5, -5))
    places.append(Place(places_ontology[4], -5, 5))

    actual_pos.x=0
    actual_pos.y=0
    actual_pos.name=places_ontology[2] # Starts in the oracle room

    ## Add people, weapons and places o the ontology
    for index_person, x in enumerate(people_ontology):
        res=MyArmor.add_item(x, 'PERSON')
        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        
        ## Disjoint all the elements
        if index_person!=0:
            count=index_person
            while count!=0:
                MyArmor.disjoint(x, people_ontology[count-1])
                count = count -1


    for index_place, x in enumerate(places_ontology):
        res=MyArmor.add_item(x, 'PLACE')
        if res.armor_response.success==False:
            print("\nError in loading PLACE in the ontology")

        ## Disjoint all the elements
        if index_place!=0:
            count=index_place
            while count!=0:
                MyArmor.disjoint(x, places_ontology[count-1])
                count = count -1

    for index_weapon, x in enumerate(weapons_onotology):
        res=MyArmor.add_item(x, 'WEAPON')
        if res.armor_response.success==False:
            print("\nError in loading WEAPON in the ontology")
        
        ## Disjoint all the elements
        if index_weapon!=0:
            count=index_weapon
            while count!=0:
                MyArmor.disjoint(x, weapons_onotology[count-1])
                count = count -1
    
    res=MyArmor.reason()

    
    



class Explore(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['move'])
        
    def execute(self, userdata):
        global pub_move, actual_pos, places

        random.seed(datetime.now())
        index=random.randint(0, len(places)-1)

        ## Can't be the same place in wich it is
        while places[index].name==actual_pos.name:
            index=random.randint(0, len(places)-1)
        destination=places[index]

        msg=MoveRequest()
        msg.x_start=actual_pos.x
        msg.y_start=actual_pos.y
        msg.x_end=destination.x
        msg.y_end=destination.y

        res=pub_move(msg)
        print("\nMoving from " + actual_pos.name + " to " + destination.name)

        rospy.wait_for_service('move_point')

        if(res.reached==True):
            actual_pos.x=destination.x
            actual_pos.y=destination.y
            actual_pos.name=destination.name
        else:
            print("\nPosition " + destination.name + " not reached")

        return 'move'


class Enter_Room(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['enter_room'])
        
    def execute(self, userdata):
        global pub_ask_hint, armor_interface

        #rospy.wait_for_service('hint_request')
        res=pub_ask_hint()
        print("RES: "+ str(res))
        
        ## Add hints to the ontology 
        count=0
        while(count!=len(res.what)):
            request=MyArmor.add_hipotesis('what', res.ID, res.what[count])
            #add_res=armor_interface(request)
            if request.armor_response.success==False:
                print("\nError, cannot add " + res.what[count])
            count = count +1

        count=0
        while(count!=len(res.where)):
            request=MyArmor.add_hipotesis('where', res.ID, res.where[count])
            #add_res=armor_interface(request)
            if request.armor_response.success==False:
                print("\nError, cannot add " + res.where[count])
            count = count +1

        count=0
        while(count!=len(res.who)):
            request=MyArmor.add_hipotesis('who', res.ID, res.who[count])
            #add_res=armor_interface(request)
            if request.armor_response.success==False:
                print("\nError, cannot add " + res.who[count])
            count = count +1

        ## Reason
        reason=MyArmor.reason()
        MyArmor.save()
        #res_reason=armor_interface(reason)
        if reason.armor_response.success==False:
                print("\nError, cannot perform reasoning")

        return 0


class Try_Solution(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['generate_solution'])
        
    def execute(self):
        global armor_interface, pub_solution

        id_inconsistent=None

        ## First asks for all consistent queries
        response=MyArmor.ask_complete()
        if response.armor_response.success==False:
            print("\nError in asking query")
            return 1
        
        else:
            response_inconsistent=MyArmor.ask_inconsistent()
        
            if sizeof(response_inconsistent.armor_response.queried_objects)!=0:
                str_inconsitent=response_inconsistent.armor_response.queried_objects[0]
                str_inconsitent=str_inconsitent[39:]
                id_inconsistent=str_inconsitent[:-1]
                res=MyArmor.remove(id_inconsistent)
                
                if res.armor_response.success==False:
                    print("Error in removing\n")

                return 1
            
            else:
                id_consistent=response.armor_response.queried_objects[0]
                id_consistent=id_consistent[39:]
                id_consistent=id_consistent[:-1]

                print("\n\nID consistent: " + id_consistent)

                res_what=MyArmor.ask_item('what', id_consistent)
                what=res_what.armor_response.queried_objects[0]
                what=what[39:]
                what=what[:-1]

                res_where=MyArmor.ask_item('where', id_consistent)
                where=res_where.armor_response.queried_objects[0]
                where=where[39:]
                where=where[:-1]

                res_who=MyArmor.ask_item('who', id_consistent)
                who=res_who.armor_response.queried_objects[0]
                who=who[39:]
                who=who[:-1]

                print("\nTrying solution : " + what + ", " + where + ", " + who)

                ## Send to oracle the solution
                res=pub_solution(what, where, who, id_consistent)
                rospy.wait_for_service('solution')
                if res.correct==True:
                    print("\nSolution is correct, the game finished!!!!!!")
                elif res.correct==False:
                    print("\nIt's not the correct solution")
                    res=MyArmor.remove(id_consistent)

        return 0
    



def main():
    global armor_interface, pub_move, pub_ask_hint, pub_solution
    rospy.init_node('state_machine')

    pub_move=ServiceProxy('/move_point', Move)
    pub_ask_hint=ServiceProxy('/hint_request', AskHint)
   # armor_interface=ServiceProxy('/armor_interface_srv', ArmorDirective)
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
        smach.StateMachine.add('EXPLORE', Explore(), 
                               transitions={'move':'ENTER_ROOM' })

        smach.StateMachine.add('ENTER_ROOM', Enter_Room(), 
                               transitions={'enter_room':'SOLUTION',
                                            'enter_room':'EXPLORE'})
        
        smach.StateMachine.add('SOLUTION', Try_Solution(),
                                transitions={'generate_solution':'EXPLORE'})


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

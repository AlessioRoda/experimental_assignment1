#!/usr/bin/env python

from posixpath import dirname, realpath
import rospy
from classes.myArmor import MyArmor
from erl2.srv import Update, UpdateResponse, Consistency, ConsistencyResponse, Oracle
from erl2.msg import ErlOracle
from armor_msgs.srv import *
from armor_msgs.msg import *

people_ontology=["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
''' Define all the people of the scene
'''
places_ontology=["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
''' Define all the places of the scene
'''
weapons_ontology=["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]


ID=[]
key=[]
value=[]
ask_solution=None
update_service=None
consistency_service=None

solution=None

def init_scene():
    '''
    Function to initialize the scene, it defines three rooms as Place objects, then defines the starting position in the Oracle_Room (x=0, y=0)
    then it adds all the information about the scene to the armor ontology, by loading all the people, places and weapons.
    When a new element is loaded in the ontology it must be disjointed respect to the others of the same class, in order to notify the ontology
    that they are different. Finally it preforms 'REASON' command to update the ontology.
    
    '''

    global actual_pos, oracle_room

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




def receive_hint(hint):
    global ID, key, value
    ID.append(hint.ID)
    key.append(hint.key)
    value.append(hint.value)



def update_ontology():
    global ID, key, value
    i=0
    while i<ID.count():
        MyArmor.add_hypothesis(key[i], ID[i], value[i])
        i+=1


    #Clear the arrays after having sent the hypotesis
    for i in ID:
        ID.remove(i)
        key.remove(i)
        value.remove(i)
    
    MyArmor.reason()
    res=UpdateResponse()
    res.updated=True
    update_service(res)
        
def try_solution():
    global ask_solution, solution

    print("Trying solution: ")
    response_complete=[]
    response_complete.append(MyArmor.ask_complete())
    if response_complete.armor_response.success==False:
        print("\nError in asking query")
    
    for j in response_complete:
        if len(response_complete[j].armor_response.queried_objects)!=0:
            response_inconsistent=[]
            response_inconsistent.append(MyArmor.ask_inconsistent())
        
            # Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:

                for i in response_inconsistent:
                    str_inconsistent=response_inconsistent[i].armor_response.queried_objects[0]
                    str_inconsistent=str_inconsistent[40:]
                    id_inconsistent=str_inconsistent[:-1]
                    print("ID_inconsistent "+str(id_inconsistent) +"\n")
                    res=MyArmor.remove(id_inconsistent)
                    response_complete.remove(id_inconsistent)

                    if res.armor_response.success==False:
                        print("Error in removing\n")

    else:
        solution=ask_solution()
        for res in response_complete:
            id_consistent=res.armor_response.queried_objects[0]
            if solution == id_consistent:
                print("\nSolution found!: "+ solution)
    res=ConsistencyResponse()
    res.ok=True
    consistency_service(res)


def main():
    '''
    Main function 
    '''
    global ask_solution, update_service, consistency_service
    rospy.init_node('main') 
    init_scene()
    update_service= rospy.Service('/update_request', Update, update_ontology)
    consistency_service=rospy.Service('/conosistency_request', Consistency, try_solution)
    ask_solution=rospy.ServiceProxy('/oracle_solution', Oracle)
    rospy.Service('/oracle_hint', ErlOracle, receive_hint)

    rospy.spin()



if __name__ == '__main__':
    main()
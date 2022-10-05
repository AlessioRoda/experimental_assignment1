#! /usr/bin/env python3

'''
.. module:: ontology_interface
   :platform: Unix
   :synopsis: Node implementing an interface for communicating with the ARMOR ontology

.. moduleauthor:: Alessio Roda alessioroda98@gmail.com

This node allows to modify the ontology and perform queries
	                                                                                                                            
Services:
    /check_consistency service to ask for complete and consistent hypothesis in the ontology and
                        to use them as possible solutions to the cluedo game

    /update_request service to update the ontology by adding the hints to the ontology and by performing the "REASON" command
 
 It's the node that allows to initialize the ontology, add items and asking queries; it receives the hints from the /oracle_hint service,
 then add them in a list. It also updates the ontology when a request is received: it loads all the new hints in the ontology and performs the REASON operation; 
 then it searches for complete and consistent hypotheses when /check_consistency is called, removes the inconsistent ones from the ontology and  
 compares the complete and consistent ones with the solution ID provided by the /oracle_solution service. If the solution is correct it 
 also queries the person, the place and the weapon of the solution and prints them, then sends the ResetRequest with the "finish" parameter equal to True;
 otherwise it removes the incorrect IDs and returns the ResetRequest with the "finish" parameter equal to False 

'''

import rospy
from classes.myArmor import MyArmor
from erl2.srv import Update, UpdateResponse, Consistency, ConsistencyResponse, Oracle, OracleRequest, Reset, ResetRequest
from erl2.msg import ErlOracle
from armor_msgs.srv import *
from armor_msgs.msg import *

people_ontology=["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
''' list[str]: Define all the people of the scene

'''
places_ontology=["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
''' list[str]: Define all the places of the scene

'''
weapons_ontology=["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
''' list[str]: Define all the weapons of the scene

'''

ID=[]
''' list: Initialize ID list 

'''
key=[]
''' list: Initialize key list (each element is referred to the corresponding element in the ID list)

'''
value=[]
''' list: Initialize value list (each element is referred to the corresponding element in the ID list)

'''
ask_solution=None
''' Initialize /oracle_solution service client

'''
update_service=None
''' Initialize /update_request service server

'''
consistency_service=None
''' Initialize /consistency_request service server

'''
reset_client=None
''' Initialize /reset_planning service client

'''

def init_scene():
    '''
    Function to initialize the scene, it adds all the information about the scene to the armor ontology by loading all the people, places and weapons.
    When a new element is loaded in the ontology it must be disjointed respect to the others of the same class, in order to notify the ontology
    that they are different. Finally it preforms 'REASON' command to update the ontology.
    
    '''

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
    '''
    Callback to get the hints from the simulation node; each element of the hint is appended in a list 

    Args: 
        hint(Hint): is the hint received from the simulation node

    '''
    global ID, key, value
    
    print("Received hint: "+str(hint))
    ID.append(str(hint.ID))
    key.append(hint.key)
    value.append(hint.value)


def update_ontology(msg):
    '''
    Callback to update the ontology when the /update_request is called; it adds all the hypothesis in the ontology,
    then updates it by using the REASON command.

    Args: 
        msg(UpdateRequest): not utilized
    Returns:
        res(UpdateResponse): not utilized

    '''

    global ID, key, value
    i=0
    while i<len(ID):
        MyArmor.add_hypothesis(key[i], ID[i], value[i])
        i+=1
    #Clear the lists after having sent the hypothesis
    ID.clear()
    key.clear()
    value.clear()
    
    MyArmor.reason()
    res=UpdateResponse()
    res.updated=True
    return res

        
def try_solution(msg):
    '''
    Callback to find complete and consistent hypothesis when the /check_consistency service is called, it also removes
    inconsistent IDs. It also uses the complete and consistent hypothesis as possible solutions, by comparing their IDs with the 
    solution ID provided by the /oracle_solution service; in case one of them corresponds to the solution ID, it 
    queries the place, the person and the weapon of the solution and sets the ResetRequest to True, 
    otherwise the ID is removed by the ontology and the ResetRequest is set to False  

    Args: 
        msg(ConsistencyRequest): not utilized
    Returns:
        res(ConsistencyResponse): not utilized

    '''
    global ask_solution

    print("Check consistent and complete IDs")
    response_complete=MyArmor.ask_complete()
    list_complete=[]
    print("Response complete: "+str(response_complete))
    if response_complete.armor_response.success==False:
        print("\nError in asking query")
    else:
        if len(response_complete.armor_response.queried_objects)!=0:

            for complete in response_complete.armor_response.queried_objects:
                complete=complete[40:]
                id_complete=complete[:-1]
                print("ID_complete "+str(id_complete) +"\n")
                list_complete.append(id_complete)

            response_inconsistent=MyArmor.ask_inconsistent()
            print("Inconsistent responses: "+str(response_inconsistent))
            print("Complete responses: "+str(list_complete))
        
            # Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:

                for str_inconsistent in response_inconsistent.armor_response.queried_objects:
                    str_inconsistent=str_inconsistent[40:]
                    id_inconsistent=str_inconsistent[:-1]
                    print("ID_inconsistent "+str(id_inconsistent) +"\n")
                    res=MyArmor.remove(id_inconsistent)
                    list_complete.remove(id_inconsistent)

                    if res.armor_response.success==False:
                        print("Error in removing inconsistent ID\n")

    req=ResetRequest()

    if len(list_complete)>0:
        print("Trying solution: ")
        print("Complete and consistent: "+str(list_complete))
        solution=ask_solution(OracleRequest())
        for id in list_complete:
            if str(solution.ID) == id:
                print("\nSolution found!: "+ str(solution.ID))

                #Query the corresponding person, weapon and place of the solution
                person=None
                weapon=None
                place=None

                #Ask weapon
                armor_res=MyArmor.ask_item(ID=id, type="what")
                print(str(armor_res))
                if len(armor_res.armor_response.queried_objects)==0:
                    print("Error in asking query")
                else:
                    for str_what in armor_res.armor_response.queried_objects:
                                str_what=str_what[40:]
                                weapon=str_what[:-1]
                #Ask person
                armor_res=MyArmor.ask_item(ID=id, type="who")
                print(str(armor_res))
                if len(armor_res.armor_response.queried_objects)==0:
                    print("Error in asking query")
                else:
                    for str_who in armor_res.armor_response.queried_objects:
                                str_who=str_who[40:]
                                person=str_who[:-1]
                #Ask place
                armor_res=MyArmor.ask_item(ID=id, type="where")
                print(str(armor_res))
                if len(armor_res.armor_response.queried_objects)==0:
                    print("Error in asking query")
                else:
                    for str_where in armor_res.armor_response.queried_objects:
                                str_where=str_where[40:]
                                place=str_where[:-1]

                print("Solution: \nPerson: "+person+"\nPlace: "+place+"\nWeapon: "+weapon)
                req.finished=True
                break
                
            else:
                print(id+" is not the solution")
                MyArmor.remove(id)
                req.finished=False

    res=reset_client(req)

    return ConsistencyResponse()


def main():
    '''
    Main function of the ontology_interface node; it declares the node itself, initializes all the services and calls the init_scene
    function to initialize the ontology

    '''
    global ask_solution, update_service, consistency_service, reset_client
    rospy.init_node('ontology_interface')
   
    consistency_service=rospy.Service('/consistency_request', Consistency, try_solution)
    update_service= rospy.Service('/update_request', Update, update_ontology)
    ask_solution=rospy.ServiceProxy('/oracle_solution', Oracle)
    rospy.Subscriber('/oracle_hint', ErlOracle, receive_hint)
    reset_client=rospy.ServiceProxy("/reset_planning", Reset)
    
    path=rospy.get_param("~ontology")
    print("PATH: "+str(path))
    
    # Loads the ontology
    response=MyArmor.load(path)
    if response.armor_response.success==True:
        print("\nOntology loaded successfully")
    else:
        print("\nERROR: Ontology not loaded correctly")
    
    init_scene()
    rospy.spin()



if __name__ == '__main__':
    main()
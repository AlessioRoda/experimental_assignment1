#! /usr/bin/env python3

'''
.. module:: ontology_interface
   :platform: Unix
   :synopsis: Node implementing an interface for comunicating with the ARMOR ontology
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com
This node allows to modify the ontology and perform queries
Service:
 	/ontology_interface/check_consistency
    /ontology_interface/update_request

It's the node that allows to initialize the ontology, add items and asking queries; it receives the hints from the state_machine node,
then add them in a list. It also updates the ontology when a request is received: it loads all the new hints in the ontology and performs the REASON operation; 
finally it also searches for complete and consistent hypothesys when /ontology_interface/check_consistency is called
'''

import rospy
from classes.myArmor import MyArmor
from erl2.srv import Update, UpdateResponse, Consistent, ConsistentResponse, Hint, HintResponse, Solution, SolutionResponse
from armor_msgs.srv import *
from armor_msgs.msg import *

people_ontology=["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
''' Define all the people of the scene
'''
places_ontology=["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
''' Define all the places of the scene
'''
weapons_ontology=["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
''' Define all the weapons of the scene
'''


ID=[]
''' Initialize ID list 
'''
key=[]
''' Initialize key list (each element is referred to the corresponding element in the ID list)
'''
value=[]
''' Initialize value list (each element is referred to the corresponding element in the ID list)
'''
update_service=None
''' Initialize /ontology_interface/update_request service server
'''
consistency_service=None
''' Initialize value /ontology_interface/check_consistency service server
'''


def init_scene():
    '''
    Function to initialize the scene, it defines three rooms as Place objects, then defines the starting position in the Oracle_Room (x=0, y=0)
    then it adds all the information about the scene to the armor ontology, by loading all the people, places and weapons.
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
        Callback to get the hints from the state_machine node; eache element of the hint is appended in a list 

        Args: 
            hint(Hint): is the hint received from the state_machine node
        Returns:
            res(HintResponse): not utilized
    '''
    global ID, key, value
    print("Received hint: "+str(hint))
    ID.append(str(hint.oracle_hint.ID))
    key.append(hint.oracle_hint.key)
    value.append(hint.oracle_hint.value)
    return HintResponse()



def update_ontology(msg):
    '''
        Callback to upadte the ontology when the /ontology_interface/update_request is called; it adds all the hypothesis in the ontology,
        then updates it by using the REASON command.

        Args: 
            msg(Update): not utilized
        Returns:
            res(UpdateResponse): not utilized
    '''
    global ID, key, value
    i=0
    while i<len(ID):
        MyArmor.add_hypothesis(key[i], ID[i], value[i])
        i+=1
    #Clear the arrays after having sent the hypotesis
    ID.clear()
    key.clear()
    value.clear()
    
    MyArmor.reason()
    res=UpdateResponse()
    res.updated=True
    return res
        
def find_consistent(msg):
    '''
        Callback to to find coplete and consistent hypothesis when the /ontology_interface/check_consistency is called

        Args: 
            msg(Consistent): not utilized
        Returns:
            res(ConsistentResponse): not utilized
    '''

    print("Trying solution ")
    response_complete=MyArmor.ask_complete()
    list_complete=[]
    print(str(response_complete))
    if response_complete.armor_response.success==False:
        print("\nError in asking query")
    else:
        if len(response_complete.armor_response.queried_objects)!=0:

            for complete in response_complete.armor_response.queried_objects:
                complete=complete[40:]
                id_complete=complete[:-1]
                print("ID_inconsistent "+str(id_complete) +"\n")
                list_complete.append(id_complete)

            response_inconsistent=MyArmor.ask_inconsistent()
            print(str(response_inconsistent))
            print("Risposte compelte: "+str(list_complete))
        
            # Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:

                for str_inconsistent in response_inconsistent.armor_response.queried_objects:
                    str_inconsistent=str_inconsistent[40:]
                    id_inconsistent=str_inconsistent[:-1]
                    print("ID_inconsistent "+str(id_inconsistent) +"\n")
                    res=MyArmor.remove(id_inconsistent)
                    list_complete.remove(id_inconsistent)

                    if res.armor_response.success==False:
                        print("Error in removing\n")

        print("Conistent hypotheses: "+str(list_complete))
        res=ConsistentResponse()
        if len(list_complete)>0:
            res.consistent=list_complete
        return res

def ask_solution(msg):
    res=SolutionResponse()

    #Ask weapon
    armor_res=MyArmor.ask_item(ID=msg.ID, type="what")
    print(str(armor_res))
    if len(armor_res.armor_response.queried_objects)==0:
        print("Error in asking query")
    else:
        for str_what in armor_res.armor_response.queried_objects:
                    str_what=str_what[40:]
                    res.weapon=str_what[:-1]
     #Ask person
    armor_res=MyArmor.ask_item(ID=msg.ID, type="who")
    print(str(armor_res))
    if len(armor_res.armor_response.queried_objects)==0:
        print("Error in asking query")
    else:
        for str_who in armor_res.armor_response.queried_objects:
                    str_who=str_who[40:]
                    res.person=str_who[:-1]
     #Ask place
    armor_res=MyArmor.ask_item(ID=msg.ID, type="where")
    print(str(armor_res))
    if len(armor_res.armor_response.queried_objects)==0:
        print("Error in asking query")
    else:
        for str_where in armor_res.armor_response.queried_objects:
                    str_where=str_where[40:]
                    res.place=str_where[:-1]
    return res


def main():
    '''
    Main function of the onotlogy_interface node; it declares the node itself, initializes all the services and calls the init_scene
    function to initialize the ontology
    '''
    global update_service, consistency_service
    rospy.init_node('ontology_interface')
   
    consistency_service=rospy.Service('/ontology_interface/check_consistency', Consistent, find_consistent)
    update_service= rospy.Service('/ontology_interface/update_request', Update, update_ontology)
    rospy.Service('/ontology_interface/add_hint', Hint, receive_hint)
    rospy.Service('/ontology_interface/ask_solution', Solution, ask_solution)

    rospy.wait_for_service("/armor_interface_srv")
    rospy.wait_for_service("/armor_interface_serialized_srv")

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
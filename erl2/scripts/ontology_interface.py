#! /usr/bin/env python3

from posixpath import dirname, realpath
import rospy
from classes.myArmor import MyArmor
#from erl2.srv import Update, UpdateResponse, TrySolution, TrySolutionResponse, Oracle
from erl2.srv import Update, UpdateResponse, Consistent, ConsistentResponse, Hint, HintResponse
from erl2.msg import ErlOracle
from armor_msgs.srv import *
from armor_msgs.msg import *

people_ontology=["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
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
    print("Hint ricevuto: "+str(hint))
    ID.append(str(hint.oracle_hint.ID))
    key.append(hint.oracle_hint.key)
    value.append(hint.oracle_hint.value)
    return HintResponse()



def update_ontology(msg):
    global ID, key, value
    i=0
    print("Dentro Update Ontology")
    while i<len(ID):
        MyArmor.add_hypothesis(key[i], ID[i], value[i])
        i+=1
    print("Update 1")
    #Clear the arrays after having sent the hypotesis
    ID.clear()
    key.clear()
    value.clear()
    print("Update 2")
    
    MyArmor.reason()
    print("Update 3")
    res=UpdateResponse()
    res.updated=True
    return res
        
def find_cosnistent(msg):
    global ask_solution, solution

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



def main():
    '''
    Main function 
    '''
    global ask_solution, update_service, consistency_service
    rospy.init_node('ontology_interface')
   
    consistency_service=rospy.Service('/ontology_interface/check_consistency', Consistent, find_cosnistent)
    update_service= rospy.Service('/ontology_interface/update_request', Update, update_ontology)
    rospy.Service('/ontology_interface/add_hint', Hint, receive_hint)

    rospy.wait_for_service("/armor_interface_srv")
    rospy.wait_for_service("/armor_interface_serialized_srv")

    # path = dirname(realpath(__file__))
    # path = path[:-7] + "cluedo_ontology.owl"
    # print("PATH: "+str(path))

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
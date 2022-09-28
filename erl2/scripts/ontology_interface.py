#! /usr/bin/env python3

from posixpath import dirname, realpath
import rospy
from classes.myArmor import MyArmor
#from erl2.srv import Update, UpdateResponse, Consistency, ConsistencyResponse, Oracle
from erl2.srv import Update, UpdateResponse, Consistency, ConsistencyResponse, Oracle, OracleRequest
from erl2.msg import ErlOracle, ResetAction, ResetGoal
from armor_msgs.srv import *
from armor_msgs.msg import *
import actionlib

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
reset_action_client=None

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
    
    print("Received hint: "+str(hint))
    ID.append(str(hint.ID))
    key.append(hint.key)
    value.append(hint.value)


def update_ontology(msg):
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
    global ask_solution, solution

    print("Check consistent and complete IDs")
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

    if len(list_complete>0):
        print("Trying solution: ")
        solution=ask_solution(OracleRequest())
        for id in list_complete:
            if str(solution.ID) == id:
                print("\nSolution found!: "+ str(solution.ID))

                return ConsistencyResponse()

    print("Reload the plan")
    goal=ResetGoal()
    reset_action_client.wait_for_server()
    reset_action_client.send_goal(goal)
    res=reset_action_client.wait_for_result()
    if res.succeed==False:
        print("Error in replanning")

    return ConsistencyResponse()


def main():
    '''
    Main function 
    '''
    global ask_solution, update_service, consistency_service, reset_action_client
    rospy.init_node('ontology_interface')
   
    consistency_service=rospy.Service('/consistency_request', Consistency, try_solution)
    update_service= rospy.Service('/update_request', Update, update_ontology)
    ask_solution=rospy.ServiceProxy('/oracle_solution', Oracle)
    rospy.Subscriber('/oracle_hint', ErlOracle, receive_hint)
    reset_action_client=actionlib.SimpleActionClient("reset_planning_action", ResetAction)
    
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
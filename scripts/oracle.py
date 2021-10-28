#! /usr/bin/env python

import roslib
import rospy
import random
from datetime import datetime
from experimental_assignment1.srv import Solution, AskHint
from armor_py_api.scripts.armor_api.armor_client import ArmorClient

from os.path import dirname, realpath

#people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock', 'Mrs.White', 'Prof.Plum', 'Rev.Green']
#places=['Ballroom', 'Biliard_Room', 'Conservatory', 'Dining_Room', 'Hall', 'Kitchen', 'Library', 'Lounge','Study']
#weapons=['Candlestick', 'Dagger','LeadPipe', 'Revolver', 'Rope', 'Spanner']

people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
places=['Ballroom', 'Biliard_Room', 'Conservatory', 'Dining_Room']
weapons=['Candlestick', 'Dagger','LeadPipe']
solution=[]

armor_interface= None
solution_service=None


def init_scene():
    global people, places, weapons, solution


    random.seed(datetime.now())
    solution.append(random.randint(0, len(weapons)-1))
    solution.append(random.randint(0, len(places)-1))
    solution.append(random.randint(0, len(people)-1))



def receive_solution(sol):
    global solution_service
    
    if(solution[0]==sol.what and solution[1]==sol.where and solution[2]==sol.who):
        print("\nSolution is correct!!")
        solution_service(True)

    else:
        solution_service(False)

def generate_hint(hint):
    global armor_interface, people, weapons, places

    if hint.req==True:
        random.seed(datetime.now())
        num_hints= random.randint(1, 3)
        
        i=0
        ## The number of elements per hint is casual
        while i!=num_hints:

            ## The type of element in the hint is casual
            hint_type=random.randint(0,2)
            if hint_type==0:
                index_people= random.randint(0, len(people)-1) ## The kind of people is casual
                armor_interface.manipulation.add_ind_to_class(people[index_people], 'PERSON') 
            elif hint_type==1:
                index_places= random.randint(0, len(places)-1)
                armor_interface.manipulation.add_ind_to_class(places[index_places], 'PLACE') 
            else:
                index_weapons= random.randint(0, len(weapons)-1)
                armor_interface.manipulation.add_ind_to_class(weapons[index_weapons], 'WEAPON')

            i=i+1
        armor_interface.utils.apply_buffered_changes()
        armor_interface.utils.sync_buffered_reasoner()



def main():
    global people, places, weapons, armor_interface, solution_service
    rospy.init_node('oracle')  

    solution_service = rospy.Service('/solution', Solution, receive_solution)
    solution_service = rospy.Service('/hint_request', AskHint, generate_hint)

    path = dirname(realpath(__file__))+"cluedo_ontology.owl"
    armor_interface = ArmorClient("client", "reference")
    armor_interface.utils.load_ref_from_file(path, "http://www.emarolab.it/cluedo-ontology",
                                True, "PELLET", True)
    armor_interface.utils.mount_on_ref()
    

    init_scene() ## generate a solution
    rospy.spin()



if __name__ == '__main__':
    main()

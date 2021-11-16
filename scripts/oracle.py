#! /usr/bin/env python

import rospy
import random
from datetime import datetime
from experimental_assignment1.srv import Solution, SolutionResponse, AskHint, AskHintResponse

#people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock', 'Mrs.White', 'Prof.Plum', 'Rev.Green']
#places=['Ballroom', 'Biliard_Room', 'Conservatory', 'Dining_Room', 'Hall', 'Kitchen', 'Library', 'Lounge','Study']
#weapons=['Candlestick', 'Dagger','LeadPipe', 'Revolver', 'Rope', 'Spanner']

people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
places=['Ballroom', 'Biliard_Room', 'Conservatory']
weapons=['Candlestick', 'Dagger','LeadPipe']
solution=[]

armor_interface= None
solution_service=None
hint_service=None
num_ID_hint=0

def init_scene():
    global people, places, weapons, solution

    index=[]

    random.seed(datetime.now())
    index.append(random.randint(0, len(weapons)-1))
    index.append(random.randint(0, len(places)-1))
    index.append(random.randint(0, len(people)-1))

    solution.append(weapons[index[0]])
    solution.append(places[index[1]])
    solution.append(people[index[2]])

    print("Solution: "+str(solution))



def receive_solution(sol):
    global solution_service

    print("\nSolution received: " + sol.what + ", " + sol.where + ", " + sol.who)
    
    res=SolutionResponse()
    if(solution[0]==sol.what and solution[1]==sol.where and solution[2]==sol.who):
        print("\nSolution is correct!!")
        res.correct=True

    else:
        res.correct=False
        print("\nSolution is not correct, try again")

    return res

def generate_hint():
    global armor_interface, people, weapons, places, solution_service, num_ID_hint

    random.seed(datetime.now())
    num_hints= random.randint(1, 4)
    
    i=0
    what=[]
    where=[]
    who=[]
    ## The number of elements per hint is casual
    while i!=num_hints:

        ## The type of element in the hint is casual
        hint_type=random.randint(0,3)
        if hint_type==0:
            index_people= random.randint(0, len(people)-1) ## The kind of people is casual 
            who.append(people[index_people])

        elif hint_type==1:
            index_places= random.randint(0, len(places)-1)
            where.append(places[index_places])

        else:
            index_weapons= random.randint(0, len(weapons)-1)
            what.append(weapons[index_weapons])

        i=i+1
    
    num_ID_hint+=1
    print("\nHint" + str(num_ID_hint) +": " + str(what) +", "+ str(where) + ", " + str(who))

    return [what, where, who]


def hint_req(req):
    global num_ID_hint

    hint=generate_hint()

    res=AskHintResponse()
    res.what=hint[0]
    res.where=hint[1]
    res.who=hint[2]
    res.ID="HP" + str(num_ID_hint)
    return res




def main():
    global people, places, weapons, armor_interface, solution_service, hint_service
    rospy.init_node('oracle')  

    solution_service = rospy.Service('/solution', Solution, receive_solution)
    hint_service = rospy.Service('/hint_request', AskHint, hint_req)


    init_scene() ## generate a solution
    rospy.spin()



if __name__ == '__main__':
    main()

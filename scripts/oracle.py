#! /usr/bin/env python

"""
.. module:: oracle
   :platform: Unix
   :synopsis: Node implementing an algorithm to simulate the Cluedo oracle
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com
This node simulate the oracle that knows the solution and that generates hints

Service:
 	/solution 
    /hint_request

It's performed as the node that permits to simulate the oracle of the Cluedo game. It generates a random solution by sorting a person, a place 
and a weapon between the ones provided by the scene, then it mainly performs two operations:

    -Generate hints when the state_machine node sends the request, the hints are a random number of elements between the elements of the scene
        that are stored in the AskHint custom message and sent to the state_machine node.

    -Receive the solution provided from the state_machine node with the Solution custom message and checks if it's correct.
        In the case it is correct it returns True and the game ends, otherwise it returns False and game continue.

"""

import rospy
import random
from datetime import datetime
from experimental_assignment1.srv import Solution, SolutionResponse, AskHint, AskHintResponse

people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
''' Define all the people of the scene

'''
places=['Ballroom', 'Billiard_Room', 'Conservatory']
''' Define all the places of the scene

'''
weapons=['Candlestick', 'Dagger','LeadPipe']
''' Define all the weapons of the scene

'''
solution=[]
''' Define the array in which will be stored the solution of the game

'''
num_ID_hint=0
''' To count the number of hints provided by the oracle node

'''

def init_scene():
    """
    Function to initialize the scene, it defines the solution to the Cluedo game by sorting a weapon, a place and a person between the 
    ones in the current scene and saves them in the solution variable.

    """

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
    """
    Callback to execute when the oracle receives a possible solution from the state_machine node.
    It checks if the person, the place and the weapon provided from the state_machine corresponds with the ones of the solution and,
    in this case, returns True to confirm, otherwise returns False.

        Args: 
            sol(Solution): is the solution received from the state_machine node
        Returns:
            res(SolutionResponse): the response if the provided solution is correct or not

    """

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
    """
    Function to generate a hint, the number of the element for each hint is random and even the type of hint is casual:
    there's the possibility to generate hint without one of PERSON, PLACE and WEAPON or even cases in which for each one of them
    there are more than one. 
    For example a i's possible to generate
    
    sol=[[], [], ['Mrs.Peacock']]

    or

    sol=[['Dagger'], ['Ballroom', 'Conservatory'], ['Mrs.Peacock']]

        Returns:
            [what, where, who]: the elements of the generated hint
        
    """

    global people, weapons, places, num_ID_hint

    random.seed(datetime.now())

    #There's at least one hint from PERSON, PLACE and WEAPON
    num_hints= random.randint(1, 4)
    
    i=0
    what=[]
    where=[]
    who=[]
    ## The number of elements per hint is random
    while i!=num_hints:

        ## The type of element in the hint is random
        hint_type=random.randint(0,3)
        if hint_type==0:
            index_people= random.randint(0, len(people)-1) 
            who.append(people[index_people])

        elif hint_type==1:
            index_places= random.randint(0, len(places)-1)
            where.append(places[index_places])

        else:
            index_weapons= random.randint(0, len(weapons)-1)
            what.append(weapons[index_weapons])

        i=i+1
    
    # Update the number of hints generated 
    num_ID_hint+=1
    print("\nHint" + str(num_ID_hint) +": " + str(what) +", "+ str(where) + ", " + str(who))

    return [what, where, who]


def hint_req(req):
    """
    Callback to execute when the oracle receives a hint request from the state_machine node, it calls the function generate_hint(),
    then returns the hint to the state_machine node.

        Args: 
            req(AskHint): the hint generated
        Returns:
            res(AskHintResponse): the message with the hint for the state_machine node
        
    """
    global num_ID_hint

    hint=generate_hint()

    res=AskHintResponse()
    res.what=hint[0]
    res.where=hint[1]
    res.who=hint[2]
    res.ID="HP" + str(num_ID_hint)
    return res




def main():
    """
    The main of the oracle node, it initializes the node itself, then creates a service on topic /solution and /hint_request.
    Finally it asks to initialize the scene.
        
    """

    global people, places, weapons

    # Initialize the node
    rospy.init_node('oracle')  

    rospy.Service('/solution', Solution, receive_solution)
    rospy.Service('/hint_request', AskHint, hint_req)

    # Generate a solution
    init_scene() 
    rospy.spin()



if __name__ == '__main__':
    main()

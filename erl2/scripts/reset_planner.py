#! /usr/bin/env python3

import rospy
from erl2.msg import ResetAction, ResetActionResult
import actionlib

from classes.Planner import Planner

reset_action_server=None

def reset():
    print("Reset the planning")

    #1 Clear all
    res=Planner.clear_planner()
    print("Clear planner: "+str(res))

    #2 Add all the insances
    res=Planner.add_instance(name="sherlock", instance_type="robot")
    print("Add instance: "+str(res))
    Planner.add_instance(name="w1", instance_type="waypoint")
    Planner.add_instance(name="w2", instance_type="waypoint")
    Planner.add_instance(name="w3", instance_type="waypoint")
    Planner.add_instance(name="w4", instance_type="waypoint")
    Planner.add_instance(name="oracle_room", instance_type="waypoint")

    #3 Add the attributes
    Planner.add_attribute(attribute_name="not_can_check", key="", value="")
    Planner.add_attribute(attribute_name="ontology_updated", key="", value="")
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w1"])
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w2"])
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w3"])
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w4"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["w1"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value= ["w2"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["w3"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["w4"])

    key=[]
    value=[]
    key.append("waypoint")
    value.append("oracle_room")
    key.append("robot")
    value.append("sherlock")
    Planner.add_attribute(attribute_name="in_position", key=key, value=value)

    ## VEDI TU SE AGGIUNGERE ANCHE LE DISTANZE FATTE BENE

    #4 Add the goal
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w1"])
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w2"])
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w3"])
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w4"])
    Planner.add_goal(attribute_name="end_game", key=[], value=[])
    key_goal=[]
    value_goal=[]
    key_goal.append("waypoint")
    value_goal.append("oracle_room")
    key_goal.append("robot")
    value_goal.append("sherlock")
    Planner.add_goal(attribute_name="in_position", key=key_goal, value=value_goal)

    #5 Generate the problem
    Planner.generate_problem()
    #6 Generate the plan
    Planner.generate_plan()
    #7 Parse the plan
    Planner.parse_plan()
    #8 Dispatch the plan and get solution
    solution=Planner.dispatch_plan()

    result=ResetActionResult()

    if solution.goal_achieved:
        print("A solution was found")
        result.result.succeed=True
    else:
        print("No solution for the planner")
        result.result.succeed=False

    reset_action_server.set_succeeded(result.result)



def main():
    '''
    Main function 
    '''
    rospy.init_node('reset_planning')
   
    reset_action_server=actionlib.SimpleActionServer("reset_planning_action", ResetAction, reset, auto_start=False)
    reset_action_server.start()
    
    rospy.spin()



if __name__ == '__main__':
    main()
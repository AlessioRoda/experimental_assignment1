#! /usr/bin/env python3

from queue import Empty
import rospy
from erl2.msg import ResetAction, ResetActionResult
import actionlib
import actionlib_msgs
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService


plan_dispatch=None
knowledge_update=None
problem_generation=None
planning_server=None
parse_plan=None
clear_knowledge=None


def reset():
    print("")






def main():
    '''
    Main function 
    '''
    global plan_dispatch, knowledge_update, problem_generation, planning_server, parse_plan, clear_knowledge
    rospy.init_node('reset_planning')

    rospy.wait_for_service('rosplan_problem_interface/problem_generation_server')
    rospy.wait_for_service('rosplan_planner_interface/planning_server')
    rospy.wait_for_service('rosplan_parsing_interface/parse_plan')
    rospy.wait_for_service('rosplan_plan_dispatcher/dispatch_plan')
    rospy.wait_for_service('rosplan_knowledge_base/update')
    rospy.wait_for_service('rosplan_knowledge_base/clear')

    plan_dispatch=rospy.ServiceProxy('rosplan_plan_dispatcher/dispatch_plan', DispatchService) 
    knowledge_update=rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
    problem_generation=rospy.ServiceProxy('rosplan_problem_interface/problem_generation_server', Empty)
    planning_server=rospy.ServiceProxy('rosplan_planner_interface/planning_server', Empty)
    parse_plan=rospy.ServiceProxy('rosplan_parsing_interface/parse_plan', Empty)
    clear_knowledge=rospy.ServiceProxy('rosplan_knowledge_base/clear', Empty)

   
    reset_action_server=actionlib.SimpleActionServer("reset_planning_action", ResetAction, reset, auto_start=False)
    reset_action_server.start()

    rospy.spin()



if __name__ == '__main__':
    main()
/*************************************************************************************************************************//**
 * \file   init.cpp
 * 
 * \brief Node to move the arm in the correct initial position
 * 
 * \version 1.0
 * \author Alessio Roda
 * \date   October 2022
 * 
 * description:
 *    This node moves the robotic arm to the initial_pose defined with the moveit setup assistant
 * 
*****************************************************************************************************************************/

#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <unistd.h>


/** 
 * int main(argc, argv)
 * 
 * \brief Main function of the node
 * 
 * \param argc: the number of argument passed as parameters
 * 
 * \param argv: the vector of string containing each argument
 * 
 * \return 0 when the program ends
 * 
 * description:
 *    The main function, initializes the node, loads the arm model of the robot described with the moveit_setup_assistant, 
 *    gets the "arm" move group to move the robotic arm and moves it in the initial_pose position
 *    
 **/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "random_position_server");
    ros::NodeHandle n;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    moveit::planning_interface::MoveGroupInterface group("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    group.setNamedTarget("initial_pose");
    group.move();

}
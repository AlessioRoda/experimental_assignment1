/*************************************************************************************************************************//**
 * \file   movearm.cpp
 * 
 * \brief Node to perform the motion of the robotic arm
 * 
 * \version 1.0
 * \author Alessio Roda
 * \date   September 2022
 * 
 * description:
 *    It's a simple node in which the robot arm moves in five different positions already defined with the moveit_setup_assistant.
 * 
*****************************************************************************************************************************/

#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetPlanningScene.h>
#include<unistd.h>


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
 *    The main function, makes all the initializations as follows:
 * 
 *    - Initializess the node
 *    - Loads the arm model of the robot described with the moveit_setup_assistant
 *    - Get the "arm" move group to move the robotic arm
 *    - Moves the arm in an infinite loop in five different poses, such that the robot can explore the environment with the camera,
 *      It starts from the default position, than it rotates around 360 degrees and folds in order to see better the markers that are 
 *      on the floor
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

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(1)
    {
        group.setNamedTarget("search_aruco_01");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_02");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_03");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_04");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_03");
        group.move();
        sleep(0.1);

        //Return to initial position
        group.setNamedTarget("initial_pose");
        group.move();
        sleep(0.1);
    }
}
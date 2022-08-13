#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
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
#include <erl2/MoveAction.h>


ros::ServiceClient validity_service;
ros::ServiceClient planning_scene_service;
actionlib::SimpleActionServer<erl2::MoveAction> movearm_server;

void init_arm()
{
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

void movearm_request(const erl2::MoveGoalConstPtr &goal)
{
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    moveit::planning_interface::MoveGroupInterface group("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    group.setNamedTarget("explore_pose1");
    group.move();
    sleep(0.5);

    /// Ruota di 180° in una direzione e poi nell'altra per esplorare tutto ///

    sleep(0.1)
    group.setNamedTarget("explore_pose2");
    group.move();
    sleep(0.5);

    /// Ruota di 180° in una direzione e poi nell'altra per esplorare tutto ///

    //Return to initial position
    group.setNamedTarget("initial_pose");
    group.move();
    sleep(0.1);

    movearm_server.setSucceeded();

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "movearm", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    validity_service=nh.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");
    planning_scene_service=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    init_arm();
    movearm_server(nh, "movearm_action", boost::bind(&movearm_request, this, _1), false);
    movearm_server.start();
    ros::AsyncSpinner spinner(1); //Nuova aggiunta, vedi se funziona
    spinner.start();

    return 0;
    

}
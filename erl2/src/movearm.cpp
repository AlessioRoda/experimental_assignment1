#include "erl2/movearm.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <motion_plan/PlanningAction.h>
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


#define _USE_MATH_DEFINES


ros::ServiceClient validity_service;
ros::ServiceClient planning_scene_service;

namespace KCL_rosplan {

	MovearmActionInterface::MovearmActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool MovearmActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

        moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
        moveit::planning_interface::MoveGroupInterface group("arm");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        
       // Poition to reach
        geometry_msgs::Pose pose;

        if(msg->parameters[1].value == "w1"){
            pose.position.x=-3;
            pose.position.y=0;
        }
        else if (msg->parameters[1].value == "w2"){
		pose.position.x=3;
		pose.position.y=0.0;
		}
		else if (msg->parameters[1].value == "w3"){
		pose.position.x=0.0;
		pose.position.y=3;
		}
		else if (msg->parameters[1].value == "w4"){
		pose.position.x=0.0;
		pose.position.y=-3;
		}

        pose.position.z=1.25;
        pose.orientation.w = 0;
	    pose.orientation.x = 0;
	    pose.orientation.y = 0;
	    pose.orientation.z = 0;

        // std::cout << "Pose: " <<pose<< std::endl;
        // group.setStartStateToCurrentState();
        // group.setApproximateJointValueTarget(pose, "cluedo_link");
        
        // moveit_msgs::GetPlanningScene req_scene;
        // planning_scene_service.waitForExistence();
        
        // if (planning_scene_service.call(req_scene))
        // {
        //     moveit_msgs::PlanningScene scene = req_scene.response.scene;

        //     moveit_msgs::GetStateValidity req;
        //     req.request.robot_state=scene.robot_state; 
        //     req.request.group_name ="arm";
        //     if(validity_service.call(req))
        //     {
        //         bool valid=req.response.valid;
        //         if(valid==true)
        //         {
        //             std::cout << "Position to reach is valid " << std::endl;
        //         }
        //         else
        //         {
        //             std::cout << "Position to reach is NOT VALID " << std::endl;
        //         }
        //     }
        //     else
        //     {
        //         std::cout << "Error in getting the validity of the pose" << std::endl;
        //     }
        // }
        // else 
        // {
        //     std::cout << "Error in getting the scene " << std::endl;
        // }
        
        // std::vector<double> joint_values;
        // double timeout = 1;
        // bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

        // if (found_ik)
        // {
        //     kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        //     for (std::size_t i = 0; i < joint_names.size(); ++i)
        //     {
        //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        //     }
        // }
        // else
        // {
        //     ROS_INFO("Did not find IK solution");
        // }

        // group.setJointValueTarget(joint_values);
        // group.setStartStateToCurrentState();
        // group.setGoalOrientationTolerance(2*M_PI); //Don't care about orientation
        // group.setGoalPositionTolerance(0.1);

        // // Plan and execute
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // group.plan(my_plan); 
        // group.execute(my_plan);
        
        // std::cout << "Quote 1.75 reached" << std::endl;
        // sleep(2.0);

        // pose.position.z =  0.75;

        // group.setStartStateToCurrentState();
        // group.setApproximateJointValueTarget(pose, "cluedo_link");
        // found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

        // if (found_ik)
        // {
        //     kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        //     for (std::size_t i = 0; i < joint_names.size(); ++i)
        //     {
        //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        //     }
        // }
        // else
        // {
        //     ROS_INFO("Did not find IK solution");
        // }
        
        
        // group.setJointValueTarget(joint_values);
        // group.setStartStateToCurrentState();
        // group.setGoalOrientationTolerance(2*M_PI);
        // group.setGoalPositionTolerance(0.01);

        // // Plan and execute
        // group.plan(my_plan); 
        // group.execute(my_plan);

        // sleep(2.0);

        group.setNamedTarget("initial_pose");
	    group.move(); 
        sleep(1);

        group.setNamedTarget("reach_target");
	    group.move(); 
        sleep(1);

        group.setNamedTarget("initial_pose");
	    group.move(); 
        sleep(1);
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}

}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "movearm_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
        validity_service=nh.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");
        planning_scene_service=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
        ros::AsyncSpinner spinner(1); //Nuova aggiunta, vedi se funziona
        spinner.start();
		KCL_rosplan::MovearmActionInterface my_aci(nh);
		my_aci.runActionInterface();

		return 0;
	}

#include "erl2/movearm.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>




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

        geometry_msgs::Pose pose;


        float pose_x=0.5;
        float pose_y=0;
        float pose_z=1.25;
        float orientation_x=0;
        float orientation_y=0;
        float orientation_z=0;
        float orientation_w=0;

        pose.orientation.w = orientation_w;
        pose.orientation.x = orientation_x;
        pose.orientation.y = orientation_y;
        pose.orientation.z = orientation_z;
        pose.position.x =  pose_x;
        pose.position.y =  pose_y;
        pose.position.z =  pose_z;

        group.setStartStateToCurrentState();
        group.setApproximateJointValueTarget(pose, "cluedo_link");
        std::vector<double> joint_values;
        double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
        }
        else
        {
            ROS_INFO("Did not find IK solution");
        }

        group.setJointValueTarget(joint_values);
        group.setStartStateToCurrentState();
        group.setGoalOrientationTolerance(0.01);
        group.setGoalPositionTolerance(0.01);

        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        group.plan(my_plan); 
        group.execute(my_plan);
        
        std::cout << "Quote 1.75 reached -> IK + setJointValue" << std::endl;
        sleep(2.0);

        pose_z=0.75

        pose.orientation.w = orientation_w;
        pose.orientation.x = orientation_x;
        pose.orientation.y = orientation_y;
        pose.orientation.z = orientation_z;
        pose.position.x =  pose_x;
        pose.position.y =  pose_y;
        pose.position.z =  pose_z;

        group.setStartStateToCurrentState();
        group.setApproximateJointValueTarget(pose, "cluedo_link");
        found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
        }
        else
        {
            ROS_INFO("Did not find IK solution");
        }
        
        
        group.setJointValueTarget(joint_values);
        group.setStartStateToCurrentState();
        group.setGoalOrientationTolerance(0.01);
        group.setGoalPositionTolerance(0.01);

        // Plan and execute
        group.plan(my_plan); 
        group.execute(my_plan);

        sleep(2.0);

        group.setNamedTarget("zero");
	    group.move(); 
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}



}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "movearm_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		KCL_rosplan::MovearmActionInterface my_aci(nh);
		my_aci.runActionInterface();

		return 0;
	}

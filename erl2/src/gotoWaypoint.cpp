#include "erl2/go_to_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/MoveAction.h>
#include <erl2/MoveActionResult.h>
#include <math.h>

#define _USE_MATH_DEFINES

float pose_x;
float pose_y;
float orientation;



namespace KCL_rosplan {

	GoToWaypointActionInterface::GoToWaypointActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool GoToWaypointActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		actionlib::SimpleActionClient<erl2::MoveAction> ac("reaching_goal", true);
		erl2::MoveAction goal;
		ac.waitForServer();

		if(msg->parameters[1].value == "w1"){
		pose_x=-2;
		pose_y=0.0;
		orientation=M_PI;
		}
		else if (msg->parameters[1].value == "w2"){
		pose_x=2;
		pose_y=0.0;
		orientation=0.0;
		}
		else if (msg->parameters[1].value == "w3"){
		pose_x=0.0;
		pose_y=2;
		orientation=1.5*M_PI;
		}
		else if (msg->parameters[1].value == "w4"){
		pose_x=0.0;
		pose_y=-2;
		orientation=M_PI_2;
		}
		else { //Go to oracle room
		pose_x=0.0;
		pose_y=0.0;
		orientation=0.0;
		}

		goal.action_goal.goal.x_pos=pose_x;
		goal.action_goal.goal.y_pos=pose_y;
		goal.action_goal.goal.theta=orientation;

		ac.sendGoal(goal.action_goal.goal);
		ac.waitForResult();

		erl2::MoveResultConstPtr res = ac.getResult();

		if (res->reached==true)
		{
			ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
			return true;
		}
		else
		{
			ROS_ERROR("The position reached by the robot is not correct");
			return false;	
		}
	}
}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "goto_waypoint_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
		KCL_rosplan::GoToWaypointActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

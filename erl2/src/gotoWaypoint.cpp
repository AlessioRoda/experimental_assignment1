#include "erl2/gotoWaypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/MoveAction.h>


float pose_x;
float pose_y;
float orientation;



namespace KCL_rosplan {

	GoToWaypointActionInterface::GoToWaypointActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool GoToWaypointActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		actionlib::SimpleActionClient<erl2::MoveAction> ac("reaching_goal", true);
		erl2::MoveAction goal;
		ac.waitForServer();
		if(msg->parameters[2].value == "wp1"){
		pose_x=-2.5;
		pose_y=0.0;
		orientation=0.0;
		}
		else if (msg->parameters[2].value == "wp2"){
		pose_x=2.5;
		pose_y=0.0;
		orientation=0.0;
		}
		else if (msg->parameters[2].value == "wp3"){
		pose_x=0.0;
		pose_y=-2.5;
		orientation=0.0;
		}
		else if (msg->parameters[2].value == "wp4"){
		pose_x=0.0;
		pose_y=-2.5;
		orientation=0.0;
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
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "goto_waypoint_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::GoToWaypointActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

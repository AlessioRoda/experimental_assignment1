#include "erl2/goto_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <motion_plan/PlanningAction.h>


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
		
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
		motion_plan::PlanningGoal goal;
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

		goal.target_pose.pose.position.x = pose_x;
		goal.target_pose.pose.position.y = pose_y;
		goal.target_pose.pose.orientation.w = orientation;

		ac.sendGoal(goal);
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

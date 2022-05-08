#include "erl2/check_Consistency.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/Consistency.h>
#include <motion_plan/PlanningAction.h>


float pose_x;
float pose_y;
float orientation;
ros::ServiceServer pub_consistency;


namespace KCL_rosplan {

	CheckConsistencyActionInterface::CheckConsistencyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool CheckConsistencyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		
		res=pub_consistency.publish();
        if res.updated==true:
		{
            return true;
        }
        else
        {
            return false;
        }
	}
}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "check_consistency", ros::init_options::AnonymousName);
        ros::ServiceServer pub_consistency= nh.advertiseService("/conosistency_request", Consistency);
		ros::NodeHandle nh("~");
		KCL_rosplan::Update_OnotlogyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

#include "erl2/checkConsistency.h"
#include <erl2/Consistency.h>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <ros/ros.h>
//#include <motion_plan/PlanningAction.h>


float pose_x;
float pose_y;
float orientation;
ros::ServiceClient client_conistency;


namespace KCL_rosplan {

	CheckConsistencyActionInterface::CheckConsistencyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool CheckConsistencyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		erl2::Consistency req;
		req.request.req=true;
	    client_conistency.call(req);
        if (req.response.res==true)
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
		ros::NodeHandle nh("~");
        client_conistency= nh.serviceClient<erl2::Consistency>("/conosistency_request");
		KCL_rosplan::CheckConsistencyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

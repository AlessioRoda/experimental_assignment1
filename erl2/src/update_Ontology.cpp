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
ros::ServiceServer pub_update;


namespace KCL_rosplan {

	Update_OnotlogyActionInterface::Update_OnotlogyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool Update_OnotlogyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		
		res=pub_update.publish();
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
		ros::init(argc, argv, "update_ontology", ros::init_options::AnonymousName);
        ros::ServiceServer pub_update= nh.advertiseService("/update_request", Consistency);
		ros::NodeHandle nh("~");
		KCL_rosplan::Update_OnotlogyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

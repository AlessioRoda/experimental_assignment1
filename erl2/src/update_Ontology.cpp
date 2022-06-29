#include "erl2/update_Ontology.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/Update.h>


float pose_x;
float pose_y;
float orientation;
ros::ServiceClient client_update;


namespace KCL_rosplan {

	Update_OnotlogyActionInterface::Update_OnotlogyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool Update_OnotlogyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		erl2::Update req;
		req.request.req=true;
	    client_update.call(req);
        if (req.response.updated==true)
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
		ros::NodeHandle nh("~");
		
        client_update= nh.serviceClient<erl2::Update>("/update_request");
		KCL_rosplan::Update_OnotlogyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

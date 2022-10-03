/*************************************************************************************************************************//**
 * \file   check_consistency.cpp
 * 
 * \brief Node to perform the check_consistency action of the PDDL domain
 * 
 * \version 1.0
 * \author Alessio Roda
 * \date   October 2022
 * 
 * description:
 *    This node implements the ROSPlan CheckConsistencyActionInterface: the corresponding behaviour for the check_consistency
 * 	  action in the PDDl file
 * 
*****************************************************************************************************************************/

#include "erl2/check_consistency.h"
#include <erl2/Consistency.h>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <ros/ros.h>

//Initialize the /consistency_request ServiceClient 
ros::ServiceClient client_consistency;


namespace KCL_rosplan {

	//Initialization of the CheckConsistencyActionInterface
	CheckConsistencyActionInterface::CheckConsistencyActionInterface(ros::NodeHandle &nh) {
			
	}

	/**
	 * bool CheckConsistencyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	 * 
	 * \brief Callback of the CheckConsistencyActionInterface
	 * 
	 * \param msg: action request for the CheckConsistencyActionInterface
	 *  
	 * 
	 * description:
	 *    This callback sends a service request on the topic /consistency_request, so to perform the check_consistency operation
	 * 	  in the ontology_interface node and waits until the operation finishes
	 **/
	bool CheckConsistencyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		erl2::Consistency req;
		req.request.req=true;
	    client_consistency.call(req);
        if (req.response.res==true)
		{
            return true;
        }
        else
        {	printf("Error occurred in checkConsistency");
            return false;
        }
	}
}

/** 
 * int main(argc, argv)
 * 
 * \brief Main function of the node
 * 
 * \param argc: the number of argument passed as parameters
 * 
 * \param argv: the vector of string containing each argument
 * 
 * \return 0 when the program ends
 * 
 * description:
 *    The main function, initializes the client for the /consistency_request service, declares the CheckConsistencyActionInterface
 * 	  and runs it
 *    
 **/
	int main(int argc, char **argv) {
		ros::init(argc, argv, "check_consistency", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
        client_consistency= nh.serviceClient<erl2::Consistency>("/consistency_request");
		KCL_rosplan::CheckConsistencyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

/*************************************************************************************************************************//**
 * \file   update_ontology.cpp
 * 
 * \brief Node to perform the UpdateOntologyActionInterface action of the PDDL domain
 * 
 * \version 1.0
 * \author Alessio Roda
 * \date   October 2022
 * 
 * description:
 *    This node implements the ROSPlan UpdateOntologyActionInterface: the corresponding behaviour for the update_ontology
 * 	  action in the PDDl file
 * 
*****************************************************************************************************************************/

#include "erl2/update_ontology.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/Update.h>

ros::ServiceClient client_update;

namespace KCL_rosplan {

	UpdateOntologyActionInterface::UpdateOntologyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	/**
	 * bool UpdateOntologyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	 * 
	 * \brief Callback of the UpdateOntologyActionInterface
	 * 
	 * \param msg: action request for the UpdateOntologyActionInterface
	 *  
	 * 
	 * description:
	 *    This callback sends a service request on the topic /update_request, so to perform the update_ontology operation
	 * 	  in the ontology_interface node and waits until the operation finishes
	 **/
	bool UpdateOntologyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
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
	 *    The main function, initializes the client for the /update_request service, declares the UpdateOntologyActionInterface
	 * 	  and runs it
	 *    
	 **/
	int main(int argc, char **argv) {
		ros::init(argc, argv, "update_ontology", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
        client_update= nh.serviceClient<erl2::Update>("/update_request");
		KCL_rosplan::UpdateOntologyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

#! /usr/bin/env python


import roslib
import rospy
import smach
import smach_ros
import time
import random
from datetime import datetime
from armor_py_api.scripts.armor_api.armor_client import ArmorClient
from os.path import dirname, realpath

armor_interface=None




def main():
    global armor_interface, people, places, weapons
    rospy.init_node('state_machine')

    path = dirname(realpath(__file__))+"cluedo_ontology.owl"
    armor_interface = ArmorClient("client", "reference")
    armor_interface.utils.load_ref_from_file(path, "http://www.emarolab.it/cluedo-ontology",
                                True, "PELLET", True)
    armor_interface.utils.mount_on_ref()

    


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LOCKED', Locked(), 
                               transitions={'push':'LOCKED', 
                                            'coin':'UNLOCKED'},
                               remapping={'locked_counter_in':'sm_counter', 
                                          'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('UNLOCKED', Unlocked(), 
                               transitions={'push':'LOCKED', 
                                            'coin':'UNLOCKED'},
                               remapping={'unlocked_counter_in':'sm_counter',
                                          'unlocked_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

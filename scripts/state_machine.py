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

people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock', 'Mrs.White', 'Prof.Plum', 'Rev.Green']
places=['Ballroom', 'Biliard_Room', 'Conservatory', 'Dining_Room', 'Hall', 'Kitchen', 'Library', 'Lounge','Study']
weapons=['Candlestick', 'Dagger','LeadPipe', 'Revolver', 'Rope', 'Spanner']

solution=None



def init_scene():
    global armor_interface, solution, people, places, weapons

    random.seed(datetime.now())
    index_people=random.randint(0, 5)
    index_places=random.randint(0, 8)
    index_weapons=random.randint(0, 5)
    
    solution=[people[index_people], places[index_places], weapons[index_weapons]]

    



def main():
    global armor_interface
    rospy.init_node('state_machine')

    path = dirname(realpath(__file__))+"cluedo_ontology.owl"
    armor_interface = ArmorClient("client", "reference")
    armor_interface.utils.load_ref_from_file(path, "http://www.emarolab.it/cluedo-ontology",
                                True, "PELLET", True)
    armor_interface.utils.mount_on_ref()

    armor_interface.manipulation.add_ind_to_class()

    init_scene()

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

#! /usr/bin/env python


import roslib
import rospy
from rospy.impl.tcpros_service import Service, ServiceProxy
import smach
import smach_ros
import time
import random
from datetime import datetime
from geometry_msgs.msg import Point
from experimental_assignment1.srv import Move
from experimental_assignment1.scripts import Place
from armor_py_api.scripts.armor_api.armor_client import ArmorClient
from os.path import dirname, realpath

armor_interface=None
pub_move=None
places=[]
actual_pos=Point()


def init_scene():
    global places, actual_pos
    places.append(Place("Ballroom", 5, 5))
    places.append(Place("Biliard_Room", -5, -5))
    places.append(Place("Dining_Room", 0, 0))
    places.append(Place("Conservatory", 5, -5))

    actual_pos.x=0
    actual_pos.y=0
    actual_pos.z=0 #Don't care




def user_action():
    return random.choice(['coin','push'])


class Explore(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['explore','enter in room'],
                             input_keys=['unlocked_counter_in'],
                             output_keys=['unlocked_counter_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(5)
        rospy.loginfo('Executing state UNLOCKED (users = %f)'%userdata.unlocked_counter_in)
        userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
        return user_action()


class Unlocked(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['push','coin'],
                             input_keys=['unlocked_counter_in'],
                             output_keys=['unlocked_counter_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(5)
        rospy.loginfo('Executing state UNLOCKED (users = %f)'%userdata.unlocked_counter_in)
        userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
        return user_action()
    

# define state Locked
class Locked(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['push','coin'],
                             input_keys=['locked_counter_in'],
                             output_keys=['locked_counter_out'])
        self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        while not rospy.is_shutdown():  
            time.sleep(1)
            if self.sensor_input < 5: 
                rospy.loginfo('Executing state LOCKED (users = %f)'%userdata.locked_counter_in)
                userdata.locked_counter_out = userdata.locked_counter_in + 1
                return user_action()
            self.sensor_input += 1
            self.rate.sleep




def main():
    global armor_interface, pub_move
    rospy.init_node('state_machine')

    pub_move=Service('/move_point', Move, Explore)

    init_scene()

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


     # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Explore', Explore(), 
                               transitions={'move':'Explore', 
                                            'enter room':'Enter Room'},
                               remapping={'moving_counter_in':'sm_counter', 
                                          'moving_counter_out':'sm_counter'})
        smach.StateMachine.add('Enter Room', Enter_Room(), 
                               transitions={'enter room':'Enter Room', 
                                            'generate solution':'Solution'},
                               remapping={'enter_room_in':'sm_counter',
                                          'ener_room_out':'sm_counter'})


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

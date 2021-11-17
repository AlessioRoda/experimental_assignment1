#! /usr/bin/env python

"""
.. module:: go_to_point
   :platform: Unix
   :synopsis: Class implementing the main operations to use ARMOR
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com
This class implements the ARMOR operations to use during the Cluedo game.  


This class is created to use in a more comfortable way the ARMOR commands, providing some methods that already create and
sends the correct messages to the armor_interface_srv service.

"""

from posixpath import dirname, realpath
from armor_msgs.srv import *
from armor_msgs.msg import *
from rospy.impl.tcpros_service import ServiceProxy

armor_interface=ServiceProxy('/armor_interface_srv', ArmorDirective)
'''
Defines the armor_interface_srv service to send the messages to ARMOR
'''

class MyArmor(object):
    """
    Simple class to use in a more comfortable way the ARMOR commands, providing some methods that already create and
    sends the correct messages to the armor_interface_srv service.
    """


    def __init__(self):
        """
        Initialize the class
        """
        print("\nClass initialized")

    def load(path):
        """
        Load the ontology from file

            Args: 
                path(string): the path in which there's the file with the ontology to load
            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        request=ArmorDirectiveReq()
        request.client_name='tutorial'
        request.reference_name='ontoTest'
        request.command='LOAD'
        request.primary_command_spec='FILE'
        request.secondary_command_spec=''
        request.args=[path, 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        request=armor_interface(request)
        return request

    def add_hypothesis(type, ID, arg):
        """
        Add the hypothesis to the ontology

            Args: type(string): the type of element to add (what, where or who)
                  ID(string): the ID of the hypothesis
                  arg(string): the name of the place/weapon/person to add to the ontology
            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='ADD'
        req.primary_command_spec='OBJECTPROP'
        req.secondary_command_spec='IND'
        req.args=[type, ID, arg]
        req=armor_interface(req)
        return req

    def reason():
        """
        Perform reason to update the ontology

            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        req=armor_interface(req)
        return req

    def ask_complete():
        """
        Asks to the ontology for completed hypothesis

            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        req=armor_interface(req)
        return req


    def ask_inconsistent():
        """
        Asks to the ontology for inconsistent hypothesis

            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        req=armor_interface(req)
        return req

    def remove(name):
        """
        Remove a hypothesis from the ontology

            Args: 
                name(string): the name of the hypothesis to remove
            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REMOVE'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= ''
        req.args= [name]
        req=armor_interface(req)
        return req

    def ask_item(type, ID):
        """
        Ask the name of a certain element of a hypothesis

            Args: 
                type(string): the type of the element (what, where or who)
                ID(string): the ID of the hypothesis
            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [type,ID]
        req=armor_interface(req)
        return req

    def add_item(name, type):
        """
        Add an element to the ontology

            Args: 
                name(string): the name of the element 
                type(string): the type of the element (PERSON/PLACE/WEAPON)
            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, type]
        req=armor_interface(req)
        return req

    def disjoint(ind1, ind2):
        """
        Disjoint two elements to notify the otology that they are different

            Args: 
                ind1(string): the name of the first element 
                ind1(string): the name of the second element 
            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= ''
        req.args= [ind1, ind2]
        req=armor_interface(req)
        return req


    def save():
        """
        Save the current ontology

            Returns:
                request(ArmorDirectiveReq): the response received from ARMOR
        """

        req=ArmorDirectiveReq()
        path = dirname(realpath(__file__))
        path = path[:-15] + "solution_cluedo_ontology.owl"
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'SAVE'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= [path]
        req=armor_interface(req)
        return req





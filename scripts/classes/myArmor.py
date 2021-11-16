#! /usr/bin/env python

from posixpath import dirname, realpath
from armor_msgs.srv import *
from armor_msgs.msg import *
from rospy.impl.tcpros_service import ServiceProxy

armor_interface=ServiceProxy('/armor_interface_srv', ArmorDirective)

class MyArmor(object):


    def __init__(self):
	    print("\nClass initialized")

    def load(path):
        request=ArmorDirectiveReq()
        request.client_name='tutorial'
        request.reference_name='ontoTest'
        request.command='LOAD'
        request.primary_command_spec='FILE'
        request.secondary_command_spec=''
        request.args=[path, 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        request=armor_interface(request)
        return request

    def add_hipotesis(type, ID, arg):
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
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= ''
        req.args= [ind1, ind2]
        req=armor_interface(req)
        return req

    def disjoint_class(ind1, ind2):
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'CLASS'
        req.secondary_command_spec= ''
        req.args= [ind1, ind2]
        req=armor_interface(req)
        return req


    def save():
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





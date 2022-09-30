#! /usr/bin/env python3

'''
.. module:: go_to_point
   :platform: Unix
   :synopsis: Class to define the places in the scene
	
.. moduleauthor:: Alessio Roda alessioroda98@gmail.com

This class represents a way to define the places of the Cluedo game

This class provides a simple way to define the places of the scene as objects characterized by a name and their x and y coordinates

'''

class Place(object):

    name=None
    '''
    Name of the place

    '''
    x=0
    '''
    x coordinate of the place

    '''
    y=0
    '''
    y coordinate of the place

    '''

    def __init__(self, name, x_coord, y_coord):
        self.name=name
        self.x=x_coord
        self.y=y_coord



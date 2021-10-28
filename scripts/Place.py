#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point


class Place(object):

    name=None
    position=Point()


    def __init__(self, name, x_coord, y_coord):
        self.name=name
        self.position.x=x_coord
        self.position.y=y_coord
        self.position.z=0 ## Don't care



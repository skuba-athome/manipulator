#!/usr/bin/env python
import rospy
from manipulator.srv import *

def manipulator_client(x, y, z):
	
    try:
        manipulator_service = rospy.ServiceProxy('manipulator_service', manipulator_S)
        resp1 = add_two_ints(x, y, z)
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

manipulator_client(float(input()), float(input()), float(input()))
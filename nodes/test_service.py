#!/usr/bin/env python
import roslib; roslib.load_manifest('manipulator')

import sys

import rospy
from manipulator.srv import *

if __name__ == "__main__":

	rospy.wait_for_service('isManipulable')

	isMani = rospy.ServiceProxy('isManipulable', isManipulable)
	respl = isMani(0.0,0.0,0.0)
	print respl.isManipulable
	respl = isMani(0.4,-0.12,-0.35)
	print respl.isManipulable
	respl = isMani(0.7,0.7,0.7)
	print respl.isManipulable

#!/usr/bin/env python
import rospy
import roslib
import time
import math
import os

from inverseKinematic import invKinematic
from geometry_msgs.msg import Vector3

from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import SetTorqueLimit

from manipulator_S.srv import *

import tf



def main():
    global tf_listener
    pub =['','','','','','','']
    pub[0] = rospy.Publisher('/shoulder_L_20/command', Float64)
    pub[1] = rospy.Publisher('/shoulder_L_21/command', Float64)
    pub[2] = rospy.Publisher('/elbow_L_22/command', Float64)
    pub[3] = rospy.Publisher('/hand_L_40/command', Float64)
    pub[4] = rospy.Publisher('/hand_L_41/command', Float64)
    pub[5] = rospy.Publisher('/hand_L_42/command', Float64)
    pub[6] = rospy.Publisher('/gripper_L_43/command', Float64)

#    pub['is_fin'] = rospy.Publisher('/manipulator/is_fin', String)
#    rospy.init_node('manipulator')
#    rospy.Subscriber("joy_cmd_manipulate", String, init_joy_cmd)
#    rospy.Subscriber("/manipulator/action", String, init_movement)
#    rospy.Subscriber("/manipulator/object_point", Vector3, init_point)
#    rospy.Subscriber("/manipulator/object_point_split", Vector3, init_point_split)
    #rospy.Subscriber("/manipulator/grasp", Vector3, grasp)
    #rospy.Subscriber("/manipulator/pour", Vector3, pour)
#    rospy.Subscriber("/diagnostics", DiagnosticArray, diag)

    #rospy.Service("isManipulable", isManipulable, is_manipulable_handle)

    tf_listener = tf.TransformListener()

    rospy.loginfo('Manipulator Start')
    s = rospy.Service('manipulator_service', manipulator_S, is_manipulable_handle)
    print "joeey service is ready"

    #does this need to remove away?#
    rospy.spin()
    #################################







def init_split(data):
    global pub
    print 'send data :',data
    #actionList['object_point'] = []
    #actionList['object_point'] = actionList['object_point'] + actionList['normal_for_get']

    try:
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    x, y, z = data.x - trans[0], data.y - trans[1], data.z - trans[2]
    #==== offset ====
    #x -= 0.03
    #y -= 0.01
    #z += 0.08
    #===============
    # extend
    print '####', 'x', x, 'y', y, 'z', z
    x0 = x

    #theta0 = invKinematic(x, y, z)
    try:
        x-=0.30
        for i in frange(x, x+0.30, 0.03):
            thetai = invKinematic(x, i, z)
            print thetai

        x = x0
        x-=0.30
        for i in frange(x, x+0.30, 0.03):
            thetai = invKinematic(x, i, z)
            motor_pub(thetai)
            pub['gripper_L_43'].publish(0)
            return True
        
    except:
        return False
    ##################################################################################


def motor_pub(theta):
    #pub['pan_kinect']
    #pub['tilt_kinect']
    pub[0].publish(theta[0])
    pub[1].publish(theta[1])
    pub[2].publish(theta[2])
    pub[3].publish(theta[3])
    pub[4].publish(theta[4])
    pub[5].publish(theta[5])
    pub[6].publish(theta[6])

def is_manipulable_handle(req):
    print "in isManipulableHandle method" + str(req)
    try:
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))

        print req
        data = invKinematic(req.x - trans[0] , req.y - trans[1], req.z - trans[2])
        
        #init_split(data)
        return isManipulableResponse(init_split(data))

    except:
        return isManipulableResponse(False)


def manipulator_server():
    rospy.init_node('manipulator_server')
    main()
    rospy.spin()

if __name__ == '__main__':

    try:
        rospy.loginfo('Manipulator service activated')
        manipulator_server()
#        action_path = roslib.packages.get_pkg_dir('manipulator') + '/action'
#        for action_filename in os.listdir(action_path):
#            if action_filename.endswith(".txt"):
#                action_name = os.path.splitext(os.path.basename(action_filename))[0]
#                action_file = open(os.path.join(action_path, action_filename))
#                actionList[action_name] = []
#                for step in action_file:
#                    if (step.strip() == ''):
#                        continue
#                    actionList[action_name].append(step.strip())
#        rospy.loginfo('Readfile Complete')
#        main()
    except rospy.ROSInterruptException:
        pass

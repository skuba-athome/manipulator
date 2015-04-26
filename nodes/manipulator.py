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
#from manipulator.srv import *

import tf

actionstep = {}
count = 0
pub = {}

actionList = {}
dynamixel = {10: 'pan_kinect', 11: 'tilt_kinect', 20: 'shoulder_L_20', 21: 'shoulder_L_21', 22: 'elbow_L_22', 40: 'hand_L_40', 41: 'hand_L_41', 42: 'hand_L_42', 43: 'gripper_L_43_L_43', 50: 'torso_M_50'}

tf_listener = []


def sendCommand(motorID, value):
    global pub
    #rospy.loginfo(motorID+':'+value)
    data = motorID + "," + value
    try:
        motorID = int(motorID)
        value = float(value)
        pub[dynamixel[motorID]].publish(value)
    except ValueError:
        value = float(value)
        rospy.wait_for_service('/' + motorID + '/set_speed')
        try:
            #rospy.loginfo('setspeed')
            setSpeed = rospy.ServiceProxy('/' + motorID + '/set_speed', SetSpeed)
            if motorID == 'elbow_L_22':
                respSpeed = setSpeed(0.15)
            else:
                respSpeed = setSpeed(0.4)
        except rospy.ServiceException, e:
            print "Service Speed call failed %s" % e
        if (motorID == 'gripper_L_43'):
            rospy.wait_for_service('/' + motorID + '/set_torque_limit')
            try:
                rospy.loginfo('settorque')
                setTorque = rospy.ServiceProxy('/' + motorID + '/set_torque_limit', SetTorqueLimit)
                respTorque = setTorque(0.30)
            except rospy.ServiceException, e:
                print "Service Torque call failed %s" % e
        pub[motorID].publish(value)
    rospy.loginfo('#' + data)


def frange(start, end, step):
    while start <= end:
        yield start
        start += step
    yield end


def init_point(data):
    global pub
    actionList['object_point'] = []
    actionList['object_point'].append('gripper_L_43,-0.4')
    actionList['object_point'].append('gripper_L_43,0.5')

    # format : x,y,z
    try:
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    x, y, z = data.x - trans[0], data.y - trans[1], data.z - trans[2]
    #x,y,z = data.x,data.y,data.z
    z += 0.1
    y -= 0.1
    print "DataAfter Trans:" + str((x, y, z, trans))

    theta = invKinematic(x, y, z)
    print 'invkine : ', x, y, z
    print theta
###################################################################################################
    actionList['object_point'].append('shoulder_L_20,' + str(theta[0]) + '/shoulder_L_21,' + str(theta[1]))
    actionList['object_point'].append('elbow_L_22,' + str(theta[2]))

    actionList['object_point'].append('gripper_L_43,-0.4')
    actionList['object_point'] = actionList['object_point'] + actionList['pullback']
    init_movement(String('object_point'))
#################^^^^^^ NEED CARE ^^^^^^##########################################################

def init_split(data):
    global pub
    print 'send data :',data
    actionList['object_point'] = []
    actionList['object_point'] = actionList['object_point'] + actionList['normal_for_get']

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

    #theta0 = invKinematic(x, y, z)

    for i in frange(y+0.1, y, 0.02):
        thetai = invKinematic(x, i, z)
        actionList['object_point'].append('shoulder_L_20,' + str(thetai[0]) + '/shoulder_L_21,' + str(thetai[1]) + '/elbow_L_22,' + str(thetai[2]) + '/hand_L_40,' + str(thetai[3]) + '/hand_L_41,' + str(thetai[4]) + '/hand_L_42,' + str(thetai[5]) + '/hand_L_43,' + str(thetai[6]))

    actionList['object_point'].append('gripper_L_43,-0.4')
    ##################################################################################


def init_point_split(data):
    global pub
    try:
        init_split(data)
        actionList['object_point'] = actionList['object_point'] + actionList['pullback']
        #actionList['object_point'] = actionList['object_point'] + actionList['normal_pullback']
        init_movement(String('object_point'))
    except Exception, e:
        print str(e)
        pub['is_fin'].publish('error')


################chalenge has been deleted###################

def init_movement(data):
    global actionstep, count
    actionname = data.data
    if (actionname in actionList):
        count = -1
        #count = 0
        actionstep = actionList[actionname]
        rospy.loginfo('##### init action #####')
        rospy.loginfo('Action Name : ' + actionname)
        rospy.loginfo('Sum Step : ' + str(len(actionstep)))
        rospy.loginfo('#######################')
    else:
        rospy.logerr('No action : ' + actionname)

def movement_step():
    global count
    if (count == len(actionstep)):
        return
    rospy.loginfo('step ' + str(count) + ' : ' + actionstep[count])
    #print 'actionstep :',actionstep
    for motor in actionstep[count].split('/'):
        motorID, value = motor.split(',')
        sendCommand(motorID, value)

def check_goal(motor_id, current_pos):
    global count
    if count >= len(actionstep) or count == -1:
        return False
    if (actionstep != {}):
        for motor in actionstep[count].split('/'):
            motorID, value = motor.split(',')
            if motorID in motor_id:
                if abs(current_pos - float(value)) > 0.1:
                    return True
                else:
                    return False

def diag(data):
    global count
    all_moving = 0
    for i in data.status:
        if (i.name[:16] == 'Joint Controller'):
            #if (i.hardware_id[19:21] in '21 22 23'):
            #   print i.hardware_id[19:21],float(i.values[2].value)
            #print i.values[5].value
            if (i.hardware_id[19:21] in '26'):
                continue
            if (i.values[5].value == 'True'):
                all_moving = 1
            #elif (i.hardware_id[19:21] in '21 22 23'):
            #   if check_goal(dynamixel[int(i.hardware_id[19:21])],float(i.values[1].value)):
            #        all_moving = 1
        else:  #there are some not joint package from diagnostics
            return
    if (count > len(actionstep)):
        rospy.loginfo('Waiting for Action...')
        return
    #rospy.loginfo('________')
    if (all_moving == 0):
        if (count == len(actionstep)):
            pub['is_fin'].publish('finish')
            rospy.loginfo('Finish Action!!!')
            count += 1
            return
        count += 1
    movement_step()


def main():
    global pub, tf_listener
    pub['pan_kinect'] = rospy.Publisher('/pan_kinect/command', Float64)
    pub['tilt_kinect'] = rospy.Publisher('/tilt_kinect/command', Float64)
    pub['shoulder_L_20'] = rospy.Publisher('/shoulder_L_20/command', Float64)
    pub['shoulder_L_21'] = rospy.Publisher('/shoulder_L_21/command', Float64)
    pub['elbow_L_22'] = rospy.Publisher('/elbow_L_22/command', Float64)
    pub['hand_L_40'] = rospy.Publisher('/hand_L_40/command', Float64)
    pub['hand_L_41'] = rospy.Publisher('/hand_L_41/command', Float64)
    pub['hand_L_42'] = rospy.Publisher('/hand_L_42/command', Float64)
    pub['gripper_L_43_L'] = rospy.Publisher('/gripper_L_43/command', Float64)

    pub['is_fin'] = rospy.Publisher('/manipulator/is_fin', String)
    rospy.init_node('manipulator')
    rospy.Subscriber("joy_cmd_manipulate", String, init_joy_cmd)
    rospy.Subscriber("/manipulator/action", String, init_movement)
    rospy.Subscriber("/manipulator/object_point", Vector3, init_point)
    rospy.Subscriber("/manipulator/object_point_split", Vector3, init_point_split)
    rospy.Subscriber("/manipulator/grasp", Vector3, grasp)
    rospy.Subscriber("/manipulator/pour", Vector3, pour)
    rospy.Subscriber("/diagnostics", DiagnosticArray, diag)

    #rospy.Service("isManipulable", isManipulable, is_manipulable_handle)

    tf_listener = tf.TransformListener()

    rospy.loginfo('Manipulator Start')
    rospy.spin()

def init_joy_cmd(action):
        init_movement(action)

def is_manipulable_handle(req):
    print "in isManipulableHandle method" + str(req)
    try:
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))
        theta = invKinematic(req.x - trans[0], req.y - trans[1], req.z - trans[2])
        return isManipulableResponse(True)
    except:
        return isManipulableResponse(False)


if __name__ == '__main__':
    try:
        rospy.loginfo('Manipulator Start Readfile LOL')
        action_path = roslib.packages.get_pkg_dir('manipulator') + '/action'
        for action_filename in os.listdir(action_path):
            if action_filename.endswith(".txt"):
                action_name = os.path.splitext(os.path.basename(action_filename))[0]
                action_file = open(os.path.join(action_path, action_filename))
                actionList[action_name] = []
                for step in action_file:
                    if (step.strip() == ''):
                        continue
                    actionList[action_name].append(step.strip())
        rospy.loginfo('Readfile Complete')
        main()
    except rospy.ROSInterruptException:
        pass

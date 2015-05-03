#!/usr/bin/env python
import rospy
import roslib
import time
import math
import os

from inverseKinematic import invKinematic
from geometry_msgs.msg import Vector3

#roslib.load_manifest('manipulator')
#roslib.load_manifest('dynamixel_controllers')

from std_msgs.msg import String
from std_msgs.msg import Float64
#from dynamixel_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import SetTorqueLimit
from manipulator.srv import *


import tf


actionstep = {}
count = 0


finish_manipulate = True


pub = {}

actionList = {}
dynamixel = {20: 'right_1', 21: 'right_2', 22: 'right_3', 40: 'joint1',41: 'joint2', 42: 'joint3', 43: 'gripper'}
dynamixeljointerror = {}
reachgoalstatus = {}
dynamixelerrlength = {'right_1' : 0.1 ,'right_2' : 0.1,'right_3' :0.1 ,'joint1' : 0.1 , 'joint2' : 0.1 ,'joint3':0.1 , 'gripper':0.1  }
#mark-44 Torelance = 65 encoder value -> 0.35 degree = 0.006 rad
#orion-45 Torelance = +- 3 deg = 0.05
searchaddress = '10 11 20 21 22 40 41 42 43'
tf_listener = []


def sendCommand(motorID, value):
    global pub
    #rospy.loginfo(motorID+':'+value)
    data = motorID + "," + value
    try:
        print 'sendcommand - > ' + str(motorID) + ' value:' + str(value) 
        #motorID = int(motorID)
        value = float(value)
        
        pub[motorID].publish(value)
    except ValueError:
        rospy.loginfo('failed in sendcommand')
        # value = float(value)
        # rospy.wait_for_service('/' + motorID + '/set_speed')
        # try:
        #     #rospy.loginfo('setspeed')
        #     setSpeed = rospy.ServiceProxy('/' + motorID + '/set_speed', SetSpeed)
        #     if motorID == 'mark44_3':
        #         respSpeed = setSpeed(0.15)
        #     else:
        #         respSpeed = setSpeed(0.4)
        # except rospy.ServiceException, e:
        #     print "Service Speed call failed %s" % e
        if (motorID == 'gripper'):
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

def init_split(data):
    global pub
    print 'send data :',data
    actionList['object_point'] = []
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
    ## Table Tall (DiningRoom z = 70cm)
    #===============
    # extend
    print '####SetPoint', 'x', x, 'y', y, 'z', z

    try:
        gripper_length = 0.11
        x -= gripper_length
        xendpts = x
        x-= 0.3
        if ((x < 0.4) and (xendpts >= 0.4)) : 
            x = 0.4
        step = 1
        for eachstep in frange(x,xendpts, 0.03):
            print 'step : ' + str(step) + ' x = ' + str(eachstep) + 'y = ' + str(y) + 'z = ' + str(z)  
            theta = invKinematic(eachstep, y, z)
            #print theta
            ##Add action movement in each forloop
            actionList['object_point'].append('right_1,'+ str(-theta[0]) + '/right_2,'+str(theta[1]) ) ##Do 2 dof of sholder together
            actionList['object_point'].append('right_3,'+ str(theta[2]) ) ##Do Elbow
            #actionList['object_point'].append('joint1,'+ str(theta[3]) + '/joint2,' + str(theta[4]) + '/joint3,' + str(theta[5]) )
            print ' action : right_1,' + str(-theta[0]) + '/right_2,'+ str(theta[1]) 
            step += 1
            print 'right_3,'+ str(theta[2])
            print 'joint1,'+ str(theta[3]) + '/joint2,' + str(theta[4]) + '/joint3,' + str(theta[5])
            print '---------------------'
        #actionList['object_point'].append('gripper,0') ##close_grip
        #actionList['object_point'].append(actionList['grip_close']) ##close_grip
    except:
        rospy.loginfo('Init Split Error')
        return False


def init_point_split(data):
    global pub
    try:
        init_split(data)
        actionList['object_point'] = actionList['object_point'] + actionList['pullback']
        #actionList['object_point'] = actionList['object_point'] + actionList['normal_pullback']
        init_movement(String('object_point'))
        return True
    except Exception, e:
        print str(e)
        #pub['is_fin'].publish('error')
        return False



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
    print '__________Current Action step Length = ' + str(len(actionstep)) + '  / COUNT = ' + str(count)
    if (count == len(actionstep)):
        return
    rospy.loginfo('*********** count = ' + str(count) + ' : ' + actionstep[count] + '*************')
    #print 'actionstep :',actionstep
    for motor in actionstep[count].split('/'):
        motorID, value = motor.split(',')
        sendCommand(motorID, value)
    #time.sleep(1)

# def check_goal(motor_id, current_pos):
#     global count
#     if count >= len(actionstep) or count == -1:
#         return False
#     if (actionstep != {}):
#         for motor in actionstep[count].split('/'):
#             motorID, value = motor.split(',')
#             if motorID in motor_id:
#                 if abs(current_pos - float(value)) > 0.1:
#                     return True
#                 else:
#                     return False

def diag(data):
    global count,dynamixeljointerror,dynamixel,reachgoalstatus,finish_manipulate,searchaddress,dynamixelerrlength
    #all_moving = 0
    for i in data.status:
        if (i.name[:16] == 'Joint Controller'):
            
            if (i.hardware_id[19:21] in searchaddress):#Search for ID in sentense
                #print 'Motor No.' + str(i.hardware_id[19:21]) + ' current_pos = ' + str(i.values[1].value)
                for key in dynamixel:
                    #if joint[key].id is int(i.hardware_id[19:21]):
                    if int(key) is int(i.hardware_id[19:21]):
                        #joint[key].currentAngle = i.values[1].value
                        dynamixeljointerror[dynamixel[key]] = float(i.values[2].value) # read joint Error in Radian
                        #print 'Motor No.' + str(joint[key].id) + ' current_pos = ' + str(joint[key].currentAngle)
        else:  #there are some not joint package from diagnostics
            return
    
    #print '***Current dynamixeljointerror***'
    
    #for key in dynamixeljointerror:
        #print key + ': Error = ' + str(dynamixeljointerror[key])    
        #pass
    ##Check Manip
    if (count > len(actionstep)):
        rospy.loginfo('Waiting for Action...')
        return

    #Process Action Step
    for key in dynamixeljointerror:
        #print "Joint Error : "+ key+ "=" + str(dynamixeljointerror[key])
        #print type(dynamixeljointerror[key]) , type(dynamixelerrlength[key])
        #print "dynamixelerrlength" + key + "=" + str(dynamixelerrlength[key])

        if dynamixeljointerror[key]  <= dynamixelerrlength[key]:
            reachgoalstatus[key] = True
            #print 'CHANGED STATUS : ' , reachgoalstatus[key]       
        else:
            reachgoalstatus[key] = False

        #print 'Reachgoal STATUS : ' , reachgoalstatus[key]
        #print 'ALL STATUS: ', reachgoalstatus
        if all(val == True for val in reachgoalstatus.values()):
            #finishstep = True
            print "-----------------STEP CHANGED--------------"
            for key in dynamixeljointerror:
                reachgoalstatus[key] = False
            time.sleep(0.5)    
            count += 1
            
        
    if (count == len(actionstep)):
        finish_manipulate = True
        rospy.loginfo('Finish Action!!!')
        #count += 1
        ##Clear step status
        for key in dynamixeljointerror:
            reachgoalstatus[key] = False
        return
    if finish_manipulate is not True:
        movement_step()
    


def main():
    global pub, tf_listener
    pub['pan_kinect'] = rospy.Publisher('/dynamixel/pan_kinect/command', Float64)
    pub['tilt_kinect'] = rospy.Publisher('/dynamixel/tilt_kinect/command', Float64)
    pub['right_1'] = rospy.Publisher('/dynamixel/right_1/command', Float64)
    pub['right_2'] = rospy.Publisher('/dynamixel/right_2/command', Float64)
    pub['right_3'] = rospy.Publisher('/dynamixel/right_3/command', Float64)
    pub['joint1'] = rospy.Publisher('/dynamixel/joint1/command', Float64)
    pub['joint2'] = rospy.Publisher('/dynamixel/joint2/command', Float64)
    pub['joint3'] = rospy.Publisher('/dynamixel/joint3/command', Float64)
    pub['gripper'] = rospy.Publisher('/dynamixel/gripper/command', Float64)

    #pub['is_fin'] = rospy.Publisher('/manipulator/is_fin', String)
    rospy.init_node('manipulator')
    rospy.Subscriber("joy_cmd_manipulate", String, init_joy_cmd)
    rospy.Subscriber("/diagnostics", DiagnosticArray, diag)

    rospy.Service("isManipulable", isManipulable, is_manipulable_handle)
    rospy.Service("ManipulateAction",ManipulateAction,manipulate_action_handle)
    tf_listener = tf.TransformListener()

    rospy.loginfo('Manipulator Start')
    rospy.spin()

def init_joy_cmd(action):
    init_movement(action)

def manipulate_action_handle(req):
    global dynamixelerrlength,dynamixeljointerror,reachgoalstatus,finish_manipulate
    finish_manipulate = False
    init_movement(String(req.action))
    
    ##init Reach goal status
    for key in dynamixeljointerror:
        reachgoalstatus[key] = False
    
    while finish_manipulate is not True:
        try:
            #print 'Wait for action'
            pass
        except:
            rospy.loginfo('Fail Service'+ str(1))
            return ManipulateActionResponse(False)
    rospy.loginfo('Finish ACTION')                
    return ManipulateActionResponse(True)
        


def is_manipulable_handle(req):
    global dynamixelerrlength,dynamixeljointerror,reachgoalstatus,finish_manipulate
    print "in isManipulableHandle method " + str(req)

    try:
        finish_manipulate = False

        (trans, rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))
        ##check if the points is reachable
        #print "tf : tx = " + str(trans[0]) + " ty = " + str(trans[1]) + " tz = " + str(trans[2])
        theta = invKinematic(req.x - trans[0], req.y - trans[1], req.z - trans[2])
    
        ##send data to process
        tempdata = Vector3()
        tempdata.x = req.x
        tempdata.y = req.y
        tempdata.z = req.z
        retstaus = init_point_split(tempdata)
    
        print "retstaus ->" + str(retstaus)
        if retstaus is True:
            ##init Reach goal status
            for key in dynamixeljointerror:
                reachgoalstatus[key] = False
                

            while finish_manipulate is not True:
                try:
                    #print 'Wait for action'
                    pass
                except:
                    rospy.loginfo('Fail Service'+ str(1))
                    return isManipulableResponse(False)
                    
            return isManipulableResponse(True)
        
        else:
            rospy.loginfo('Fail Service'+ str(2))
            return isManipulableResponse(False)

        return isManipulableResponse(True)
    except Exception as e:
        rospy.loginfo('Fail Service'+ str(3)) 
        print str(e)
        return isManipulableResponse(False)


if __name__ == '__main__':
    try:
        rospy.loginfo('Manipulator Start Readfile')
        action_path = roslib.packages.get_pkg_dir('manipulator') + '/action'
        #action_path = rospack.get_path('manipulator') + '/action'
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

#!/usr/bin/env python
import rospy
import roslib
import time
import math

from inverseKinematic import invKinematic
from geometry_msgs.msg import Vector3

roslib.load_manifest('manipulator')
roslib.load_manifest('dynamixel_controllers')

from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import SetTorqueLimit
from manipulator.srv import *

import tf

actionstep = {}
count = 0
pub = {}

packagePath = roslib.packages.get_pkg_dir('manipulator')

actionList = {}
dynamixel = {10: 'pan_kinect',11: 'tilt_kinect',21: 'mark43_1',22: 'mark43_2',25: 'mark43_3', 40: 'joint1', 41: 'joint2', 42: 'joint3', 43: 'gripper'}

tf_listener = []

def sendCommand(motorID,value):
	global pub
	#rospy.loginfo(motorID+':'+value)
	data = motorID+","+value
	try:
    		motorID = int(motorID)
		value = float(value)
		pub[dynamixel[motorID]].publish(value)
	except ValueError:
		value = float(value)
		rospy.wait_for_service('/'+motorID+'/set_speed')
		try:
			#rospy.loginfo('setspeed')
			setSpeed = rospy.ServiceProxy('/'+motorID+'/set_speed',SetSpeed)
			respSpeed = setSpeed(0.4)
		except rospy.ServiceException, e:
			print "Service Speed call failed %s"%e
		if(motorID=='gripper'):
			rospy.wait_for_service('/'+motorID+'/set_torque_limit')
			try:
				rospy.loginfo('settorque')
				setTorque = rospy.ServiceProxy('/'+motorID+'/set_torque_limit',SetTorqueLimit)
				respTorque = setTorque(0.25)
			except rospy.ServiceException, e:
				print "Service Torque call failed %s"%e
		pub[motorID].publish(value)
	rospy.loginfo('#'+data)

def frange(start, end, step):
    while start <= end:
        yield start
        start += step

def init_point(data):
	global pub
	actionList['object_point'] = []
	actionList['object_point'].append('gripper,-1.38')
	actionList['object_point'].append('gripper,-0.1')
	#actionList['object_point'].append('gripper,0.5')
	
	# format : x,y,z
       	try:
      		(trans,rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))
  	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
  		return

	x,y,z = data.x-trans[0],data.y-trans[1],data.z-trans[2]
	#x,y,z = data.x,data.y,data.z
	z += 0.1
	y -= 0.1
        print "DataAfter Trans:"+str((x,y,z,trans))

	theta = invKinematic(x,y,z)
	print 'invkine : ',x,y,z
	print theta
	actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0]+theta[1])*2)+'/mark43_2,'+str(math.radians(theta[0]+theta[1])*2))
	actionList['object_point'].append('mark43_3,'+str(math.radians(theta[2])))

	actionList['object_point'].append('gripper,-1.38')
	actionList['object_point'] = actionList['object_point']+actionList['pullback']
	init_movement(String('object_point'))

def init_point_split(data):
	global pub
	actionList['object_point'] = []
#	actionList['object_point'].append('joint1,0/joint2,0.5/joint3,0/gripper,-1.38')
#	actionList['object_point'].append('mark43_1,1.02/mark43_2,1.02/mark43_3,0')
#	actionList['object_point'].append('mark43_1,1.02/mark43_2,1.02/mark43_3,0')
#	actionList['object_point'].append('mark43_1,1.02/mark43_2,1.02/mark43_3,0')
#	actionList['object_point'].append('joint1,0/joint2,0/joint3,0/gripper,-0.1')
	actionList['object_point'] = actionList['object_point']+actionList['normal_for_get']
	actionList['object_point'].append('mark43_3,0')
	
	# format : x,y,z
	#x,y,z = data.x,data.y,data.z
	#z += 0.04
	#if(y>=0):
	#	y -= (0.1)
	#else:
	#	y -= 0.1

       	try:
      		(trans,rot) = tf_listener.lookupTransform('/base_link', '/mani_link', rospy.Time(0))
  	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
  		return

	x,y,z = data.x-trans[0],data.y-trans[1],data.z-trans[2]
	z += 0.04
	y -= 0.06
	# extend
	print '####','x',x,'y',y,'z',z
	dist = math.sqrt(x*x+y*y)
	zeeta = math.atan(y/x)
	print "##################### last step : ",dist*math.cos(zeeta),dist*math.sin(zeeta),z
	print '#####','dist',dist,'zeeta',zeeta
	try:
		theta0 = invKinematic((dist)*math.cos(zeeta),(dist)*math.sin(zeeta),z)
		actionList['object_point'].append('mark43_1,'+str(math.radians(-theta0[0])*2)+'/mark43_2,'+str(math.radians(theta0[0])*2))
		x_2 = 0.11*math.sin(math.radians(theta0[0]))
		y_2 = 0.11*math.cos(math.radians(theta0[0]))
		dist_true = math.sqrt(dist*dist-0.11*0.11)
		print '#####','theta0',theta0[0]
		print "#####",'x_2',x_2,'y_2',y_2
		print '#####','dist_true',dist_true
		#for i in frange(0.45, dist+0.04, 0.05):
		for i in frange(0.45, dist_true+0.04, 0.05):
			#theta = invKinematic(i*math.cos(zeeta),i*math.sin(zeeta),z)
			#print 'step',i,':',i*math.cos(zeeta),i*math.sin(zeeta),z
			theta = invKinematic(i*math.cos(math.radians(theta0[0]))+x_2,i*math.sin(math.radians(theta0[0]))-y_2,z)
			print '######','step',i,':',i*math.cos(math.radians(theta0[0]))+x_2,i*math.sin(math.radians(theta0[0]))-y_2,z
			#print theta
			actionList['object_point'].append('joint3,'+str(-3.67-math.radians(theta[3])))
			actionList['object_point'].append('gripper,-0.1')
			#actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0]+theta[1])*2)+'/mark43_2,'+str(math.radians(theta[0]+theta[1])*2)+'/mark43_3,'+str(math.radians(theta[2])))
			actionList['object_point'].append('mark43_1,'+str(math.radians(-theta0[0]+theta[1])*2)+'/mark43_2,'+str(math.radians(theta0[0]+theta[1])*2)+'/mark43_3,'+str(math.radians(theta[2])))
		
		actionList['object_point'].append('gripper,-1.38')
		actionList['object_point'] = actionList['object_point']+actionList['pullback']
		init_movement(String('object_point'))
	except Exception, e:
		print str(e)
		pub['is_fin'].publish('error')

def init_point_split_top(data):
	global pub
	actionList['object_point'] = []
	actionList['object_point'].append('joint1,1.57/joint3,-1.57/gripper,-1.38')
	actionList['object_point'].append('mark43_1,1.57/mark43_2,1.57/mark43_3,0/joint2,-1.57')
	actionList['object_point'].append('joint1,0/joint2,-1/joint3,-1.57/gripper,-0.1')

	# format : x,y,z
	x,y,z = data.x,data.y,data.z
	y -= 0.01
	# extend
	src = z+0.1
	print src,z

	delay = Delay()
	publish = Publish()
        readObject(object_list)
	readLocation(location_list)
	readLocationSequence(search_sequence)
        main()
	
	try:
		for i in frange(src, z, -0.01):
			theta = invKinematic(x,y,i)
			print 'step',i,':',x,y,i
			print theta
			actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0]+theta[1])*2)+'/mark43_2,'+str(math.radians(theta[0]+theta[1])*2)+'/mark43_3,'+str(math.radians(theta[2])))

		actionList['object_point'].append('gripper,-1.38')
		actionList['object_point'] = actionList['object_point']+actionList['pullback']
		init_movement(String('object_point'))
	except:
		pub['is_fin'].publish('error')

def init_movement(data):
	global actionstep,count
	actionname = data.data
	if(actionname in actionList):
		count = -1
		actionstep = actionList[actionname]
		rospy.loginfo('##### init action #####')
		rospy.loginfo('Action Name : '+actionname)
		rospy.loginfo('Sum Step : '+str(len(actionstep)))
		rospy.loginfo('#######################')
	else:
		rospy.logerr('No action : '+actionname)	
		
def movement_step():
	global count
	if(count==len(actionstep)):
        	return
	rospy.loginfo('step '+str(count) + ' : ' + actionstep[count])
	for motor in actionstep[count].split('/'):
		motorID,value = motor.split(',')
		sendCommand(motorID,value)
	
def diag(data):
	global count
	all_moving = 0
	for i in data.status:
		if(i.name[:16]=='Joint Controller'):
			if(i.values[5].value=='True'):
				all_moving = 1
			#print i.name,i.values[5].value
		else: 					#there are some not joint package from diagnostics
			return
	if(count>len(actionstep)):
		rospy.loginfo('Waiting for Action...')
		return
	#print all_moving
	#rospy.loginfo('________')
	if(all_moving==0):
		if(count==len(actionstep)):
			pub['is_fin'].publish('finish')
			rospy.loginfo('Finish Action!!!')
			count+=1
			return
		count+=1
	movement_step()
	

def main():
    global pub,tf_listener
    pub['pan_kinect'] = rospy.Publisher('/pan_kinect/command', Float64)
    pub['tilt_kinect'] = rospy.Publisher('/tilt_kinect/command', Float64)
    pub['mark43_1'] = rospy.Publisher('/mark43_1/command', Float64)
    pub['mark43_2'] = rospy.Publisher('/mark43_2/command', Float64)
    pub['mark43_3'] = rospy.Publisher('/mark43_3/command', Float64)
    pub['joint1'] = rospy.Publisher('/joint1/command', Float64)
    pub['joint2'] = rospy.Publisher('/joint2/command', Float64)
    pub['joint3'] = rospy.Publisher('/joint3/command', Float64)
    pub['gripper'] = rospy.Publisher('/gripper/command', Float64)

    pub['is_fin'] = rospy.Publisher('/manipulator/is_fin', String)
    rospy.init_node('manipulator')

    rospy.Subscriber("/manipulator/action", String, init_movement)
    rospy.Subscriber("/manipulator/object_point", Vector3, init_point)
    rospy.Subscriber("/manipulator/object_point_split", Vector3, init_point_split)
    rospy.Subscriber("/manipulator/object_point_split_top", Vector3, init_point_split_top)
    rospy.Subscriber("/diagnostics", DiagnosticArray, diag)

    rospy.Service("isManipulable", isManipulable, isManipulableHandle)

    tf_listener = tf.TransformListener()

    rospy.loginfo('Manipulator Start')
    rospy.spin()

def isManipulableHandle(req):
    try:
        theta = invKinematic(req.x, req.y, req.z)
        return isManipulableResponse(True)
    except:
	return isManipulableResponse(False)

if __name__ == '__main__':
    try:
	rospy.loginfo('Manipulator Start Readfile')
	temp = open(packagePath+'/action/actionList.txt')
	for line in temp:
		line = line.strip()
		actionFile = open(packagePath+'/action/'+line+'.txt')
		actionList[line] = []
		for step in actionFile:
			if(step.strip()==''):
				continue
			actionList[line].append(step.strip())
	rospy.loginfo('Readfile Complete')
        main()
    except rospy.ROSInterruptException:
        pass

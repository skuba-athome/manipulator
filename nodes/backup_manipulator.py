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

actionstep = {}
count = 0
pub = {}
is_split = 0

packagePath = roslib.packages.get_pkg_dir('manipulator')

actionList = {}
dynamixel = {10: 'pan_kinect',11: 'tilt_kinect',21: 'mark43_1',22: 'mark43_2',25: 'mark43_3', 40: 'joint1', 41: 'joint2', 42: 'joint3', 43: 'gripper'}

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
	


def cb_point(data):
	global pub
	# format : x,y,z
	x,y,z = data.x,data.y,data.z
	z += 0.28
	y += 0.15
	# extend
	theta = invKinematic(x,y,z)
	print theta
	print math.floor(730-theta[0]*2+theta[1]*2),math.floor(730+theta[0]*2+theta[1]*2),math.floor(theta[2]-52),math.radians(theta[3])
	sendCommand('43','0.5')
	time.sleep(0.01)
	sendCommand(bush[2],str(int(3)))
	time.sleep(1)
	sendCommand(bush[0],str(int(math.floor(730-theta[0]*2+theta[1]*2))))
	time.sleep(0.01)
	sendCommand(bush[1],str(int(math.floor(730+theta[0]*2+theta[1]*2))))
	time.sleep(25)
	sendCommand('43','-1.5')
	time.sleep(0.01)
	sendCommand('41',str(math.radians(theta[3])))
	time.sleep(1)
	sendCommand(bush[2],str(int(math.floor(theta[2]-52))))
	time.sleep(10)
	sendCommand('43','0.5')
	time.sleep(5)
	sendCommand(bush[2],'3')

def frange(start, end, step):
    while start <= end:
        yield start
        start += step

def init_point(data):
	global pub
	actionList['object_point'] = []
	actionList['object_point'].append('gripper,0.5')
	actionList['object_point'].append('gripper,-1.4')
	#actionList['object_point'].append('gripper,0.5')
	
	# format : x,y,z
	x,y,z = data.x,data.y,data.z
	z += 0.28
	y += 0.15
	# extend

	theta = invKinematic(x,y,z)
	print 'invkine : ',x,y,z
	print theta
	#print math.radians(-theta[0]+theta[1])*2,math.radians(theta[0]+theta[1])*2,math.radians(theta[2])
	#actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0])*2)+'/mark43_2,'+str(math.radians(theta[0])*2)+'/mark43_3,'+str(math.radians(0)))
	actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0]+theta[1])*2)+'/mark43_2,'+str(math.radians(theta[0]+theta[1])*2)+'/mark43_3,'+str(math.radians(theta[2])))
	actionList['object_point'].append('gripper,0.5')
	'''
	for step in actionFile:
		if(step.strip()==''):
			continue
		actionList['object_point'].append(step.strip())
	'''
	init_movement(String('object_point'))

def init_point_split(data):
	global pub
	actionList['object_point'] = []
	actionList['object_point'].append('joint1,0/joint2,0/joint3,0')
	actionList['object_point'].append('gripper,0.5')
	actionList['object_point'].append('gripper,-1.4')
	actionList['object_point'].append('mark43_1,1.57/mark43_2,1.57/mark43_3,2')
	#actionList['object_point'].append('gripper,0.5')
	
	# format : x,y,z
	x,y,z = data.x,data.y,data.z
	z += 0.13
	y += 0.14
	# extend

	dist = math.sqrt(x*x+y*y)
	zeeta = math.atan(y/x)
	#print dist,zeeta,dist*math.sin(zeeta)

	for i in frange(0.45, dist, 0.01):
		theta = invKinematic(i*math.cos(zeeta),i*math.sin(zeeta),z)
		print 'step',i,':',i*math.cos(zeeta),i*math.sin(zeeta),z
		#print theta
		#print math.radians(-theta[0]+theta[1])*2,math.radians(theta[0]+theta[1])*2,math.radians(theta[2])
		#actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0])*2)+'/mark43_2,'+str(math.radians(theta[0])*2)+'/mark43_3,'+str(math.radians(0)))
		actionList['object_point'].append('mark43_1,'+str(math.radians(-theta[0]+theta[1])*2)+'/mark43_2,'+str(math.radians(theta[0]+theta[1])*2)+'/mark43_3,'+str(math.radians(theta[2])))

	actionList['object_point'].append('gripper,0.5')
	'''
	for step in actionFile:
		if(step.strip()==''):
			continue
		actionList['object_point'].append(step.strip())
	'''
	init_movement(String('object_point'))

def init_movement_split(data):
	global actionstep,count,is_split
	actionname = data.data
	if(actionname in actionList):
		count = 0
		is_split = 1
		actionstep = actionList[actionname]
		rospy.loginfo('##### init action #####')
		rospy.loginfo('Action Name : '+actionname)
		rospy.loginfo('Sum Step : '+str(len(actionstep)))
		rospy.loginfo('#######################')
		'''
		for step in actionList[actionname]:
			rospy.loginfo('step '+str(i) + ' : ' + step)
			if(step != ''):
				for motor in step.split('/'):
					motorID,value = motor.split(',')
					sendCommand(motorID,value)
			time.sleep(1)
			i+=1
			if(i>len(actionList[actionname])):
				rospy.loginfo('step !!!!!!!!!!!!!!!!!')
		'''
	else:
		rospy.logerr('No action : '+actionname)	

def init_movement(data):
	global actionstep,count
	actionname = data.data
	if(actionname in actionList):
		count = 0
		is_split = 0
		actionstep = actionList[actionname]
		rospy.loginfo('##### init action #####')
		rospy.loginfo('Action Name : '+actionname)
		rospy.loginfo('Sum Step : '+str(len(actionstep)))
		rospy.loginfo('#######################')
	else:
		rospy.logerr('No action : '+actionname)	
		
def movement_step():
	global count
	rospy.loginfo('step '+str(count) + ' : ' + actionstep[count])
	for motor in actionstep[count].split('/'):
		motorID,value = motor.split(',')
		sendCommand(motorID,value)
	
def diag(data):
	global count,is_split
	all_moving = 0
	for i in data.status:
		if(i.name[:16]=='Joint Controller'):
			if(i.values[5].value=='True'):
				all_moving = 1
			#print i.name,i.values[5].value
		else: 					#there are some not joint package from diagnostics
			return
	if(count>len(actionstep)):
		rospy.loginfo('Waiting for Action...'+str(is_split))
		return
	#print all_moving
	#rospy.loginfo('________')
	if(is_split==0):
		if(all_moving==0):
			if(count==len(actionstep)):
				pub['is_fin'].publish('finish')
				rospy.loginfo('Finish Action!!!')
				count+=1
				return
			movement_step()
			count+=1
	else:
		if(all_moving==0):
			if(count==len(actionstep)):
				pub['is_fin'].publish('finish')
				rospy.loginfo('Finish Action!!!')
				count+=1
				return
			if(count==3):
				for i in range(len(actionstep)-4):
					movement_step()
					time.sleep(0.3)
					count+=1
			movement_step()
			count+=1
		

def cb_action(data):
	global pub
	action = data.datadynamixel_msgs/JointState
	if(action in actionList):
		for step in actionList[action]:
			if(step != ''):
				for motor in step.split('/'):
					motorID,value = motor.split(',')
					sendCommand(motorID,value)
			time.sleep(1)
	else:
		rospy.logerr('No action :'+action)

def main():
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
    #pub['bushless'] = rospy.Publisher('/set_multiposition', String)
    #pub['manipulator'] = rospy.Publisher('/manipulator/set_action', String)
    rospy.init_node('inverse_kine')

    rospy.Subscriber("/object_point", Vector3, cb_point)
    rospy.Subscriber("/manipulator/action", String, init_movement)
    rospy.Subscriber("/manipulator/object_point", Vector3, init_point)
    rospy.Subscriber("/manipulator/object_point_split", Vector3, init_point_split)
    rospy.Subscriber("/diagnostics", DiagnosticArray, diag)

    rospy.loginfo('Manipulator Start')
    rospy.spin()

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

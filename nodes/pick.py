#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, oskInc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import sys
import rospy
import actionlib
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from dynamixel_controllers.srv import SetTorqueLimit
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped

GRIPPER_EFFORT = 0.8


class GripperActionController:
    def __init__(self, gripper_side):
        # gripper_side should be right_gripper or left_gripper
        self.gripper_client = actionlib.SimpleActionClient('/dynamixel/' + gripper_side + '_controller/gripper_action',
                                                           GripperCommandAction)
        self.set_torque_limit = rospy.ServiceProxy('/dynamixel/right_gripper_joint_controller/set_torque_limit',
                                                   SetTorqueLimit)

    def gripper_open(self, gripper_effort=GRIPPER_EFFORT):
        action_open = GripperCommandGoal()
        action_open.command.position = 0.8
        self.set_torque_limit(gripper_effort)
        self.gripper_client.send_goal(action_open)

    def gripper_close(self, gripper_effort=GRIPPER_EFFORT):
        action_close = GripperCommandGoal()
        action_close.command.position = 0.0
        self.set_torque_limit(gripper_effort)
        self.gripper_client.send_goal(action_close)


if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo')
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    gripper = GripperActionController("right_gripper")
    robot.right_arm.set_planning_time(100)
    robot.left_arm.set_planning_time(100)
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    scene.remove_world_object("part1")
    rospy.sleep(1)

    gripper.gripper_close()

    robot.right_arm.set_named_target("right_normal") 
    robot.right_arm.go()

    gripper.gripper_open()
    #robot.left_arm.set_named_target("left_normal") 
    #robot.left_arm.go()
    rospy.sleep(1)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    print robot.get_planning_frame()
    p.pose.position.x = 0.8
    p.pose.position.y = -0.15
    p.pose.position.z = 0.85
    p.pose.orientation.w = 1.0
    #scene.add_box("pole", p, (0.3, 0.05, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.25
    scene.add_box("table", p, (0.33, 0.9, 0.5))
    
    p.pose.position.x = 0.66
    p.pose.position.y = -0.18
    p.pose.position.z = 0.6
    scene.add_box("part1", p, (0.05, 0.05, 0.2 ))

    # p.pose.position.x = 0.6
    # p.pose.position.y = -0.29
    # p.pose.position.z = 0.45
    # scene.add_box("part2", p, (0.075, 0.05, 0.2))
    rospy.sleep(1)

    # pick an object
    robot.right_arm.pick("part1")
    rospy.sleep(1)

    gripper.gripper_close()
    rospy.sleep(1)
    # robot.right_arm.set_named_target("right_normal") 
    # robot.right_arm.go()
    scene.remove_world_object("part1")
    robot.right_arm.set_named_target("right_pregrasp") 
    robot.right_arm.go()
    # robot.right_arm.pick("part2")
    #robot.right_arm.set_named_target("right_normal") 
    #robot.right_arm.go()

    rospy.spin()
    roscpp_shutdown()


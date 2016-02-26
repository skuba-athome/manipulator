rostopic pub /dynamixel/right_shoulder_1_controller/command std_msgs/Float64 "data: -0.15" | 
rostopic pub /dynamixel/right_shoulder_2_controller/command std_msgs/Float64 "data: 0.38" | 
rostopic pub /dynamixel/right_elbow_controller/command std_msgs/Float64 "data: 0.3" | 
rostopic pub /dynamixel/right_wrist_1_controller/command std_msgs/Float64 "data: 0.0" | 
rostopic pub /dynamixel/right_wrist_2_controller/command std_msgs/Float64 "data: -0.8" | 
rostopic pub /dynamixel/right_wrist_3_controller/command std_msgs/Float64 "data: 1.4" |
rostopic pub /dynamixel/right_gripper_joint_controller/command std_msgs/Float64 "data: -0.8"

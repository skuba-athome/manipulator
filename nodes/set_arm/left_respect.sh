rostopic pub /dynamixel/left_shoulder_1_controller/command std_msgs/Float64 "data: 0.15" | 
rostopic pub /dynamixel/left_shoulder_2_controller/command std_msgs/Float64 "data: -0.10" | 
rostopic pub /dynamixel/left_elbow_controller/command std_msgs/Float64 "data: -0.2" | 
rostopic pub /dynamixel/left_wrist_1_controller/command std_msgs/Float64 "data: 0.0" | 
rostopic pub /dynamixel/left_wrist_2_controller/command std_msgs/Float64 "data: -1.8" | 
rostopic pub /dynamixel/left_wrist_3_controller/command std_msgs/Float64 "data: 1.6"

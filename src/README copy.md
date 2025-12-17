cd ROS2ARM
hash -r
colcon build
source install/setup.bash

# run urdf:
source ~/.bashrc
ros2 launch urdf_tutorial display.launch.py model:=/home/wzk/ARM/src/arm_description/urdf/arm.urdf
ros2 launch urdf_tutorial display.launch.py model:=/home/wzk/ROS2ARM/src/arm_description/urdf/arm.urdf

# launch by main xacro:
ros2 launch urdf_tutorial display.launch.py model:=/home/wzk/ROS2ARM/src/arm_description/urdf/arm_main.urdf.xacro

# launch by uviz:
ros2 launch arm_description display.launch.xml

# launch the assstiant:
ros2 launch moveit_setup_assistant setup_assistant.launch.py 

# launch demo:
ros2 launch arm_moveit_config demo.launch.py 

# launch with the bring up file:
ros2 launch arm_bringup my_robot.launch.xml 

# Control demo
ros2 run arm_commander_c test_moveit 

# Start to control:
ros2 run arm_commander_c commander

# control command template
Joint Goal Control:
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data:[0,0,0,0,0]}"
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data:[-3.14,-3.14,-3.14,-3.14,-3.14]}"
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data:[1,1,0,0,0]}"

# Pose Goal Control:
ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand "{x: 0.391407, y: -0.000063, z: 0.226419, ox: 0.017067, oy: 0.706935, oz: 0.016974, ow: 0.706869}"

# Position: [0.39137, -4.21521e-05, 0.22646]
Orientation: [0.0172663, 0.706912, 0.0172053, 0.706881]

# Orginal State:
ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand "{x: 0.391407, y: -0.000063, z: 0.226419, ox: 0.017067, oy: 0.706935, oz: 0.016974, ow: 0.706869}"

# Random State 1:
ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand "{x: 0.058887, y: 0.146824, z: 0.449133, ox: -0.046032, oy: 0.113452, oz: 0.844689, ow: 0.521067}"

# Random State 2:
ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand "{x: -0.023763, y: -0.358257, z: 0.224518, ox: 0.256116, oy: -0.163100, oz: 0.354659, ow: 0.884319}"

# Random State 3:
ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand "{x: 0.257882, y: 0.229724, z: 0.058750, ox: -0.621570, oy: 0.370204, oz: -0.137256, ow: 0.676580}"

# Random State 4:
ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand "{x: 0.035492, y: 0.253320, z: 0.177651, ox: 0.213835, oy: 0.922607, oz: 0.313785, ow: -0.067892}"

ros2 topic pub -1 /cartesian_command cm/msg/CaCommand "{dx: 0.0, dy: 0.0, dz: -0.2}"

# Listen the topic joint_from_pose:
ros2 topic echo /joint_from_pose

# "Ros_to_pi_node" will sent the joint angle to motor driver:
ros2 run ros_to_pi ros_to_pi_node

# the motor control topic
ros2 topic echo /control_cmd



---------------------The Motor Control ---------------------
The Motor Control 
# Window 1: turn on the control node
sudo chmod 666 /dev/ttyACM0
ros2 run scservo_ros2 scservo_node

# Window 2: send the control command to the node
# Zero state:
ros2 topic pub --once /control_cmd arm_interface/msg/MotorPos "{servo_ids: [1, 2, 3, 4, 5, 6], position: [820, 850, 467, 1851, 1069, 3050], speed: [2400,2400,2400,2400,2400,2400], acceleration: [50, 50, 50, 50, 50, 50]}"

# Mid Finger Fuck You
ros2 topic pub --once /control_cmd arm_interface/msg/MotorPos "{servo_ids: [1, 2, 3, 4, 5, 6], position: [3000, 850, 467, 1500, 1069, 4095], speed: [1000,1000,1000,1000,1000,1000], acceleration: [50, 50, 50, 50, 50, 50]}"




ID:6 
Zero state:1069

ID:5 
Zero state:1069

ID:4 
Zero state:1851

ID:3 
Zero state:500

ID:2 
Zero state:900
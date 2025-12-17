


# Due to differences in the zero-position configuration between the 3D simulation model and the actual motor setup, 
# the current motion planning in simulation cannot yet be applied directly to the physical robotic arm.
# The configuration is being updated.
# At this stage, only the packages related to controlling the physical ARM are functional.


# --------------------- The Motor Control ---------------------
# The Motor Control 
# Window 1: turn on the control node
sudo chmod 666 /dev/ttyACM0
ros2 run scservo_ros2 scservo_node

# Window 2: send the control command to the node
# Zero state:
ros2 topic pub --once /control_cmd arm_interface/msg/MotorPos "{servo_ids: [1, 2, 3, 4, 5, 6], position: [820, 850, 467, 1851, 1069, 3050], speed: [2400,2400,2400,2400,2400,2400], acceleration: [50, 50, 50, 50, 50, 50]}

# Mid Finger Fuck You
ros2 topic pub --once /control_cmd arm_interface/msg/MotorPos "{servo_ids: [1, 2, 3, 4, 5, 6], position: [3000, 850, 467, 1500, 1069, 4095], speed: [1000,1000,1000,1000,1000,1000], acceleration: [50, 50, 50, 50, 50, 50]}
# --------------------- The Motor Control ---------------------
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




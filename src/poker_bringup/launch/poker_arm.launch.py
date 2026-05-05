import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    mode_arg = DeclareLaunchArgument(
        'mode', 
        default_value='pc_hardware', 
        description='Modes: pc_hardware, pi_hardware, pi_hardware_headless, sim'
    )
    port_arg = DeclareLaunchArgument(
        'port', 
        default_value='/dev/ttyACM0',
        description='Serial port for the hardware driver'
    )
    dashboard_only_arg = DeclareLaunchArgument(
        'dashboard_only',
        default_value='false',
        description='Set to true to launch only the dashboard (for remote control)'
    )
    
    mode = LaunchConfiguration('mode')
    port = LaunchConfiguration('port')
    use_sim_time = PythonExpression(["'", mode, "' == 'sim'"])
    dashboard_only = LaunchConfiguration('dashboard_only')

    # --- Paths ---
    lerobot_pkg = get_package_share_directory('lerobot_description')
    
    # --- 1. Simulation Environment (Gazebo) ---
    # Launch for sim OR digital twin (any mode except pi_hardware_headless)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lerobot_pkg, 'launch', 'so101_gazebo.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'pi_hardware_headless' and '", dashboard_only, "' == 'false'"])
        )
    )

    # --- 2. Dashboard ---
    # Run UNLESS the mode is headless.
    dashboard_node = Node(
        package='poker_dashboard',
        executable='dashboard', 
        condition=UnlessCondition(
            PythonExpression(["'", mode, "' == 'pi_hardware_headless'"])
        )
    )

    # --- 3. Controller (Brain) ---
    # Run unless we are in "Dashboard Only" mode
    controller_node = Node(
        package='poker_control',
        executable='controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}], # <--- ADD THIS LINE
        condition=UnlessCondition(dashboard_only)
    )

    # --- 4. Hardware Driver (Hardware Modes) ---
    # Run if NOT sim AND NOT dashboard_only
    driver_node = Node(
        package='scservo_driver',
        executable='driver_node',
        parameters=[{'port': port}],
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'sim' and '", dashboard_only, "' == 'false'"])
        )
    )

    # --- 5. Simulation Bridge (Sim Mode & Digital Twin) ---
    # Run if Gazebo is running (so we can bridge commands to it)
    sim_bridge_node = Node(
        package='poker_control',
        executable='sim_bridge',
        parameters=[{'mode': mode, 'use_sim_time': use_sim_time}],
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'pi_hardware_headless' and '", dashboard_only, "' == 'false'"])
        )
    )

    # --- 6. Controller Spawners (Gazebo Controllers) ---
    # Activate the controllers in Gazebo (Needed whenever Gazebo runs)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'pi_hardware_headless' and '", dashboard_only, "' == 'false'"])
        )
    )

    forward_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'pi_hardware_headless' and '", dashboard_only, "' == 'false'"])
        )
    )

    # --- 7. Robot State Publisher (Hardware Modes) ---
    # In Sim, this is handled by so101_gazebo.launch.py.
    # In Hardware, we need to run it manually.
    
    urdf_file = os.path.join(lerobot_pkg, 'urdf', 'so101.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'sim' and '", dashboard_only, "' == 'false'"])
        )
    )

    # --- 8. Pump/Button GPIO Node (Pi Hardware Modes Only) ---
    # Required by the paramiko-bridge dashboard for /button_count and /pump_control.
    pump_test_node = Node(
        package='poker_control',
        executable='pump_test',
        name='pump_test',
        parameters=[{'num_players': 3}],
        condition=IfCondition(
            PythonExpression(["'", mode, "' in ['pi_hardware', 'pi_hardware_headless']"])
        ),
    )

    return LaunchDescription([
        mode_arg,
        port_arg,
        dashboard_only_arg,
        gazebo_sim,
        dashboard_node,
        controller_node,
        driver_node,
        sim_bridge_node,
        joint_state_broadcaster,
        forward_position_controller,
        robot_state_publisher,
        pump_test_node,
    ])
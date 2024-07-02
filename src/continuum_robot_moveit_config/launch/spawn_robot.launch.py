import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable

def generate_launch_description():

    ld = LaunchDescription()

    bridge_params = os.path.join(get_package_share_directory('continuum_robot'), 
                                 'config', 'bridge.yaml')
    rviz_file = os.path.join(get_package_share_directory('continuum_robot_moveit_config'), 'config', 'moveit.rviz')

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Launch configuration variables specific to simulation
    arg_x_pos = LaunchConfiguration('x_pos')
    arg_y_pos = LaunchConfiguration('y_pos')
    arg_z_pos = LaunchConfiguration('z_pos')
    arg_roll_ang = LaunchConfiguration('roll_ang')
    arg_pitch_ang = LaunchConfiguration('pitch_ang')
    arg_yaw_ang = LaunchConfiguration('yaw_ang')

    ld.add_action(DeclareLaunchArgument('x_pos', default_value='0.0',
                                        description='Specify x_position of the robot'))
    ld.add_action(DeclareLaunchArgument('y_pos', default_value='0.0',
                                        description='Specify y_position of the robot'))
    ld.add_action(DeclareLaunchArgument('z_pos', default_value='0.0',
                                        description='Specify z_position of the robot'))
    ld.add_action(DeclareLaunchArgument('roll_ang', default_value='0.0',
                                        description='Specify initial roll angle of the robot'))
    ld.add_action(DeclareLaunchArgument('pitch_ang', default_value='0.0',
                                        description='Specify initial pitch angle of the robot'))
    ld.add_action(DeclareLaunchArgument('yaw_ang', default_value='0.0',
                                        description='Specify initial yaw angle of the robot'))
    ld.add_action(DeclareLaunchArgument('rviz_file', default_value=rviz_file))

    robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                LaunchConfiguration("robot_urdf_directory"),
            ]
        )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],    
        )
    ld.add_action(node_robot_state_publisher)

    node_gazebo_ros_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='gz_ros_spawner',
        arguments=[
            '-name', "XTa",
            # '-file', arg_robot,
            '-topic', robot_description,
            '-x', arg_x_pos,
            '-y', arg_y_pos,
            '-z', arg_z_pos,
            '-R', arg_roll_ang,
            '-P', arg_pitch_ang,
            '-Y', arg_yaw_ang
        ],
        output='screen',
    )
    ld.add_action(node_gazebo_ros_spawner)

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'xta_arm_controller'],
        output='screen'
    )

    event_load_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_gazebo_ros_spawner,
                on_exit=[load_joint_state_broadcaster],
            )
    )
    ld.add_action(event_load_joint_state_broadcaster)

    event_load_joint_trajectory_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    ld.add_action(event_load_joint_trajectory_controller)

    node_gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                "config_file": bridge_params,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )
    # ld.add_action(node_gz_ros_bridge)

    node_gazebo_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['rgbd/camera/image', 'rgbg/camera/depth_image'],
        output='screen',
    )
    # ld.add_action(node_gazebo_ros_image_bridge)

    return ld
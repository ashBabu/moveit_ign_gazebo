import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    pkg_directory = get_package_share_directory('continuum_robot_moveit_config')

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
    arg_use_gz_tf = LaunchConfiguration("use_gz_tf")
                                                                           
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
    ld.add_action(DeclareLaunchArgument("use_gz_tf", default_value="true", description="Use Gazebo TF"))

    robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("continuum_robot_moveit_config"),
                    "config", "continuum_robot_gazebo.urdf.xacro"]
                ),
            ]
        )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = os.path.join(pkg_directory, 'config', 'ros2_controllers.yaml')
    
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    ld.add_action(node_controller_manager)

    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        )
    ld.add_action(node_robot_state_publisher)

    node_gazebo_ros_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='gz_ros_spawner',
        arguments=[
            '-name', "XTa",
            # '-file', arg_robot,
            '-topic', "robot_description",
            '-x', arg_x_pos,
            '-y', arg_y_pos,
            '-z', arg_z_pos,
            # '-R', arg_roll_ang,
            # '-P', arg_pitch_ang,
            # '-Y', arg_yaw_ang
        ],
        output='screen',
    )
    ld.add_action(node_gazebo_ros_spawner)

    node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ld.add_action(node_joint_state_broadcaster_spawner)

    node_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xta_arm_controller", "--controller-manager", "/controller_manager"],
    )
    ld.add_action(node_joint_trajectory_controller_spawner)

    a1 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_gazebo_ros_spawner,
                on_exit=[node_joint_state_broadcaster_spawner],
            )
    )
    ld.add_action(a1)

    a2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_joint_state_broadcaster_spawner,
                on_exit=[node_joint_trajectory_controller_spawner],
            )
    )
    ld.add_action(a2)

    bridge_params = os.path.join(pkg_directory, 'config', 'bridge.yaml')

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
        arguments=['rgbd_camera/image', 'rgbg_camera/depth_image', '/flexx2'],
        output='screen',
    )
    ld.add_action(node_gazebo_ros_image_bridge)

 

    return ld
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    
    arg_robot = LaunchConfiguration('robot_urdf')  
    robot_urdf = arg_robot.perform(context)
    relative_urdf_path = "config/" + robot_urdf
    moveit_config = MoveItConfigsBuilder("continuum_robot")
    moveit_config.robot_description(file_path=relative_urdf_path)
    moveit_config.trajectory_execution(file_path="config/moveit_controllers.yaml")
    moveit_config.planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    moveit_config.robot_description_kinematics(file_path="config/kinematics.yaml")
    moveit_config.planning_pipelines(pipelines=["ompl"]).to_moveit_configs()

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    return [node_move_group]

def generate_launch_description():

    ld = LaunchDescription()

    rviz_file = os.path.join(get_package_share_directory('continuum_robot_moveit_config'), 'config', 'moveit.rviz')
    
    arg_rviz_file = LaunchConfiguration('rviz_file')
    
    ld.add_action(DeclareLaunchArgument('rviz_file', default_value=rviz_file))
    ld.add_action(DeclareLaunchArgument('robot_urdf', default_value='continuum_robot_gazebo.urdf.xacro'))

    ld.add_action(OpaqueFunction(function=launch_setup))

    # RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', arg_rviz_file],
    )
    ld.add_action(node_rviz)

    node_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        )
    ld.add_action(node_joint_state_publisher)

    return ld
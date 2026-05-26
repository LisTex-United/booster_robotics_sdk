from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys

def generate_launch_description():
    # URDF path - using the fixed version with absolute mesh paths
    urdf_path = '/home/master/Workspace/booster_robotics_sdk/urdf/T1_fixed.urdf'

    # RViz config path
    # We assume this file is in the same workspace or can be found relative to this script
    # For simplicity in this environment, we will use the absolute path we just created
    rviz_config_path = '/home/master/Workspace/booster_robotics_sdk/rviz/booster_t1.rviz'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)}],
            arguments=[urdf_path]
        ),
        # Using existing /joint_states topic from the robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        )
    ])

def main():
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()

if __name__ == '__main__':
    from launch import LaunchService
    main()

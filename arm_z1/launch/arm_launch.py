from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os


def generate_launch_description():

    return LaunchDescription([
        # Launch the z1_ros node with the specified executable
        ExecuteProcess(cmd=['bash', './src/arm/arm_z1/launch/launch_arm_controller.sh'], name='z1_controller', output='both'),
        Node(
            package='arm_z1', 
            executable='z1_ros',
            name='z1_ros',
            output='screen', # to show on terminal
            
        ),

        Node(
            package='arm_z1',
            executable='static_arm_transformations',
            name='static_arm_transformations',
            output='log',
        ),

        
       ])



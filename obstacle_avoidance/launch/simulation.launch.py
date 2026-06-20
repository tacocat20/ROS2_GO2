"""
Launch file for obstacle avoidance simulation in Gazebo.
Runs: Gazebo + Robot + Obstacle Avoider Node
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'world',
            default_value='obstacle_world.world',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'safe_distance',
            default_value='0.8',
            description='Safe distance for obstacle avoidance'
        ),
        DeclareLaunchArgument(
            'forward_speed',
            default_value='0.3',
            description='Forward speed of robot'
        ),
        
        # Start Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                ['/home/krish/ros2_ws/src/obstacle_avoidance/worlds/', 
                 LaunchConfiguration('world')]
            ],
            output='screen'
        ),
        
        # Start RViz for visualization
        ExecuteProcess(
            cmd=['rviz2', '-d', 
                 '/home/krish/ros2_ws/src/obstacle_avoidance/rviz/simulation.rviz'],
            output='screen'
        ),
        
        # Obstacle Avoidance Node
        Node(
            package='obstacle_avoidance',
            executable='avoider',
            name='obstacle_avoider',
            output='screen',
            parameters=[
                {'safe_distance': LaunchConfiguration('safe_distance')},
                {'forward_speed': LaunchConfiguration('forward_speed')},
                {'turn_speed': 1.0},
                {'backup_speed': -0.2},
            ]
        ),
    ])

import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, OpaqueFunction

# List of pickup points and drop-off points
pickup_points = [f'r{i+1}' for i in range(209)]
dropoff_points = [f'd{i+1}' for i in range(3)]

def generate_random_patrol_task(context):
    start_point = random.choice(pickup_points)
    dropoff_point = random.choice(dropoff_points)
    return [
        Node(
            package='rmf_demos_tasks',
            executable='dispatch_patrol',
            name='dispatch_patrol',
            output='screen',
            arguments=[
                '-p', start_point, dropoff_point, start_point,
                '-n', '1',
            ]
        )
    ]

def repeat_patrol_task(context):
    return [
        TimerAction(
            period=30.0,
            actions=[
                OpaqueFunction(function=generate_random_patrol_task),
                OpaqueFunction(function=repeat_patrol_task)
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=repeat_patrol_task)
    ])

if __name__ == '__main__':
    generate_launch_description()
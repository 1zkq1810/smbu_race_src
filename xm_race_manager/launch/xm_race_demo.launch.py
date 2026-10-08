# launch 用 Python（战队规范：节点 C++，launch Python）
# use_fake_nav：true=VM演全剧（不接Nav2）；false=实车真导航
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_nav', default_value='true',
            description='true=假导航演全剧；false=真调Nav2'),
        Node(
            package='xm_race_manager',
            executable='race_manager_node',
            name='race_manager_node',
            output='screen',
            parameters=[{'use_fake_nav': LaunchConfiguration('use_fake_nav')}],
        ),
    ])

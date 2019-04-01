"""Launch a talker and a listener."""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='bdi_ros2', node_executable='belief_node', output='screen'),
	launch_ros.actions.Node(
            package='bdi_ros2', node_executable='goal_node', output='screen'),
    ])

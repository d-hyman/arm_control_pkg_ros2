import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ctrl_share = get_package_share_directory('arm_control_pkg')
    desc_share = get_package_share_directory('robot_arm_assem_v5')

    params = os.path.join(ctrl_share, 'config', 'arm_control.yaml')
    urdf_path = os.path.join(desc_share, 'urdf', 'robot_arm_assem_v5.urdf')
    rviz_config = os.path.join(desc_share, 'config', 'urdf.rviz')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    fake = LaunchConfiguration('fake_detector')
    mode = LaunchConfiguration('driver_mode')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('fake_detector', default_value='true',
                              description='Publish fake detections instead of using the AI node'),
        DeclareLaunchArgument('driver_mode', default_value='sim',
                              description='"sim" (RViz only) or "serial" (real servos)'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # No joint_state_publisher_gui here: servo_driver_node owns /joint_states
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(rviz)),

        Node(package='arm_control_pkg', executable='sorter_node', name='sorter_node',
             parameters=[params], output='screen'),
        Node(package='arm_control_pkg', executable='servo_driver_node', name='servo_driver_node',
             parameters=[params, {'mode': mode}], output='screen'),
        Node(package='arm_control_pkg', executable='fake_detector_node', name='fake_detector_node',
             parameters=[params], output='screen', condition=IfCondition(fake)),
    ])
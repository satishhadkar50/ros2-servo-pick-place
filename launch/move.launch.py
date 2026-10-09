import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    pkg_path = get_package_share_directory('arm_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm.urdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'display.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()
         
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    pose_node = Node(                       # 🏷️ pose_node
        package='arm_description',
        executable='pose_pb',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        pose_node,
        rviz,
    ])
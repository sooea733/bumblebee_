import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 기본 경로
    share_dir = get_package_share_directory('bumblebee_MobileManipulator_ver100_description')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')

    # URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'bumblebee_MobileManipulator_ver4.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # RViz
    default_rviz_config = os.path.join(share_dir, 'config', 'display.rviz')

    # 런치 인자
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=default_rviz_config, description='RViz config file')
    database_path_arg = DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map DB path')

    rviz_config = LaunchConfiguration('rviz_config')
    db_path = LaunchConfiguration('database_path')

    # URDF TF 퍼블리셔
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ✅ rtabmap.launch.py 포함
    rtabmap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
        ),
    )

    return LaunchDescription([
        rviz_config_arg,
        database_path_arg,
        robot_state_publisher_node,
        #rviz_node,
        rtabmap_include
    ])

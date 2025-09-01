import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 기본 경로
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')

    # 런치 인자
    database_path_arg = DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map DB path')
    db_path = LaunchConfiguration('database_path')

    # ✅ rtabmap.launch.py를 로컬라이제이션 모드로 포함
    rtabmap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
        ),
        launch_arguments={
            'localization': 'true',
            'database_path': db_path
        }.items()
    )

    return LaunchDescription([
        database_path_arg,
        rtabmap_include
    ])
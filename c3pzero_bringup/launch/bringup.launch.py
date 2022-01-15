import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='c3pzero_bringup').find('c3pzero_bringup')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/bringup_config.rviz')

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='False',
                                            description='Flag to show rviz'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),
        rviz_node
    ])

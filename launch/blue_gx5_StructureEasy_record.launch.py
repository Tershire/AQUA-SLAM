import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


RECORD_TOPICS = [
    '/aqua_slam/orb_odom',
    '/aqua_slam/orb_path',
    '/aqua_slam/orb_odom_body',
    '/aqua_slam/orb_path_body',
    '/aqua_slam/dvl_imu_pose',
    '/aqua_slam/dvl_imu_path',
    '/aqua_slam/dvl_imu_pose_ref',
    '/aqua_slam/dvl_imu_path_ref',
    '/apriltag_slam/GT',
]


def generate_launch_description():
    pkg_share = get_package_share_directory('aqua_slam')

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    results_dir = Path(pkg_share).parents[3] / 'src' / 'AQUA-SLAM' / 'results'
    results_dir.mkdir(parents=True, exist_ok=True)
    os.chmod(results_dir, 0o777)  # allow host user (non-root) to write plots alongside bags
    default_result_bag = str(results_dir / f'slam_{timestamp}')

    result_bag_arg = DeclareLaunchArgument(
        'result_bag',
        default_value=default_result_bag,
        description='Output path prefix for the result bag',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 alongside the SLAM node',
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'blue_gx5_StructureEasy.launch.py')
        ),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
    )

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('result_bag'), *RECORD_TOPICS],
        output='screen',
    )

    return LaunchDescription([
        result_bag_arg,
        use_rviz_arg,
        slam_launch,
        record,
    ])

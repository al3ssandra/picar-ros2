import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import launch_ros

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_nav').find('carlikebot_nav')
    slam_config_path = os.path.join(nav_pkg_share, 'config/mapper_params_online_async.yaml')
    nav2_config_path = os.path.join(nav_pkg_share, 'config/nav2_params.yaml')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('carlikebot_bringup'), 'launch'), '/carlikebot_gazebo_classic.launch.py'])),
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
                                                launch_arguments={'slam_params_file': slam_config_path}.items()),
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                                                launch_arguments={'params_file': nav2_config_path}.items())
    ])
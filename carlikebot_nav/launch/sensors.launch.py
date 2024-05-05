import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import launch_ros

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_nav').find('carlikebot_nav')
    slam_config_path = os.path.join(nav_pkg_share, 'config/mapper_params_online_async.yaml')
    nav2_config_path = os.path.join(nav_pkg_share, 'config/nav2_params.yaml')
    twist_mux_params = os.path.join(nav_pkg_share,'config/twist_mux.yaml')
    map_yaml_path = os.path.join(nav_pkg_share, 'maps/turtlebot3_world.yaml')
    # use_sim_time = True

    twist_mux = launch_ros.actions.Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/bicycle_steering_controller/reference_unstamped')]
            # remappings=[('/cmd_vel_out','/bicycle_steering_controller/reference')]
    )

    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('carlikebot_bringup'), 'launch'), '/carlikebot_gazebo_classic.launch.py'])),
        # launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
        #                                         launch_arguments={'slam_params_file': slam_config_path, 
        #                                                           'use_sim_time': 'True'
        #                                                           # 'use_lifecycle_manager': 'True'
        #                                                           }.items()),
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/localization_launch.py']),
                                                launch_arguments={'params_file': nav2_config_path,
                                                                  'map': map_yaml_path, 
                                                                  'use_sim_time': 'True'
                                                                  }.items()),
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                                                launch_arguments={'params_file': nav2_config_path, 
                                                                  'use_sim_time': 'True'}.items()),
        twist_mux
    ])
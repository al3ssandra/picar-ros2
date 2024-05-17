import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_description').find('carlikebot_description')
    default_model_path = os.path.join(description_pkg_share, 'urdf/carlikebot.urdf.xacro')
    default_rviz_config_path = os.path.join(description_pkg_share, 'rviz/carlikebot.rviz')
    bringup_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_bringup').find('carlikebot_bringup')
    default_controllers_config_path = os.path.join(bringup_pkg_share, 'config/carlikebot_controllers.yaml')


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'), 
                                                   " ", "use_mock_hardware:=", LaunchConfiguration('use_mock_hardware'), 
                                                   " ", "mock_sensor_commands:=", LaunchConfiguration('mock_sensor_commands')])}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    control_node_remapped = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[default_controllers_config_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            # ("/diff_drive_controller/tf_odometry", "/tf"),
        ],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('use_ekf')),
    )
    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[default_controllers_config_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_ekf')),
    )
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    imu_sensor_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_diff_drive_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(bringup_pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
       condition=launch.conditions.IfCondition(LaunchConfiguration('use_ekf')),
)
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_mock_hardware', default_value='true',
                                            description='Start robot with fake hardware mirroring command to its states.'),
        launch.actions.DeclareLaunchArgument(name='mock_sensor_commands', default_value='false',
                                            description="Enable fake command interfaces for sensors used for simple simulations. \
                                            Used only if 'use_mock_hardware' parameter is true."),
        launch.actions.DeclareLaunchArgument(name="use_ekf", default_value="true",
                                            description="Use extended kalman filter for sensor fusion"),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        control_node_remapped,
        control_node,
        joint_state_broadcaster_spawner,
        robot_diff_drive_controller_spawner,
        imu_sensor_broadcaster_spawner,
        robot_localization_node,
        rviz_node
    ])
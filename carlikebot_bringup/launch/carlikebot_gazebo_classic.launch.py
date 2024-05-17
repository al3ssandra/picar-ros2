import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
import os


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_description').find('carlikebot_description')
    default_model_path = os.path.join(description_pkg_share, 'urdf/carlikebot.urdf.xacro')
    default_rviz_config_path = os.path.join(description_pkg_share, 'rviz/carlikebot.rviz')
    # default_world_path = os.path.join(description_pkg_share, 'world/my_world.sdf')
    default_world_path = os.path.join(description_pkg_share, 'world/turtlebot3_world.model')
    bringup_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_bringup').find('carlikebot_bringup')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'), " ", "use_gazebo_classic:=true", " ",
                                                   "use_ekf:=", LaunchConfiguration('use_ekf')])}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_diff_drive_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        # arguments=["-topic", "robot_description", "-entity", "carlikebot"],
        arguments=["-topic", "robot_description", "-entity", "carlikebot", "-x", "0.5", "-y", "0.5"],
        output="screen",
    )
    imu_sensor_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(bringup_pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
       condition=launch.conditions.IfCondition(LaunchConfiguration('use_ekf')),
    )
    gazebo = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [PathJoinSubstitution([launch_ros.substitutions.FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]), 
        launch_arguments={'world': default_world_path}.items()
    )

    return launch.LaunchDescription([
        gazebo,
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name="use_ekf", default_value="true",
                                            description="Use extended kalman filter to fuse sensors"),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                            description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_diff_drive_controller_spawner,
        imu_sensor_broadcaster_spawner,
        robot_localization_node,
        rviz_node
    ])
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
import os


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='carlikebot_description').find('carlikebot_description')
    default_model_path = os.path.join(description_pkg_share, 'urdf/carlikebot.urdf.xacro')
    default_rviz_config_path = os.path.join(description_pkg_share, 'rviz/carlikebot.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'), " ", "use_gazebo_classic:=true"])}]
    )
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_bicycle_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller", "--controller-manager", "/controller_manager"],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "carlikebot"],
        output="screen",
    )
    imu_sensor_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )
    gazebo = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [PathJoinSubstitution([launch_ros.substitutions.FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]), 
        launch_arguments={"verbose": "false"}.items())

    return launch.LaunchDescription([
        gazebo,
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_bicycle_controller_spawner,
        imu_sensor_broadcaster_spawner,
        rviz_node
    ])
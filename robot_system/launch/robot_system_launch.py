# robot_system/launch/robot_system_launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_system')

    # World file argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, '..', 'worlds', 'cupboard_world.world'),
        description='Full path to the Ignition world file'
    )

    # Include Ignition Gazebo launch (ros_ign_gazebo)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            )
        ),
        launch_arguments={
            'ign_args': ['-r', LaunchConfiguration('world')]
        }.items()
    )

    # Sensor nodes
    camera_node = Node(package='sensors', executable='camera_node', name='camera_node')
    odom_node   = Node(package='sensors', executable='odom_node',   name='odom_node')
    imu_node    = Node(package='sensors', executable='imu_node',    name='imu_node')
    range_node  = Node(package='sensors', executable='range_node',  name='range_node')

    # Perception nodes
    fid_node = Node(package='perception', executable='fiducial_detector',      name='fiducial_detector')
    loc_node = Node(package='perception', executable='localization_node',      name='localization_node')

    # Planning nodes
    lane_svc_node = Node(package='planning', executable='lane_change_service', name='lane_change_service')
    gp_node       = Node(package='planning', executable='global_planner',       name='global_planner')
    lt_node       = Node(package='planning', executable='local_trajectory',     name='local_trajectory')

    # Control node
    vc_node = Node(package='control', executable='velocity_controller', name='velocity_controller')

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        camera_node,
        odom_node,
        imu_node,
        range_node,
        fid_node,
        loc_node,
        lane_svc_node,
        gp_node,
        lt_node,
        vc_node,
    ])


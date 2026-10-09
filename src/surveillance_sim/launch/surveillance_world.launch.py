import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    world_file = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ['true', '1']
    paused = LaunchConfiguration('paused').perform(context).lower() in ['true', '1']

    gz_args = []
    if not paused:
        gz_args.append('-r')
    if headless:
        gz_args.append('-s')
    gz_args.append(world_file)

    ros_gz_sim_share = FindPackageShare('ros_gz_sim').perform(context)
    gz_sim_launch_path = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items(),
    )

    return [gz_sim]


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare('surveillance_sim'),
        'worlds',
        'surveillance_world.sdf'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to the SDF world file to load.'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless/server-only mode (-s, no GUI).'
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start simulation paused. If false, passes -r to run on start.'
        ),
        OpaqueFunction(function=launch_setup),
    ])

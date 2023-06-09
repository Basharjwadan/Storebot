import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    package_name='storebot_sim'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join( get_package_share_directory(package_name), 'worlds', 'world.model')
    launch_file_dir = os.path.join( get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/rsp.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'storebot'],
                        output='screen'),

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
        ),



    
    ])

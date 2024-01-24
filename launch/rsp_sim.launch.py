import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'urdf_example'
    file_subpath = 'description/example_robot.urdf.xacro'

    world_subpath = 'worlds/test_world.world'
    world_path = os.path.join(get_package_share_directory(pkg_name),world_subpath)


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #         launch_arguments={'world' : world_path, 'verbose':"true"}.items()
    #     )
    
    ign_gaz = ExecuteProcess(
            cmd=['ign','gazebo', '--verbose'],
            output='screen'
        )


    spawn_entity = Node(package='ros_gz_sim', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')


    # Run the node
    return LaunchDescription([
        ign_gaz,
        node_robot_state_publisher,
        spawn_entity
    ])
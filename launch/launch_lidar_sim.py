
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_name = 'lidar_vis'
    file_subpath = 'description/example_robot.urdf.xacro'
    world_subpath = 'worlds/test_world.world'
    world_path = os.path.join(get_package_share_directory(pkg_name), world_subpath)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
        # launch_arguments={
        #    'gz_args': '-r gpu_lidar.sdf'
        # }.items(),
    )

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       # FIXME: Generate new RViz config once this demo is usable again
       # arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'gpu_lidar.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # parameters=[{'name': 'example_robot',
        #             '-topic': '/robot_description'}],
        arguments=['-topic', 'robot_description',
        '-entity', 'my_bot'], output='screen',
    )

    # Gz - ROS Bridge
    bridge1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (IGN -> ROS2)
            #'/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        # remappings=[
        #     ('/world/empty/model/rrbot/joint_state', 'joint_states'),
        # ],
        output='screen'
    )

    # Bridge
    bridge2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )
                                                                  

    # FIXME: need a SDF file (gpu_lidar.sdf) inside ros_gz_point_cloud/
    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn,
        bridge1,
        bridge2,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),

        rviz
    ])
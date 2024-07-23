import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('mpu_viz'), 'urdf', 'plane.urdf'),
        description='Full path to the URDF file to be loaded'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable GUI'
    )

    robot_description_param = DeclareLaunchArgument(
        name='robot_description',
        default_value=os.path.join(get_package_share_directory('mpu_viz'), 'urdf', 'plane.urdf'),
        description='Full path to the URDF file to be loaded'
    )

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
    )

    imu_node = Node(
        package='mpu_viz',
        executable='imu_node.py',
        name='imu_node',
        output='screen'
    )

    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter_node_for_orientation',
        output='screen'
    )

    tf_broadcaster_node = Node(
        package='mpu_viz',
        executable='tf_broadcaster_imu.py',
        name='rpy_tf',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add all actions to the launch description
    ld.add_action(model_arg)
    ld.add_action(gui_arg)
    ld.add_action(robot_description_param)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(imu_node)
    ld.add_action(imu_filter_node)
    ld.add_action(tf_broadcaster_node)
    # Uncomment the following line if RViz configuration is ready
    # ld.add_action(rviz_node)

    return ld

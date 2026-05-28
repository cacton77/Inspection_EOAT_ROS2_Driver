from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    declared_arguments = [
        # Add any declared arguments here if needed in the future
        # DeclareLaunchArgument("cell", default_value="alpha"),
    ]

    d405_camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="d405_camera",
        output="screen",
        parameters=[{
            'spatial_filter.enable': True,
            'temporal_filter.enable': True,
            # SENSOR_DATA (BEST_EFFORT + VOLATILE) for image streams.
            # Default SYSTEM_DEFAULT resolves to RELIABLE, which stalls the
            # writer on cross-host UDP fragment loss (seen as multi-second
            # gaps in `ros2 topic hz` from remote subscribers).
            'depth_qos': 'SENSOR_DATA',
            'depth_info_qos': 'SENSOR_DATA',
            'infra_qos': 'SENSOR_DATA',
            'infra1_qos': 'SENSOR_DATA',
            'infra2_qos': 'SENSOR_DATA',
            'color_qos': 'SENSOR_DATA',
            'color_info_qos': 'SENSOR_DATA',
        }],
    )

    joy_node = Node(
        package='joy',
        executable="joy_node",
        name='joy'
    )

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888', '-v6'],
    )

    macro_ps_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='macro_ps_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200', '-v6'],
    )

    imu_processor_node = Node(
        package='inspection_eoat',
        executable='imu_processor_node',
        name='imu_processor',
        output='screen',
    )

    return LaunchDescription(declared_arguments + [
        joy_node,
        d405_camera_node,
        # micro_ros_agent_node,
        # macro_ps_agent_node,
        # imu_processor_node,
    ])

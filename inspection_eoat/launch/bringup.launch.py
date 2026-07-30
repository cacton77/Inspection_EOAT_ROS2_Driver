from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    declared_arguments = [
        # Add any declared arguments here if needed in the future
        # DeclareLaunchArgument("cell", default_value="alpha"),
        DeclareLaunchArgument(
            "pi_camera_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("inspection_eoat"),
                "config",
                "pi_camera_2028x1520.yaml",
            ]),
            description=(
                "Parameter file for the pi_camera node. One file per sensor "
                "mode: it pairs width/height with the camera_info_url of the "
                "calibration derived for that resolution."
            ),
        ),
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

    pi_camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        # The parameter file is keyed on this node name, and so are its
        # qos_overrides entries — keep them in sync if this is renamed.
        name="pi_camera",
        output="screen",
        parameters=[LaunchConfiguration("pi_camera_config")],
    )

    joy_node = Node(
        package='joy',
        executable="joy_node",
        name='joy'
    )

    turntable_agent_node = Node(
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
        # output='screen',
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
        pi_camera_node,
        # turntable_agent_node,
        macro_ps_agent_node,
        # imu_processor_node,
    ])

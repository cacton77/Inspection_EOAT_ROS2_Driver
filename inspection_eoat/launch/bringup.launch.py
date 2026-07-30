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

    pi_camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="pi_camera",
        output="screen",
        parameters=[{
            'camera': 0,
            'width': 1920,
            'height': 1080,
            'format': 'RGB888',
            'frame_id': 'eoat_camera_link',
            # Theoretical (uncalibrated) intrinsics for the macro-PS rig at
            # minimum magnification (0.12x). camera_info_manager resolves the
            # package:// URL to config/macro_ps_imx477_min_mag.yaml at runtime.
            # Replace with measured intrinsics once ChArUco calibration is run.
            'camera_info_url': 'package://inspection_eoat/config/macro_ps_imx477_min_mag.yaml',
            # BEST_EFFORT + VOLATILE for image streams, same rationale as the
            # D405 above: default RELIABLE stalls over cross-host UDP loss.
            # camera_ros gained per-topic qos_overrides in PR #155 (June 2026).
            'qos_overrides./pi_camera/image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./pi_camera/image_raw.publisher.durability': 'volatile',
            'qos_overrides./pi_camera/image_raw.publisher.history': 'keep_last',
            'qos_overrides./pi_camera/image_raw.publisher.depth': 5,
            'qos_overrides./pi_camera/camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./pi_camera/camera_info.publisher.durability': 'volatile',
            'qos_overrides./pi_camera/camera_info.publisher.history': 'keep_last',
            'qos_overrides./pi_camera/camera_info.publisher.depth': 5,
        }],
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

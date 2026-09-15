from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
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
        arguments=['udp4', '--port', '8888', '-v2'],   # see macro_ps_agent_node
    )

    lens_tuner_node = Node(
        package='inspection_eoat',
        executable='lens_tuner_node',
        name='lens_tuner',
        output='screen',
    )

    macro_ps_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='macro_ps_agent',
        # output='screen',
        # Verbosity is a real-time parameter here, not just noise control. At -v6
        # the agent hex-dumps every message; with the IMU at 200 Hz that is ~220
        # log records a second, written synchronously to the SD card. A card
        # flush stalls the agent for long enough to break the /lens/command
        # stream, and the firmware's 150 ms velocity deadman then halts the jog
        # and re-ramps -- visible as the lens stuttering mid-jog. It had grown a
        # 56 GB launch.log before this was caught, and raising the host command
        # rate made the stutter worse rather than better, which is the tell.
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200', '-v2'],
    )

    imu_processor_node = Node(
        package='inspection_eoat',
        executable='imu_processor_node',
        name='imu_processor',
        output='screen',
    )

    # Foxglove bridge, so the EOAT's images and state can be viewed from any
    # machine on the LAN (ws://<this-host>:8765) without running a second ROS
    # stack. It runs HERE rather than on another host sharing the ROS domain for
    # a bandwidth reason: a remote bridge would first pull every image across the
    # LAN over DDS (UDP, fragmented) and then re-send it over its websocket. Run
    # here, the bytes cross the network once, over TCP.
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start foxglove_bridge on port 8765.')
    try:
        get_package_share_directory('foxglove_bridge')
        foxglove_available = True
    except PackageNotFoundError:
        foxglove_available = False

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('foxglove')),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            # Raw image topics are refused outright. Measured on this Pi:
            # /pi_camera/image_raw is ~154 MB/s (~1.2 Gbit/s) at 4056x3040 --
            # more than the gigabit link it would have to cross -- and the D405
            # raw streams are ~190-290 Mbit/s each. A viewer subscribing to one
            # by accident would saturate the network the robot also depends on.
            # View the /compressed variants instead. std::regex (ECMAScript), so
            # the negative lookahead is supported.
            'topic_whitelist': ['^(?!.*/(image_raw|image_rect_raw)$).*$'],
            # READ-ONLY. The defaults also grant clientPublish, parameters and
            # services, with no authentication: anyone who can reach port 8765
            # could publish /camera_head/lens/command or /led_ring/command. A
            # Foxglove panel publishing on a timer would recreate exactly the
            # two-publishers-fighting failure already hit on /led_ring/command.
            'capabilities': ['connectionGraph', 'assets'],
            # Never transcode to video on the Pi. Likely only relevant to
            # Foxglove remote access (off here), but a 12 MP transcode would be
            # expensive enough not to leave to a default.
            'video_transcode_topic_denylist': ['.*'],
        }],
    )
    foxglove_actions = (
        [foxglove_arg, foxglove_bridge_node] if foxglove_available else
        [LogInfo(msg='foxglove_bridge is not installed in this image; skipping it. '
                     'Rebuild the image to get it (docker/overlay_packages.jazzy.txt).')]
    )

    # Appended here rather than written into the list below, deliberately: that
    # list is where per-rig node selection gets toggled locally, and touching the
    # line that opens it would conflict with those local edits on pull.
    declared_arguments += foxglove_actions

    return LaunchDescription(declared_arguments + [
        # joy_node,
        # d405_camera_node,
        # pi_camera_node,
        # turntable_agent_node,
        lens_tuner_node,
        macro_ps_agent_node,
        # imu_processor_node,
    ])

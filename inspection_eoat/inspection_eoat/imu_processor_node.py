"""
Subscribes to a sensor_msgs/Imu stream (default /camera_head/imu) and publishes
a geometry_msgs/AccelStamped combining:

  * linear  — gravity-free linear acceleration. The low-frequency component of
    the raw accelerometer signal is treated as gravity and subtracted. This
    works because /camera_head/imu has no orientation
    (orientation_covariance[0] = -1 on the firmware side), so we cannot rotate
    a known g vector into the body frame. The low-pass approach is the standard
    substitute and is tunable via `gravity_alpha`.

  * angular — angular acceleration via finite difference of angular_velocity
    using each message's header stamp as the time base. Optional IIR smoothing
    on the output to tame differentiation noise.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import AccelStamped


class ImuProcessorNode(Node):
    def __init__(self):
        super().__init__('imu_processor')

        self.declare_parameter('imu_topic', '/camera_head/imu')
        # Per-sample EMA weight for the gravity estimate. At 200 Hz, 0.005 ≈
        # 1 s time constant — slow enough that typical hand motion above ~0.2 Hz
        # survives the subtraction, fast enough to track slow tilt.
        self.declare_parameter('gravity_alpha', 0.005)
        # Output IIR smoothing (1.0 = passthrough). Differentiation amplifies
        # high-frequency noise, so some smoothing on angular_acceleration is
        # usually wanted.
        self.declare_parameter('angular_acceleration_alpha', 0.3)

        imu_topic = self.get_parameter('imu_topic').value
        self.gravity_alpha = float(self.get_parameter('gravity_alpha').value)
        self.alpha_smooth = float(
            self.get_parameter('angular_acceleration_alpha').value)

        self.gravity = None              # running gravity estimate, m/s^2
        self.prev_omega = None           # last angular_velocity sample
        self.prev_stamp_ns = None        # last header stamp in nanoseconds
        self.alpha_filt = (0.0, 0.0, 0.0)  # smoothed angular acceleration

        # Match the firmware's BEST_EFFORT publisher; RELIABLE would never
        # complete the QoS handshake.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos)

        self.accel_pub = self.create_publisher(AccelStamped, '~/accel', 10)

        self.get_logger().info(
            f'Subscribed to {imu_topic} '
            f'(gravity_alpha={self.gravity_alpha}, '
            f'angular_acceleration_alpha={self.alpha_smooth})')

    def imu_callback(self, msg: Imu):
        a = (msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z)
        omega = (msg.angular_velocity.x,
                 msg.angular_velocity.y,
                 msg.angular_velocity.z)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Seed the gravity estimate from the first sample. Assumes the camera
        # is roughly stationary at startup; if it isn't, the EMA will converge
        # within a few time constants anyway.
        if self.gravity is None:
            self.gravity = a
        else:
            k = self.gravity_alpha
            self.gravity = (
                (1.0 - k) * self.gravity[0] + k * a[0],
                (1.0 - k) * self.gravity[1] + k * a[1],
                (1.0 - k) * self.gravity[2] + k * a[2],
            )

        a_linear = (a[0] - self.gravity[0],
                    a[1] - self.gravity[1],
                    a[2] - self.gravity[2])

        # Skip publishing until we have a previous sample to finite-difference
        # against; otherwise angular would be a misleading zero.
        if self.prev_omega is None or self.prev_stamp_ns is None:
            self.prev_omega = omega
            self.prev_stamp_ns = stamp_ns
            return

        dt_ns = stamp_ns - self.prev_stamp_ns
        if dt_ns <= 0:
            return

        dt = dt_ns * 1e-9
        raw_alpha = (
            (omega[0] - self.prev_omega[0]) / dt,
            (omega[1] - self.prev_omega[1]) / dt,
            (omega[2] - self.prev_omega[2]) / dt,
        )
        k = self.alpha_smooth
        self.alpha_filt = (
            (1.0 - k) * self.alpha_filt[0] + k * raw_alpha[0],
            (1.0 - k) * self.alpha_filt[1] + k * raw_alpha[1],
            (1.0 - k) * self.alpha_filt[2] + k * raw_alpha[2],
        )

        self.prev_omega = omega
        self.prev_stamp_ns = stamp_ns

        out = AccelStamped()
        out.header = msg.header
        out.accel.linear.x, out.accel.linear.y, out.accel.linear.z = a_linear
        out.accel.angular.x, out.accel.angular.y, out.accel.angular.z = self.alpha_filt
        self.accel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

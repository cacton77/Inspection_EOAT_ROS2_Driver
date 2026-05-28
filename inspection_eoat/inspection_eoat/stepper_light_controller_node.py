"""
ROS2 node wrapper for StepperLightController.

Subscribes to /joy and controls stepper velocity based on joystick input.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

from .stepper_light_controller import StepperLightController


class StepperLightControllerNode(Node):
    def __init__(self):
        super().__init__('stepper_light_controller')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('home_on_start', True)
        self.declare_parameter('velocity_scale', 127.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('deadzone', 0.1)

        port = self.get_parameter('port').value
        home_on_start = self.get_parameter('home_on_start').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.deadzone = self.get_parameter('deadzone').value

        self.get_logger().info(f'Connecting to {port}...')
        self.controller = StepperLightController(port)
        self.controller.connect(home=home_on_start)
        self.get_logger().info('Controller connected')

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.position_pub = self.create_publisher(Int32, '~/position', 10)
        self.velocity_pub = self.create_publisher(Int32, '~/velocity', 10)

        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def joy_callback(self, msg: Joy):
        if len(msg.axes) < 1:
            return

        axis_value = msg.axes[0]
        if abs(axis_value) < self.deadzone:
            axis_value = 0.0

        velocity = int(axis_value * self.velocity_scale)
        velocity = max(-127, min(127, velocity))

        d_pad_value = msg.axes[7]
        if d_pad_value != 0.0:
            velocity = int(d_pad_value)

        if velocity == 0 and self.controller.velocity != 0:
            # Transitioning to stop: set position to current so motor holds in place
            self.controller.position = self.controller.feedback_position

        self.controller.velocity = velocity

    def timer_callback(self):
        pos_msg = Int32()
        pos_msg.data = self.controller.feedback_position

        vel_msg = Int32()
        vel_msg.data = self.controller.feedback_velocity

        self.position_pub.publish(pos_msg)
        self.velocity_pub.publish(vel_msg)

    def destroy_node(self):
        self.get_logger().info('Disconnecting controller...')
        self.controller.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperLightControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

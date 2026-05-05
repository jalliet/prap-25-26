import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

try:
    from gpiozero import Button, OutputDevice
except ImportError:  # pragma: no cover - exercised on non-Pi development machines.
    Button = None
    OutputDevice = None


class PokerGpioNode(Node):
    """ROS 2 bridge for the poker button and active-low pump GPIO circuit."""

    def __init__(self):
        super().__init__('poker_gpio')

        self.declare_parameter('pump_pin', 27)
        self.declare_parameter('pump_active_low', True)
        self.declare_parameter('button_pin', 17)
        self.declare_parameter('button_pull_up', False)
        self.declare_parameter('button_sample_period_s', 0.02)
        self.declare_parameter('button_debounce_samples', 3)
        self.declare_parameter('button_count_min', 1)
        self.declare_parameter('button_count_max', 5)

        self._pump_pin = self.get_parameter('pump_pin').value
        self._pump_active_low = self.get_parameter('pump_active_low').value
        self._button_pin = self.get_parameter('button_pin').value
        self._button_pull_up = self.get_parameter('button_pull_up').value
        self._sample_period_s = self.get_parameter('button_sample_period_s').value
        self._debounce_samples = max(
            1, int(self.get_parameter('button_debounce_samples').value))
        self._count_min = int(self.get_parameter('button_count_min').value)
        self._count_max = int(self.get_parameter('button_count_max').value)

        if self._count_min > self._count_max:
            raise ValueError('button_count_min must be <= button_count_max')

        if Button is None or OutputDevice is None:
            raise RuntimeError(
                'gpiozero is required for poker_gpio. Install gpiozero and lgpio '
                'in the Pi workspace venv before launching this node.'
            )

        # active_high=False makes on() drive the pin low and off() drive it high.
        self._pump = OutputDevice(
            self._pump_pin,
            active_high=not self._pump_active_low,
            initial_value=False,
        )
        self._button = Button(
            self._button_pin,
            pull_up=self._button_pull_up,
            bounce_time=None,
        )

        self._button_count = self._count_min
        self._stable_button_value = int(self._button.is_pressed)
        self._candidate_value = self._stable_button_value
        self._candidate_samples = 0

        self._button_pub = self.create_publisher(Int32, '/button_count', 10)
        self._pump_sub = self.create_subscription(
            Int32, '/pump_control', self._pump_callback, 10)

        self._timer = self.create_timer(
            float(self._sample_period_s), self._sample_button)

        self._publish_button_count()
        self.get_logger().info(
            f'poker_gpio ready: pump GPIO {self._pump_pin} '
            f'({"active-low" if self._pump_active_low else "active-high"}), '
            f'button GPIO {self._button_pin}'
        )

    def _pump_callback(self, msg: Int32):
        enabled = int(msg.data) != 0
        if enabled:
            self._pump.on()
        else:
            self._pump.off()
        self.get_logger().info(f'pump {"on" if enabled else "off"}')

    def _sample_button(self):
        value = int(self._button.is_pressed)

        if value != self._candidate_value:
            self._candidate_value = value
            self._candidate_samples = 1
            return

        self._candidate_samples += 1
        if self._candidate_samples < self._debounce_samples:
            return

        if value == self._stable_button_value:
            return

        previous_value = self._stable_button_value
        self._stable_button_value = value

        if previous_value == 0 and value == 1:
            self._advance_button_count()

    def _advance_button_count(self):
        if self._button_count >= self._count_max:
            self._button_count = self._count_min
        else:
            self._button_count += 1
        self._publish_button_count()
        self.get_logger().info(f'button_count={self._button_count}')

    def _publish_button_count(self):
        msg = Int32()
        msg.data = self._button_count
        self._button_pub.publish(msg)

    def shutdown_gpio(self):
        try:
            self._pump.off()
            self._pump.close()
            self._button.close()
        except Exception as exc:
            self.get_logger().warn(f'GPIO cleanup failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PokerGpioNode()
        rclpy.spin(node)
    except Exception as exc:
        print(f'poker_gpio failed: {exc}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.shutdown_gpio()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""ROS 2 GPIO node for the table-side button + suction pump.

Ported from ROS 1 (rospy) original on origin/feature/arm_controller commit 8a3ccf5
(authored by addy-boop). Targets ROS 2 Jazzy + Raspberry Pi GPIO.

Behaviour:
    - Pin 27 (BUTTON_PIN, RISING edge): increments a 1..num_players cycling counter,
      publishes Int32 to /button_count. The bridge in services/table_io_bridge.py
      maps this to a GameState.next_turn() call.
    - Pin 17 (PUMP_PIN): subscribes to /pump_control (Int32, 1=ON, 0=OFF).
      Wiring is active-low (relay/MOSFET module): GPIO LOW energises the pump,
      GPIO HIGH releases it. The bridge publishes 1, then 0 after pump
      settle delay.

ROS 2 parameters:
    num_players (int, default 3): cycle range for the button counter.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    _GPIO_AVAILABLE = False


PUMP_PIN = 17
BUTTON_PIN = 27


class PumpTestNode(Node):
    def __init__(self):
        super().__init__('pump_test')
        self.declare_parameter('num_players', 3)
        self.num_players = self.get_parameter('num_players').value

        self.button_count = 0
        self.button_pub = self.create_publisher(Int32, '/button_count', 10)
        self.create_subscription(Int32, '/pump_control', self._on_pump_control, 10)

        if _GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(PUMP_PIN, GPIO.OUT)
            GPIO.output(PUMP_PIN, GPIO.HIGH)
            GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(
                BUTTON_PIN, GPIO.RISING,
                callback=self._on_button_pressed, bouncetime=300)
            self.get_logger().info(
                f"GPIO pump/button node started (num_players={self.num_players})")
        else:
            self.get_logger().warn(
                "RPi.GPIO unavailable - pump_test running in stub mode (no hardware).")

    def _on_button_pressed(self, channel):
        self.button_count = (self.button_count % self.num_players) + 1
        msg = Int32()
        msg.data = self.button_count
        self.button_pub.publish(msg)
        self.get_logger().info(f"Button pressed. Count = {self.button_count}")

    def _on_pump_control(self, msg: Int32):
        if not _GPIO_AVAILABLE:
            self.get_logger().info(f"[stub] pump_control = {msg.data}")
            return
        if msg.data == 1:
            GPIO.output(PUMP_PIN, GPIO.LOW)
            self.get_logger().info("Pump ON")
        elif msg.data == 0:
            GPIO.output(PUMP_PIN, GPIO.HIGH)
            self.get_logger().info("Pump OFF")
        else:
            self.get_logger().warn(f"Use only 1 or 0 for pump control (got {msg.data})")

    def destroy_node(self):
        if _GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PumpTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

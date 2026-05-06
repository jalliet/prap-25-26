import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from poker_interfaces.action import MoveJoints, MovePose
from std_msgs.msg import Int32


# Pose x per button slot along the dealer line (left-to-right or fixed mount order).
_CARD_X = {1: 0.11, 2: 0.065, 3: 0.0, 4: -0.065, 5: -0.11}


class PickDemoNode(Node):
    """Startup homes once, runs min-slot card sequence; /button_count edges run pick slots 1..max."""

    def __init__(self):
        super().__init__('pick_demo')

        self.declare_parameter('pump_on_settle_sec', 0.5)
        self.declare_parameter('pump_off_settle_sec', 0.5)
        self.declare_parameter('action_server_timeout_sec', 120.0)
        self.declare_parameter('button_count_min', 1)
        self.declare_parameter('button_count_max', 5)

        self._pump_on_settle_sec = (
            float(self.get_parameter('pump_on_settle_sec').value))
        self._pump_off_settle_sec = (
            float(self.get_parameter('pump_off_settle_sec').value))

        self._executor = None
        self._server_timeout = float(self.get_parameter(
            'action_server_timeout_sec').value)

        self._mutex = threading.Lock()
        self._busy = False
        self._startup_done = threading.Event()
        self._last_count = None

        self._btn_min = int(self.get_parameter('button_count_min').value)
        self._btn_max = int(self.get_parameter('button_count_max').value)
        self._edge_pick_slots = tuple(
            range(self._btn_min, self._btn_max + 1))

        self._clients_joints = ActionClient(self, MoveJoints, '/move_joints')
        self._clients_pose = ActionClient(self, MovePose, '/move_pose')
        self._pump_pub = self.create_publisher(Int32, '/pump_control', 10)
        self._button_sub = self.create_subscription(
            Int32, '/button_count', self._on_button, 10)

    def set_executor(self, executor: MultiThreadedExecutor) -> None:
        self._executor = executor

    def begin_startup(self) -> None:
        threading.Thread(target=self._startup_worker, daemon=True).start()

    def _startup_worker(self):
        ok = False
        try:
            with self._mutex:
                if self._busy:
                    self.get_logger().warn('startup skipped: demo already busy')
                    return
                self._busy = True
            if not self._wait_for_servers():
                return
            if not self._send_home():
                return
            if not self._pick_sequence(slot=self._btn_min, long_intro=True):
                return
            ok = True
        finally:
            with self._mutex:
                self._busy = False
            self._startup_done.set()
            if ok:
                self.get_logger().info('startup sequence finished')
            else:
                self.get_logger().error('startup sequence aborted')

    def _on_button(self, msg: Int32):
        if self._executor is None:
            return
        v = int(msg.data)
        self.get_logger().debug(f'/button_count data={v}')

        with self._mutex:
            if not self._startup_done.is_set():
                self._last_count = v
                return
            if self._busy:
                self._last_count = v
                return
            prev = self._last_count
            self._last_count = v
            if prev is None:
                return
            trigger_pick = v != prev and v in self._edge_pick_slots

        if trigger_pick:
            threading.Thread(
                target=self._slot_worker,
                kwargs={
                    'slot': v,
                    'long_intro': v == self._btn_min,
                },
                daemon=True,
            ).start()

    def _slot_worker(self, slot: int, *, long_intro: bool = False):
        with self._mutex:
            if self._busy:
                return
            self._busy = True
        try:
            self._pick_sequence(slot=slot, long_intro=long_intro)
        finally:
            with self._mutex:
                self._busy = False

    def _require_executor(self):
        if self._executor is None:
            raise RuntimeError('executor not configured (internal error)')

    def _spin_until(self, fut, timeout_sec=None):
        self._require_executor()
        return rclpy.spin_until_future_complete(
            self,
            fut,
            executor=self._executor,
            timeout_sec=timeout_sec,
        )

    def _wait_for_servers(self):
        deadline = time.monotonic() + self._server_timeout
        clients = [(self._clients_joints, 'move_joints'),
                   (self._clients_pose, 'move_pose')]
        for client, name in clients:
            while rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.get_logger().error(
                        f'{name} server not reachable after timeout')
                    return False
                if client.wait_for_server(timeout_sec=min(1.0, remaining)):
                    break
                self.get_logger().info(f'waiting for /{name} ...')
            self.get_logger().info(f'/ {name} available')
        return True

    def _send_goal_joints(self, joints, duration):
        goal = MoveJoints.Goal()
        goal.joints = list(float(x) for x in joints)
        goal.duration = float(duration)
        return self._send_goal(self._clients_joints, goal)

    def _send_goal_pose(self, x, y, z, pitch, roll, duration):
        goal = MovePose.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.pitch = float(pitch)
        goal.roll = float(roll)
        goal.duration = float(duration)
        return self._send_goal(self._clients_pose, goal)

    def _send_goal(self, client, goal):
        send_future = client.send_goal_async(goal)
        self._spin_until(send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('move goal rejected or failed to send')
            return False
        result_future = gh.get_result_async()
        self._spin_until(result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('empty action result')
            return False
        res = wrapped.result
        ok = getattr(res, 'success', False)
        if not ok:
            err = getattr(res, 'final_error', '?')
            self.get_logger().error(
                f'move failed success=False final_error={err}')
        return bool(ok)

    def _send_home(self):
        self.get_logger().info('homing...')
        zeros = [0.0] * 6
        return self._send_goal_joints(zeros, duration=10.0)

    def _pump(self, on: bool):
        msg = Int32()
        msg.data = 1 if on else 0
        self._pump_pub.publish(msg)
        delay = (
            self._pump_on_settle_sec if on else self._pump_off_settle_sec)
        time.sleep(delay)

    def _pick_sequence(self, slot: int, *, long_intro: bool) -> bool:
        card_x = _CARD_X.get(slot)
        if card_x is None:
            self.get_logger().warn(f'bad slot {slot}; expected 1..5')
            return False

        d_pick = 10.0 if long_intro else 5.0
        d_other = 5.0

        self.get_logger().info(f'slot {slot} pick low')
        if not self._send_goal_pose(
                card_x, 0.265, 0.057, 0.0, 0.0, duration=d_pick):
            return False

        self.get_logger().info(f'slot {slot} pump ON')
        self._pump(True)

        self.get_logger().info(f'slot {slot} lift')
        if not self._send_goal_pose(
                card_x, 0.265, 0.15, 0.0, 0.0, duration=d_other):
            return False

        self.get_logger().info(f'slot {slot} flip')
        if not self._send_goal_pose(
                card_x, 0.265, 0.2, -0.4, 3.13, duration=d_other):
            return False

        self.get_logger().info(f'slot {slot} drop')
        if not self._send_goal_pose(
                card_x, 0.2, 0.15, -0.4, 3.13, duration=d_other):
            return False

        self.get_logger().info(f'slot {slot} pump OFF')
        self._pump(False)

        self.get_logger().info(f'slot {slot} pose restore')
        if not self._send_goal_pose(
                card_x, 0.2, 0.15, 0.0, 0.0, duration=d_other):
            return False

        self.get_logger().info(f'slot {slot} sequence OK')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickDemoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.set_executor(executor)

    node.begin_startup()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import threading
import time
from collections import deque

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

        # Startup / homing: Cartesian via /move_pose by default; use home_mode:=joints for rad list.
        self.declare_parameter('home_mode', 'pose')
        self.declare_parameter('home_pose_x', 0.11)
        self.declare_parameter('home_pose_y', 0.265)
        self.declare_parameter('home_pose_z', 0.15)
        self.declare_parameter('home_pose_pitch', 0.0)
        self.declare_parameter('home_pose_roll', 0.0)
        self.declare_parameter('home_pose_duration_sec', 10.0)
        self.declare_parameter(
            'home_joint_positions',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('home_joint_duration_sec', 10.0)

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
        self._pick_queue = deque()

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

    def run_startup_blocking(self) -> bool:
        """Run homing + first slot on caller thread before executor.spin() main loop."""
        ok = False
        try:
            with self._mutex:
                self._busy = True
            if not self._wait_for_servers():
                return False
            if not self._send_home():
                return False
            if not self._pick_sequence(slot=self._btn_min, long_intro=True):
                return False
            ok = True
            return True
        finally:
            with self._mutex:
                self._busy = False
            self._startup_done.set()
            if ok:
                self.get_logger().info('startup sequence finished')
            else:
                self.get_logger().error('startup sequence aborted')

    def drain_scheduled_pick(self) -> None:
        """Call from main loop (same thread as spin_once waits) to process one queued pick."""
        with self._mutex:
            if self._busy or not self._startup_done.is_set() or not self._pick_queue:
                return
            slot, intro = self._pick_queue.popleft()
            self._busy = True

        try:
            self._pick_sequence(slot=slot, long_intro=intro)
        finally:
            with self._mutex:
                self._busy = False

    def _on_button(self, msg: Int32):
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
            if not (v != prev and v in self._edge_pick_slots):
                return
            self._pick_queue.append(
                (v, v == self._btn_min),
            )

    def _require_executor(self):
        if self._executor is None:
            raise RuntimeError('executor not configured (internal error)')

    def _wait_future(self, future, timeout_sec=None) -> bool:
        """Drain executor callbacks until future completes (never use spin_until_future_complete
        on the same executor that is spinning — Jazzy raises RuntimeError)."""
        self._require_executor()
        deadline = None if timeout_sec is None else (
            time.monotonic() + float(timeout_sec))
        while rclpy.ok() and not future.done():
            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().error('action wait timed out')
                return False
            self._executor.spin_once(timeout_sec=0.05)
        return True

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
        if not self._wait_future(send_future):
            return False
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('move goal rejected or failed to send')
            return False
        result_future = gh.get_result_async()
        if not self._wait_future(result_future):
            return False
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

    def _send_home(self) -> bool:
        self.get_logger().info('homing...')
        mode = str(self.get_parameter('home_mode').value).lower().strip()
        if mode == 'joints':
            joints = list(self.get_parameter('home_joint_positions').value)
            if len(joints) != 6:
                self.get_logger().error(
                    f'home_joint_positions must have 6 values, got {len(joints)}')
                return False
            dur = float(self.get_parameter('home_joint_duration_sec').value)
            return self._send_goal_joints(joints, duration=dur)
        return self._send_home_pose()

    def _send_home_pose(self) -> bool:
        x = float(self.get_parameter('home_pose_x').value)
        y = float(self.get_parameter('home_pose_y').value)
        z = float(self.get_parameter('home_pose_z').value)
        pitch = float(self.get_parameter('home_pose_pitch').value)
        roll = float(self.get_parameter('home_pose_roll').value)
        dur = float(self.get_parameter('home_pose_duration_sec').value)
        self.get_logger().info(
            f'home MovePose x={x} y={y} z={z} pitch={pitch} roll={roll} dur={dur}')
        return self._send_goal_pose(x, y, z, pitch, roll, duration=dur)

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
                card_x, 0.265, 0.059, 0.0, 0.0, duration=d_pick):
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
                card_x, 0.2, 0.2, -1.2, 3.13, duration=d_other):
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

    node.run_startup_blocking()

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node.drain_scheduled_pick()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

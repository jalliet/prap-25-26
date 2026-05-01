"""Tests for ``services.arm_choreographer``.

These tests exercise the sequencing logic without ROS 2 by mocking
``ArmRosBridge``. They require PySide6 because the choreographer is a
``QObject`` and uses ``Signal``; if PySide6 is not installed the tests are
skipped rather than failing the suite.
"""
from __future__ import annotations

import unittest

try:
    from PySide6.QtCore import QObject, Signal  # noqa: F401
    _PYSIDE_AVAILABLE = True
except Exception:  # pragma: no cover - environment-dependent
    _PYSIDE_AVAILABLE = False


@unittest.skipUnless(_PYSIDE_AVAILABLE, "PySide6 not installed")
class TestTableMap(unittest.TestCase):
    """``TableMap`` is pure Python; verify its setters/getters in isolation."""

    def setUp(self) -> None:
        from services.arm_choreographer import TableMap
        self.m = TableMap()

    def test_unknown_returns_none(self):
        self.assertIsNone(self.m.deck_xy())
        self.assertIsNone(self.m.pot_xy())
        self.assertIsNone(self.m.seat_xy(0))
        self.assertEqual(self.m.known_seats(), [])

    def test_set_and_get(self):
        self.m.set_deck_xy(0.1, 0.2)
        self.m.set_pot_xy(0.0, 0.0)
        self.m.set_seat_xy(0, 0.3, -0.2)
        self.m.set_seat_xy(2, 0.3, 0.2)

        self.assertEqual(self.m.deck_xy(), (0.1, 0.2))
        self.assertEqual(self.m.pot_xy(), (0.0, 0.0))
        self.assertEqual(self.m.seat_xy(0), (0.3, -0.2))
        self.assertEqual(self.m.seat_xy(2), (0.3, 0.2))
        self.assertIsNone(self.m.seat_xy(1))
        self.assertEqual(self.m.known_seats(), [0, 2])

    def test_clear_resets_state(self):
        self.m.set_deck_xy(0.1, 0.2)
        self.m.set_seat_xy(0, 0.3, -0.2)
        self.m.clear()
        self.assertIsNone(self.m.deck_xy())
        self.assertEqual(self.m.known_seats(), [])

    def test_int_coercion_for_seat_keys(self):
        """``set_seat_xy(0)`` and ``seat_xy(0)`` should agree even if a caller
        accidentally passes a numeric string."""
        self.m.set_seat_xy("3", 0.4, 0.0)  # type: ignore[arg-type]
        self.assertEqual(self.m.seat_xy(3), (0.4, 0.0))


class _FakeSignal:
    """Minimal stand-in for ``PySide6.QtCore.Signal`` for our mock bridge.

    The choreographer only uses ``move_completed.connect(slot)`` — it never
    introspects the signal — so a list of slots is enough to drive sequences
    deterministically from the tests.
    """

    def __init__(self) -> None:
        self._slots = []

    def connect(self, slot) -> None:
        self._slots.append(slot)

    def emit(self, *args) -> None:
        for slot in list(self._slots):
            slot(*args)


class _FakeBridge:
    """In-memory mock of ``ArmRosBridge``.

    Records every ``move_pose`` / ``move_joints`` call. Calling
    ``ack(success)`` simulates the controller finishing the most recent
    move, which is how the real bridge advances the choreographer through
    its sequence (via ``move_completed``).
    """

    def __init__(self) -> None:
        self.move_completed = _FakeSignal()
        self.calls: list[tuple] = []
        self.cancel_count: int = 0

    def move_pose(self, x, y, z, pitch, roll, duration):
        self.calls.append(("pose", (x, y, z, pitch, roll), duration))

    def move_joints(self, joints, duration):
        self.calls.append(("joints", tuple(joints), duration))

    def cancel_move(self):
        self.cancel_count += 1

    def ack(self, success: bool = True, final_error: float = 0.0):
        """Simulate the controller completing the most recent move."""
        self.move_completed.emit(success, final_error)


@unittest.skipUnless(_PYSIDE_AVAILABLE, "PySide6 not installed")
class TestArmChoreographer(unittest.TestCase):
    """End-to-end behaviour of the sequencer with a mocked bridge."""

    def _make(self):
        from services.arm_choreographer import ArmChoreographer, TableMap
        bridge = _FakeBridge()
        tm = TableMap()
        choreo = ArmChoreographer(bridge, table_map=tm)
        return bridge, tm, choreo

    # --- rejection cases (no XY data, busy guard) ------------------------

    def test_pick_up_deck_without_xy_is_rejected(self):
        bridge, _tm, choreo = self._make()
        finished: list[tuple] = []
        choreo.sequence_finished.connect(
            lambda n, ok: finished.append((n, ok)))

        accepted = choreo.pick_up_deck()
        self.assertFalse(accepted)
        self.assertEqual(bridge.calls, [])
        self.assertEqual(finished, [("pick_up_deck", False)])
        self.assertFalse(choreo.is_busy)

    def test_deal_card_unknown_seat_is_rejected(self):
        bridge, _tm, choreo = self._make()
        finished: list[tuple] = []
        choreo.sequence_finished.connect(
            lambda n, ok: finished.append((n, ok)))

        accepted = choreo.deal_card_to_seat(7)
        self.assertFalse(accepted)
        self.assertEqual(bridge.calls, [])
        self.assertEqual(finished, [("deal_card_to_seat_7", False)])

    def test_collect_pot_without_pot_xy_is_rejected(self):
        bridge, _tm, choreo = self._make()
        accepted = choreo.collect_pot()
        self.assertFalse(accepted)
        self.assertEqual(bridge.calls, [])

    # --- happy paths ------------------------------------------------------

    def test_home_dispatches_one_joint_move(self):
        bridge, _tm, choreo = self._make()
        accepted = choreo.home(duration=1.5)
        self.assertTrue(accepted)
        self.assertTrue(choreo.is_busy)
        self.assertEqual(len(bridge.calls), 1)
        kind, args, duration = bridge.calls[0]
        self.assertEqual(kind, "joints")
        self.assertEqual(len(args), 6)
        self.assertEqual(duration, 1.5)

        bridge.ack(True)
        self.assertFalse(choreo.is_busy)

    def test_deal_card_runs_three_pose_steps_in_order(self):
        bridge, tm, choreo = self._make()
        tm.set_seat_xy(2, 0.30, 0.20)

        starts: list[str] = []
        steps: list[tuple] = []
        finished: list[tuple] = []
        choreo.sequence_started.connect(starts.append)
        choreo.sequence_step.connect(lambda n, i: steps.append((n, i)))
        choreo.sequence_finished.connect(
            lambda n, ok: finished.append((n, ok)))

        choreo.deal_card_to_seat(2)
        self.assertEqual(starts, ["deal_card_to_seat_2"])

        # Step 1 of 3: hover above the drop point.
        self.assertEqual(len(bridge.calls), 1)
        kind, args, _ = bridge.calls[0]
        self.assertEqual(kind, "pose")
        self.assertEqual(args[0], 0.30)
        self.assertEqual(args[1], 0.20)
        # z = drop_z + hover_dz
        self.assertAlmostEqual(
            args[2], choreo.config.drop_z + choreo.config.hover_dz)
        bridge.ack(True)

        # Step 2 of 3: descend to drop_z.
        self.assertEqual(len(bridge.calls), 2)
        _, args, _ = bridge.calls[1]
        self.assertAlmostEqual(args[2], choreo.config.drop_z)
        bridge.ack(True)

        # Step 3 of 3: lift back up.
        self.assertEqual(len(bridge.calls), 3)
        _, args, _ = bridge.calls[2]
        self.assertAlmostEqual(
            args[2], choreo.config.drop_z + choreo.config.hover_dz)
        bridge.ack(True)

        # Sequence terminated successfully on the third ack.
        self.assertEqual(steps, [
            ("deal_card_to_seat_2", 0),
            ("deal_card_to_seat_2", 1),
            ("deal_card_to_seat_2", 2),
        ])
        self.assertEqual(finished, [("deal_card_to_seat_2", True)])
        self.assertFalse(choreo.is_busy)

    def test_pick_up_deck_uses_deck_xy_from_table_map(self):
        bridge, tm, choreo = self._make()
        tm.set_deck_xy(0.123, -0.456)
        choreo.pick_up_deck()
        self.assertEqual(len(bridge.calls), 1)
        _, (x, y, _z, _p, _r), _ = bridge.calls[0]
        self.assertEqual((x, y), (0.123, -0.456))

    # --- failure / cancel paths ------------------------------------------

    def test_failed_move_aborts_remaining_steps(self):
        bridge, tm, choreo = self._make()
        tm.set_seat_xy(0, 0.30, -0.20)
        finished: list[tuple] = []
        choreo.sequence_finished.connect(
            lambda n, ok: finished.append((n, ok)))

        choreo.deal_card_to_seat(0)
        self.assertEqual(len(bridge.calls), 1)

        bridge.ack(False, 99.9)  # First step "fails".

        self.assertEqual(finished, [("deal_card_to_seat_0", False)])
        self.assertFalse(choreo.is_busy)
        # No further steps were dispatched after the failure.
        self.assertEqual(len(bridge.calls), 1)

    def test_busy_rejects_concurrent_sequence(self):
        bridge, tm, choreo = self._make()
        tm.set_seat_xy(0, 0.30, -0.20)
        tm.set_seat_xy(1, 0.35, 0.0)
        finished: list[tuple] = []
        choreo.sequence_finished.connect(
            lambda n, ok: finished.append((n, ok)))

        choreo.deal_card_to_seat(0)
        # Second call while still busy — should be rejected.
        accepted = choreo.deal_card_to_seat(1)
        self.assertFalse(accepted)
        self.assertEqual(finished, [("deal_card_to_seat_1", False)])
        self.assertTrue(choreo.is_busy)

    def test_cancel_clears_state_and_calls_bridge(self):
        bridge, tm, choreo = self._make()
        tm.set_deck_xy(0.1, 0.0)
        choreo.pick_up_deck()
        self.assertTrue(choreo.is_busy)

        choreo.cancel()
        # cancel_move is best-effort; the controller will eventually emit
        # move_completed(False) which finalises the sequence.
        self.assertEqual(bridge.cancel_count, 1)
        bridge.ack(False, -1.0)
        self.assertFalse(choreo.is_busy)


if __name__ == "__main__":
    unittest.main()

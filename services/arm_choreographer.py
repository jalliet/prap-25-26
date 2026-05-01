"""
Arm choreography for the poker dealing robot.

This module owns the *what* of dealing — multi-step sequences and their
sequencing logic — while ``services.arm_ros_bridge.ArmRosBridge`` handles
the *how* (action client plumbing to the ROS 2 controller).

Coordinate sources
------------------
The (x, y) ground positions of the deck, pot and each seat come from the
computer-vision pipeline at runtime; they are *not* hard-coded constants
here. Z height, pitch and roll come from a fixed ``TableConfig`` derived
from table geometry. The choreographer reads positions through a
``TableMap`` abstraction so that:

A "sequence" is an ordered list of ``Step`` records. The choreographer
dispatches one step at a time and advances on the bridge's
``move_completed`` signal so the Qt event loop is never blocked. If any
step fails, the rest is aborted and ``sequence_finished`` is emitted
with success=False.

Coordinate convention (matches ``poker_control.controller_node`` IK):
    x, y, z   : metres, base frame
    pitch,roll: radians, end-effector orientation
    duration  : seconds for the trajectory
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from PySide6.QtCore import QObject, Signal

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

# A pose is (x, y, z, pitch, roll). All values metres / radians, base frame.
Pose = Tuple[float, float, float, float, float]
# Joint vector ordered q1..q6 in radians.
JointVec = Tuple[float, float, float, float, float, float]
XY = Tuple[float, float]


@dataclass(frozen=True)
class TableConfig:
    """Fixed geometric parameters of the playing surface.

    All values are placeholders pending real measurement; only the *fixed*
    parameters live here. Per-feature XY coordinates come from CV at
    runtime via ``TableMap``.
    """
    pickup_z: float = 0.05      # TODO: calibrate (m) — height to grasp the deck.
    drop_z: float = 0.05        # TODO: calibrate (m) — height to release a card.
    pot_z: float = 0.08         # TODO: calibrate (m) — pot working height.
    pitch: float = 0.0          # rad — neutral end-effector pitch.
    roll: float = 0.0           # rad — neutral end-effector roll.
    hover_dz: float = 0.07      # m   — extra clearance for the hover pose.
    flip_roll: float = math.pi  # rad — TODO: calibrate flip-card wrist roll.


# Default duration for any single move in seconds.
DEFAULT_DURATION_S: float = 2.5

# TODO: calibrate — joint vector for the safe parking / home pose.
HOME_JOINTS: JointVec = (0.0, -0.3, 0.6, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Table map — runtime XY positions, populated by the CV pipeline
# ---------------------------------------------------------------------------

class TableMap:
    """In-memory store of CV-derived XY positions.

    All getters return ``None`` when the corresponding feature has not yet
    been observed. The CV controller is expected to call the ``set_*``
    methods whenever it produces a fresh measurement; the choreographer
    only reads.

    This class is intentionally framework-agnostic (no Qt, no ROS) so it
    can be unit-tested in isolation and the CV team can populate it from
    whatever thread their pipeline runs on. Use a lock externally if you
    expect concurrent writes from non-Qt threads.
    """

    def __init__(self) -> None:
        self._deck_xy: Optional[XY] = None
        self._pot_xy: Optional[XY] = None
        self._seat_xy: dict[int, XY] = {}

    # --- setters (CV pipeline -> map) -------------------------------------

    def set_deck_xy(self, x: float, y: float) -> None:
        self._deck_xy = (float(x), float(y))

    def set_pot_xy(self, x: float, y: float) -> None:
        self._pot_xy = (float(x), float(y))

    def set_seat_xy(self, seat: int, x: float, y: float) -> None:
        self._seat_xy[int(seat)] = (float(x), float(y))

    def clear(self) -> None:
        self._deck_xy = None
        self._pot_xy = None
        self._seat_xy.clear()

    # --- getters (choreographer -> map) -----------------------------------

    def deck_xy(self) -> Optional[XY]:
        return self._deck_xy

    def pot_xy(self) -> Optional[XY]:
        return self._pot_xy

    def seat_xy(self, seat: int) -> Optional[XY]:
        return self._seat_xy.get(int(seat))

    def known_seats(self) -> List[int]:
        return sorted(self._seat_xy)


# ---------------------------------------------------------------------------
# Step / Sequence data structures
# ---------------------------------------------------------------------------

@dataclass
class Step:
    """A single command dispatched to the arm bridge.

    ``kind`` is either ``'pose'`` or ``'joints'``. For ``'pose'`` ``args``
    is a 5-tuple ``(x, y, z, pitch, roll)``; for ``'joints'`` ``args`` is
    a 6-tuple of joint angles in radians. ``duration`` is seconds.
    """
    kind: str
    args: Tuple[float, ...]
    duration: float = DEFAULT_DURATION_S


@dataclass
class Sequence:
    """A named ordered list of steps."""
    name: str
    steps: List[Step] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Choreographer
# ---------------------------------------------------------------------------

class ArmChoreographer(QObject):
    """Sequencer that drives ``ArmRosBridge`` through multi-step routines.

    The choreographer is non-blocking. Each public action method enqueues
    a ``Sequence`` and returns immediately. Progress is reported via Qt
    signals.

    Only one sequence may run at a time. Calls received while busy are
    rejected (logged + ``sequence_finished`` emitted with success=False),
    matching the controller node's "reject during tracking" policy. Use
    ``cancel()`` to stop the current sequence.
    """

    # name -- Sequence accepted and first step dispatched.
    sequence_started = Signal(str)
    # name, step_index -- a step within the sequence completed successfully.
    sequence_step = Signal(str, int)
    # name, success -- the entire sequence finished (or aborted on failure).
    sequence_finished = Signal(str, bool)

    def __init__(
        self,
        bridge,
        table_map: Optional[TableMap] = None,
        config: Optional[TableConfig] = None,
        parent: Optional[QObject] = None,
    ) -> None:
        """Args:
            bridge: an ``ArmRosBridge`` instance (shared singleton in production).
            table_map: runtime XY source. Defaults to a fresh empty ``TableMap``.
            config: fixed table geometry. Defaults to placeholder ``TableConfig``.
            parent: standard Qt parent.
        """
        super().__init__(parent)
        self._bridge = bridge
        self.table_map: TableMap = table_map if table_map is not None else TableMap()
        self.config: TableConfig = config if config is not None else TableConfig()

        self._current: Optional[Sequence] = None
        self._step_idx: int = 0
        self._cancelled: bool = False

        # Bridge feeds us per-move success/failure on this signal.
        self._bridge.move_completed.connect(self._on_move_completed)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_busy(self) -> bool:
        """True while a sequence is in flight."""
        return self._current is not None

    # ------------------------------------------------------------------
    # Pose construction helpers
    # ------------------------------------------------------------------

    def _pose(self, x: float, y: float, z: float) -> Pose:
        """Compose an (x, y, z, pitch, roll) pose using the configured
        neutral orientation."""
        return (x, y, z, self.config.pitch, self.config.roll)

    def _hover(self, pose: Pose) -> Pose:
        """Return ``pose`` lifted by ``config.hover_dz`` in z."""
        x, y, z, p, r = pose
        return (x, y, z + self.config.hover_dz, p, r)

    # ------------------------------------------------------------------
    # Public API — high-level routines
    # ------------------------------------------------------------------

    def home(self, duration: float = DEFAULT_DURATION_S) -> bool:
        """Move the arm to ``HOME_JOINTS``.

        Returns False immediately if the choreographer is already busy.
        """
        return self._run(Sequence(
            name="home",
            steps=[Step(kind="joints", args=HOME_JOINTS, duration=duration)],
        ))

    def pick_up_deck(self) -> bool:
        """Hover over the deck, descend to grasp, lift back up.

        Requires ``table_map.deck_xy()`` to be populated by the CV pipeline.
        """
        xy = self.table_map.deck_xy()
        if xy is None:
            return self._reject("pick_up_deck", "deck_xy unset")
        pickup = self._pose(xy[0], xy[1], self.config.pickup_z)
        return self._run(Sequence(
            name="pick_up_deck",
            steps=[
                Step(kind="pose", args=self._hover(pickup)),
                Step(kind="pose", args=pickup),
                # TODO: insert gripper-close primitive once available.
                Step(kind="pose", args=self._hover(pickup)),
            ],
        ))

    def deal_card_to_seat(self, seat: int) -> bool:
        """Carry the held card to the seat's drop pose, release, retreat.

        Args:
            seat: integer seat index matching ``poker.player.Player.seat``.

        Requires ``table_map.seat_xy(seat)`` to be populated.
        """
        xy = self.table_map.seat_xy(seat)
        if xy is None:
            return self._reject(
                f"deal_card_to_seat_{seat}",
                f"seat_xy({seat}) unset (known: {self.table_map.known_seats()})",
            )
        drop = self._pose(xy[0], xy[1], self.config.drop_z)
        return self._run(Sequence(
            name=f"deal_card_to_seat_{seat}",
            steps=[
                Step(kind="pose", args=self._hover(drop)),
                Step(kind="pose", args=drop),
                # TODO: insert gripper-open primitive once available.
                Step(kind="pose", args=self._hover(drop)),
            ],
        ))

    def flip_card(self, seat: int) -> bool:
        """Flip the card already lying at ``seat``.

        Placeholder kinematics: descends with a roll offset, then ascends.
        A real flip primitive will need a dedicated wrist motion (TODO).
        Requires ``table_map.seat_xy(seat)`` to be populated.
        """
        xy = self.table_map.seat_xy(seat)
        if xy is None:
            return self._reject(
                f"flip_card_{seat}",
                f"seat_xy({seat}) unset (known: {self.table_map.known_seats()})",
            )
        base = self._pose(xy[0], xy[1], self.config.drop_z)
        flipped: Pose = (xy[0], xy[1], self.config.drop_z,
                         self.config.pitch, self.config.flip_roll)
        return self._run(Sequence(
            name=f"flip_card_{seat}",
            steps=[
                Step(kind="pose", args=self._hover(base)),
                Step(kind="pose", args=flipped),
                Step(kind="pose", args=self._hover(base)),
            ],
        ))

    def collect_pot(self) -> bool:
        """Move chips from the centre of the table to the pot pose, then home.

        Placeholder: drives to the pot pose and back to home. The actual
        sweep / scoop primitive depends on the gripper choice (TODO).
        Requires ``table_map.pot_xy()`` to be populated.
        """
        xy = self.table_map.pot_xy()
        if xy is None:
            return self._reject("collect_pot", "pot_xy unset")
        pot = self._pose(xy[0], xy[1], self.config.pot_z)
        return self._run(Sequence(
            name="collect_pot",
            steps=[
                Step(kind="pose", args=self._hover(pot)),
                Step(kind="pose", args=pot),
                Step(kind="pose", args=self._hover(pot)),
                Step(kind="joints", args=HOME_JOINTS),
            ],
        ))

    def cancel(self) -> None:
        """Cancel any in-flight sequence. The current arm move is also
        cancelled via the bridge if it has a goal handle."""
        if self._current is None:
            return
        logger.info(
            "ArmChoreographer: cancelling sequence '%s' at step %d",
            self._current.name, self._step_idx)
        self._cancelled = True
        try:
            self._bridge.cancel_move()
        except Exception as e:  # pragma: no cover - defensive
            logger.warning("ArmChoreographer: cancel_move raised %s", e)

    # ------------------------------------------------------------------
    # Internal sequencing
    # ------------------------------------------------------------------

    def _reject(self, name: str, reason: str) -> bool:
        """Emit a synchronous failure for sequences that cannot start."""
        logger.warning("ArmChoreographer: '%s' rejected: %s", name, reason)
        self.sequence_finished.emit(name, False)
        return False

    def _run(self, seq: Sequence) -> bool:
        """Begin executing ``seq``. Returns True on dispatch, False if busy."""
        if self.is_busy:
            return self._reject(
                seq.name,
                f"busy with '{self._current.name if self._current else '?'}'",
            )
        if not seq.steps:
            logger.warning(
                "ArmChoreographer: '%s' has no steps; skipping", seq.name)
            self.sequence_finished.emit(seq.name, True)
            return True

        self._current = seq
        self._step_idx = 0
        self._cancelled = False
        self.sequence_started.emit(seq.name)
        logger.info(
            "ArmChoreographer: starting sequence '%s' (%d steps)",
            seq.name, len(seq.steps))
        self._dispatch_current_step()
        return True

    def _dispatch_current_step(self) -> None:
        """Send the step at ``self._step_idx`` to the bridge."""
        assert self._current is not None
        step = self._current.steps[self._step_idx]
        if step.kind == "pose":
            if len(step.args) != 5:
                logger.error(
                    "ArmChoreographer: '%s' step %d has bad pose arity %d",
                    self._current.name, self._step_idx, len(step.args))
                self._finish(False)
                return
            x, y, z, pitch, roll = step.args
            self._bridge.move_pose(x, y, z, pitch, roll, step.duration)
        elif step.kind == "joints":
            if len(step.args) != 6:
                logger.error(
                    "ArmChoreographer: '%s' step %d has bad joint arity %d",
                    self._current.name, self._step_idx, len(step.args))
                self._finish(False)
                return
            self._bridge.move_joints(list(step.args), step.duration)
        else:
            logger.error(
                "ArmChoreographer: '%s' step %d unknown kind %r",
                self._current.name, self._step_idx, step.kind)
            self._finish(False)

    def _on_move_completed(self, success: bool, _final_error: float) -> None:
        """Handler for ``ArmRosBridge.move_completed``."""
        if self._current is None:
            return  # Not our move (or cancelled and cleaned up).

        if self._cancelled:
            self._finish(False)
            return

        if not success:
            logger.warning(
                "ArmChoreographer: '%s' aborted at step %d (move failed)",
                self._current.name, self._step_idx)
            self._finish(False)
            return

        self.sequence_step.emit(self._current.name, self._step_idx)
        self._step_idx += 1

        if self._step_idx >= len(self._current.steps):
            self._finish(True)
            return

        self._dispatch_current_step()

    def _finish(self, success: bool) -> None:
        """Emit the terminal signal and clear bookkeeping."""
        name = self._current.name if self._current else "?"
        self._current = None
        self._step_idx = 0
        self._cancelled = False
        self.sequence_finished.emit(name, success)
        logger.info(
            "ArmChoreographer: sequence '%s' finished (success=%s)",
            name, success)

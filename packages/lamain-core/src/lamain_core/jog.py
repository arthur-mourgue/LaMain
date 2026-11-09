# SPDX-License-Identifier: Apache-2.0
"""Keyboard jogging of the hand joints.

A `JogController` owns the current normalized pose and applies a fixed angular
step when a key is pressed. It always goes through `HandController`, so the
calibration bounds and safety filter apply. This module is pure logic: it takes
keys as strings, so it is testable without a keyboard.
"""
from __future__ import annotations

import math

from .controller import HandController
from .demos import Pose

# Key order follows 0..9: two keys per joint (negative / positive).
DEFAULT_KEYMAP: dict[str, tuple[str, int]] = {
    "0": ("index_flex", -1),
    "1": ("index_flex", +1),
    "2": ("middle_flex", -1),
    "3": ("middle_flex", +1),
    "4": ("index_middle_abd", -1),
    "5": ("index_middle_abd", +1),
    "6": ("thumb_rot", -1),
    "7": ("thumb_rot", +1),
    "8": ("thumb_flex", -1),
    "9": ("thumb_flex", +1),
    # Arrow keys: a dedicated pair for each thumb axis.
    "up": ("thumb_flex", +1),
    "down": ("thumb_flex", -1),
    "left": ("thumb_rot", -1),
    "right": ("thumb_rot", +1),
}

DEFAULT_STEP_DEG = 3.0


class JogController:
    def __init__(
        self,
        controller: HandController,
        step_deg: float = DEFAULT_STEP_DEG,
        keymap: dict[str, tuple[str, int]] | None = None,
    ):
        self.controller = controller
        self.step_deg = step_deg
        self.keymap = dict(keymap or DEFAULT_KEYMAP)
        self.q: dict[str, float] = dict(controller.get_normalized())

    # ------------------------------------------------------------------ #
    def _span_deg(self, joint: str, sign: int) -> float:
        return max(1e-6, self.controller.q_span_deg(joint, sign))

    def handle_key(self, key: str) -> bool:
        """Apply one jog step. Returns True if the key mapped to a joint."""
        if key not in self.keymap:
            return False
        joint, sign = self.keymap[key]
        dq = sign * self.step_deg / self._span_deg(joint, sign)
        target = self.q.get(joint, 0.0) + dq
        applied = self.controller.set_normalized({joint: target})
        span = self._span_deg(joint, 1 if applied[joint] >= 0 else -1)
        self.q[joint] = applied[joint] / math.radians(span)
        return True

    def capture(self) -> Pose:
        """Read the real joint positions and return a normalized + rad + ticks pose."""
        rad = self.controller.get_joint_positions()
        q = {}
        for name, value in rad.items():
            span = self._span_deg(name, 1 if value >= 0 else -1)
            q[name] = value / math.radians(span)
        self.q = dict(q)
        ticks = {
            j.name: self.controller.bus.read_position(j.servo_id)
            for j in self.controller.model.joints.values()
        }
        return Pose(
            q=q, rad=rad, ticks=ticks, hand=self.controller.calibration.hand_serial
        )

    def goto(self, pose: Pose) -> None:
        """Move the hand to a stored pose (e.g. a gesture) and resync state."""
        self.controller.set_normalized(dict(pose.q))
        self.q = dict(pose.q)

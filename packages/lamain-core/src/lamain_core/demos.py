# SPDX-License-Identifier: Apache-2.0
"""Gestures, episodes, and playback.

- A `Gesture` is a single named pose (reusable brick).
- An `Episode` is an ordered list of `Step`s, each being either a plain
  keypoint or a gesture *snapshot* (the resolved pose is stored, so the episode
  keeps playing even if the gesture is later renamed/deleted).
- The `Player` interpolates between poses and drives the hand through
  `HandController` (bounds + safety always enforced).
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from . import units
from .controller import HandController


# --------------------------------------------------------------------------- #
# Data model
# --------------------------------------------------------------------------- #
@dataclass
class Pose:
    q: dict[str, float]              # normalized [-1, 1] (portable across hands)
    rad: dict[str, float] = field(default_factory=dict)  # direct joint radians
    ticks: dict[str, int] = field(default_factory=dict)  # absolute servo ticks (same hand)
    hand: str = ""                   # hand serial the pose was recorded on

    def to_dict(self) -> dict:
        return {
            "q": dict(self.q),
            "rad": dict(self.rad),
            "ticks": dict(self.ticks),
            "hand": self.hand,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Pose":
        return cls(
            q=dict(data["q"]),
            rad=dict(data.get("rad", {})),
            ticks={k: int(v) for k, v in data.get("ticks", {}).items()},
            hand=data.get("hand", ""),
        )


@dataclass
class Gesture:
    name: str
    pose: Pose
    hand: str = ""
    model_version: str = ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "hand": self.hand,
            "model_version": self.model_version,
            **self.pose.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Gesture":
        return cls(
            name=data["name"],
            pose=Pose.from_dict(data),
            hand=data.get("hand", ""),
            model_version=data.get("model_version", ""),
        )


@dataclass
class Step:
    type: str                        # "keypoint" | "gesture"
    pose: Pose | None = None         # keypoint pose; None for a gesture reference
    name: str | None = None          # gesture name (resolved at play time)
    hold: float | None = None        # seconds

    def to_dict(self) -> dict:
        data = {"type": self.type, "name": self.name, "hold": self.hold}
        if self.pose is not None:
            data.update(self.pose.to_dict())
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "Step":
        pose = Pose.from_dict(data) if "q" in data else None
        return cls(
            type=data.get("type", "keypoint"),
            pose=pose,
            name=data.get("name"),
            hold=data.get("hold"),
        )


@dataclass
class Episode:
    name: str
    steps: list[Step] = field(default_factory=list)
    hand: str = ""
    model_version: str = ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "hand": self.hand,
            "model_version": self.model_version,
            "steps": [s.to_dict() for s in self.steps],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Episode":
        return cls(
            name=data["name"],
            steps=[Step.from_dict(s) for s in data.get("steps", [])],
            hand=data.get("hand", ""),
            model_version=data.get("model_version", ""),
        )


# --------------------------------------------------------------------------- #
# Stores
# --------------------------------------------------------------------------- #
class GestureStore:
    """One JSON file per gesture. Saving an existing name overwrites it."""

    def __init__(self, directory: str | Path):
        self.dir = Path(directory)

    def _path(self, name: str) -> Path:
        return self.dir / f"{name}.json"

    def list(self) -> list[str]:
        if not self.dir.exists():
            return []
        return sorted(p.stem for p in self.dir.glob("*.json"))

    def exists(self, name: str) -> bool:
        return self._path(name).exists()

    def save(self, gesture: Gesture) -> Path:
        self.dir.mkdir(parents=True, exist_ok=True)
        path = self._path(gesture.name)
        path.write_text(json.dumps(gesture.to_dict(), indent=2), encoding="utf-8")
        return path

    def load(self, name: str) -> Gesture:
        return Gesture.from_dict(json.loads(self._path(name).read_text(encoding="utf-8")))

    def rename(self, old: str, new: str) -> bool:
        if not self.exists(old) or not new or old == new:
            return self.exists(old) and bool(new)
        gesture = self.load(old)
        gesture.name = new
        self.save(gesture)
        self.delete(old)
        return True

    def delete(self, name: str) -> bool:
        path = self._path(name)
        if path.exists():
            path.unlink()
            return True
        return False


class EpisodeStore:
    """One JSON file per episode. Saving never overwrites: names get a numeric
    suffix (demo1.json, demo1_2.json, ...)."""

    def __init__(self, directory: str | Path):
        self.dir = Path(directory)

    def list(self) -> list[str]:
        if not self.dir.exists():
            return []
        return sorted(p.stem for p in self.dir.glob("*.json"))

    def next_path(self, name: str) -> Path:
        self.dir.mkdir(parents=True, exist_ok=True)
        base = self.dir / f"{name}.json"
        if not base.exists():
            return base
        n = 2
        while True:
            candidate = self.dir / f"{name}_{n}.json"
            if not candidate.exists():
                return candidate
            n += 1

    def save(self, episode: Episode) -> Path:
        path = self.next_path(episode.name)
        episode.name = path.stem
        path.write_text(json.dumps(episode.to_dict(), indent=2), encoding="utf-8")
        return path

    def write(self, episode: Episode) -> Path:
        """Overwrite `demos/episodes/<name>.json` (used when editing)."""
        self.dir.mkdir(parents=True, exist_ok=True)
        path = self.dir / f"{episode.name}.json"
        path.write_text(json.dumps(episode.to_dict(), indent=2), encoding="utf-8")
        return path

    def load(self, name: str) -> Episode:
        path = self.dir / f"{name}.json"
        return Episode.from_dict(json.loads(path.read_text(encoding="utf-8")))

    def rename(self, old: str, new: str) -> bool:
        if not (self.dir / f"{old}.json").exists() or not new:
            return False
        if old == new:
            return True
        episode = self.load(old)
        episode.name = new
        self.write(episode)
        (self.dir / f"{old}.json").unlink()
        return True

    def delete(self, name: str) -> bool:
        path = self.dir / f"{name}.json"
        if path.exists():
            path.unlink()
            return True
        return False


# --------------------------------------------------------------------------- #
# Playback
# --------------------------------------------------------------------------- #
class PlayerStopped(RuntimeError):
    pass


@dataclass
class PlayOptions:
    fps: int = 100
    speed: float = 2.0           # playback rate multiplier
    default_step_s: float = 0.15
    min_step_s: float = 0.05     # floor for a segment duration
    settle_timeout_s: float = 2.0
    loops: int = 0               # 0 = infinite
    max_step_deg: float | None = None
    approach: bool = True        # glide to the first pose instead of jumping
    start_at_zero: bool = True   # always begin at the zero pose (q = 0)
    monitor: bool = True         # watch temperature / voltage and stop if bad


def lerp(a: dict[str, float], b: dict[str, float], t: float) -> dict[str, float]:
    return {k: a[k] + (b[k] - a[k]) * t for k in a}


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class Player:
    def __init__(self, controller: HandController, options: PlayOptions | None = None):
        self.controller = controller
        self.options = options or PlayOptions()

    # ------------------------------------------------------------------ #
    def _sleep(self, seconds: float) -> None:
        if self.controller.bus.simulated:
            return
        time.sleep(seconds / max(1e-3, self.options.speed))

    def _health(self) -> None:
        for j in self.controller.model.joints.values():
            temp = self.controller.bus.read_temperature(j.servo_id)
            volt = self.controller.bus.read_voltage(j.servo_id)
            if self.controller.safety.check_health(temp, volt, 0) == "stop":
                raise PlayerStopped(
                    f"servo {j.servo_id}: unhealthy (temp={temp} C, V={volt:.1f})"
                )

    def _drive(self, q: dict[str, float], frame: int) -> None:
        self.controller.set_normalized(q)
        if self.options.monitor and frame % max(1, self.options.fps // 5) == 0:
            self._health()

    def _segment_duration(self, qa: dict[str, float], qb: dict[str, float]) -> float:
        """Fastest duration for the move, proportional to the distance: the
        biggest joint travel divided by its max speed."""
        speed_dps = min(
            self.controller.model.joint(n).max_speed_dps for n in qa
        ) or 120.0
        max_deg = 0.0
        for name in qa:
            max_deg = max(
                max_deg,
                abs(
                    self.controller.q_to_deg(name, qb[name])
                    - self.controller.q_to_deg(name, qa[name])
                ),
            )
        return max(self.options.min_step_s, max_deg / speed_dps)

    def _wait_settle(self, stop_flag, ticks: bool = False) -> None:
        """Wait until the hand actually reaches the commanded pose."""
        if self.controller.bus.simulated:
            return
        deadline = time.monotonic() + self.options.settle_timeout_s
        last = None
        stable = 0
        eps = 2 if ticks else 0.005
        while time.monotonic() < deadline:
            if stop_flag is not None and stop_flag():
                raise PlayerStopped("stopped")
            pos = (
                self.controller.get_joint_ticks()
                if ticks
                else self.controller.get_normalized()
            )
            if last is not None and all(abs(pos[n] - last[n]) < eps for n in pos):
                stable += 1
                if stable >= 3:
                    return
            else:
                stable = 0
            last = pos
            time.sleep(0.03)

    def _interpolate_to(self, target: dict[str, float], duration: float, stop_flag) -> None:
        start = self.controller.get_normalized()
        n = max(2, int(duration * self.options.fps))
        for k in range(n):
            if stop_flag is not None and stop_flag():
                raise PlayerStopped("stopped")
            self._drive(lerp(start, target, smoothstep(k / (n - 1))), k)
            self._sleep(1.0 / self.options.fps)
        self._wait_settle(stop_flag)

    # --- absolute-tick playback (same hand, calibration-independent) -------- #
    def _drive_ticks(self, ticks: dict[str, int], frame: int) -> None:
        self.controller.set_joint_ticks(ticks)
        if self.options.monitor and frame % max(1, self.options.fps // 5) == 0:
            self._health()

    def _ticks_duration(self, a: dict[str, int], b: dict[str, int]) -> float:
        speed_dps = min(
            self.controller.model.joint(n).max_speed_dps for n in a
        ) or 120.0
        max_deg = units.ticks_to_deg(
            max(abs(b[n] - a[n]) for n in a), self.controller.model.servo
        )
        return max(self.options.min_step_s, max_deg / speed_dps)

    def _interpolate_ticks(self, target: dict[str, int], duration: float, stop_flag) -> None:
        start = self.controller.get_joint_ticks()
        n = max(2, int(duration * self.options.fps))
        for k in range(n):
            if stop_flag is not None and stop_flag():
                raise PlayerStopped("stopped")
            self._drive_ticks(lerp(start, target, smoothstep(k / (n - 1))), k)
            self._sleep(1.0 / self.options.fps)
        self._wait_settle(stop_flag, ticks=True)

    def _use_ticks(self, poses: list[Pose]) -> bool:
        current = self.controller.calibration.hand_serial
        return all(p.ticks for p in poses) and all(
            p.hand == "" or p.hand == current for p in poses
        )

    def play(self, steps, stop_flag: Callable[[], bool] | None = None, gestures=None) -> None:
        poses: list[Pose] = []
        for s in steps:
            if isinstance(s, Pose):
                poses.append(s)
            elif s.type == "gesture" and s.pose is None:
                # resolve the gesture by name (so replacing it updates episodes)
                if gestures is None:
                    raise PlayerStopped(f"gesture '{s.name}' needs a gesture store")
                poses.append(gestures.load(s.name).pose)
            else:
                poses.append(s.pose)
        holds = [s.hold if isinstance(s, Step) else None for s in steps]
        if not poses:
            raise PlayerStopped("empty episode")

        if self.options.max_step_deg is not None:
            self.controller.set_max_step_deg(self.options.max_step_deg)
        try:
            if self._use_ticks(poses):
                self._play_ticks(poses, holds, stop_flag)
            else:
                self._play_q(poses, holds, stop_flag)
        finally:
            if self.options.max_step_deg is not None:
                self.controller.set_max_step_deg(None)

    def _play_q(self, poses: list[Pose], holds, stop_flag) -> None:
        if self.options.start_at_zero:
            zero = {name: 0.0 for name in poses[0].q}
            current = self.controller.get_normalized()
            self._interpolate_to(zero, self._segment_duration(current, zero), stop_flag)
        if self.options.approach:
            current = self.controller.get_normalized()
            self._interpolate_to(
                poses[0].q, self._segment_duration(current, poses[0].q), stop_flag
            )
        loops = 0
        while True:
            for i in range(len(poses) - 1):
                duration = (
                    holds[i]
                    if holds[i] is not None
                    else self._segment_duration(poses[i].q, poses[i + 1].q)
                )
                self._interpolate_to(poses[i + 1].q, duration, stop_flag)
            loops += 1
            if self.options.loops and loops >= self.options.loops:
                break

    def _play_ticks(self, poses: list[Pose], holds, stop_flag) -> None:
        if self.options.start_at_zero:
            zero = self.controller.mount_ticks()
            current = self.controller.get_joint_ticks()
            self._interpolate_ticks(zero, self._ticks_duration(current, zero), stop_flag)
        if self.options.approach:
            current = self.controller.get_joint_ticks()
            self._interpolate_ticks(
                poses[0].ticks, self._ticks_duration(current, poses[0].ticks), stop_flag
            )
        loops = 0
        while True:
            for i in range(len(poses) - 1):
                duration = (
                    holds[i]
                    if holds[i] is not None
                    else self._ticks_duration(poses[i].ticks, poses[i + 1].ticks)
                )
                self._interpolate_ticks(poses[i + 1].ticks, duration, stop_flag)
            loops += 1
            if self.options.loops and loops >= self.options.loops:
                break

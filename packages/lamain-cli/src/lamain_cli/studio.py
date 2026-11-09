# SPDX-License-Identifier: Apache-2.0
"""Interactive studio: jog the hand with the keyboard, save gestures and
episodes, and replay them fast for a demo video."""
from __future__ import annotations

import time
from pathlib import Path

from lamain_core import (
    Episode,
    EpisodeStore,
    Gesture,
    GestureStore,
    HandController,
    JogController,
    PlayOptions,
    Player,
    PlayerStopped,
    Pose,
    Step,
    load_calibration,
)

JOG_HELP = (
    "Jog keys (one step per press):\n"
    "  0/1 index flex   2/3 middle flex   4/5 abduction\n"
    "  6/7 thumb base   8/9 thumb flex\n"
    "  Up/Down = thumb flex   Left/Right = thumb rotation"
)


class Studio:
    def __init__(
        self,
        model,
        bus,
        calibration_path: str | Path,
        gestures: GestureStore,
        episodes: EpisodeStore,
        keys,
        step_deg: float = 3.0,
        out=print,
    ):
        self.model = model
        self.bus = bus
        self.calibration_path = Path(calibration_path)
        self.gestures = gestures
        self.episodes = episodes
        self.keys = keys
        self.step_deg = step_deg
        self.out = out

        self.calibration = load_calibration(self.calibration_path)
        self.controller = HandController(bus, model, self.calibration)
        self.jog = JogController(self.controller, step_deg=self.step_deg)

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        self._home()  # always start from the initial zero
        try:
            while True:
                self._menu()
                choice = self.keys.prompt_line("> ").strip().lower()
                if choice in ("1", "g"):
                    self._create_gesture()
                elif choice in ("2", "e"):
                    self._create_episode()
                elif choice in ("3",):
                    self._play_menu()
                elif choice in ("4",):
                    self._play_gesture_menu()
                elif choice in ("5",):
                    self._manage()
                elif choice in ("6", "q", ""):
                    break
                else:
                    self.out("Unknown choice.")
        finally:
            self.controller.disable_torque()

    def _menu(self) -> None:
        self.out("\n=== La Main Studio ===")
        self.out(f"Calibration : {self.calibration_path} "
                 f"(hand {self.calibration.hand_serial})")
        self.out("  1) Create a gesture")
        self.out("  2) Create an episode")
        self.out("  3) Play an episode")
        self.out("  4) Play a gesture")
        self.out("  5) Manage (list / edit / rename / delete)")
        self.out("  6) Quit")

    # ------------------------------------------------------------------ #
    # Human-readable labels for keys that are not printable.
    _KEY_LABELS = {
        " ": "[space]",
        "\x7f": "[backspace]",
        "\x08": "[backspace]",
        "\t": "[tab]",
    }

    @classmethod
    def _key_label(cls, key: str) -> str:
        return cls._KEY_LABELS.get(key, key)

    def _jog_loop(self, actions: dict[str, str]) -> str:
        """Read keys; digits jog, keys in `actions` are special. Returns the
        special key that ended the loop (or 'q')."""
        self.out(JOG_HELP)
        # collapse duplicate labels (DEL and BS both = backspace)
        shown = []
        for key, label in actions.items():
            text = f"{self._key_label(key)} = {label}"
            if text not in shown:
                shown.append(text)
        self.out("Special keys:  " + "   ".join(shown))
        while True:
            key = self.keys.read_key()
            if key is None:
                continue
            if key in actions:
                return key
            if self.jog.handle_key(key):
                self._status()
            else:
                self.out(f"(unused key {self._key_label(key)})")

    def _status(self) -> None:
        pose = self.jog.q
        pretty = " ".join(f"{n[:4]}={pose[n]:+.2f}" for n in self.model.joints)
        self.out(f"  q: {pretty}")

    # ------------------------------------------------------------------ #
    def _move_to_pose(self, pose: Pose) -> None:
        """Smoothly move the hand all the way to a pose (not just one step)."""
        self.controller.enable_torque("default")
        player = Player(
            self.controller,
            PlayOptions(loops=1, approach=True, start_at_zero=False, max_step_deg=90.0),
        )
        player.play([pose])

    def _home(self) -> None:
        """Bring every joint to the initial mount zero before any action."""
        self.out("  homing to the initial zero...")
        self.controller.enable_torque("default")
        pose = Pose(
            q={name: 0.0 for name in self.controller.joint_names},
            ticks=self.controller.mount_ticks(),
            hand=self.calibration.hand_serial,
        )
        player = Player(
            self.controller,
            PlayOptions(loops=1, approach=True, start_at_zero=False, max_step_deg=90.0),
        )
        player.play([pose])

    def _settle(self, timeout: float = 1.5) -> None:
        """Wait until the hand stops moving (so capture = what you see)."""
        if self.controller.bus.simulated:
            return
        deadline = time.monotonic() + timeout
        last = None
        stable = 0
        while time.monotonic() < deadline:
            pos = self.controller.get_joint_positions()
            if last is not None and all(abs(pos[n] - last[n]) < 0.005 for n in pos):
                stable += 1
                if stable >= 3:
                    return
            else:
                stable = 0
            last = pos
            time.sleep(0.03)

    def _create_gesture(self) -> None:
        self.out("\n--- Create a gesture ---")
        self._home()
        self.jog = JogController(self.controller, step_deg=self.step_deg)
        key = self._jog_loop({"s": "save", "q": "back"})
        if key != "s":
            self.controller.disable_torque()
            return
        self._settle()
        name = self.keys.prompt_line("Gesture name > ").strip()
        if not name:
            self.controller.disable_torque()
            self.out("Cancelled.")
            return
        gesture = Gesture(
            name=name,
            pose=self.jog.capture(),
            hand=self.calibration.hand_serial,
            model_version=self.model.model_version,
        )
        path = self.gestures.save(gesture)
        self.out(f"Saved gesture -> {path}")
        self.controller.disable_torque()

    # ------------------------------------------------------------------ #
    def _create_episode(self) -> None:
        self.out("\n--- Create an episode ---")
        episode = Episode(
            name="episode",
            hand=self.calibration.hand_serial,
            model_version=self.model.model_version,
        )
        self._episode_session(episode, overwrite=False)

    def _edit_episode(self, name: str) -> None:
        episode = self.episodes.load(name)
        self.out(f"\n--- Edit episode '{name}' ({len(episode.steps)} steps) ---")
        self._episode_session(episode, overwrite=True)

    def _episode_session(self, episode: Episode, overwrite: bool) -> None:
        self._home()
        self.jog = JogController(self.controller, step_deg=self.step_deg)
        actions = {
            " ": "add keypoint",
            "g": "insert gesture",
            "\x7f": "undo",
            "\x08": "undo",
            "p": "preview",
            "s": "save",
            "q": "back",
        }
        while True:
            key = self._jog_loop(actions)
            if key == " ":
                self._settle()
                episode.steps.append(Step("keypoint", self.jog.capture()))
                self.out(f"  keypoint added ({len(episode.steps)} steps)")
            elif key == "g":
                self._insert_gesture(episode)
            elif key in ("\x7f", "\x08"):
                if episode.steps:
                    episode.steps.pop()
                    self.out(f"  removed last step ({len(episode.steps)} left)")
            elif key == "p":
                self._play(episode.steps, loops=1)
            elif key == "s":
                name = self.keys.prompt_line(
                    f"Episode name > [{episode.name}] "
                ).strip() or episode.name
                episode.name = name
                if overwrite:
                    path = self.episodes.write(episode)
                else:
                    path = self.episodes.save(episode)
                self.out(f"Saved episode -> {path}")
                break
            elif key == "q":
                break
        self.controller.disable_torque()

    def _edit_gesture(self, name: str) -> None:
        gesture = self.gestures.load(name)
        self.out(f"\n--- Edit gesture '{name}' ---")
        self._home()
        self.jog = JogController(self.controller, step_deg=self.step_deg)
        self._move_to_pose(gesture.pose)     # start from the stored pose
        self.jog.q = dict(gesture.pose.q)
        self._settle()
        key = self._jog_loop({"s": "save", "n": "save as", "q": "cancel"})
        if key == "q":
            self.controller.disable_torque()
            return
        self._settle()
        if key == "n":
            new_name = self.keys.prompt_line("New name > ").strip()
            if not new_name:
                self.controller.disable_torque()
                return
        else:
            new_name = gesture.name
        saved = Gesture(
            name=new_name,
            pose=self.jog.capture(),
            hand=self.calibration.hand_serial,
            model_version=self.model.model_version,
        )
        path = self.gestures.save(saved)
        self.out(f"Saved gesture -> {path}")
        self.controller.disable_torque()

    def _insert_gesture(self, episode: Episode) -> None:
        names = self.gestures.list()
        if not names:
            self.out("No gestures yet: create some first.")
            return
        self.out("Available gestures:")
        for i, name in enumerate(names, 1):
            self.out(f"  {i}) {name}")
        self.out("  0) cancel")
        choice = self.keys.prompt_line("> ").strip()
        if not choice or choice == "0":
            return
        try:
            name = names[int(choice) - 1]
        except (ValueError, IndexError):
            self.out("Invalid choice.")
            return
        gesture = self.gestures.load(name)
        self._move_to_pose(gesture.pose)     # the hand actually goes to the gesture
        self.jog.q = dict(gesture.pose.q)    # continue jogging from there
        self._settle()
        episode.steps.append(Step("gesture", None, name=name))  # reference by name
        self.out(f"  gesture '{name}' inserted (reference); hand moved to it")

    # ------------------------------------------------------------------ #
    def _play_menu(self) -> None:
        names = self.episodes.list()
        if not names:
            self.out("No episodes yet.")
            return
        self.out("Episodes:")
        for i, name in enumerate(names, 1):
            self.out(f"  {i}) {name}")
        choice = self.keys.prompt_line("> ").strip()
        try:
            name = names[int(choice) - 1]
        except (ValueError, IndexError):
            self.out("Invalid choice.")
            return
        episode = self.episodes.load(name)
        self.out("Playing (Ctrl-C to stop)...")
        self._play(episode.steps, loops=0)

    def _play_gesture_menu(self) -> None:
        names = self.gestures.list()
        if not names:
            self.out("No gestures yet.")
            return
        self.out("Gestures:")
        for i, name in enumerate(names, 1):
            self.out(f"  {i}) {name}")
        choice = self.keys.prompt_line("> ").strip()
        try:
            name = names[int(choice) - 1]
        except (ValueError, IndexError):
            self.out("Invalid choice.")
            return
        gesture = self.gestures.load(name)
        self.out(f"Moving to gesture '{name}' (from zero)...")
        self._play([Step("gesture", gesture.pose, name=name)], loops=1)

    def _play(self, steps, loops: int) -> None:
        self.controller.enable_torque("default")
        options = PlayOptions(loops=loops, max_step_deg=90.0, approach=True)
        player = Player(self.controller, options)
        try:
            player.play(steps, gestures=self.gestures)
        except PlayerStopped as exc:
            self.out(f"Stopped: {exc}")
        except KeyboardInterrupt:
            self.out("Interrupted.")
        finally:
            self.controller.disable_torque()

    # ------------------------------------------------------------------ #
    def _manage(self) -> None:
        while True:
            self.out("\n--- Manage ---")
            self.out("Gestures: " + (", ".join(self.gestures.list()) or "(none)"))
            self.out("Episodes: " + (", ".join(self.episodes.list()) or "(none)"))
            kind = self.keys.prompt_line(
                "[g]estures / [e]pisodes / [c]ancel > "
            ).strip().lower()
            if kind == "g":
                self._manage_gestures()
            elif kind == "e":
                self._manage_episodes()
            else:
                return

    def _pick(self, names, what: str) -> str | None:
        if not names:
            self.out(f"No {what}.")
            return None
        for i, name in enumerate(names, 1):
            self.out(f"  {i}) {name}")
        choice = self.keys.prompt_line("number > ").strip()
        try:
            return names[int(choice) - 1]
        except (ValueError, IndexError):
            self.out("Invalid choice.")
            return None

    def _manage_gestures(self) -> None:
        name = self._pick(self.gestures.list(), "gestures")
        if name is None:
            return
        action = self.keys.prompt_line(
            "[e]dit / [r]ename / [d]elete / [c]ancel > "
        ).strip().lower()
        if action == "e":
            self._edit_gesture(name)
        elif action == "r":
            new = self.keys.prompt_line("New name > ").strip()
            self.out("Renamed." if self.gestures.rename(name, new) else "Failed.")
        elif action == "d":
            self.out("Deleted." if self.gestures.delete(name) else "Failed.")

    def _manage_episodes(self) -> None:
        name = self._pick(self.episodes.list(), "episodes")
        if name is None:
            return
        action = self.keys.prompt_line(
            "[e]dit / [r]ename / [d]elete / [c]ancel > "
        ).strip().lower()
        if action == "e":
            self._edit_episode(name)
        elif action == "r":
            new = self.keys.prompt_line("New name > ").strip()
            self.out("Renamed." if self.episodes.rename(name, new) else "Failed.")
        elif action == "d":
            self.out("Deleted." if self.episodes.delete(name) else "Failed.")

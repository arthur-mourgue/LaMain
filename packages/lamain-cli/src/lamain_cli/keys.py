# SPDX-License-Identifier: Apache-2.0
"""Non-blocking keyboard input for the studio (Linux, no extra dependency)."""
from __future__ import annotations

import select
import sys
import termios
import tty
from typing import Protocol

ARROWS = {
    "\x1b[A": "up",
    "\x1b[B": "down",
    "\x1b[C": "right",
    "\x1b[D": "left",
}


class KeySource(Protocol):
    def read_key(self, timeout: float | None = None) -> str | None: ...

    def prompt_line(self, prompt: str) -> str: ...

    def close(self) -> None: ...


class KeyReader:
    """Single-key reader in cbreak mode; `prompt_line` restores cooked mode
    temporarily so the user can type a name. Falls back to line-based input when
    stdin is not a TTY (pipes, scripts)."""

    def __init__(self, stream=None):
        self._stream = stream or sys.stdin
        self._fd = self._stream.fileno()
        self._saved = None
        self._interactive = False
        self._opened = False

    def __enter__(self) -> "KeyReader":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        if self._opened:
            return
        self._opened = True
        try:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._interactive = True
        except termios.error:
            self._saved = None
            self._interactive = False

    def close(self) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None

    def read_key(self, timeout: float | None = None) -> str | None:
        if self._saved is None:
            self.open()
        if not self._interactive:
            line = self._stream.readline()
            if not line:
                return None
            return line[0]
        ready, _, _ = select.select([self._stream], [], [], timeout)
        if not ready:
            return None
        ch = self._stream.read(1)
        if ch == "\x1b":
            # Arrow keys arrive as ESC [ A/B/C/D; consume the tail promptly.
            seq = ch
            while len(seq) < 3:
                ready, _, _ = select.select([self._stream], [], [], 0.01)
                if not ready:
                    break
                seq += self._stream.read(1)
            return ARROWS.get(seq, seq)
        return ch or None

    def prompt_line(self, prompt: str) -> str:
        if not self._interactive:
            try:
                return input(prompt)
            except EOFError:
                return ""
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        try:
            return input(prompt)
        except EOFError:
            return ""
        finally:
            tty.setcbreak(self._fd)


class ScriptedKeySource:
    """Deterministic key source for tests."""

    def __init__(self, keys=None, lines=None):
        self._keys = list(keys or [])
        self._lines = list(lines or [])

    def read_key(self, timeout: float | None = None) -> str | None:
        return self._keys.pop(0) if self._keys else "q"

    def prompt_line(self, prompt: str) -> str:
        return self._lines.pop(0) if self._lines else ""

    def close(self) -> None:
        pass

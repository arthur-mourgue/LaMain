import os
import pty

import pytest

from lamain_cli.keys import KeyReader


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="pty not available")
def test_arrow_sequences_are_decoded():
    master, slave = pty.openpty()
    stream = os.fdopen(slave, "r", encoding="utf-8", buffering=1)
    reader = KeyReader(stream)
    try:
        reader.open()  # tty.setcbreak flushes pending input
        for data, expected in (
            (b"\x1b[A", "up"),
            (b"\x1b[B", "down"),
            (b"\x1b[C", "right"),
            (b"\x1b[D", "left"),
        ):
            os.write(master, data)
            assert reader.read_key(timeout=1.0) == expected
    finally:
        reader.close()
        stream.close()
        os.close(master)

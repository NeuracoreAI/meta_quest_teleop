"""Tests for the AXOn MetaQuestNode using a fake logcat stream (no headset needed)."""

from __future__ import annotations

import multiprocessing as mp
import queue
import subprocess
import sys
import time
from typing import Any

import numpy as np
import pytest
from axon.node import Node
from axon.runtime import RuntimeManager

from meta_quest.messages import QuestHandsMsg
from meta_quest.meta_quest_node import MetaQuestNode

# Right controller at (0.1, 0.2, 0.3) rotated 90 degrees about z; left untracked.
RIGHT_MATRIX = "0 -1 0 0.1 1 0 0 0.2 0 0 1 0.3 0 0 0 1"
LINE = f"r:{RIGHT_MATRIX}&R,A,RG,RThU,rightGrip 0.8,rightTrig 0.2,rightJS 0.1 -0.2"
LOGCAT_LINE = f"10-14 12:00:00.000  1234  1234 I {MetaQuestNode.LOGCAT_TAG}: {LINE}"


def _fake_logcat(script: str) -> subprocess.Popen[bytes]:
    """Run ``script`` as a stand-in for ``adb logcat``, writing to a pipe."""
    return subprocess.Popen(
        [sys.executable, "-c", script], stdout=subprocess.PIPE, bufsize=0
    )


class LiveQuestNode(MetaQuestNode):
    """Stream a fresh sample every 10 ms, like a live headset."""

    def _open_logcat(self) -> subprocess.Popen[bytes]:
        return _fake_logcat(
            "import sys, time\n"
            "while True:\n"
            f"    print({LOGCAT_LINE!r}, flush=True)\n"
            "    time.sleep(0.01)\n"
        )


class StaleQuestNode(MetaQuestNode):
    """Send one sample, then go quiet, like an unplugged headset."""

    def _open_logcat(self) -> subprocess.Popen[bytes]:
        return _fake_logcat(
            f"import time\nprint({LOGCAT_LINE!r}, flush=True)\ntime.sleep(60)\n"
        )

    def _diagnose_link(self) -> str | None:
        return None


class ProbeNode(Node):
    """Forward MetaQuestNode output to the test process."""

    def __init__(self, results: mp.Queue) -> None:
        super().__init__(name="probe")
        self.results = results

    def setup(self) -> None:
        self.create_subscriber("/meta_quest/hands", QuestHandsMsg, self.on_hands)
        self.results.put(("ready", None))

    def on_hands(self, msg: QuestHandsMsg) -> None:
        self.results.put(("hands", msg))


def _collect(results: mp.Queue, kinds: set[str], timeout: float = 10.0) -> dict:
    found: dict[str, Any] = {}
    deadline = time.monotonic() + timeout
    while not kinds <= found.keys() and time.monotonic() < deadline:
        try:
            kind, msg = results.get(timeout=0.2)
        except queue.Empty:
            continue
        found.setdefault(kind, msg)
    return found


@pytest.fixture
def runtime() -> Any:
    yield
    RuntimeManager.shutdown()


def test_publishes_hands(runtime: None) -> None:
    results: mp.Queue = mp.get_context("spawn").Queue()
    probe = ProbeNode.start(results)
    assert _collect(results, {"ready"})
    quest = LiveQuestNode.start()
    try:
        found = _collect(results, {"hands"})
    finally:
        quest.terminate()
        probe.terminate()

    hands: QuestHandsMsg = found["hands"]
    assert hands.right.pose.valid
    assert hands.right.pose.position == pytest.approx((0.1, 0.2, 0.3))
    s = np.sqrt(0.5)
    assert hands.right.pose.orientation == pytest.approx((0.0, 0.0, s, s))
    assert hands.right.buttons.button_primary
    assert hands.right.buttons.thumb_up
    assert not hands.left.buttons.thumb_up
    assert not hands.right.buttons.button_secondary
    assert hands.right.buttons.grip_pressed
    assert hands.right.buttons.grip_value == pytest.approx(0.8)
    assert hands.right.buttons.trigger_value == pytest.approx(0.2)
    assert hands.right.buttons.joystick == pytest.approx((0.1, -0.2))
    assert not hands.left.pose.valid


def test_stops_when_data_goes_stale(runtime: None) -> None:
    quest = StaleQuestNode.start(disconnect_timeout=0.3)
    quest.join(timeout=10.0)
    try:
        assert not quest.is_alive()
    finally:
        quest.terminate()


def test_parse_logcat_line() -> None:
    sample = MetaQuestNode.parse_logcat_line(LOGCAT_LINE)
    assert sample is not None and "r" in sample[0]
    assert MetaQuestNode.parse_logcat_line("unrelated logcat line") is None
    assert (
        MetaQuestNode.parse_logcat_line(f"I {MetaQuestNode.LOGCAT_TAG}: no separator")
        is None
    )


@pytest.mark.parametrize("bad_value", ["reflection", "nan", "inf", "-inf"])
def test_invalid_pose_preserves_buttons(bad_value: str) -> None:
    matrix = np.eye(4)
    if bad_value == "reflection":
        matrix[0, 0] = -1.0
    else:
        matrix[0, 3] = float(bad_value)
    values = " ".join(str(value) for value in matrix.flat)
    sample = MetaQuestNode.parse_logcat_line(
        f"I {MetaQuestNode.LOGCAT_TAG}: r:{values}&R,A"
    )
    assert sample is not None
    transforms, buttons = sample
    hand = MetaQuestNode._hand_state("r", transforms, buttons)
    assert not hand.pose.valid
    assert hand.pose.position == (0.0, 0.0, 0.0)
    assert hand.pose.orientation == (0.0, 0.0, 0.0, 1.0)
    assert hand.buttons.button_primary

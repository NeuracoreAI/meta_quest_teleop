"""AXOn node that publishes Meta Quest controller poses and button presses.

Start it with ``MetaQuestNode.start()`` so it shares the AXOn broker (and, after
``RuntimeManager.connect_neuracore()``, the Neuracore session) with other nodes.

The node talks to the headset through the ``adb`` command-line tool, which must
be installed and on PATH (``sudo apt install android-tools-adb``).
"""

from __future__ import annotations

import dataclasses
import os
import select
import subprocess
import time

import numpy as np
from axon.messages import LogLevel
from axon.node import Node

from axon_meta_quest import parsing
from axon_meta_quest import transforms as tf
from axon_meta_quest.messages import HandButtons, HandPose, HandState, QuestHandsMsg


class MetaQuestNode(Node):
    """Publish Meta Quest controller state on the AXOn bus.

    Publishes one :class:`QuestHandsMsg` per sample on ``hands``; the node is
    named ``meta_quest``, so the topic is ``/meta_quest/hands``. Poses are in
    the OpenXR frame (see README.md "Coordinate Systems: ROS vs OpenXR").
    """

    APK_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "APK")

    DEFAULT_APK_PATH = os.path.join(APK_DIR, "teleop-pointer-frame-relative.apk")

    APK_PACKAGE = "com.rail.oculus.teleop"

    LOGCAT_TAG = "wE9ryARX"

    @classmethod
    def _adb(
        cls, *args: str, serial: str | None = None, timeout: float = 30.0
    ) -> subprocess.CompletedProcess:
        """Run an ``adb`` command and return the finished process (never raises on exit code)."""
        cmd = ["adb", *(["-s", serial] if serial else []), *args]
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        except FileNotFoundError:
            raise RuntimeError(
                "`adb` is not installed or not on PATH. Install it with "
                "`sudo apt install android-tools-adb`."
            ) from None
        except subprocess.TimeoutExpired:
            raise RuntimeError(
                f"`{' '.join(cmd)}` timed out. Try `adb kill-server && adb start-server`."
            ) from None

    @classmethod
    def _list_devices(cls) -> dict[str, str]:
        """Return ``{serial: state}`` for every device ``adb devices`` lists."""
        output = cls._adb("devices", timeout=10).stdout
        # Skip daemon start-up chatter and the "List of devices attached" header.
        lines = output.split("List of devices attached", 1)[-1].splitlines()
        return dict(line.split("\t", 1) for line in lines if "\t" in line)

    @classmethod
    def _is_network_serial(cls, serial: str) -> bool:
        return serial.count(".") >= 3

    @classmethod
    def diagnose_adb(cls) -> str | None:
        """Describe what is wrong with the USB ADB link to the Quest, or None if fine."""
        try:
            devices = cls._list_devices()
        except RuntimeError as error:
            return str(error)
        states = [
            s for serial, s in devices.items() if not cls._is_network_serial(serial)
        ]
        if not states:
            return (
                "No Meta Quest found over USB. Check the headset is powered on and "
                "plugged in with a data cable, and that `adb devices` lists it."
            )
        if "device" in states:
            return None
        state = states[0]
        if state == "unauthorized":
            return (
                "The Meta Quest has not authorised this computer. Put on the headset "
                "and accept the 'Allow USB debugging' prompt."
            )
        if state.startswith("no permissions"):
            return (
                "This user has no permission to access the Meta Quest over USB. Add "
                "Android udev rules (e.g. `sudo apt install "
                "android-sdk-platform-tools-common`) and reconnect the headset."
            )
        return (
            f"The Meta Quest is listed by adb as '{state}'. Reconnect the cable or run "
            "`adb kill-server && adb start-server`."
        )

    @classmethod
    def find_device(cls, ip_address: str | None = None, port: int = 5555) -> str:
        """Find the Meta Quest and return its adb serial.

        Args:
            ip_address: Quest IP address for wireless ADB. None uses USB.
            port: Wireless ADB port.

        Raises:
            RuntimeError: If the headset cannot be reached.
        """
        if ip_address is None:
            for serial, state in cls._list_devices().items():
                if not cls._is_network_serial(serial) and state == "device":
                    return serial
            raise RuntimeError(
                "Cannot connect to the Meta Quest. "
                + (
                    cls.diagnose_adb()
                    or "Run `adb devices` to verify the headset is visible."
                )
            )

        # Try `adb connect`, then switch a USB-attached headset to TCP/IP mode and retry.
        serial = f"{ip_address}:{port}"
        for attempt in range(2):
            if attempt == 1:
                cls._adb("tcpip", str(port))
                time.sleep(1.0)
            cls._adb("connect", serial)
            if cls._list_devices().get(serial) == "device":
                return serial
        raise RuntimeError(
            f"Could not connect to the Meta Quest at {serial}. Make sure the headset "
            "is on and on the same network, and run `adb shell ip route` over USB "
            "to verify its IP address."
        )

    @classmethod
    def is_installed(cls, serial: str) -> bool:
        """Return whether the teleop APK is installed on the headset."""
        result = cls._adb(
            "shell", "pm", "list", "packages", cls.APK_PACKAGE, serial=serial
        )
        if result.returncode != 0:
            raise RuntimeError(
                "The Meta Quest is visible but could not be accessed. "
                + (
                    cls.diagnose_adb()
                    or "Put on the headset and allow USB debugging access."
                )
            )
        return f"package:{cls.APK_PACKAGE}" in result.stdout.split()

    @classmethod
    def install_apk(
        cls, serial: str, apk_path: str | None = None, reinstall: bool = False
    ) -> None:
        """Install the teleop APK on the headset unless it is already installed.

        Args:
            serial: adb serial of the headset.
            apk_path: Path to the APK file. If None, the bundled pointer APK is used.
            reinstall: Whether to reinstall the APK if it is already installed.
        """
        if cls.is_installed(serial) and not reinstall:
            return
        apk_path = apk_path or cls.DEFAULT_APK_PATH
        with open(apk_path, "rb") as apk:
            if apk.read(7) == b"version":
                raise RuntimeError(
                    f"{apk_path} is a Git LFS pointer, not the APK. Install "
                    "axon_meta_quest with Git LFS enabled: add `lfs = true` to its "
                    "[tool.uv.sources] entry or set UV_GIT_LFS=1, then reinstall."
                )
        args = ["install", "-t"] + (["-r"] if reinstall else [])
        result = cls._adb(*args, apk_path, serial=serial, timeout=120)
        if result.returncode != 0 or not cls.is_installed(serial):
            raise RuntimeError(
                "APK install failed: " + (result.stdout + result.stderr).strip()
            )

    @classmethod
    def parse_logcat_line(cls, line: str) -> tuple[dict[str, np.ndarray], dict] | None:
        """Return the sample in one teleop logcat line, or None if it has none."""
        _, sep, data = line.partition(cls.LOGCAT_TAG + ": ")
        if not sep:
            return None
        try:
            return parsing.parse_data(data.strip())
        except ValueError:
            return None

    @classmethod
    def _hand_state(cls, hand: str, transforms: dict, buttons: dict) -> HandState:
        """Build one controller's message from a parsed sample."""
        matrix = transforms.get(hand)
        if matrix is not None and tf.is_valid_transform(np.asarray(matrix)):
            position, orientation = tf.matrix_to_pose(np.asarray(matrix))
            pose = HandPose(position=position, orientation=orientation, valid=True)
        else:
            pose = HandPose(
                position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0), valid=False
            )
        inputs = parsing.hand_inputs(buttons, hand)  # type: ignore[arg-type]
        return HandState(pose=pose, buttons=HandButtons(**dataclasses.asdict(inputs)))

    def __init__(
        self,
        frequency: float = 30.0,
        ip_address: str | None = None,
        port: int = 5555,
        disconnect_timeout: float | None = 5.0,
    ) -> None:
        """Create the node.

        Args:
            frequency: Publish rate in Hz.
            ip_address: Quest IP address for wireless ADB. None uses USB.
            port: Wireless ADB port.
            disconnect_timeout: Seconds without new controller data before the
                node stops itself. None keeps it running.
        """
        super().__init__(name="meta_quest")
        self.frequency = frequency
        self._ip_address = ip_address
        self._port = port
        self._disconnect_timeout = disconnect_timeout
        self._disconnected_since: float | None = None
        self._received_data = False
        self._stale_reason = ""
        self._logcat: subprocess.Popen[bytes] | None = None
        self._logcat_closed = False
        self._buffer = b""

    def setup(self) -> None:
        """Connect to the headset and start the publish loop."""
        self._logcat = self._open_logcat()
        self.hands_pub = self.create_publisher("hands", QuestHandsMsg)
        self.create_loop(self.publish_state, frequency=self.frequency)

    def shutdown(self) -> None:
        """Stop the adb logcat process."""
        if self._logcat is not None:
            self._logcat.terminate()
            self._logcat.wait(timeout=5)

    def _open_logcat(self) -> subprocess.Popen[bytes]:
        """Find the headset, install and launch the teleop app, and stream its log."""
        serial = self.find_device(self._ip_address, self._port)
        self.install_apk(serial)
        self._adb(
            "shell",
            f'am start -n "{self.APK_PACKAGE}/{self.APK_PACKAGE}.MainActivity" '
            "-a android.intent.action.MAIN -c android.intent.category.LAUNCHER",
            serial=serial,
        )
        # Two different "-s" flags here, for two different programs:
        #   adb -s <serial>   selects the target device (adb global option).
        #   logcat -s <tag>   silences everything except the teleop app's pose
        #                     lines, so we needn't drain the full device log.
        # The separate adb process drains the device socket into a local pipe
        # independently of this process's GIL, so under heavy thread contention
        # the node only has to keep up with that pipe.
        return subprocess.Popen(
            ["adb", "-s", serial, "shell", "logcat", "-T", "0", "-s", self.LOGCAT_TAG],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
            close_fds=True,
            start_new_session=True,
        )

    def _diagnose_link(self) -> str | None:
        """Describe what is wrong with the adb link, or None if it looks fine."""
        return self.diagnose_adb() if self._ip_address is None else None

    def _read_latest_sample(self) -> tuple[dict[str, np.ndarray], dict] | None:
        """Drain the logcat pipe and return the newest sample, or None if none arrived.

        Reads in large chunks without blocking, so the node stays ahead of the
        stream and never publishes poses that sat in the pipe.
        """
        assert self._logcat is not None and self._logcat.stdout is not None
        fd = self._logcat.stdout.fileno()
        while not self._logcat_closed and select.select([fd], [], [], 0)[0]:
            chunk = os.read(fd, 65536)
            if not chunk:
                self._logcat_closed = True
                break
            self._buffer += chunk
        if b"\n" not in self._buffer:
            return None
        complete, self._buffer = self._buffer.rsplit(b"\n", 1)
        for raw_line in reversed(complete.split(b"\n")):
            sample = self.parse_logcat_line(raw_line.decode("utf-8", errors="replace"))
            if sample is not None and sample[0]:
                return sample
        return None

    def _describe_stale(self) -> str:
        """Explain why no controller data is arriving."""
        if not self._received_data:
            return (
                "Waiting for Meta Quest controller data. If none arrives, check the "
                "controllers are on and tracked and the teleop app is open in the "
                "headset."
            )
        problem = self._diagnose_link()
        if problem is not None:
            return (
                f"Lost connection to the Meta Quest: {problem} The node does not "
                "reconnect, so restart it once the headset is back."
            )
        return (
            "Meta Quest stopped sending controller data. Check the headset is awake "
            "and worn, the controllers are on and tracked, and the teleop app is "
            "open."
        )

    def publish_state(self) -> None:
        """Publish the latest controller sample, or track how long data is stale."""
        sample = self._read_latest_sample()
        if sample is None:
            now = time.monotonic()
            if self._disconnected_since is None:
                self._disconnected_since = now
                self._stale_reason = self._describe_stale()
                self.log(self._stale_reason, LogLevel.WARNING)
            elif (
                self._disconnect_timeout is not None
                and now - self._disconnected_since > self._disconnect_timeout
            ):
                raise RuntimeError(
                    f"No Meta Quest data for over {self._disconnect_timeout:g}s, "
                    f"stopping node. {self._stale_reason} Pass "
                    "disconnect_timeout=None to keep the node running instead."
                )
            return

        if self._disconnected_since is not None:
            gap = time.monotonic() - self._disconnected_since
            if self._received_data:
                self.log(f"Meta Quest data resumed after {gap:.1f}s.")
            else:
                self.log(f"Receiving Meta Quest data (after {gap:.1f}s).")
            self._disconnected_since = None
        self._received_data = True

        transforms, buttons = sample
        self.hands_pub.publish(
            QuestHandsMsg(
                left=self._hand_state("l", transforms, buttons),
                right=self._hand_state("r", transforms, buttons),
            )
        )

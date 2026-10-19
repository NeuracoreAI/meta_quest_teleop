"""Tests for the node's adb handling, with a fake adb binary."""

import subprocess
from pathlib import Path

import pytest

from meta_quest import meta_quest_node as node_module
from meta_quest.meta_quest_node import MetaQuestNode


def fake_adb(
    monkeypatch: pytest.MonkeyPatch,
    devices: str,
    installed: bool = True,
    calls: list[list[str]] | None = None,
) -> None:
    """Replace ``subprocess.run`` in the node module with a scripted adb."""
    state = {"installed": installed}

    def run(cmd: list[str], **_: object) -> subprocess.CompletedProcess:
        if calls is not None:
            calls.append(cmd)
        args = cmd[3:] if cmd[1] == "-s" else cmd[1:]
        stdout = ""
        if args[0] == "devices":
            stdout = "List of devices attached\n" + devices
        elif args[:3] == ["shell", "pm", "list"]:
            stdout = (
                f"package:{MetaQuestNode.APK_PACKAGE}\n" if state["installed"] else ""
            )
        elif args[0] == "install":
            state["installed"] = True
            stdout = "Success\n"
        return subprocess.CompletedProcess(cmd, 0, stdout, "")

    monkeypatch.setattr(node_module.subprocess, "run", run)


def test_finds_usb_device(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_adb(monkeypatch, "192.168.1.5:5555\tdevice\nQUEST000000\tdevice\n")
    assert MetaQuestNode.find_device() == "QUEST000000"


def test_installs_bundled_apk_when_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[list[str]] = []
    fake_adb(monkeypatch, "QUEST000000\tdevice\n", installed=False, calls=calls)
    MetaQuestNode.install_apk("QUEST000000")
    install = next(cmd for cmd in calls if "install" in cmd)
    assert install == [
        "adb",
        "-s",
        "QUEST000000",
        "install",
        "-t",
        MetaQuestNode.DEFAULT_APK_PATH,
    ]


def test_skips_install_when_present(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[list[str]] = []
    fake_adb(monkeypatch, "QUEST000000\tdevice\n", calls=calls)
    MetaQuestNode.install_apk("QUEST000000")
    assert not any("install" in cmd for cmd in calls)


def test_bundled_apk_is_packaged() -> None:
    with open(MetaQuestNode.DEFAULT_APK_PATH, "rb") as apk:
        # A real APK (zip), not a Git LFS pointer file.
        assert apk.read(2) == b"PK"


def test_lfs_pointer_apk_is_reported(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    pointer = tmp_path / "pointer.apk"
    pointer.write_text("version https://git-lfs.github.com/spec/v1\n")
    fake_adb(monkeypatch, "QUEST000000\tdevice\n")
    with pytest.raises(RuntimeError, match="lfs = true"):
        MetaQuestNode.install_apk("QUEST000000", str(pointer), reinstall=True)


@pytest.mark.parametrize(
    ("devices", "expected"),
    [
        ("", "No Meta Quest found over USB"),
        ("QUEST000000\tunauthorized\n", "Allow USB debugging"),
        ("QUEST000000\tno permissions (user not in plugdev)\n", "udev rules"),
    ],
)
def test_connection_error_explains_problem(
    monkeypatch: pytest.MonkeyPatch, devices: str, expected: str
) -> None:
    fake_adb(monkeypatch, devices)
    with pytest.raises(RuntimeError, match=expected):
        MetaQuestNode.find_device()


def test_missing_adb_binary(monkeypatch: pytest.MonkeyPatch) -> None:
    def run(cmd: list[str], **_: object) -> None:
        raise FileNotFoundError("adb")

    monkeypatch.setattr(node_module.subprocess, "run", run)
    assert "not installed" in (MetaQuestNode.diagnose_adb() or "")
    with pytest.raises(RuntimeError, match="not installed"):
        MetaQuestNode.find_device()

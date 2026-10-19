"""Tests for parsing APK data lines."""

import numpy as np
import pytest

from meta_quest import parsing

RIGHT = "1 0 0 0.1 0 1 0 0.2 0 0 1 0.3 0 0 0 1"
LEFT = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"


def test_parse_data_splits_transforms_and_buttons() -> None:
    transforms, buttons = parsing.parse_data(
        f"r:{RIGHT}|l:{LEFT}&R,B,RTr,rightTrig 0.5,rightJS 0.25 -1.0,L,Y,LJ"
    )
    np.testing.assert_allclose(transforms["r"][:3, 3], [0.1, 0.2, 0.3])
    assert transforms["l"].shape == (4, 4)
    assert buttons["B"] and buttons["RTr"] and buttons["Y"] and buttons["LJ"]
    assert not buttons["A"] and not buttons["X"]
    assert buttons["rightTrig"] == (0.5,)
    assert buttons["rightJS"] == (0.25, -1.0)


def test_parse_transforms_skips_malformed_entries() -> None:
    transforms = parsing.parse_transforms(f"r:{RIGHT}|l:1 2 3|garbage")
    assert list(transforms) == ["r"]


def test_parse_data_rejects_line_without_separator() -> None:
    with pytest.raises(ValueError):
        parsing.parse_data(f"r:{RIGHT}")


def test_hand_inputs_maps_buttons_per_hand() -> None:
    buttons = parsing.parse_buttons(
        "R,A,RG,rightGrip 0.8,rightTrig 0.2,rightJS 0.1 -0.2,L,Y,LTr,leftTrig 0.9"
    )
    right = parsing.hand_inputs(buttons, "right")
    assert right == parsing.HandInputs(
        button_primary=True,
        button_secondary=False,
        trigger_pressed=False,
        trigger_value=0.2,
        grip_pressed=True,
        grip_value=0.8,
        joystick_pressed=False,
        joystick=(0.1, -0.2),
    )
    left = parsing.hand_inputs(buttons, "l")
    assert not left.button_primary and left.button_secondary
    assert left.trigger_pressed and left.trigger_value == pytest.approx(0.9)
    assert left.grip_value == 0.0 and left.joystick == (0.0, 0.0)


def test_hand_inputs_defaults_when_hand_missing() -> None:
    inputs = parsing.hand_inputs({}, "left")
    assert not any([inputs.button_primary, inputs.grip_pressed, inputs.trigger_pressed])
    assert inputs.joystick == (0.0, 0.0)


def test_normalize_hand_rejects_unknown() -> None:
    with pytest.raises(ValueError):
        parsing.normalize_hand("middle")  # type: ignore[arg-type]


@pytest.mark.parametrize("flags", ["", "LThU", "RThU", "LThU,RThU"])
def test_thumb_up_maps_independently_per_hand(flags: str) -> None:
    buttons = parsing.parse_buttons(f"L,R,{flags}")
    assert parsing.hand_inputs(buttons, "left").thumb_up == ("LThU" in flags)
    assert parsing.hand_inputs(buttons, "right").thumb_up == ("RThU" in flags)

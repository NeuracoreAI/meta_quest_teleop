"""Parse the controller data lines the teleop APK writes to logcat.

Each line is ``<transforms>&<buttons>``, where ``<transforms>`` is
``r:<16 floats>|l:<16 floats>`` (row-major 4x4 matrices, OpenXR frame) and
``<buttons>`` is a comma-separated list of pressed buttons and analog values.
"""

from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

Hand = Literal["left", "right", "l", "r"]


@dataclass(frozen=True)
class HandInputs:
    """Button, trigger, grip, and joystick state of one controller."""

    button_primary: bool
    """A on the right controller, X on the left controller."""
    button_secondary: bool
    """B on the right controller, Y on the left controller."""
    trigger_pressed: bool
    trigger_value: float
    grip_pressed: bool
    grip_value: float
    joystick_pressed: bool
    joystick: tuple[float, float]
    thumb_up: bool = False


def normalize_hand(hand: Hand) -> str:
    """Normalize a hand identifier to ``"l"`` or ``"r"``.

    Args:
        hand: Hand identifier ('left', 'right', 'l', or 'r').

    Returns:
        'l' or 'r'.
    """
    if hand in ("left", "l"):
        return "l"
    if hand in ("right", "r"):
        return "r"
    raise ValueError(f"Invalid hand: {hand}. Must be 'left', 'right', 'l', or 'r'")


def parse_buttons(text: str) -> dict[str, Any]:
    """Parse buttons from text.

    Args:
        text: Text to parse.

    Returns:
        Dictionary with button states.
    """
    split_text = text.split(",")
    buttons: dict[str, Any] = {}
    if "R" in split_text:  # right hand if available
        split_text.remove("R")  # remove marker
        buttons.update(
            {
                "A": False,
                "B": False,
                "RThU": (
                    False
                ),  # indicates that right thumb is up from the rest position
                "RJ": False,  # joystick pressed
                "RG": False,  # trigger on the grip
                "RTr": False,  # trigger on the index finger
            }
        )
        # besides following keys are provided:
        # 'rightJS' / 'leftJS' -
        #   (x, y) position of joystick. x, y both in range (-1.0, 1.0)
        # 'rightGrip' / 'leftGrip' -
        #   float value for trigger on the grip in range (0.0, 1.0)
        # 'rightTrig' / 'leftTrig' -
        #   float value for trigger on the index finger in range (0.0, 1.0)

    if "L" in split_text:  # left hand accordingly
        split_text.remove("L")  # remove marker
        buttons.update(
            {
                "X": False,
                "Y": False,
                "LThU": False,
                "LJ": False,
                "LG": False,
                "LTr": False,
            }
        )
    for key in buttons.keys():
        if key in list(split_text):
            buttons[key] = True
            split_text.remove(key)
    for elem in split_text:
        split_elem = elem.split(" ")
        if len(split_elem) < 2:
            continue
        key = split_elem[0]
        value = tuple([float(x) for x in split_elem[1:]])
        buttons[key] = value
    return buttons


def parse_transforms(text: str) -> dict[str, np.ndarray]:
    """Parse ``r:<16 floats>|l:<16 floats>`` into 4x4 matrices keyed by hand.

    Entries that are malformed or do not hold exactly 16 values are skipped.

    Args:
        text: Transforms part of a data line.

    Returns:
        Dictionary mapping 'r' and/or 'l' to 4x4 matrices.
    """
    transforms = {}
    for pair_string in text.split("|"):
        pair = pair_string.split(":")
        if len(pair) != 2:
            continue
        values = [float(value) for value in pair[1].split(" ") if value]
        if len(values) == 16:
            transforms[pair[0]] = np.array(values).reshape(4, 4)
    return transforms


def parse_data(text: str) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Parse one data line into transforms and button states.

    Args:
        text: Data line without the logcat prefix.

    Returns:
        Tuple of transforms and button states.

    Raises:
        ValueError: If the line is not ``<transforms>&<buttons>``.
    """
    transforms_string, buttons_string = text.split("&")
    return parse_transforms(transforms_string), parse_buttons(buttons_string)


def _first_value(value: Any, default: float = 0.0) -> float:
    # Analog values are parsed as tuples, e.g. ('rightGrip', (0.8,)).
    if isinstance(value, tuple):
        return float(value[0]) if value else default
    return float(value) if value else default


def grip_value(buttons: dict[str, Any], hand: Hand) -> float:
    """Return the analog grip value of ``hand`` in [0, 1]."""
    side = "left" if normalize_hand(hand) == "l" else "right"
    return _first_value(buttons.get(f"{side}Grip", 0.0))


def trigger_value(buttons: dict[str, Any], hand: Hand) -> float:
    """Return the analog index trigger value of ``hand`` in [0, 1]."""
    side = "left" if normalize_hand(hand) == "l" else "right"
    return _first_value(buttons.get(f"{side}Trig", 0.0))


def joystick_value(buttons: dict[str, Any], hand: Hand) -> tuple[float, float]:
    """Return the joystick (x, y) of ``hand``, each in [-1, 1]."""
    side = "left" if normalize_hand(hand) == "l" else "right"
    value = buttons.get(f"{side}JS", (0.0, 0.0))
    if isinstance(value, tuple) and len(value) >= 2:
        return (float(value[0]), float(value[1]))
    return (0.0, 0.0)


def hand_inputs(buttons: dict[str, Any], hand: Hand) -> HandInputs:
    """Collect all button, trigger, grip, and joystick state of one controller.

    Args:
        buttons: Button states from :func:`parse_buttons`.
        hand: Which hand ('left', 'right', 'l', or 'r').

    Returns:
        The controller's inputs; missing values default to released / zero.
    """
    side = normalize_hand(hand).upper()
    primary, secondary = ("A", "B") if side == "R" else ("X", "Y")
    return HandInputs(
        button_primary=bool(buttons.get(primary, False)),
        button_secondary=bool(buttons.get(secondary, False)),
        trigger_pressed=bool(buttons.get(f"{side}Tr", False)),
        trigger_value=trigger_value(buttons, hand),
        grip_pressed=bool(buttons.get(f"{side}G", False)),
        grip_value=grip_value(buttons, hand),
        joystick_pressed=bool(buttons.get(f"{side}J", False)),
        joystick=joystick_value(buttons, hand),
        thumb_up=bool(buttons.get(f"{side}ThU", False)),
    )

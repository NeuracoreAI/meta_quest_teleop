"""Buttons parser for Meta Quest Reader."""

from typing import Any


def parse_buttons(text: str) -> dict[str, Any]:
    """Parse buttons from text.

    Args:
        text: Text to parse.

    Returns:
        Dictionary with button states.
    """
    # Normalize whitespace so tokens like " A " are treated as "A".
    split_text = [token.strip() for token in text.split(",") if token.strip()]
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
    for key in list(buttons.keys()):
        if key in split_text:
            buttons[key] = True
            split_text.remove(key)
    for elem in split_text:
        split_elem = [part for part in elem.split(" ") if part]
        if len(split_elem) < 2:
            continue
        key = split_elem[0]
        try:
            value = tuple(float(x) for x in split_elem[1:])
        except ValueError:
            # Ignore malformed numeric fields while preserving other parsed inputs.
            continue
        buttons[key] = value
    return buttons

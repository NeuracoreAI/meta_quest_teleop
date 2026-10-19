"""Run the Meta Quest AXOn node and print the controller state it publishes.

Usage:
    python examples/meta_quest_example.py                    # USB
    python examples/meta_quest_example.py --ip 10.0.32.101   # wireless ADB
"""

from __future__ import annotations

import argparse
import time

from axon.node import Node
from axon.runtime import RuntimeManager

from meta_quest.messages import QuestHandsMsg
from meta_quest.meta_quest_node import MetaQuestNode


class PrintNode(Node):
    """Print all published controller data from ``/meta_quest/hands``."""

    def __init__(self) -> None:
        """Create the node."""
        super().__init__(name="quest_printer")

    def setup(self) -> None:
        """Subscribe to the Quest hands topic."""
        self.create_subscriber("/meta_quest/hands", QuestHandsMsg, self.on_hands)

    def on_hands(self, msg: QuestHandsMsg) -> None:
        """Log one sample as a table with both controllers side by side."""

        def vector(values: tuple[float, ...]) -> str:
            return "(" + ", ".join(f"{value: .4f}" for value in values) + ")"

        rows: list[tuple[str, str, str]] = []
        for label, values in (
            ("Pose valid", [str(hand.pose.valid) for hand in (msg.left, msg.right)]),
            (
                "Position (x, y, z)",
                [vector(hand.pose.position) for hand in (msg.left, msg.right)],
            ),
            (
                "Quaternion (x, y, z, w)",
                [vector(hand.pose.orientation) for hand in (msg.left, msg.right)],
            ),
            (
                "Primary button (X / A)",
                [str(hand.buttons.button_primary) for hand in (msg.left, msg.right)],
            ),
            (
                "Secondary button (Y / B)",
                [str(hand.buttons.button_secondary) for hand in (msg.left, msg.right)],
            ),
            (
                "Trigger pressed",
                [str(hand.buttons.trigger_pressed) for hand in (msg.left, msg.right)],
            ),
            (
                "Trigger value",
                [f"{hand.buttons.trigger_value:.4f}" for hand in (msg.left, msg.right)],
            ),
            (
                "Grip pressed",
                [str(hand.buttons.grip_pressed) for hand in (msg.left, msg.right)],
            ),
            (
                "Grip value",
                [f"{hand.buttons.grip_value:.4f}" for hand in (msg.left, msg.right)],
            ),
            (
                "Joystick pressed",
                [str(hand.buttons.joystick_pressed) for hand in (msg.left, msg.right)],
            ),
            (
                "Joystick (x, y)",
                [vector(hand.buttons.joystick) for hand in (msg.left, msg.right)],
            ),
            (
                "Thumb up",
                [str(hand.buttons.thumb_up) for hand in (msg.left, msg.right)],
            ),
        ):
            rows.append((label, values[0], values[1]))

        label_width = max(len(label) for label, _, _ in rows)
        value_width = max(
            len(value) for _, left, right in rows for value in (left, right)
        )
        header = f"{'Meta Quest':<{label_width}} | {'LEFT':<{value_width}} | RIGHT"
        divider = "-" * (label_width + 2 * value_width + 6)
        lines = [header, divider]
        lines.extend(
            f"{label:<{label_width}} | {left:<{value_width}} | {right}"
            for label, left, right in rows
        )
        self.log("\n" + "\n".join(lines))


def main() -> None:
    """Start the Quest and printer nodes until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default=None, help="Quest IP for wireless ADB")
    args = parser.parse_args()

    quest = MetaQuestNode.start(ip_address=args.ip)
    printer = PrintNode.start()
    try:
        while quest.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        quest.terminate()
        printer.terminate()
        RuntimeManager.shutdown()


if __name__ == "__main__":
    main()

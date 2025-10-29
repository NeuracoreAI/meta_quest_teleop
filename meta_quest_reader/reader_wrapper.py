#!/usr/bin/env python3
"""Wrapper for MetaQuestReader to handle new APK from Neuracore."""

from typing import Any, Callable, Literal

import numpy as np
from scipy.spatial.transform import Rotation

from meta_quest_reader.reader import MetaQuestReader


class MetaQuestReaderWrapper:
    """Wrapper class for the MetaQuestReader class from RAIL Lab.

    This class handles the new APK created by Neuracore and provides clean
    APIs to access the hand controller transforms in the OpenXR coordinate
    system and ROS coordinate system, and button event callbacks.
    """

    def __init__(
        self,
        ip_address: str | None = None,
        port: int = 5555,
        APK_name: str = "com.rail.oculus.teleop",
        print_FPS: bool = False,
        run: bool = True,
    ):
        """Initialize the MetaQuestReaderWrapper.

        Args:
            ip_address: IP address of Meta Quest device
            port: Port number for connection
            APK_name: Android package name
            print_FPS: Whether to print FPS statistics
            run: Whether to start reader immediately
        """
        self.reader = MetaQuestReader(
            ip_address=ip_address,
            port=port,
            APK_name=APK_name,
            print_FPS=print_FPS,
            run=run,
        )

        # Button state tracking for edge detection
        self._prev_button_states: dict[str, bool] = {}

        # Callback system
        # TODO: add more button event callbacks.
        self._callbacks: dict[str, list[Callable]] = {
            "button_b_pressed": [],
            "button_a_pressed": [],
            "button_x_pressed": [],
            "button_y_pressed": [],
        }

        # Cache latest transforms and button values
        self._latest_transforms: dict[str, np.ndarray] = {}
        self._latest_buttons: dict[str, Any] = {}

    def __del__(self) -> None:
        """Destructor."""
        if hasattr(self, "reader"):
            self.reader.stop()

    def update(self) -> bool:
        """Poll for new transforms and button states.

        Call this in your main loop to get latest data.

        Returns:
            True if new data was received, False otherwise
        """
        transforms, buttons = self.reader.get_transformations_and_buttons()

        if transforms is None or len(transforms) == 0:
            return False

        # Validate and store transforms
        for key, matrix in transforms.items():
            validated = self._validate_transform(matrix)

            # only store the transform if it is valid
            # TODO: maybe print a warning under a debug flag?
            if validated is not None:
                self._latest_transforms[key] = validated

        # Store button states
        if buttons is not None:
            self._latest_buttons = buttons
            self._handle_button_events(buttons)

        return True

    def get_hand_controller_transform_openxr(
        self,
        hand: Literal["left", "right", "l", "r"] = "right",
        transform_type: Literal["grip", "pointer", "model"] = "grip",
    ) -> np.ndarray | None:
        """Get the 4x4 transformation matrix for a hand controller.

        The transform is in the OpenXR coordinate system.

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')
            transform_type: Type of transform to get:
                - 'grip': Grip pose on the handle (tilted, default for
                  backwards compatibility)
                - 'pointer': Pointer/ray-cast pose (straight forward,
                  recommended for control)
                - 'model': Device model pose

        Returns:
            4x4 numpy array transformation matrix, or None if not
            available
        """
        hand_key = self._normalize_hand_key(hand)

        # Map transform type to key suffix
        type_suffix = {"grip": "g", "pointer": "p", "model": "m"}.get(
            transform_type, "g"
        )

        # e.g. 'rg' for right grip, 'lp' for left pointer
        key = hand_key + type_suffix
        if key in self._latest_transforms:
            return self._latest_transforms[key].copy()

        return None

    def get_hand_controller_transform_ros(
        self,
        hand: Literal["left", "right", "l", "r"] = "right",
        transform_type: Literal["grip", "pointer", "model"] = "grip",
    ) -> np.ndarray | None:
        """Get the 4x4 transformation matrix for a hand controller.

        The transform is in the ROS coordinate system. This function applies
        a quaternion [0.5, -0.5, -0.5, 0.5] to the
        transform to convert from OpenXR coordinate system to ROS coordinate
        system.

        OpenXR coordinate system: X=right, Y=up, Z=backward
        ROS coordinate system: X=forward, Y=left, Z=up

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')
            transform_type: Type of transform ('grip', 'pointer', or
                'model')

        Returns:
            4x4 transformation matrix in ROS coordinates, or None if not
            available
        """
        transform_openxr = self.get_hand_controller_transform_openxr(
            hand, transform_type
        )

        if transform_openxr is None:
            return None

        # Apply static transform: quaternion [0.5, -0.5, -0.5, 0.5]
        Q = Rotation.from_quat([0.5, -0.5, -0.5, 0.5])
        T_static = np.eye(4)
        T_static[:3, :3] = Q.as_matrix()

        return T_static @ transform_openxr

    def get_button_state(self, button_name: str) -> bool:
        """Get current state of a button.

        Args:
            button_name: Button name (e.g., 'A', 'B', 'X', 'Y', 'RJ',
                'LJ')

        Returns:
            True if button is pressed, False otherwise
        """
        # TODO: think about this. if the button was not found and we just
        # return False.
        return self._latest_buttons.get(button_name, False)

    def get_grip_value(
        self, hand: Literal["left", "right", "l", "r"] = "right"
    ) -> float:
        """Get the continuous grip value (analog trigger).

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            Float value in range [0.0, 1.0] where 0.0 is not pressed and
            1.0 is fully pressed
        """
        hand_key = self._normalize_hand_key(hand)
        button_name = "leftGrip" if hand_key == "l" else "rightGrip"
        value = self._latest_buttons.get(button_name, 0.0)

        # Handle case where value might be a tuple from parsing
        if isinstance(value, tuple):
            return float(value[0]) if len(value) > 0 else 0.0
        return float(value) if value else 0.0

    def get_trigger_value(
        self, hand: Literal["left", "right", "l", "r"] = "right"
    ) -> float:
        """Get the continuous trigger value (index finger trigger).

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            Float value in range [0.0, 1.0] where 0.0 is not pressed and
            1.0 is fully pressed
        """
        hand_key = self._normalize_hand_key(hand)
        button_name = "leftTrig" if hand_key == "l" else "rightTrig"
        value = self._latest_buttons.get(button_name, 0.0)

        # Handle case where value might be a tuple from parsing
        if isinstance(value, tuple):
            return float(value[0]) if len(value) > 0 else 0.0
        return float(value) if value else 0.0

    def get_joystick_value(
        self, hand: Literal["left", "right", "l", "r"] = "right"
    ) -> tuple[float, float]:
        """Get the joystick position.

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            Tuple (x, y) where both x and y are in range [-1.0, 1.0]
            Returns (0.0, 0.0) if not available
        """
        hand_key = self._normalize_hand_key(hand)
        button_name = "leftJS" if hand_key == "l" else "rightJS"
        value = self._latest_buttons.get(button_name, (0.0, 0.0))

        if isinstance(value, tuple) and len(value) >= 2:
            return (float(value[0]), float(value[1]))
        return (0.0, 0.0)

    def on(self, event: str, callback: Callable) -> None:
        """Register a callback for an event.

        Available events:
        - 'button_b_pressed': Called when Button B is pressed
        - 'button_a_pressed': Called when Button A is pressed
        - 'button_x_pressed': Called when Button X is pressed
        - 'button_y_pressed': Called when Button Y is pressed

        Args:
            event: Event name
            callback: Function to call when event occurs
        """
        # make sure the event is a valid event
        if event not in self._callbacks:
            raise ValueError(
                f"Invalid event: {event}. Must be one of: "
                f"{list(self._callbacks.keys())}"
            )

        self._callbacks[event].append(callback)

    def _validate_transform(self, matrix: np.ndarray) -> np.ndarray | None:
        """Validate transformation matrix.

        Args:
            matrix: 4x4 transformation matrix

        Returns:
            The same matrix if valid, None if invalid
        """
        if np.allclose(matrix, 0.0):
            return None

        det = np.linalg.det(matrix[:3, :3])
        if abs(abs(det) - 1.0) > 0.1:
            return None

        return matrix

    def _normalize_hand_key(self, hand: Literal["left", "right", "l", "r"]) -> str:
        """Normalize hand identifier to 'l' or 'r'.

        Args:
            hand: Hand identifier ('left', 'right', 'l', or 'r')

        Returns:
            'l' or 'r'
        """
        if hand in ("left", "l"):
            return "l"
        elif hand in ("right", "r"):
            return "r"
        else:
            raise ValueError(
                f"Invalid hand: {hand}. Must be 'left', 'right', " f"'l', or 'r'"
            )

    def _handle_button_events(self, buttons: dict) -> None:
        """Handle button press events and trigger callbacks.

        Args:
            buttons: Dictionary of button states
        """
        # Check for button presses (rising edge detection)
        button_map = {
            "B": "button_b_pressed",
            "A": "button_a_pressed",
            "X": "button_x_pressed",
            "Y": "button_y_pressed",
        }

        for button_key, event_name in button_map.items():
            current_state = buttons.get(button_key, False)
            prev_state = self._prev_button_states.get(button_key, False)

            # Rising edge detected
            if current_state and not prev_state:
                # Trigger callbacks
                for callback in self._callbacks[event_name]:
                    callback()

            self._prev_button_states[button_key] = current_state

"""AXOn message types published by :class:`meta_quest.meta_quest_node.MetaQuestNode`."""

from __future__ import annotations

from axon.registry import register_message
from pydantic import BaseModel


class HandPose(BaseModel):
    """6-DoF pose of one Quest controller, in the OpenXR tracking frame."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    """Orientation as an (x, y, z, w) quaternion."""
    valid: bool
    """Whether this controller was tracked when the pose was sampled."""


class HandButtons(BaseModel):
    """Button, trigger, grip, and joystick state of one Quest controller."""

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
    """Whether the controller reports the thumb lifted from its rest position."""


class HandState(BaseModel):
    """Combined pose and button state of one Quest controller."""

    pose: HandPose
    buttons: HandButtons


@register_message
class QuestHandsMsg(BaseModel):
    """Combined left- and right-hand Quest controller state for one sample."""

    left: HandState
    right: HandState

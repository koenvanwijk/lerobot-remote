"""
modes.py — ActionMode serialization helpers for the signaling handshake.

Converts ActionMode and FrameDefinition to/from plain dicts so they can be
sent over the signaling channel as JSON during capabilities negotiation.
"""

from __future__ import annotations

from typing import Any

from lerobot_action_space import ActionMode, FrameDefinition


def action_mode_to_dict(mode: ActionMode) -> dict[str, Any]:
    return {
        "name": mode.name,
        "space_type": mode.space_type,
        "unit": mode.unit,
        "command_mode": mode.command_mode,
        "frame": _frame_to_dict(mode.frame) if mode.frame else None,
        "is_default": mode.is_default,
        "preferred_hz": mode.preferred_hz,
        "requires": mode.requires,
        "description": mode.description,
    }


def action_mode_from_dict(d: dict[str, Any]) -> ActionMode:
    return ActionMode(
        name=d["name"],
        space_type=d["space_type"],
        unit=d["unit"],
        command_mode=d["command_mode"],
        frame=_frame_from_dict(d["frame"]) if d.get("frame") else None,
        is_default=d.get("is_default", False),
        preferred_hz=d.get("preferred_hz"),
        requires=d.get("requires", []),
        description=d.get("description", ""),
    )


def _frame_to_dict(frame: FrameDefinition) -> dict[str, Any]:
    return {
        "name": frame.name,
        "x_axis": frame.x_axis,
        "y_axis": frame.y_axis,
        "z_axis": frame.z_axis,
        "handedness": frame.handedness,
        "origin": frame.origin,
    }


def _frame_from_dict(d: dict[str, Any]) -> FrameDefinition:
    return FrameDefinition(
        name=d["name"],
        x_axis=d["x_axis"],
        y_axis=d["y_axis"],
        z_axis=d["z_axis"],
        handedness=d.get("handedness", "right"),
        origin=d.get("origin"),
    )

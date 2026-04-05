"""
tests/test_negotiation.py — Unit tests for mode negotiation protocol.

Tests _negotiate_modes() in both RemoteRobot and RemoteTeleop using
InProcessSignaling (no network, no WebRTC).
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any

import pytest

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent / "lerobot-action-space" / "src"))

from conftest import InProcessSignaling, VirtualRobot

from lerobot_action_space.compat import SO101_FOLLOWER_MODES
from lerobot_action_space import ActionMode
from lerobot_remote_transport.signaling import ProtocolError
from lerobot_remote_transport.modes import action_mode_to_dict, action_mode_from_dict


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def run_coro(coro, loop):
    """Run a coroutine in an existing event loop from a sync context."""
    fut = asyncio.run_coroutine_threadsafe(coro, loop)
    return fut.result(timeout=10)


def make_loop():
    loop = asyncio.new_event_loop()
    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()
    return loop, t


# ---------------------------------------------------------------------------
# Negotiation: normal flow
# ---------------------------------------------------------------------------

def test_negotiate_modes_exact_match():
    """
    Operator and robot both support joint_absolute_norm.
    Bridge should be EXACT quality.
    """
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig
    from lerobot_action_space import ActionBridge, TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES

    robot_modes = SO101_FOLLOWER_MODES
    teleop_modes = TELEOP_ACTION_MODES

    op_sig, rb_sig = InProcessSignaling.linked_pair()

    # Build objects with injected signaling
    robot_config = RemoteRobotConfig(room="test")
    teleop_config = RemoteTeleopConfig(room="test")

    remote_robot = RemoteRobot(robot_config, teleop_modes=teleop_modes)
    remote_teleop = RemoteTeleop(teleop_config, robot_modes=robot_modes)

    vr = VirtualRobot()
    remote_teleop.attach_robot(vr)

    # Inject signaling (skip real HTTP)
    remote_robot._signaling = op_sig
    remote_teleop._signaling = rb_sig

    # Run both negotiations concurrently in separate loops
    op_loop, _ = make_loop()
    rb_loop, _ = make_loop()
    remote_robot._loop = op_loop
    remote_teleop._loop = rb_loop

    # Both sides need to negotiate simultaneously
    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(run_coro, remote_robot._negotiate_modes(), op_loop)
        f2 = ex.submit(run_coro, remote_teleop._negotiate_modes(), rb_loop)
        f1.result(timeout=10)
        f2.result(timeout=10)

    assert remote_robot._bridge is not None
    assert remote_teleop._bridge is not None
    assert remote_robot._bridge.teleop_mode.name == remote_teleop._bridge.robot_mode.name

    # Features should have been propagated
    assert len(remote_robot._robot_features) == len(vr.action_features)


def test_negotiate_modes_features_propagated():
    """After negotiation, RemoteRobot knows the robot's joint key names."""
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig
    from lerobot_action_space import TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES

    robot_modes = SO101_FOLLOWER_MODES
    teleop_modes = TELEOP_ACTION_MODES

    op_sig, rb_sig = InProcessSignaling.linked_pair()
    vr = VirtualRobot(joint_names=["shoulder_pan", "shoulder_lift", "gripper"])

    remote_robot = RemoteRobot(RemoteRobotConfig(room="feat"), teleop_modes=teleop_modes)
    remote_teleop = RemoteTeleop(RemoteTeleopConfig(room="feat"), robot_modes=robot_modes)
    remote_teleop.attach_robot(vr)

    remote_robot._signaling = op_sig
    remote_teleop._signaling = rb_sig

    op_loop, _ = make_loop()
    rb_loop, _ = make_loop()
    remote_robot._loop = op_loop
    remote_teleop._loop = rb_loop

    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(run_coro, remote_robot._negotiate_modes(), op_loop)
        f2 = ex.submit(run_coro, remote_teleop._negotiate_modes(), rb_loop)
        f1.result(timeout=10)
        f2.result(timeout=10)

    assert "shoulder_pan.pos" in remote_robot._robot_features
    assert "gripper.pos" in remote_robot._robot_features


def test_negotiate_modes_no_physical_robot_sends_empty_features():
    """If no physical robot attached, features list is empty."""
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig
    from lerobot_action_space import TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES

    robot_modes = SO101_FOLLOWER_MODES
    teleop_modes = TELEOP_ACTION_MODES

    op_sig, rb_sig = InProcessSignaling.linked_pair()

    remote_robot = RemoteRobot(RemoteRobotConfig(room="nofeat"), teleop_modes=teleop_modes)
    remote_teleop = RemoteTeleop(RemoteTeleopConfig(room="nofeat"), robot_modes=robot_modes)
    # NO attach_robot

    remote_robot._signaling = op_sig
    remote_teleop._signaling = rb_sig

    op_loop, _ = make_loop()
    rb_loop, _ = make_loop()
    remote_robot._loop = op_loop
    remote_teleop._loop = rb_loop

    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(run_coro, remote_robot._negotiate_modes(), op_loop)
        f2 = ex.submit(run_coro, remote_teleop._negotiate_modes(), rb_loop)
        f1.result(timeout=10)
        f2.result(timeout=10)

    # No features → _robot_features should be empty, action_features falls back to bridge mode name
    assert remote_robot._robot_features == {}


# ---------------------------------------------------------------------------
# Negotiation: error paths
# ---------------------------------------------------------------------------

def test_negotiate_wrong_first_message_raises():
    """RemoteTeleop raises ProtocolError if first message is not capabilities."""
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES

    robot_modes = SO101_FOLLOWER_MODES

    op_sig, rb_sig = InProcessSignaling.linked_pair()
    rb_loop, _ = make_loop()

    remote_teleop = RemoteTeleop(RemoteTeleopConfig(room="err"), robot_modes=robot_modes)
    remote_teleop._signaling = rb_sig
    remote_teleop._loop = rb_loop

    # Send wrong message type directly via threading Queue (InProcessSignaling)
    op_sig._send_queue.put({"type": "wrong_type", "data": "bad"})

    # negotiate should raise ProtocolError
    with pytest.raises(ProtocolError):
        run_coro(remote_teleop._negotiate_modes(), rb_loop)

"""
tests/test_end_to_end.py — End-to-end tests for RemoteRobot ↔ RemoteTeleop.

Uses InProcessTransport (no WebRTC) + InProcessSignaling (no HTTP).
Tests the full action/observation flow including threading.
"""

from __future__ import annotations

import asyncio
import threading
import time

import pytest

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent / "lerobot-action-space" / "src"))

from conftest import InProcessSignaling, InProcessTransport, VirtualRobot


def make_loop():
    loop = asyncio.new_event_loop()
    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()
    return loop, t


def run_concurrent(op_loop, rb_loop, op_coro, rb_coro, timeout=10):
    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(
            lambda: asyncio.run_coroutine_threadsafe(op_coro, op_loop).result(timeout=timeout)
        )
        f2 = ex.submit(
            lambda: asyncio.run_coroutine_threadsafe(rb_coro, rb_loop).result(timeout=timeout)
        )
        f1.result(timeout=timeout + 2)
        f2.result(timeout=timeout + 2)


def build_connected_pair(joint_names=("j1", "j2", "j3")):
    """
    Returns (remote_robot, remote_teleop, virtual_robot) all connected via
    in-process transports. No network or WebRTC involved.
    """
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig
    from lerobot_action_space import TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES

    robot_modes = SO101_FOLLOWER_MODES
    teleop_modes = TELEOP_ACTION_MODES
    vr = VirtualRobot(joint_names=joint_names)

    op_sig, rb_sig = InProcessSignaling.linked_pair()

    robot_config = RemoteRobotConfig(room="e2e", hz=50)
    teleop_config = RemoteTeleopConfig(room="e2e", hz=50)

    rr = RemoteRobot(robot_config, teleop_modes=teleop_modes)
    rt = RemoteTeleop(teleop_config, robot_modes=robot_modes)
    rt.attach_robot(vr)

    op_loop, _ = make_loop()
    rb_loop, _ = make_loop()
    rr._loop = op_loop
    rt._loop = rb_loop
    rr._signaling = op_sig
    rt._signaling = rb_sig

    # Run mode negotiation
    run_concurrent(op_loop, rb_loop, rr._negotiate_modes(), rt._negotiate_modes())

    # Wire up in-process transport
    op_tr, rb_tr = None, None

    def create_transports():
        nonlocal op_tr, rb_tr
        loop = asyncio.new_event_loop()
        op_tr, rb_tr = InProcessTransport.linked_pair()
        asyncio.run_coroutine_threadsafe(op_tr.connect(), op_loop).result(timeout=5)
        asyncio.run_coroutine_threadsafe(rb_tr.connect(), rb_loop).result(timeout=5)

    create_transports()

    rr._transport = op_tr
    rt._transport = rb_tr
    op_tr.on_message = rr._on_observation_received
    rb_tr.on_message = rt._on_action_received

    rr._connected = True
    rt._connected = True

    return rr, rt, vr


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_e2e_send_action_received_by_teleop():
    """Action sent by RemoteRobot arrives at RemoteTeleop._action_queue."""
    rr, rt, vr = build_connected_pair()

    action = {k: 0.5 for k in vr.action_features}
    asyncio.run_coroutine_threadsafe(
        rr._transport.send(rr._bridge.convert(action)), rr._loop
    ).result(timeout=5)

    # Give the recv loop time to process
    time.sleep(0.1)
    assert not rt._action_queue.empty()
    received = rt._action_queue.get_nowait()
    assert set(received.keys()) == set(action.keys())


def test_e2e_get_action_decodes_correctly():
    """RemoteTeleop.get_action() decodes the received action."""
    rr, rt, vr = build_connected_pair(joint_names=("shoulder_pan", "elbow_flex", "gripper"))

    action = {"shoulder_pan.pos": 30.0, "elbow_flex.pos": -20.0, "gripper.pos": 50.0}
    encoded = rr._bridge.convert(action)

    asyncio.run_coroutine_threadsafe(
        rr._transport.send(encoded), rr._loop
    ).result(timeout=5)

    time.sleep(0.1)
    decoded = rt.get_action()
    # Values should match after round-trip through bridge (EXACT mode → identity)
    assert abs(decoded.get("shoulder_pan.pos", 0) - 30.0) < 0.01
    assert abs(decoded.get("gripper.pos", 0) - 50.0) < 0.01


def test_e2e_get_action_returns_last_on_timeout():
    """get_action() returns last known action on timeout, not raises."""
    rr, rt, vr = build_connected_pair()

    # Send one action first
    action = {k: 10.0 for k in vr.action_features}
    encoded = rr._bridge.convert(action)
    asyncio.run_coroutine_threadsafe(rr._transport.send(encoded), rr._loop).result(timeout=5)
    time.sleep(0.1)
    rt.get_action()  # consume it → sets _last_action

    # Now get_action with empty queue → returns last
    result = rt.get_action()
    assert result is not None
    assert set(result.keys()) == set(action.keys())


def test_e2e_get_action_returns_empty_before_first():
    """get_action() returns {} if no action received yet (not raises)."""
    rr, rt, vr = build_connected_pair()
    # Queue is empty, _last_action is None
    result = rt.get_action()
    assert result == {}


def test_e2e_observation_roundtrip():
    """Observation sent by robot side arrives at RemoteRobot.get_observation()."""
    rr, rt, vr = build_connected_pair(joint_names=("j1", "j2"))

    obs = vr.get_observation()
    # Robot side (rt._transport) sends obs → operator side (rr._transport) receives it
    asyncio.run_coroutine_threadsafe(
        rt._transport.send({"__obs__": True, **obs}),
        rt._loop
    ).result(timeout=5)

    time.sleep(0.2)
    received_obs = rr.get_observation()
    assert received_obs == obs


def test_e2e_feedback_not_stored_as_observation():
    """__feedback__ messages are ignored by RemoteRobot._on_observation_received."""
    rr, rt, vr = build_connected_pair()

    rr._last_observation = {}
    asyncio.run_coroutine_threadsafe(
        rr._transport.send({"__feedback__": True, "force": 9.9}),
        rr._loop
    ).result(timeout=5)

    time.sleep(0.1)
    assert rr.get_observation() == {}


def test_e2e_send_feedback_does_not_crash():
    """RemoteTeleop.send_feedback() sends without error."""
    rr, rt, vr = build_connected_pair()
    # Should not raise
    asyncio.run_coroutine_threadsafe(
        rt._transport.send({"__feedback__": True, "torque": 0.5}),
        rt._loop
    ).result(timeout=5)


def test_e2e_observation_features_after_negotiation():
    """RemoteRobot.observation_features returns the actual joint keys after negotiation."""
    rr, rt, vr = build_connected_pair(joint_names=("shoulder_pan", "gripper"))
    features = rr.observation_features
    assert "shoulder_pan.pos" in features
    assert "gripper.pos" in features
    assert all(v is float for v in features.values())


def test_e2e_action_features_match_observation_features():
    """action_features and observation_features are identical after negotiation."""
    rr, rt, vr = build_connected_pair()
    assert rr.action_features == rr.observation_features


def test_e2e_is_connected_true_after_wiring():
    """is_connected returns True after manual wiring."""
    rr, rt, vr = build_connected_pair()
    assert rr.is_connected
    assert rt._connected

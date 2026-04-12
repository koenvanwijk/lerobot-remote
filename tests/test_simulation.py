"""
tests/test_simulation.py — Integration simulation tests for RemoteRobot ↔ RemoteTeleop.

Uses InProcessSignaling + InProcessTransport (no real network/WebRTC) but runs the
full mode negotiation, ActionBridge conversion, and action/observation flow — just
like production, but both sides in the same process.

Tests cover multiple ActionMode combinations to verify that negotiation picks the
correct bridge and that values survive the round-trip correctly.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time

import pytest

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent / "lerobot-action-space" / "src"))

from conftest import InProcessSignaling, InProcessTransport, VirtualRobot


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_loop():
    loop = asyncio.new_event_loop()
    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()
    return loop


def _run_both(loop_a, coro_a, loop_b, coro_b, timeout=5):
    """Run two coroutines concurrently on separate event loops, wait for both."""
    fa = asyncio.run_coroutine_threadsafe(coro_a, loop_a)
    fb = asyncio.run_coroutine_threadsafe(coro_b, loop_b)
    fa.result(timeout=timeout)
    fb.result(timeout=timeout)


def build_sim_pair(
    teleop_modes,
    robot_modes,
    joint_names=("j1", "j2", "j3"),
    robot_type_hint: str | None = None,
):
    """
    Build a fully connected (RemoteRobot, RemoteTeleop, VirtualRobot) triple
    using in-process transports. Both sides run on separate event loops (as in
    production) so threading behaviour is realistic.

    If robot_type_hint is set, RemoteTeleop is constructed without explicit
    robot_modes and auto-detects them from the attached robot's name.
    """
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig

    vr = VirtualRobot(joint_names=joint_names)
    # Give the virtual robot a .name so auto-detection works
    if robot_type_hint:
        vr.name = robot_type_hint

    op_sig, rb_sig = InProcessSignaling.linked_pair()

    rr = RemoteRobot(RemoteRobotConfig(room="sim", hz=50), teleop_modes=teleop_modes)
    if robot_type_hint:
        rt = RemoteTeleop(RemoteTeleopConfig(room="sim", hz=50))  # no explicit modes
    else:
        rt = RemoteTeleop(RemoteTeleopConfig(room="sim", hz=50), robot_modes=robot_modes)
    rt.attach_robot(vr)

    op_loop = _make_loop()
    rb_loop = _make_loop()
    rr._loop = op_loop
    rt._loop = rb_loop
    rr._signaling = op_sig
    rt._signaling = rb_sig

    # Negotiate modes (this is the core of connect())
    _run_both(op_loop, rr._negotiate_modes(), rb_loop, rt._negotiate_modes())

    # Wire in-process transport
    op_tr, rb_tr = InProcessTransport.linked_pair()
    asyncio.run_coroutine_threadsafe(op_tr.connect(), op_loop).result(timeout=5)
    asyncio.run_coroutine_threadsafe(rb_tr.connect(), rb_loop).result(timeout=5)

    rr._transport = op_tr
    rt._transport = rb_tr
    op_tr.on_message = rr._on_observation_received
    rb_tr.on_message = rt._on_action_received

    rr._connected = True
    rt._connected = True

    return rr, rt, vr


def send_action_and_receive(rr, rt, action: dict, wait=0.15) -> dict:
    """Helper: convert + send action from operator side, return what RemoteTeleop.get_action() yields."""
    encoded = rr._bridge.convert(action)
    asyncio.run_coroutine_threadsafe(rr._transport.send(encoded), rr._loop).result(timeout=5)
    time.sleep(wait)
    return rt.get_action()


# ---------------------------------------------------------------------------
# Parametrized ActionMode scenarios
# ---------------------------------------------------------------------------

MODE_SCENARIOS = [
    pytest.param(
        "SO101 — norm↔norm (identity)",
        "so101_follower",
        ["joint_absolute_norm", "joint_absolute_deg"],   # teleop advertises
        ["joint_absolute_norm", "joint_absolute_deg"],   # robot advertises
        "joint_absolute_norm",                           # expected bridge teleop mode
        "joint_absolute_norm",                           # expected bridge robot mode
        {"j1.pos": 0.5, "j2.pos": -0.3, "j3.pos": 0.8},
        {"j1.pos": 0.5, "j2.pos": -0.3, "j3.pos": 0.8},  # expected (identity)
        0.01,                                            # tolerance
        id="so101-norm-norm",
    ),
    pytest.param(
        "Koch — same as SO101 (norm↔norm)",
        "koch_follower",
        ["joint_absolute_norm", "joint_absolute_deg"],
        ["joint_absolute_norm", "joint_absolute_deg"],
        "joint_absolute_norm",
        "joint_absolute_norm",
        {"j1.pos": -0.1, "j2.pos": 0.9, "j3.pos": 0.0},
        {"j1.pos": -0.1, "j2.pos": 0.9, "j3.pos": 0.0},
        0.01,
        id="koch-norm-norm",
    ),
    pytest.param(
        "SO101 — deg teleop → norm robot (no numeric conversion, passthrough)",
        "so101_follower",
        ["joint_absolute_deg"],   # teleop only advertises deg
        ["joint_absolute_norm"],  # robot only accepts norm
        "joint_absolute_deg",
        "joint_absolute_norm",
        {"j1.pos": 90.0, "j2.pos": -45.0, "j3.pos": 0.0},
        # ActionBridge marks this as 'exact' — no ConversionStep for deg→norm is registered.
        # Values pass through unchanged. The robot is responsible for interpreting units.
        {"j1.pos": 90.0, "j2.pos": -45.0, "j3.pos": 0.0},
        0.01,
        id="so101-deg-to-norm",
    ),
    pytest.param(
        "SO101 — norm teleop → deg robot (no numeric conversion, passthrough)",
        "so101_follower",
        ["joint_absolute_norm"],  # teleop only advertises norm
        ["joint_absolute_deg"],   # robot only accepts deg
        "joint_absolute_norm",
        "joint_absolute_deg",
        {"j1.pos": 0.5, "j2.pos": -0.25, "j3.pos": 1.0},
        # Same: passthrough, no ConversionStep registered for norm→deg.
        {"j1.pos": 0.5, "j2.pos": -0.25, "j3.pos": 1.0},
        0.01,
        id="so101-norm-to-deg",
    ),
    pytest.param(
        "LeKiwi arm — norm teleop → arm_joint_absolute_norm",
        "lekiwi",
        ["joint_absolute_norm", "joint_absolute_deg"],
        ["arm_joint_absolute_norm", "arm_joint_absolute_deg"],
        "joint_absolute_norm",
        "arm_joint_absolute_norm",
        {"j1.pos": 0.3, "j2.pos": -0.1, "j3.pos": 0.7},
        {"j1.pos": 0.3, "j2.pos": -0.1, "j3.pos": 0.7},  # same unit, different space_type label
        0.01,
        id="lekiwi-arm-norm-norm",
    ),
]


@pytest.mark.parametrize(
    "label,robot_type,teleop_mode_names,robot_mode_names,expected_teleop_mode,expected_robot_mode,action_in,action_expected,tol",
    MODE_SCENARIOS,
)
def test_mode_negotiation_selects_correct_bridge(
    label, robot_type, teleop_mode_names, robot_mode_names,
    expected_teleop_mode, expected_robot_mode,
    action_in, action_expected, tol,
):
    """
    Full simulation: negotiate, send action, verify values match expected
    after ActionBridge conversion.
    """
    from lerobot_action_space import ActionMode
    from lerobot_action_space.compat import _ROBOT_MAP, _TELEOP_MAP

    # Build ActionMode lists from names
    all_modes = {m.name: m for modes in list(_ROBOT_MAP.values()) + list(_TELEOP_MAP.values()) for m in modes}
    # Also include modes from TELEOP_ACTION_MODES
    from lerobot_action_space import TELEOP_ACTION_MODES
    all_modes.update({m.name: m for m in TELEOP_ACTION_MODES})

    teleop_modes = [all_modes[n] for n in teleop_mode_names if n in all_modes]
    robot_modes  = [all_modes[n] for n in robot_mode_names  if n in all_modes]

    assert teleop_modes, f"No teleop modes found for {teleop_mode_names}"
    assert robot_modes,  f"No robot modes found for {robot_mode_names}"

    joint_names = [k.replace(".pos", "") for k in action_in]
    rr, rt, vr = build_sim_pair(teleop_modes, robot_modes, joint_names=joint_names)

    # Verify negotiated bridge modes
    assert rr._bridge.teleop_mode.name == expected_teleop_mode, (
        f"Expected teleop_mode={expected_teleop_mode}, got {rr._bridge.teleop_mode.name}"
    )
    assert rr._bridge.robot_mode.name == expected_robot_mode, (
        f"Expected robot_mode={expected_robot_mode}, got {rr._bridge.robot_mode.name}"
    )

    # Send action and verify round-trip values
    received = send_action_and_receive(rr, rt, action_in)
    assert received, f"[{label}] No action received"
    for key, expected_val in action_expected.items():
        assert key in received, f"[{label}] Key {key!r} missing from received action"
        assert abs(received[key] - expected_val) < tol, (
            f"[{label}] {key}: expected {expected_val:.4f}, got {received[key]:.4f} "
            f"(diff={abs(received[key]-expected_val):.4f} > tol={tol})"
        )


# ---------------------------------------------------------------------------
# Auto-detection tests (robot_type_hint → no explicit robot_modes)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("robot_type", ["so101_follower", "koch_follower"])
def test_auto_detect_modes_from_robot_name(robot_type):
    """
    RemoteTeleop with no explicit robot_modes should auto-detect from
    the attached VirtualRobot.name via compat.get_modes_for_robot().
    """
    from lerobot_action_space import TELEOP_ACTION_MODES

    rr, rt, vr = build_sim_pair(
        teleop_modes=TELEOP_ACTION_MODES,
        robot_modes=None,
        robot_type_hint=robot_type,
    )
    # Bridge should have been set up correctly
    assert rr._bridge is not None
    assert rt._bridge is not None
    assert rr._bridge.teleop_mode.name == "joint_absolute_norm"


# ---------------------------------------------------------------------------
# Start-order independence tests
# ---------------------------------------------------------------------------

def test_operator_starts_first():
    """
    Operator sends capabilities before robot connects.
    The in-process queue acts as a backlog — robot must still receive correctly.
    """
    from lerobot_action_space import TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig

    vr = VirtualRobot()
    op_sig, rb_sig = InProcessSignaling.linked_pair()

    rr = RemoteRobot(RemoteRobotConfig(room="order-test", hz=50), teleop_modes=TELEOP_ACTION_MODES)
    rt = RemoteTeleop(RemoteTeleopConfig(room="order-test", hz=50), robot_modes=SO101_FOLLOWER_MODES)
    rt.attach_robot(vr)

    op_loop = _make_loop()
    rb_loop = _make_loop()
    rr._loop = op_loop
    rt._loop = rb_loop
    rr._signaling = op_sig
    rt._signaling = rb_sig

    results = {}

    def run_operator():
        # Operator sends capabilities immediately (no sleep)
        asyncio.run_coroutine_threadsafe(rr._negotiate_modes(), op_loop).result(timeout=10)
        results["operator"] = "ok"

    def run_robot():
        # Robot starts 300ms later — must still receive operator's capabilities
        time.sleep(0.3)
        asyncio.run_coroutine_threadsafe(rt._negotiate_modes(), rb_loop).result(timeout=10)
        results["robot"] = "ok"

    t1 = threading.Thread(target=run_operator)
    t2 = threading.Thread(target=run_robot)
    t1.start(); t2.start()
    t1.join(timeout=12); t2.join(timeout=12)

    assert results.get("operator") == "ok", "Operator negotiation failed"
    assert results.get("robot") == "ok", "Robot negotiation failed (started late)"
    assert rr._bridge is not None
    assert rt._bridge is not None


def test_robot_starts_first():
    """
    Robot side starts and blocks waiting for operator capabilities.
    Operator connects 300ms later — negotiation must still complete.
    """
    from lerobot_action_space import TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig

    vr = VirtualRobot()
    op_sig, rb_sig = InProcessSignaling.linked_pair()

    rr = RemoteRobot(RemoteRobotConfig(room="order-test-2", hz=50), teleop_modes=TELEOP_ACTION_MODES)
    rt = RemoteTeleop(RemoteTeleopConfig(room="order-test-2", hz=50), robot_modes=SO101_FOLLOWER_MODES)
    rt.attach_robot(vr)

    op_loop = _make_loop()
    rb_loop = _make_loop()
    rr._loop = op_loop
    rt._loop = rb_loop
    rr._signaling = op_sig
    rt._signaling = rb_sig

    results = {}

    def run_robot():
        # Robot starts first, blocks waiting for operator
        asyncio.run_coroutine_threadsafe(rt._negotiate_modes(), rb_loop).result(timeout=10)
        results["robot"] = "ok"

    def run_operator():
        # Operator starts 300ms later
        time.sleep(0.3)
        asyncio.run_coroutine_threadsafe(rr._negotiate_modes(), op_loop).result(timeout=10)
        results["operator"] = "ok"

    t1 = threading.Thread(target=run_robot)
    t2 = threading.Thread(target=run_operator)
    t1.start(); t2.start()
    t1.join(timeout=12); t2.join(timeout=12)

    assert results.get("robot") == "ok", "Robot negotiation failed (started first)"
    assert results.get("operator") == "ok", "Operator negotiation failed"
    assert rr._bridge is not None
    assert rt._bridge is not None


# ---------------------------------------------------------------------------
# Sinusoidal action stream test (realistic data collection simulation)
# ---------------------------------------------------------------------------

def test_sinusoidal_action_stream():
    """
    Simulate a real teleop session: operator sends 20 frames of sinusoidal
    joint positions at ~30Hz. Robot side must receive all of them in order.
    """
    from lerobot_action_space import TELEOP_ACTION_MODES
    from lerobot_action_space.compat import SO101_FOLLOWER_MODES

    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "gripper")
    rr, rt, vr = build_sim_pair(TELEOP_ACTION_MODES, SO101_FOLLOWER_MODES, joint_names=joint_names)

    N = 20
    sent = []
    for i in range(N):
        t = i / N * 2 * math.pi
        action = {f"{j}.pos": round(math.sin(t + k * 0.5), 4) for k, j in enumerate(joint_names)}
        sent.append(action)
        encoded = rr._bridge.convert(action)
        asyncio.run_coroutine_threadsafe(rr._transport.send(encoded), rr._loop).result(timeout=2)
        time.sleep(1 / 30)  # ~30Hz

    time.sleep(0.3)

    received = []
    while not rt._action_queue.empty():
        received.append(rt._action_queue.get_nowait())

    assert len(received) == N, f"Expected {N} actions, got {len(received)}"
    # Check first and last frame values are close
    for key in sent[0]:
        assert abs(received[0][key] - sent[0][key]) < 0.02, f"First frame mismatch on {key}"
        assert abs(received[-1][key] - sent[-1][key]) < 0.02, f"Last frame mismatch on {key}"

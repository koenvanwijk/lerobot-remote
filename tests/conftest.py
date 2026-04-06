"""
tests/conftest.py — shared fixtures and WebRTC/signaling mocks.

WebRTCTransport and SignalingClient are mocked away so tests run without
aiortc / ffmpeg. The mock wires operator ↔ robot via in-process asyncio queues,
giving us a realistic message-passing test without any network I/O.
"""

from __future__ import annotations

import asyncio
import queue as queue_module
import threading
from typing import Any
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

import sys, pathlib
_repo = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(_repo / "transport" / "src"))
sys.path.insert(0, str(_repo / "robot" / "src"))
sys.path.insert(0, str(_repo / "teleop" / "src"))
sys.path.insert(0, str(_repo.parent / "lerobot-action-space" / "src"))
sys.path.insert(0, str(_repo.parent / "lerobot-matchmaker" / "src"))


# ---------------------------------------------------------------------------
# In-process linked transport pair
# ---------------------------------------------------------------------------

class InProcessTransport:
    """
    Mimics WebRTCTransport using threading.Queue for cross-loop compatibility.
    Create a linked pair with InProcessTransport.linked_pair().
    """

    def __init__(self):
        self._send_queue: queue_module.Queue | None = None
        self._recv_queue: queue_module.Queue | None = None
        self.on_message = None
        self._recv_thread: threading.Thread | None = None
        self._running = False

    @classmethod
    def linked_pair(cls):
        op = cls()
        rb = cls()
        shared_op_to_rb: queue_module.Queue = queue_module.Queue()
        shared_rb_to_op: queue_module.Queue = queue_module.Queue()
        op._send_queue = shared_op_to_rb
        op._recv_queue = shared_rb_to_op
        rb._send_queue = shared_rb_to_op
        rb._recv_queue = shared_op_to_rb
        return op, rb

    async def connect(self):
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    async def send(self, message: dict):
        self._send_queue.put(message)

    async def close(self):
        self._running = False

    def _recv_loop(self):
        while self._running:
            try:
                msg = self._recv_queue.get(timeout=0.1)
                if self.on_message:
                    self.on_message(msg)
            except queue_module.Empty:
                continue


class InProcessSignaling:
    """
    Mimics SignalingClient using threading.Queue for cross-loop compatibility.
    Create a linked pair with InProcessSignaling.linked_pair().
    """

    def __init__(self):
        self._send_queue: queue_module.Queue | None = None
        self._recv_queue: queue_module.Queue | None = None

    @classmethod
    def linked_pair(cls):
        op = cls()
        rb = cls()
        shared_op_to_rb: queue_module.Queue = queue_module.Queue()
        shared_rb_to_op: queue_module.Queue = queue_module.Queue()
        op._send_queue = shared_op_to_rb
        op._recv_queue = shared_rb_to_op
        rb._send_queue = shared_rb_to_op
        rb._recv_queue = shared_op_to_rb
        return op, rb

    async def connect(self):
        pass

    async def send(self, message: dict):
        self._send_queue.put(message)

    async def receive(self) -> dict:
        loop = asyncio.get_event_loop()
        # Block in executor to avoid holding up the event loop
        return await loop.run_in_executor(None, self._recv_queue.get)

    async def close(self):
        pass


# ---------------------------------------------------------------------------
# Virtual robot stub
# ---------------------------------------------------------------------------

class VirtualRobot:
    """Minimal Robot stub for testing."""

    def __init__(self, joint_names=("j1", "j2", "j3")):
        self._joints = {f"{n}.pos": 0.0 for n in joint_names}
        self.received_actions: list[dict] = []
        self.is_connected = False

    @property
    def action_features(self) -> dict:
        return {k: float for k in self._joints}

    @property
    def observation_features(self) -> dict:
        return {k: float for k in self._joints}

    def connect(self, calibrate=True):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_observation(self) -> dict:
        return dict(self._joints)

    def send_action(self, action: dict) -> dict:
        self.received_actions.append(action)
        self._joints.update(action)
        return action


# ---------------------------------------------------------------------------
# Helpers to wire RemoteRobot + RemoteTeleop with in-process transports
# ---------------------------------------------------------------------------

def make_connected_pair(teleop_modes, robot_modes):
    """
    Build a (remote_robot, remote_teleop, virtual_robot) triple fully connected
    via in-process transports. Returns them ready to use.
    """
    from lerobot_robot_remote.remote_robot import RemoteRobot, RemoteRobotConfig
    from lerobot_teleoperator_remote.remote_teleop import RemoteTeleop, RemoteTeleopConfig

    virtual_robot = VirtualRobot()
    op_sig, rb_sig = InProcessSignaling.linked_pair()

    robot_config = RemoteRobotConfig(signaling_url="mock", room="test", hz=30)
    teleop_config = RemoteTeleopConfig(signaling_url="mock", room="test", hz=30)

    remote_robot = RemoteRobot(robot_config, teleop_modes=teleop_modes)
    remote_teleop = RemoteTeleop(teleop_config, robot_modes=robot_modes)
    remote_teleop.attach_robot(virtual_robot)

    # Inject in-process transports instead of real WebRTC
    op_transport, rb_transport = None, None  # created inside event loops

    def start_robot():
        loop = asyncio.new_event_loop()
        remote_robot._loop = loop
        remote_robot._signaling = rb_sig  # note: robot is "operator" role here, signaling is swapped
        # Actually: robot is RemoteRobot → operator role in signaling
        # RemoteRobot is on the operator/control side
        threading.Thread(target=loop.run_forever, daemon=True).start()
        return loop

    def start_teleop():
        loop = asyncio.new_event_loop()
        remote_teleop._loop = loop
        threading.Thread(target=loop.run_forever, daemon=True).start()
        return loop

    return remote_robot, remote_teleop, virtual_robot, op_sig, rb_sig

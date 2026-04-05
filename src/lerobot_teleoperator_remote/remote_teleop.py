"""
remote_teleop.py — RemoteTeleop

Implements the LeRobot Teleoperator interface for a teleop running on a remote machine.

connect() waits for the operator's capabilities over the signaling channel,
sends back the robot's modes, receives the agreed ActionMode pair, then builds
the ActionBridge and establishes WebRTC.
get_action() receives encoded actions over WebRTC, decodes via ActionBridge.
send_feedback() sends feedback back to the remote operator over WebRTC.

Used on the robot side as a drop-in replacement for a physical teleoperator.
"""

from __future__ import annotations

import asyncio
import logging
import queue
import threading
from dataclasses import dataclass
from typing import Any

from lerobot_action_space import ActionBridge, ActionMode

from lerobot_remote_transport import SignalingClient, WebRTCTransport
from lerobot_remote_transport.modes import action_mode_to_dict, action_mode_from_dict

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

try:
    from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig

    @TeleoperatorConfig.register_subclass("remote_teleop")
    @dataclass
    class RemoteTeleopConfig(TeleoperatorConfig):
        signaling_url: str = "http://localhost:8080"
        room: str = "default"
        hz: int = 30

except ImportError:
    @dataclass
    class RemoteTeleopConfig:  # type: ignore[no-redef]
        signaling_url: str = "http://localhost:8080"
        room: str = "default"
        hz: int = 30

    class Teleoperator:  # type: ignore[no-redef]
        pass


# ---------------------------------------------------------------------------
# RemoteTeleop
# ---------------------------------------------------------------------------

class RemoteTeleop(Teleoperator):
    """
    Proxy teleoperator that receives actions over WebRTC from a remote operator.

    During connect(), the robot's action_modes are sent to the operator, and the
    operator responds with the agreed ActionMode pair (selected via ActionBridge.auto()).
    The ActionBridge is built from the agreed modes.

    get_action() blocks up to one control interval and falls back to the last known
    action on timeout so the robot stays safe.

    Usage:
        from lerobot_action_space.compat import SO100_FOLLOWER_MODES
        config = RemoteTeleopConfig(signaling_url="http://matchmaker:8080", room="arm-1")
        teleop = RemoteTeleop(config, robot_modes=SO100_FOLLOWER_MODES)
        teleop.connect()
        action = teleop.get_action()
        teleop.send_feedback({"force": 0.3})
        teleop.disconnect()
    """

    config_class = RemoteTeleopConfig
    name = "remote_teleop"

    def __init__(self, config: RemoteTeleopConfig, robot_modes: list[ActionMode]):
        self.config = config
        self._robot_modes = robot_modes
        self._bridge: ActionBridge | None = None

        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._transport: WebRTCTransport | None = None
        self._signaling: SignalingClient | None = None

        self._connected = False
        self._action_queue: queue.Queue[dict] = queue.Queue()
        self._last_action: dict[str, Any] = {}

    # ------------------------------------------------------------------
    # LeRobot required methods

    def connect(self) -> None:
        """
        1. Start background event loop.
        2. Wait for operator's teleop_modes over signaling.
        3. Send our robot_modes back.
        4. Receive agreed modes, build ActionBridge.
        5. Establish WebRTC DataChannel (answerer role).
        """
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

        self._signaling = SignalingClient(
            server_url=self.config.signaling_url,
            room=self.config.room,
            role="robot",
        )
        self._run_async(self._signaling.connect())
        self._run_async(self._negotiate_modes())

        self._transport = WebRTCTransport(self._signaling, role="robot")
        self._transport.on_message = self._on_action_received
        self._run_async(self._transport.connect())

        self._connected = True
        assert self._bridge is not None
        logger.info(
            "RemoteTeleop connected: room=%s, agreed=%s→%s",
            self.config.room,
            self._bridge.teleop_mode.name,
            self._bridge.robot_mode.name,
        )

    def disconnect(self) -> None:
        if self._transport:
            self._run_async(self._transport.close())
        if self._signaling:
            self._run_async(self._signaling.close())
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread:
            self._loop_thread.join(timeout=5)
        self._connected = False
        logger.info("RemoteTeleop disconnected")

    def get_action(self) -> dict[str, Any]:
        """
        Return the next decoded action from the remote operator.

        Blocks up to one control interval; returns last known action on timeout.
        """
        assert self._bridge is not None, "call connect() first"
        try:
            raw = self._action_queue.get(timeout=1.0 / self.config.hz)
            decoded = self._bridge.convert(raw)
            self._last_action = decoded
            return decoded
        except queue.Empty:
            logger.debug("get_action timeout — returning last known action")
            return dict(self._last_action)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send feedback (e.g. force/torque) back to the remote operator."""
        self._run_async(self._transport.send({"__feedback__": True, **feedback}))

    # ------------------------------------------------------------------
    # Mode negotiation

    async def _negotiate_modes(self) -> None:
        """
        Wait for the operator's teleop_modes, send our robot_modes,
        receive the agreed pair, build the ActionBridge.
        """
        # 1. Wait for operator capabilities
        msg = await self._signaling.receive()
        assert msg.get("type") == "capabilities", f"Expected capabilities, got: {msg.get('type')}"
        teleop_modes = [action_mode_from_dict(m) for m in msg["teleop_modes"]]
        logger.debug("Received operator capabilities (%d modes)", len(teleop_modes))

        # 2. Send our robot capabilities
        await self._signaling.send({
            "type": "capabilities",
            "robot_modes": [action_mode_to_dict(m) for m in self._robot_modes],
        })
        logger.debug("Sent robot capabilities (%d modes)", len(self._robot_modes))

        # 3. Receive agreed modes chosen by operator
        msg = await self._signaling.receive()
        assert msg.get("type") == "mode_agreed", f"Expected mode_agreed, got: {msg.get('type')}"
        teleop_mode = action_mode_from_dict(msg["teleop_mode"])
        robot_mode = action_mode_from_dict(msg["robot_mode"])

        # 4. Build bridge with the agreed pair (operator already validated quality)
        self._bridge = ActionBridge(teleop_mode, robot_mode)
        logger.info("Mode agreed — bridge plan:\n%s", self._bridge.explain())

    # ------------------------------------------------------------------

    def _run_async(self, coro) -> Any:
        assert self._loop is not None
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=30)

    def _on_action_received(self, msg: dict) -> None:
        if msg.get("__feedback__"):
            return
        self._action_queue.put_nowait(msg)

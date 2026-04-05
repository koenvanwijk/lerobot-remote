"""
remote_robot.py — RemoteRobot

Implements the LeRobot Robot interface for a robot running on a remote machine.

connect() negotiates ActionModes with the remote robot via the signaling channel,
auto-selects the best pair via ActionBridge.auto(), then establishes WebRTC.
send_action() encodes via the negotiated ActionBridge and sends over DataChannel.
get_observation() returns the last observation received from the remote robot.

Used on the operator/control side as a drop-in replacement for a physical robot.
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
from lerobot_remote_transport.signaling import ProtocolError
from lerobot_remote_transport.modes import action_mode_to_dict, action_mode_from_dict

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

try:
    from lerobot.robots.robot import Robot
    from lerobot.robots.config import RobotConfig

    @RobotConfig.register_subclass("remote_robot")
    @dataclass
    class RemoteRobotConfig(RobotConfig):
        signaling_url: str = "http://localhost:8080"
        room: str = "default"
        hz: int = 30

except ImportError:
    @dataclass
    class RemoteRobotConfig:  # type: ignore[no-redef]
        signaling_url: str = "http://localhost:8080"
        room: str = "default"
        hz: int = 30

    class Robot:  # type: ignore[no-redef]
        pass


# ---------------------------------------------------------------------------
# RemoteRobot
# ---------------------------------------------------------------------------

class RemoteRobot(Robot):
    """
    Proxy robot that forwards actions over WebRTC to a remote physical robot.

    During connect(), capabilities are exchanged with the RemoteTeleop over the
    signaling channel. ActionBridge.auto() selects the best teleop→robot mode pair.
    The agreed modes are sent back to the robot side before WebRTC starts.

    Usage:
        from lerobot_action_space import TELEOP_ACTION_MODES
        config = RemoteRobotConfig(signaling_url="http://matchmaker:8080", room="arm-1")
        robot = RemoteRobot(config, teleop_modes=TELEOP_ACTION_MODES)
        robot.connect()
        robot.send_action({"joint1.pos": 0.5, ...})
        obs = robot.get_observation()
        robot.disconnect()
    """

    config_class = RemoteRobotConfig
    name = "remote_robot"

    def __init__(self, config: RemoteRobotConfig, teleop_modes: list[ActionMode]):
        self.config = config
        self._teleop_modes = teleop_modes
        self._bridge: ActionBridge | None = None

        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._transport: WebRTCTransport | None = None
        self._signaling: SignalingClient | None = None

        self._connected = False
        self._last_observation: dict[str, Any] = {}
        self._robot_features: dict[str, Any] = {}  # set after mode negotiation
        self._obs_lock = threading.Lock()

    # ------------------------------------------------------------------
    # LeRobot required properties

    @property
    def observation_features(self) -> dict[str, Any]:
        if self._robot_features:
            return self._robot_features
        if self._bridge:
            return {f"{self._bridge.robot_mode.name}.obs": float}
        return {"remote.obs": float}

    @property
    def action_features(self) -> dict[str, Any]:
        if self._robot_features:
            return self._robot_features
        if self._bridge:
            return {f"{self._bridge.robot_mode.name}.action": float}
        return {"remote.action": float}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True  # calibration is handled by the remote physical robot

    # ------------------------------------------------------------------
    # LeRobot required methods

    def connect(self, calibrate: bool = True) -> None:
        """
        1. Start background event loop.
        2. Negotiate ActionModes with remote robot via signaling.
        3. Build ActionBridge from negotiated modes.
        4. Establish WebRTC DataChannel.
        """
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

        self._signaling = SignalingClient(
            server_url=self.config.signaling_url,
            room=self.config.room,
            role="operator",
        )
        self._run_async(self._signaling.connect())
        self._run_async(self._negotiate_modes())

        self._transport = WebRTCTransport(self._signaling, role="operator")
        self._transport.on_message = self._on_observation_received
        self._run_async(self._transport.connect())

        self._connected = True
        assert self._bridge is not None
        logger.info(
            "RemoteRobot connected: room=%s, agreed=%s→%s",
            self.config.room,
            self._bridge.teleop_mode.name,
            self._bridge.robot_mode.name,
        )
        logger.info("Bridge plan:\n%s", self._bridge.explain())

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
        logger.info("RemoteRobot disconnected")

    def calibrate(self) -> None:
        pass  # handled by the remote physical robot

    def configure(self) -> None:
        pass  # configured at construction time

    def get_observation(self) -> dict[str, Any]:
        with self._obs_lock:
            return dict(self._last_observation)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        assert self._bridge is not None, "call connect() first"
        encoded = self._bridge.convert(action)
        self._run_async(self._transport.send(encoded))
        return encoded

    # ------------------------------------------------------------------
    # Mode negotiation

    async def _negotiate_modes(self) -> None:
        """
        Send our teleop_modes to the robot, receive its robot_modes,
        auto-select the best pair, confirm the agreed modes back.
        """
        # 1. Send our capabilities
        await self._signaling.send({
            "type": "capabilities",
            "teleop_modes": [action_mode_to_dict(m) for m in self._teleop_modes],
        })
        logger.debug("Sent teleop capabilities (%d modes)", len(self._teleop_modes))

        # 2. Wait for robot capabilities
        msg = await self._signaling.receive()
        if msg.get("type") != "capabilities":
            raise ProtocolError(f'Expected capabilities, got: {msg.get("type")!r} — full msg: {msg}')
        robot_modes = [action_mode_from_dict(m) for m in msg["robot_modes"]]
        logger.debug("Received robot capabilities (%d modes)", len(robot_modes))

        # 3. Auto-select best pair
        self._bridge = ActionBridge.auto(self._teleop_modes, robot_modes)

        # 4. Confirm agreed modes to robot (robot responds with its action_features)
        await self._signaling.send({
            "type": "mode_agreed",
            "teleop_mode": action_mode_to_dict(self._bridge.teleop_mode),
            "robot_mode": action_mode_to_dict(self._bridge.robot_mode),
        })

        # 5. Receive robot's action_features (joint key names + types)
        msg = await self._signaling.receive()
        if msg.get("type") == "features":
            self._robot_features = {k: float for k in msg.get("keys", [])}
            logger.info("Robot features received: %s", list(self._robot_features.keys()))

    # ------------------------------------------------------------------

    def _run_async(self, coro) -> Any:
        assert self._loop is not None
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=30)

    def _on_observation_received(self, msg: dict) -> None:
        # Only store messages tagged as observations; ignore feedback and other control messages
        if not msg.get("__obs__"):
            logger.debug("RemoteRobot: ignoring non-obs message (keys=%s)", list(msg.keys()))
            return
        obs = {k: v for k, v in msg.items() if not k.startswith("__")}
        with self._obs_lock:
            self._last_observation = obs

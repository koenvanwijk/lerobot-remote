# lerobot-remote

LeRobot Robot and Teleoperator plugins for remote control over WebRTC.
One pip package (`lerobot-remote`) installs three Python packages:

| Python package | LeRobot role | `name` attr | Used on |
|---|---|---|---|
| `lerobot_robot_remote` | Robot plugin | `"remote_robot"` | operator side |
| `lerobot_teleoperator_remote` | Teleoperator plugin | `"remote_teleop"` | robot side |
| `lerobot_remote_transport` | shared transport | — | internal |

Depends on `lerobot-action-space` for `ActionMode` and `ActionBridge`.

## Package layout

```
src/
  lerobot_robot_remote/
    remote_robot.py         # RemoteRobot + RemoteRobotConfig
  lerobot_teleoperator_remote/
    remote_teleop.py        # RemoteTeleop + RemoteTeleopConfig
  lerobot_remote_transport/
    webrtc.py               # WebRTCTransport (aiortc DataChannel wrapper)
    signaling.py            # SignalingClient (HTTP long-poll to matchmaker)
    modes.py                # ActionMode ↔ JSON dict serialization
```

## Architecture

```
[operator machine]                          [robot machine]
  local teleop                                physical robot
      │ get_action()                              ▲ send_action()
      ▼                                           │
  LeRobot framework                          LeRobot framework
      │ send_action()                             │ get_action()
      ▼                                           │
  RemoteRobot                              RemoteTeleop
      │ bridge.convert()                          │ bridge.convert()
      │ WebRTCTransport.send()                    │ WebRTCTransport.on_message
      ▼                                           │
  ══════════════ WebRTC DataChannel ══════════════
```

## connect() sequence

Before WebRTC starts, modes are negotiated over the signaling channel:

```
operator → matchmaker → robot:  {"type": "capabilities", "teleop_modes": [...]}
robot    → matchmaker → operator: {"type": "capabilities", "robot_modes": [...]}
operator: ActionBridge.auto(teleop_modes, robot_modes)  → best pair
operator → matchmaker → robot:  {"type": "mode_agreed", "teleop_mode": {...}, "robot_mode": {...}}
─── WebRTC SDP offer/answer ───────────────────────────────────────────────────
```

Both sides log `bridge.explain()` after mode agreement.

## Key classes

### `RemoteRobot` (`lerobot_robot_remote/remote_robot.py`)
- Constructor: `RemoteRobot(config: RemoteRobotConfig, teleop_modes: list[ActionMode])`
- `connect(calibrate=True)` — negotiates modes, builds `ActionBridge`, opens DataChannel
- `send_action(action)` — `bridge.convert(action)` → JSON → DataChannel; returns encoded action
- `get_observation()` — returns last observation received from remote robot (see open issue #1)
- `disconnect()` — closes WebRTC + signaling, stops background event loop
- Runs asyncio in a daemon thread; all public methods are synchronous (LeRobot compatible)

### `RemoteTeleop` (`lerobot_teleoperator_remote/remote_teleop.py`)
- Constructor: `RemoteTeleop(config: RemoteTeleopConfig, robot_modes: list[ActionMode])`
- `connect()` — waits for capabilities, receives agreed modes, builds `ActionBridge`, answers WebRTC offer
- `get_action()` — dequeues received action, `bridge.convert()`, returns decoded dict;
  falls back to last known action on timeout (robot safety)
- `send_feedback(feedback)` — sends `{"__feedback__": True, ...}` over DataChannel
- `disconnect()` — closes WebRTC + signaling, stops background event loop

### `WebRTCTransport` (`lerobot_remote_transport/webrtc.py`)
- `role="operator"` → WebRTC offerer (creates DataChannel, sends SDP offer)
- `role="robot"` → WebRTC answerer (waits for offer, sends SDP answer)
- Single DataChannel named `"actions"` for all messages
- `on_message: Callable[[dict], None]` — set before `connect()`

### `SignalingClient` (`lerobot_remote_transport/signaling.py`)
- HTTP long-poll to lerobot-matchmaker
- `role="operator"` or `role="robot"` — determines which queue to poll
- `send(msg)` / `receive()` are ordered; capabilities messages arrive before SDP messages

### `action_mode_to_dict` / `action_mode_from_dict` (`lerobot_remote_transport/modes.py`)
- Serialize `ActionMode` and `FrameDefinition` to plain JSON-compatible dicts
- Used during the capabilities handshake

## Usage example

**Operator side:**
```python
from lerobot_action_space import TELEOP_ACTION_MODES
from lerobot_robot_remote import RemoteRobot, RemoteRobotConfig

config = RemoteRobotConfig(signaling_url="http://matchmaker:8080", room="arm-1")
robot = RemoteRobot(config, teleop_modes=TELEOP_ACTION_MODES)
robot.connect()
# use robot like any LeRobot robot
robot.send_action({"joint1.pos": 0.5})
robot.disconnect()
```

**Robot side:**
```python
from lerobot_action_space.compat import SO100_FOLLOWER_MODES
from lerobot_teleoperator_remote import RemoteTeleop, RemoteTeleopConfig

config = RemoteTeleopConfig(signaling_url="http://matchmaker:8080", room="arm-1")
teleop = RemoteTeleop(config, robot_modes=SO100_FOLLOWER_MODES)
teleop.connect()
action = teleop.get_action()  # blocks up to 1/hz seconds
teleop.disconnect()
```

## Install

```bash
pip install git+https://github.com/koenvanwijk/lerobot-remote
```

## Open issues

- **#1 Observation reverse channel** — `RemoteRobot.get_observation()` returns the last
  dict received over the DataChannel, but `RemoteTeleop` never sends observations.
  Need to call `transport.send(obs)` from the robot side after each `get_observation()`
  on the physical robot, and filter `__observation__` messages from action messages.

- **#2 ICE / NAT traversal** — `WebRTCTransport` has no STUN/TURN configuration.
  Works on LAN; will fail across NAT without a TURN server.
  Fix: add `iceServers` config to `RemoteRobotConfig` / `RemoteTeleopConfig` and pass
  to `RTCPeerConnection(configuration=...)`.

- **#3 lerobot import paths** — `Robot`, `RobotConfig`, `Teleoperator`, `TeleopConfig`
  are imported with a try/except fallback. The actual lerobot module paths
  (`lerobot.robots.robot`, `lerobot.teleoperators.teleoperator`) are unverified —
  need to install lerobot and confirm the paths are correct.

- **#4 Single DataChannel for all message types** — capabilities, mode_agreed, SDP, actions,
  and feedback all flow through the same signaling channel or DataChannel. Order-dependent.
  Consider typed sub-channels or a message envelope with `type` routing.

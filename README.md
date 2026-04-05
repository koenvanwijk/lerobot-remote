# lerobot-remote

LeRobot Robot and Teleoperator plugins for remote control over WebRTC.

```bash
pip install git+https://github.com/koenvanwijk/lerobot-remote
```

**Operator side** — replace your local robot with a remote one:
```python
from lerobot_action_space import TELEOP_ACTION_MODES
from lerobot_robot_remote import RemoteRobot, RemoteRobotConfig

robot = RemoteRobot(
    RemoteRobotConfig(signaling_url="http://matchmaker:8080", room="arm-1"),
    teleop_modes=TELEOP_ACTION_MODES,
)
robot.connect()  # negotiates ActionMode, opens WebRTC
robot.send_action({"joint1.pos": 0.5})
robot.disconnect()
```

**Robot side** — replace your local teleop with a remote one:
```python
from lerobot_action_space.compat import SO100_FOLLOWER_MODES
from lerobot_teleoperator_remote import RemoteTeleop, RemoteTeleopConfig

teleop = RemoteTeleop(
    RemoteTeleopConfig(signaling_url="http://matchmaker:8080", room="arm-1"),
    robot_modes=SO100_FOLLOWER_MODES,
)
teleop.connect()
action = teleop.get_action()
teleop.disconnect()
```

Requires [lerobot-matchmaker](https://github.com/koenvanwijk/lerobot-matchmaker) as the signaling server.
See [CLAUDE.md](CLAUDE.md) for architecture details and open issues.

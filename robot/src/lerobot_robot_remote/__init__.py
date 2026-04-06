"""lerobot_robot_remote — LeRobot Robot plugin for remote robots over WebRTC."""

from .remote_robot import RemoteRobot, RemoteRobotConfig

# Also ensure lerobot_teleoperator_remote gets imported (both plugins live in same repo)
try:
    import lerobot_teleoperator_remote  # noqa: F401
except ImportError:
    pass

__all__ = ["RemoteRobot", "RemoteRobotConfig"]

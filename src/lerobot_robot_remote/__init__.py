"""lerobot_robot_remote — LeRobot Robot plugin for remote robots over WebRTC."""

from .remote_robot import RemoteRobot, RemoteRobotConfig

# Ensure teleoperator plugin is also registered in single-package layout.
try:
	import lerobot_teleoperator_remote  # noqa: F401
except ImportError:
	pass

__all__ = ["RemoteRobot", "RemoteRobotConfig"]

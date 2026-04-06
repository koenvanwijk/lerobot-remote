"""lerobot_remote_transport — shared WebRTC transport for lerobot-robot-remote and lerobot-teleoperator-remote."""

from .signaling import SignalingClient
from .modes import action_mode_to_dict, action_mode_from_dict

def __getattr__(name):
    if name == "WebRTCTransport":
        from .webrtc import WebRTCTransport
        return WebRTCTransport
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = ["SignalingClient", "WebRTCTransport", "action_mode_to_dict", "action_mode_from_dict"]

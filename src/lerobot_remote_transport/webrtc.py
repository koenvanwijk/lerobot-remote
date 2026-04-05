"""
webrtc.py — WebRTC DataChannel transport

Wraps aiortc to provide a simple send/receive interface over a DataChannel.
Used by lerobot-robot-remote (answerer) and lerobot-teleoperator-remote (offerer).
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Callable

from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel

from .signaling import SignalingClient

logger = logging.getLogger(__name__)

CHANNEL_NAME = "actions"


class WebRTCTransport:
    """
    Manages a single WebRTC DataChannel for action streaming.

    role="operator" → offerer (creates DataChannel, sends SDP offer)
    role="robot"    → answerer (waits for offer, sends SDP answer)

    Actions are serialized as JSON over the DataChannel.
    Set on_message to receive incoming dicts.
    """

    def __init__(self, signaling: SignalingClient, role: str):
        assert role in ("robot", "operator")
        self._signaling = signaling
        self._role = role
        self._pc = RTCPeerConnection()
        self._channel: RTCDataChannel | None = None
        self._ready = asyncio.Event()
        self.on_message: Callable[[dict], None] | None = None

    async def connect(self) -> None:
        if self._role == "operator":
            await self._connect_as_offerer()
        else:
            await self._connect_as_answerer()
        await self._ready.wait()
        logger.info("WebRTCTransport ready (role=%s)", self._role)

    async def send(self, message: dict) -> None:
        assert self._channel is not None and self._channel.readyState == "open"
        self._channel.send(json.dumps(message))

    async def close(self) -> None:
        await self._pc.close()

    async def _connect_as_offerer(self) -> None:
        channel = self._pc.createDataChannel(CHANNEL_NAME)
        self._channel = channel

        @channel.on("open")
        def on_open():
            self._ready.set()

        @channel.on("message")
        def on_message(msg):
            if self.on_message:
                self.on_message(json.loads(msg))

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        await self._signaling.send({"type": "offer", "sdp": self._pc.localDescription.sdp})

        answer_msg = await self._signaling.receive()
        answer = RTCSessionDescription(sdp=answer_msg["sdp"], type="answer")
        await self._pc.setRemoteDescription(answer)

    async def _connect_as_answerer(self) -> None:
        @self._pc.on("datachannel")
        def on_datachannel(channel: RTCDataChannel):
            self._channel = channel

            @channel.on("open")
            def on_open():
                self._ready.set()

            @channel.on("message")
            def on_message(msg):
                if self.on_message:
                    self.on_message(json.loads(msg))

        offer_msg = await self._signaling.receive()
        offer = RTCSessionDescription(sdp=offer_msg["sdp"], type="offer")
        await self._pc.setRemoteDescription(offer)

        answer = await self._pc.createAnswer()
        await self._pc.setLocalDescription(answer)
        await self._signaling.send({"type": "answer", "sdp": self._pc.localDescription.sdp})

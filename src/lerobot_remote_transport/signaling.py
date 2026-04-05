"""
signaling.py — WebRTC signaling client

Connects to lerobot-matchmaker to exchange SDP offer/answer and ICE candidates
between lerobot-teleoperator-remote (offerer) and lerobot-robot-remote (answerer).
"""

from __future__ import annotations

import asyncio
import logging

import aiohttp

logger = logging.getLogger(__name__)


class SignalingClient:
    """
    Connects to the matchmaker signaling server via HTTP long-poll.

    Usage:
        client = SignalingClient(server_url="http://localhost:8080", room="my-robot", role="robot")
        await client.connect()
        await client.send({"type": "offer", "sdp": ...})
        msg = await client.receive()
        await client.close()
    """

    def __init__(self, server_url: str, room: str, role: str):
        assert role in ("robot", "operator"), f"role must be 'robot' or 'operator', got {role!r}"
        self._url = server_url.rstrip("/")
        self._room = room
        self._role = role
        self._session: aiohttp.ClientSession | None = None
        self._recv_queue: asyncio.Queue[dict] = asyncio.Queue()
        self._poll_task: asyncio.Task | None = None

    async def connect(self) -> None:
        self._session = aiohttp.ClientSession()
        self._poll_task = asyncio.create_task(self._poll_loop())
        logger.info("SignalingClient connected: room=%s role=%s", self._room, self._role)

    async def send(self, message: dict) -> None:
        assert self._session is not None, "call connect() first"
        url = f"{self._url}/signal/{self._room}/{self._role}/send"
        async with self._session.post(url, json=message) as resp:
            resp.raise_for_status()

    async def receive(self) -> dict:
        return await self._recv_queue.get()

    async def close(self) -> None:
        if self._poll_task:
            self._poll_task.cancel()
        if self._session:
            await self._session.close()

    async def _poll_loop(self) -> None:
        assert self._session is not None
        peer_role = "operator" if self._role == "robot" else "robot"
        url = f"{self._url}/signal/{self._room}/{peer_role}/recv"
        while True:
            try:
                async with self._session.get(url, timeout=aiohttp.ClientTimeout(total=30)) as resp:
                    if resp.status == 200:
                        data = await resp.json()
                        await self._recv_queue.put(data)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.warning("Signaling poll error: %s", exc)
                await asyncio.sleep(1)

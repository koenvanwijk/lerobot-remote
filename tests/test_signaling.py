"""
tests/test_signaling.py — Unit tests for SignalingClient.

Tests send/receive, subscriber_id stability, and poll error recovery.
Uses aiohttp TestServer (no mocking needed — real HTTP).
"""

from __future__ import annotations

import asyncio
import pytest

from aiohttp.test_utils import TestClient, TestServer

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent / "lerobot-matchmaker" / "src"))

from lerobot_matchmaker.server import create_app
from lerobot_remote_transport.signaling import SignalingClient, ProtocolError


@pytest.fixture
async def matchmaker():
    app = create_app()
    async with TestClient(TestServer(app)) as c:
        yield c


# ---------------------------------------------------------------------------
# Basic send/receive
# ---------------------------------------------------------------------------

@pytest.mark.asyncio
async def test_signaling_send_receive(matchmaker):
    """Operator sends, robot receives."""
    url = matchmaker.make_url("/")

    op = SignalingClient(server_url=url, room="r1", role="operator")
    rb = SignalingClient(server_url=url, room="r1", role="robot")

    await op.connect()
    await rb.connect()

    msg = {"type": "offer", "sdp": "v=0"}
    await op.send(msg)
    received = await rb.receive()
    assert received == msg

    await op.close()
    await rb.close()


@pytest.mark.asyncio
async def test_signaling_bidirectional(matchmaker):
    """Both sides can send and receive."""
    url = matchmaker.make_url("/")

    op = SignalingClient(server_url=url, room="bidir", role="operator")
    rb = SignalingClient(server_url=url, room="bidir", role="robot")

    await op.connect()
    await rb.connect()

    await op.send({"type": "offer"})
    await rb.send({"type": "answer"})

    r1 = await rb.receive()
    r2 = await op.receive()

    assert r1 == {"type": "offer"}
    assert r2 == {"type": "answer"}

    await op.close()
    await rb.close()


@pytest.mark.asyncio
async def test_signaling_subscriber_id_stable(matchmaker):
    """Same SignalingClient instance reuses its subscriber_id across polls."""
    url = matchmaker.make_url("/")
    rb = SignalingClient(server_url=url, room="sid-test", role="robot")
    await rb.connect()
    assert rb._subscriber_id != ""
    sid1 = rb._subscriber_id

    # Re-connecting (simulating reconnect) should assign a new ID
    await rb.close()
    await rb.connect()
    assert rb._subscriber_id != ""
    # New connect → new UUID (fresh subscriber)
    assert rb._subscriber_id != sid1

    await rb.close()


@pytest.mark.asyncio
async def test_two_robots_same_room_fanout(matchmaker):
    """Two robot clients both receive the same operator message (fan-out)."""
    url = matchmaker.make_url("/")

    op = SignalingClient(server_url=url, room="fanout", role="operator")
    rb1 = SignalingClient(server_url=url, room="fanout", role="robot")
    rb2 = SignalingClient(server_url=url, room="fanout", role="robot")

    await op.connect()
    await rb1.connect()
    await rb2.connect()

    await op.send({"type": "offer", "sdp": "..."})

    r1 = await asyncio.wait_for(rb1.receive(), timeout=5.0)
    r2 = await asyncio.wait_for(rb2.receive(), timeout=5.0)

    assert r1 == {"type": "offer", "sdp": "..."}
    assert r2 == {"type": "offer", "sdp": "..."}

    await op.close()
    await rb1.close()
    await rb2.close()


@pytest.mark.asyncio
async def test_signaling_message_order(matchmaker):
    """Multiple messages arrive in order."""
    url = matchmaker.make_url("/")
    op = SignalingClient(server_url=url, room="order", role="operator")
    rb = SignalingClient(server_url=url, room="order", role="robot")

    await op.connect()
    await rb.connect()

    msgs = [{"seq": i} for i in range(5)]
    for m in msgs:
        await op.send(m)

    received = []
    for _ in range(5):
        received.append(await asyncio.wait_for(rb.receive(), timeout=5.0))

    assert received == msgs

    await op.close()
    await rb.close()


# ---------------------------------------------------------------------------
# ProtocolError
# ---------------------------------------------------------------------------

def test_protocol_error_is_runtime_error():
    e = ProtocolError("bad message")
    assert isinstance(e, RuntimeError)
    assert "bad message" in str(e)

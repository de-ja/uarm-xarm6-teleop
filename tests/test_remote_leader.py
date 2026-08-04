import asyncio
import json
import os
import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from uarm_xarm6_teleop.config import load_config
from uarm_xarm6_teleop.remote_leader import (
    REMOTE_LEADER_PATH,
    BrowserPairedRemoteLeaderFactory,
    RemoteLeader,
    RemoteLeaderError,
    RemoteLeaderService,
    leader_url_for_host,
    load_token_file,
    normalize_leader_url,
)

TOKEN = "test-token-with-at-least-thirty-two-characters"


class FakeSyncConnection:
    def __init__(self, responses):
        self.responses = deque(json.dumps(response) for response in responses)
        self.sent = []
        self.closed = False

    def send(self, message):
        self.sent.append(json.loads(message))

    def recv(self, timeout=None):
        del timeout
        return self.responses.popleft()

    def close(self):
        self.closed = True


class FakeAsyncConnection:
    def __init__(self, incoming, path=REMOTE_LEADER_PATH):
        self.incoming = deque(json.dumps(message) for message in incoming)
        self.request = SimpleNamespace(path=path)
        self.sent = []
        self.closed = None

    async def recv(self):
        if not self.incoming:
            raise ConnectionError("test client disconnected")
        return self.incoming.popleft()

    async def send(self, message):
        self.sent.append(json.loads(message))

    async def close(self, code=1000, reason=""):
        self.closed = (code, reason)


class FakeLocalLeader:
    torque_enabled_ids = (3,)

    def __init__(self, positions):
        self.positions = positions
        self.read_count = 0

    def read_positions(self):
        self.read_count += 1
        return self.positions


class RemoteLeaderTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.positions = (
            self.config.leader.midpoint,
            self.config.leader.midpoint,
            self.config.leader.midpoint,
            self.config.leader.midpoint,
            self.config.leader.midpoint,
            self.config.leader.midpoint,
            self.config.leader.gripper_zero_position,
        )

    def test_remote_client_handshake_and_fresh_sample(self):
        connection = FakeSyncConnection(
            [
                {
                    "type": "hello",
                    "protocol": 1,
                    "ids": list(self.config.serial.ids),
                    "torque_enabled_ids": [3],
                },
                {"type": "sample", "sequence": 1, "positions": list(self.positions)},
            ]
        )
        connect_calls = []

        def connect_factory(url, **kwargs):
            connect_calls.append((url, kwargs))
            return connection

        leader = RemoteLeader(
            self.config.serial,
            self.config.leader,
            url="ws://192.168.50.1:8765",
            token=TOKEN,
            timeout=0.2,
            monotonic=lambda: 123.25,
            connect_factory=connect_factory,
        )

        leader.open()
        sample = leader.read()
        leader.close()

        self.assertEqual(connect_calls[0][0], "ws://192.168.50.1:8765/ws/leader")
        self.assertEqual(connection.sent[0]["type"], "hello")
        self.assertEqual(connection.sent[0]["ids"], list(self.config.serial.ids))
        self.assertEqual(connection.sent[1], {"type": "sample", "sequence": 1})
        self.assertEqual(sample.positions, self.positions)
        self.assertEqual(sample.timestamp, 123.25)
        np.testing.assert_allclose(sample.radians, np.zeros(7), atol=1e-12)
        self.assertTrue(connection.closed)

    def test_remote_client_rejects_wrong_sequence(self):
        connection = FakeSyncConnection(
            [
                {
                    "type": "hello",
                    "protocol": 1,
                    "ids": list(self.config.serial.ids),
                    "torque_enabled_ids": [],
                },
                {"type": "sample", "sequence": 9, "positions": list(self.positions)},
            ]
        )
        leader = RemoteLeader(
            self.config.serial,
            self.config.leader,
            url="ws://leader:8765",
            token=TOKEN,
            connect_factory=lambda _url, **_kwargs: connection,
        )
        leader.open()

        with self.assertRaisesRegex(RemoteLeaderError, "unexpected sample sequence"):
            leader.read()

    def test_token_file_requires_private_permissions_and_sufficient_length(self):
        with tempfile.TemporaryDirectory() as directory:
            token_path = Path(directory) / "leader.token"
            token_path.write_text(TOKEN, encoding="utf-8")
            os.chmod(token_path, 0o600)
            self.assertEqual(load_token_file(token_path), TOKEN)

            os.chmod(token_path, 0o644)
            with self.assertRaisesRegex(RemoteLeaderError, "group or other"):
                load_token_file(token_path)

    def test_url_validation(self):
        self.assertEqual(
            normalize_leader_url("wss://leader.example:8765/"),
            "wss://leader.example:8765/ws/leader",
        )
        with self.assertRaises(RemoteLeaderError):
            normalize_leader_url("http://leader.example:8765")

    def test_browser_source_addresses_become_leader_urls(self):
        self.assertEqual(
            leader_url_for_host("10.42.0.15"),
            "ws://10.42.0.15:8765/ws/leader",
        )
        self.assertEqual(
            leader_url_for_host("fd00::15", port=9000),
            "ws://[fd00::15]:9000/ws/leader",
        )

    def test_browser_paired_factory_requires_pairing_then_uses_observed_address(self):
        factory = BrowserPairedRemoteLeaderFactory(token=TOKEN)
        with self.assertRaisesRegex(RemoteLeaderError, "Open the console"):
            factory(self.config.serial, self.config.leader)

        factory.pair_browser("10.42.0.15")
        leader = factory(self.config.serial, self.config.leader)

        self.assertEqual(leader.url, "ws://10.42.0.15:8765/ws/leader")


class RemoteLeaderServiceTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.config = load_config()
        self.positions = (2047,) * 7

    async def test_service_authenticates_and_reads_only_on_request(self):
        leader = FakeLocalLeader(self.positions)
        service = RemoteLeaderService(leader, self.config.serial, token=TOKEN)
        connection = FakeAsyncConnection(
            [
                {
                    "type": "hello",
                    "protocol": 1,
                    "token": TOKEN,
                    "ids": list(self.config.serial.ids),
                },
                {"type": "sample", "sequence": 1},
            ]
        )

        await asyncio.wait_for(service.handle(connection), timeout=1.0)

        self.assertEqual(connection.sent[0]["type"], "hello")
        self.assertEqual(connection.sent[0]["torque_enabled_ids"], [3])
        self.assertEqual(connection.sent[1]["type"], "sample")
        self.assertEqual(connection.sent[1]["sequence"], 1)
        self.assertEqual(connection.sent[1]["positions"], list(self.positions))
        self.assertEqual(leader.read_count, 1)
        self.assertIsNone(service._active)

    async def test_service_rejects_an_invalid_token_without_reading(self):
        leader = FakeLocalLeader(self.positions)
        service = RemoteLeaderService(leader, self.config.serial, token=TOKEN)
        connection = FakeAsyncConnection(
            [
                {
                    "type": "hello",
                    "protocol": 1,
                    "token": "wrong-token-that-is-still-long-enough-for-test",
                    "ids": list(self.config.serial.ids),
                }
            ]
        )

        await asyncio.wait_for(service.handle(connection), timeout=1.0)

        self.assertEqual(connection.sent[0]["type"], "error")
        self.assertIn("authentication failed", connection.sent[0]["message"])
        self.assertEqual(connection.closed[0], 1008)
        self.assertEqual(leader.read_count, 0)


if __name__ == "__main__":
    unittest.main()

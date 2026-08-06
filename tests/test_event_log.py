import json
import os
import tempfile
import unittest
from pathlib import Path

from uarm_xarm6_teleop.event_log import AsyncJsonlEventSink
from uarm_xarm6_teleop.protocol import ControllerEvent


class EventLogTests(unittest.TestCase):
    def test_async_jsonl_sink_persists_versioned_session_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sessions" / "events.jsonl"
            sink = AsyncJsonlEventSink(path)
            sink.emit("session-a", ControllerEvent(10.5, "info", "connected"))
            sink.emit("session-a", ControllerEvent(11.0, "warning", "contact"))
            sink.close()

            records = [json.loads(line) for line in path.read_text().splitlines()]
            mode = os.stat(path).st_mode & 0o777

        self.assertEqual([record["message"] for record in records], ["connected", "contact"])
        self.assertEqual({record["session_id"] for record in records}, {"session-a"})
        self.assertEqual({record["schema_version"] for record in records}, {1})
        self.assertIsNone(sink.error)
        self.assertEqual(mode, 0o600)

    def test_close_is_idempotent_and_emit_after_close_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            sink = AsyncJsonlEventSink(path)
            sink.close()
            sink.emit("late", ControllerEvent(12.0, "error", "ignored"))
            sink.close()

            self.assertEqual(path.read_text(), "")


if __name__ == "__main__":
    unittest.main()

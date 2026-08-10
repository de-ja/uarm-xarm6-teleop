import json
import os
import tempfile
import unittest
from pathlib import Path

from uarm_xarm6_teleop.event_log import EVENT_LOG_SCHEMA_VERSION, AsyncJsonlEventSink
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
        self.assertEqual(
            {record["schema_version"] for record in records}, {EVENT_LOG_SCHEMA_VERSION}
        )
        self.assertEqual({record["record"] for record in records}, {"event"})
        self.assertIsNone(sink.error)
        self.assertEqual(mode, 0o600)

    def test_metrics_records_are_persisted_alongside_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            sink = AsyncJsonlEventSink(path)
            sink.emit("session-a", ControllerEvent(10.5, "info", "running"))
            sink.emit_metrics(
                "session-a",
                {"timestamp": 11.0, "loop_rate_hz": 19.8, "leader_network_ms": 7.5},
            )
            sink.close()
            records = [json.loads(line) for line in path.read_text().splitlines()]

        self.assertEqual([record["record"] for record in records], ["event", "metrics"])
        metrics = records[1]
        self.assertEqual(metrics["session_id"], "session-a")
        self.assertEqual(metrics["loop_rate_hz"], 19.8)
        self.assertEqual(metrics["leader_network_ms"], 7.5)

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

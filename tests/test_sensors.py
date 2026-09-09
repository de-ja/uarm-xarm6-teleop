import unittest

from uarm_xarm6_teleop.config import SensorConfig
from uarm_xarm6_teleop.sensors import (
    EFleshSensor,
    SensorError,
    SensorHub,
    SensorReading,
    eflesh_labels,
)


class FakeAnySkinProcess:
    """Stand in for anyskin's reader process without touching hardware."""

    def __init__(self, num_mags, port, temp_filtered=True):
        self.num_mags = num_mags
        self.port = port
        self.temp_filtered = temp_filtered
        self.calls = []
        # anyskin reports [timestamp, *values] with three axes per magnetometer.
        self.last_reading = [1.5] + [float(i) for i in range(3 * num_mags)]

    def start(self):
        self.calls.append("start")

    def start_streaming(self):
        self.calls.append("start_streaming")

    def pause_streaming(self):
        self.calls.append("pause_streaming")

    def join(self):
        self.calls.append("join")


class FakeSensor:
    """Minimal SensorSource used to exercise hub behaviour."""

    def __init__(self, name, values=(1.0,), fail_on=None):
        self._name = name
        self._values = tuple(values)
        self._fail_on = fail_on
        self.started = False
        self.closed = False
        self.reads = 0

    @property
    def name(self):
        return self._name

    @property
    def labels(self):
        return tuple(f"v{i}" for i in range(len(self._values)))

    def start(self):
        if self._fail_on == "start":
            raise RuntimeError("cannot open")
        self.started = True

    def read_latest(self):
        self.reads += 1
        if self._fail_on == "read":
            raise RuntimeError("transport wedged")
        return SensorReading(
            name=self._name,
            timestamp=0.0,
            source_timestamp=None,
            labels=self.labels,
            values=self._values,
        )

    def close(self):
        if self._fail_on == "close":
            raise RuntimeError("cannot close")
        self.closed = True


def eflesh_config(**overrides):
    values = {
        "kind": "eflesh",
        "name": "gripper_tactile",
        "port": "/dev/serial/by-id/usb-Adafruit_QT_Py_M0_TEST-if00",
        "num_mags": 5,
        "fingers": ("left",),
    }
    values.update(overrides)
    return SensorConfig(**values)


class EFleshLabelTests(unittest.TestCase):
    def test_one_board_labels_follow_physical_position_order(self):
        labels = eflesh_labels(5, ("left",))
        self.assertEqual(len(labels), 15)
        self.assertEqual(labels[0], "left_middle_bx")
        # Position order is middle, left, right, top, bottom, not address order.
        self.assertEqual(labels[3], "left_left_bx")
        self.assertEqual(labels[-1], "left_bottom_bz")

    def test_two_boards_are_ordered_by_mux_channel(self):
        labels = eflesh_labels(10, ("left", "right"))
        self.assertEqual(len(labels), 30)
        self.assertEqual(labels[0], "left_middle_bx")
        self.assertEqual(labels[15], "right_middle_bx")

    def test_magnetometer_count_must_match_the_finger_names(self):
        with self.assertRaisesRegex(SensorError, "finger name"):
            eflesh_labels(10, ("left",))

    def test_partial_board_is_rejected(self):
        with self.assertRaisesRegex(SensorError, "multiple of 5"):
            eflesh_labels(7, ("left",))


class EFleshSensorTests(unittest.TestCase):
    def make(self, **overrides):
        created = []

        def factory(**kwargs):
            process = FakeAnySkinProcess(**kwargs)
            created.append(process)
            return process

        sensor = EFleshSensor(eflesh_config(**overrides), process_factory=factory)
        return sensor, created

    def test_start_requests_temperature_filtered_streaming(self):
        sensor, created = self.make()
        sensor.start()

        process = created[0]
        # Labels describe three field axes per magnetometer, which only holds
        # when the temperature channel is filtered out.
        self.assertTrue(process.temp_filtered)
        self.assertEqual(process.num_mags, 5)
        self.assertEqual(process.calls, ["start", "start_streaming"])

    def test_reading_splits_the_source_timestamp_from_the_values(self):
        sensor, created = self.make()
        sensor.start()
        reading = sensor.read_latest()

        self.assertEqual(reading.source_timestamp, 1.5)
        self.assertEqual(len(reading.values), 15)
        self.assertEqual(reading.labels, sensor.labels)
        # A local monotonic stamp is what aligns this against camera and leader
        # samples, so it must not be the sensor's own clock.
        self.assertNotEqual(reading.timestamp, reading.source_timestamp)

    def test_no_reading_before_the_stream_starts(self):
        sensor, _created = self.make()
        self.assertIsNone(sensor.read_latest())

    def test_a_failed_magnetometer_read_is_discarded(self):
        sensor, created = self.make()
        sensor.start()
        # Raw 0xFFFF converts to 78640.8 uT, far beyond the 126.9 mT full scale.
        created[0].last_reading = [1.5] + [78640.8] * 15
        self.assertIsNone(sensor.read_latest())

    def test_a_value_count_mismatch_is_reported(self):
        sensor, created = self.make()
        sensor.start()
        created[0].last_reading = [1.5, 0.0, 0.0]
        with self.assertRaisesRegex(SensorError, "values but"):
            sensor.read_latest()

    def test_close_stops_streaming_before_joining(self):
        sensor, created = self.make()
        sensor.start()
        sensor.close()
        self.assertEqual(created[0].calls[-2:], ["pause_streaming", "join"])

    def test_close_is_idempotent(self):
        sensor, created = self.make()
        sensor.start()
        sensor.close()
        sensor.close()
        self.assertEqual(created[0].calls.count("join"), 1)


class SensorHubTests(unittest.TestCase):
    def test_readings_are_keyed_by_sensor_name(self):
        hub = SensorHub((FakeSensor("a", (1.0,)), FakeSensor("b", (2.0, 3.0))))
        hub.start()
        readings = hub.read_all()

        self.assertEqual(set(readings), {"a", "b"})
        self.assertEqual(readings["b"].values, (2.0, 3.0))

    def test_a_sensor_that_cannot_start_is_dropped_without_raising(self):
        good = FakeSensor("good")
        hub = SensorHub((FakeSensor("bad", fail_on="start"), good))
        hub.start()

        self.assertEqual(hub.failed, ("bad",))
        self.assertEqual(hub.names, ("good",))
        self.assertTrue(good.started)
        self.assertEqual(set(hub.read_all()), {"good"})

    def test_a_read_failure_never_reaches_the_control_loop(self):
        bad = FakeSensor("bad", fail_on="read")
        hub = SensorHub((bad, FakeSensor("good")))
        hub.start()

        readings = hub.read_all()
        self.assertEqual(set(readings), {"good"})
        self.assertEqual(hub.failed, ("bad",))

    def test_a_failed_sensor_is_not_retried_on_later_cycles(self):
        bad = FakeSensor("bad", fail_on="read")
        hub = SensorHub((bad,))
        hub.start()
        hub.read_all()
        hub.read_all()
        hub.read_all()

        # Retrying a wedged transport every control cycle would cost latency in
        # the loop that matters most.
        self.assertEqual(bad.reads, 1)

    def test_close_continues_past_a_failing_sensor(self):
        good = FakeSensor("good")
        hub = SensorHub((FakeSensor("bad", fail_on="close"), good))
        hub.start()
        hub.close()

        self.assertTrue(good.closed)

    def test_an_empty_hub_is_usable(self):
        hub = SensorHub()
        hub.start()
        self.assertEqual(hub.read_all(), {})
        hub.close()


if __name__ == "__main__":
    unittest.main()

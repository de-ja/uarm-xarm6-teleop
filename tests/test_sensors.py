import time
import unittest

from eflesh import FakeEFleshSource
from uarm_xarm6_teleop.config import SensorConfig
from uarm_xarm6_teleop.sensors import (
    EFleshTactileSensor,
    SensorError,
    SensorHub,
    SensorReading,
    build_sensor,
    build_sensor_hub,
)


def eflesh_config(**overrides):
    values = {
        "kind": "eflesh",
        "name": "gripper_tactile",
        # A literal path passes through resolution untouched, so the fixture
        # needs no attached hardware. Selector resolution is tested separately.
        "port": "/dev/ttyACM-test",
        "settle": 5.0,
    }
    values.update(overrides)
    return SensorConfig(**values)


def fake_factory(num_fingers=2, record=None):
    """Build a source factory that yields hardware-free eFlesh sources."""

    def factory(port, settle, name):
        if record is not None:
            record.append({"port": port, "settle": settle, "name": name})
        return FakeEFleshSource(num_fingers=num_fingers, name=name)

    return factory


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


class EFleshDriverTests(unittest.TestCase):
    def make(self, num_fingers=2, **overrides):
        record = []
        sensor = EFleshTactileSensor(
            eflesh_config(**overrides), source_factory=fake_factory(num_fingers, record)
        )
        return sensor, record

    def test_the_finger_count_is_detected_rather_than_configured(self):
        # The firmware sizes its frame at boot from however many boards came up,
        # so num_fingers must never be passed through from configuration.
        sensor, record = self.make()
        sensor.start()

        self.assertNotIn("num_fingers", record[0])
        self.assertEqual(sensor.num_fingers, 2)
        sensor.close()

    def test_one_finger_is_handled_without_hardcoding_two(self):
        sensor, _record = self.make(num_fingers=1)
        sensor.start()

        self.assertEqual(sensor.num_fingers, 1)
        self.assertEqual(len(sensor.labels), 15)
        sensor.close()

    def test_the_configured_settle_reaches_the_source(self):
        sensor, record = self.make(settle=7.5)
        sensor.start()

        self.assertEqual(record[0]["settle"], 7.5)
        sensor.close()

    def test_the_port_is_resolved_before_the_source_is_built(self):
        # A usb: selector is meaningless to the sensor package, so it must be
        # resolved here. With nothing attached that resolution fails loudly.
        sensor, _record = self.make(port="usb:serial=NOTATTACHED")
        with self.assertRaises(Exception) as raised:
            sensor.start()
        self.assertIn("NOTATTACHED", str(raised.exception))

    def await_sample(self, sensor, timeout=2.0):
        """Wait for the reader thread to publish its first sample."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            reading = sensor.read_latest()
            if reading is not None:
                return reading
            time.sleep(0.005)
        raise AssertionError("the reader published no sample")

    def test_a_reading_is_flattened_with_one_label_per_value(self):
        sensor, _record = self.make()
        sensor.start()
        reading = self.await_sample(sensor)

        self.assertEqual(reading.name, "gripper_tactile")
        self.assertEqual(len(reading.values), 30)
        self.assertEqual(len(reading.labels), len(reading.values))
        self.assertEqual(reading.labels[0], "f0_middle_bx")
        # A local monotonic stamp aligns this against leader and camera samples,
        # so it must not be the sensor's own clock.
        self.assertIsNotNone(reading.source_timestamp)
        self.assertNotEqual(reading.timestamp, reading.source_timestamp)
        sensor.close()

    def test_no_reading_and_no_labels_before_starting(self):
        sensor, _record = self.make()
        self.assertIsNone(sensor.read_latest())
        self.assertEqual(sensor.labels, ())
        self.assertIsNone(sensor.num_fingers)

    def test_geometry_carries_the_hand_verified_constants(self):
        sensor, _record = self.make()
        sensor.start()
        geometry = sensor.geometry_payload()

        # The frontend must read these rather than hold a second copy that drifts.
        self.assertEqual(geometry["num_fingers"], 2)
        self.assertEqual(geometry["mags_per_finger"], 5)
        self.assertEqual(len(geometry["chip_positions_mm"]), 5)
        self.assertIn("contact_threshold_ut", geometry)
        self.assertIn("pad_half_mm", geometry)
        sensor.close()

    def test_geometry_before_starting_is_an_error_not_a_guess(self):
        sensor, _record = self.make()
        with self.assertRaisesRegex(SensorError, "has not started"):
            sensor.geometry_payload()

    def test_display_payloads_are_throttled_and_pad_relative(self):
        sensor, _record = self.make()
        sensor.start()
        frames = []
        for frame in sensor.display_payloads(hz=200.0, stop=lambda: len(frames) >= 3):
            frames.append(frame)

        self.assertEqual(len(frames), 3)
        finger = frames[0]["fingers"][0]
        # shear_xy arrives already rotated into the shared pad frame, so the
        # frontend needs no rotation matrices of its own.
        self.assertEqual(len(finger["shear_xy"]), 5)
        self.assertEqual(len(finger["shear_xy"][0]), 2)
        self.assertIn("balance", frames[0])
        sensor.close()

    def test_close_is_idempotent(self):
        sensor, _record = self.make()
        sensor.start()
        sensor.close()
        sensor.close()

    def test_the_gap_baseline_hook_rejects_use_before_starting(self):
        sensor, _record = self.make()
        with self.assertRaisesRegex(SensorError, "has not started"):
            sensor.set_gap_baseline([[0.0] * 30])


class RegistryTests(unittest.TestCase):
    def test_eflesh_is_registered_under_its_kind(self):
        sensor = build_sensor(eflesh_config())
        self.assertIsInstance(sensor, EFleshTactileSensor)

    def test_an_unknown_kind_is_rejected(self):
        with self.assertRaisesRegex(SensorError, "Unsupported sensor kind"):
            build_sensor(eflesh_config(kind="tacto"))

    def test_a_sensor_that_cannot_be_built_is_skipped_not_fatal(self):
        hub = build_sensor_hub((eflesh_config(kind="tacto"),))
        self.assertEqual(hub.names, ())


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

    def test_an_eflesh_source_flows_through_the_hub(self):
        sensor = EFleshTactileSensor(eflesh_config(), source_factory=fake_factory())
        hub = SensorHub((sensor,))
        hub.start()
        readings = hub.read_all()
        hub.close()

        self.assertEqual(set(readings), {"gripper_tactile"})
        self.assertEqual(len(readings["gripper_tactile"].values), 30)


class FullRateReaderTests(unittest.TestCase):
    """The sensor is read once per sample by one thread, not once per consumer."""

    def make(self):
        sensor = EFleshTactileSensor(eflesh_config(), source_factory=fake_factory())
        sensor.start()
        return sensor

    def test_derived_signals_do_not_depend_on_how_often_a_consumer_reads(self):
        # Previously both the display and the metrics path advanced the source's
        # own vibration history, so that signal changed meaning depending on
        # whether a browser was attached.
        observed = []
        for poll_hz in (60.0, 4.0):
            sensor = self.make()
            time.sleep(0.35)
            for _ in range(3):
                sensor.read_latest()
                time.sleep(1.0 / poll_hz)
            reading, _monotonic = sensor._store.latest()
            observed.append(reading.signals.fingers[0].vibration)
            sensor.close()

        fast, slow = observed
        self.assertLess(abs(fast - slow), max(fast, slow) * 0.5 + 0.5)

    def test_every_sample_is_buffered_for_a_recorder(self):
        sensor = self.make()
        time.sleep(0.3)
        drained = sensor.drain()
        sensor.close()

        # Far more than a per-second snapshot: the reader runs at sensor rate.
        self.assertGreater(len(drained), 20)
        timestamp, reading = drained[0]
        self.assertIsInstance(timestamp, float)
        self.assertEqual(len(reading.signals.fingers), 2)

    def test_draining_twice_does_not_repeat_samples(self):
        sensor = self.make()
        time.sleep(0.2)
        first = sensor.drain()
        second = sensor.drain()
        sensor.close()

        self.assertGreater(len(first), 0)
        self.assertLess(len(second), len(first))

    def test_the_observed_rate_is_reported(self):
        sensor = self.make()
        time.sleep(0.3)
        rate = sensor.sample_rate_hz
        sensor.close()
        self.assertGreater(rate, 20.0)

    def test_sample_age_distinguishes_live_from_frozen(self):
        sensor = self.make()
        time.sleep(0.2)
        self.assertLess(sensor.age_seconds(), 0.5)
        sensor.close()
        # Once the reader stops, the newest sample simply ages.
        frozen = sensor.age_seconds()
        time.sleep(0.15)
        self.assertGreater(sensor.age_seconds(), frozen)

    def test_age_is_unknown_before_any_sample(self):
        sensor = EFleshTactileSensor(eflesh_config(), source_factory=fake_factory())
        self.assertIsNone(sensor.age_seconds())

    def test_reading_the_snapshot_does_not_consume_the_recorder_buffer(self):
        sensor = self.make()
        time.sleep(0.25)
        for _ in range(5):
            sensor.read_latest()
        drained = sensor.drain()
        sensor.close()
        self.assertGreater(len(drained), 20)


if __name__ == "__main__":
    unittest.main()

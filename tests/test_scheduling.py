import unittest

from uarm_xarm6_teleop.scheduling import PeriodicScheduler, RateMeter


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, delay):
        self.sleeps.append(delay)
        self.now += delay


class SchedulingTests(unittest.TestCase):
    def test_periodic_scheduler_keeps_deadlines_and_resets_after_overrun(self):
        clock = FakeClock()
        scheduler = PeriodicScheduler(
            10.0,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )

        scheduler.wait()
        self.assertAlmostEqual(clock.sleeps[-1], 0.1)

        clock.now += 0.2
        scheduler.wait()
        self.assertEqual(len(clock.sleeps), 1)

        scheduler.wait()
        self.assertAlmostEqual(clock.sleeps[-1], 0.1)

    def test_rate_meter_reports_and_resets_each_window(self):
        clock = FakeClock()
        meter = RateMeter(window_seconds=0.5, monotonic=clock.monotonic)

        for _ in range(4):
            clock.now += 0.1
            self.assertIsNone(meter.record())
        clock.now += 0.1
        self.assertAlmostEqual(meter.record(), 10.0)

        clock.now += 0.5
        self.assertAlmostEqual(meter.record(), 2.0)


if __name__ == "__main__":
    unittest.main()

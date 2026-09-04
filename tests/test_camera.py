import tempfile
import threading
import time
import unittest
from pathlib import Path

from uarm_xarm6_teleop.camera import (
    AdaptiveJpegQuality,
    CameraCapability,
    CameraCatalog,
    CameraError,
    CameraInfo,
    CameraManager,
    _decode_fourcc,
    _encode_fourcc,
)


class CameraCatalogTests(unittest.TestCase):
    def test_stable_links_and_physical_bus_grouping(self):
        with tempfile.TemporaryDirectory() as directory:
            dev = Path(directory)
            for index in range(4):
                (dev / f"video{index}").touch()

            by_id = dev / "v4l" / "by-id"
            by_id.mkdir(parents=True)
            stable_camera = by_id / "usb-depth-camera-serial-video-index0"
            stable_camera.symlink_to("../../video2")

            capabilities = {
                "video0": CameraCapability("Depth camera", "usb-port-1", True),
                "video1": CameraCapability("Depth metadata", "usb-port-1", False),
                "video2": CameraCapability("Depth camera RGB", "usb-port-1", True),
                "video3": CameraCapability("Workspace camera", "usb-port-2", True),
            }
            catalog = CameraCatalog(
                dev,
                capability_reader=lambda path: capabilities[path.name],
            )

            cameras = catalog.list_cameras()

            self.assertEqual(len(cameras), 2)
            self.assertEqual(cameras[0].name, "Depth camera RGB")
            self.assertEqual(cameras[0].device, str(stable_camera))
            self.assertEqual(cameras[1].name, "Workspace camera")
            self.assertEqual(cameras[1].device, str(dev / "video3"))
            self.assertNotEqual(cameras[0].id, cameras[1].id)

    def test_missing_camera_id_requires_catalog_refresh(self):
        with tempfile.TemporaryDirectory() as directory:
            catalog = CameraCatalog(Path(directory), capability_reader=lambda _path: None)

            with self.assertRaisesRegex(CameraError, "refresh the camera list"):
                catalog.get("missing")


class CaptureProfileTests(unittest.TestCase):
    def test_fourcc_round_trips_through_the_v4l2_integer_code(self):
        # 0x47504a4d is the V4L2 code a driver reports back for MJPG.
        self.assertEqual(_encode_fourcc("MJPG"), 0x47504A4D)
        self.assertEqual(_decode_fourcc(0x47504A4D), "MJPG")
        self.assertEqual(_decode_fourcc(_encode_fourcc("YUYV")), "YUYV")

    def test_manager_requests_a_compressed_format_by_default(self):
        self.assertEqual(CameraManager().fourcc, "MJPG")

    def test_manager_rejects_a_malformed_pixel_format(self):
        with self.assertRaises(ValueError):
            CameraManager(fourcc="MJPEG")

    def test_granted_profile_is_logged_without_a_warning(self):
        manager = CameraManager(width=1280, height=720, fps=15)
        with self.assertLogs("uarm_xarm6_teleop.camera", level="INFO") as logs:
            manager.report_negotiated_profile(
                "/dev/video0", fourcc="MJPG", width=1280, height=720, fps=15.0
            )
        self.assertEqual(len(logs.records), 1)
        self.assertEqual(logs.records[0].levelname, "INFO")

    def test_silent_driver_downgrade_is_reported(self):
        manager = CameraManager(width=1280, height=720, fps=15)
        with self.assertLogs("uarm_xarm6_teleop.camera", level="WARNING") as logs:
            manager.report_negotiated_profile(
                "/dev/video0", fourcc="YUYV", width=640, height=480, fps=10.0
            )
        message = logs.records[0].getMessage()
        self.assertIn("format MJPG -> YUYV", message)
        self.assertIn("size 1280x720 -> 640x480", message)
        self.assertIn("rate 15 -> 10 fps", message)

    def test_unreported_driver_frame_rate_is_not_a_downgrade(self):
        manager = CameraManager(width=1280, height=720, fps=15)
        with self.assertLogs("uarm_xarm6_teleop.camera", level="INFO") as logs:
            manager.report_negotiated_profile(
                "/dev/video0", fourcc="MJPG", width=1280, height=720, fps=0.0
            )
        self.assertEqual(logs.records[0].levelname, "INFO")


class CameraManagerTests(unittest.TestCase):
    def test_adaptive_quality_reduces_fast_and_recovers_cautiously(self):
        quality = AdaptiveJpegQuality(
            initial=80,
            minimum=35,
            maximum=85,
            target_latency_ms=75,
        )

        self.assertEqual(quality.observe(160), 65)
        self.assertEqual(quality.observe(100), 57)
        for _ in range(3):
            self.assertEqual(quality.observe(20), 57)
        self.assertEqual(quality.observe(20), 60)

    def test_adaptive_quality_validates_measurements_and_bounds(self):
        quality = AdaptiveJpegQuality(initial=40, minimum=35, maximum=45)

        self.assertEqual(quality.observe(1_000), 35)
        with self.assertRaisesRegex(ValueError, "finite, non-negative"):
            quality.observe(float("nan"))

    def test_subscribers_share_capture_and_last_close_releases_device(self):
        camera = CameraInfo("camera-a", "Workspace camera", "/dev/camera-a")

        class StubCatalog:
            def list_cameras(self):
                return (camera,)

            def get(self, camera_id):
                if camera_id != camera.id:
                    raise CameraError("missing")
                return camera

        class FakeCapture:
            def __init__(self):
                self.released = threading.Event()

            def isOpened(self):
                return True

            def read(self):
                time.sleep(0.005)
                return True, b"raw-frame"

            def release(self):
                self.released.set()

        captures = []

        def open_capture(_device):
            capture = FakeCapture()
            captures.append(capture)
            return capture

        manager = CameraManager(
            catalog=StubCatalog(),
            capture_factory=open_capture,
            frame_encoder=lambda _frame, _quality: b"jpeg-frame",
            wall_time=lambda: 123.456,
        )
        first = manager.subscribe(camera.id)
        second = manager.subscribe(camera.id)
        first_stream = first.iter_mjpeg()
        second_stream = second.iter_mjpeg()

        first_frame = next(first_stream)
        self.assertIn(b"jpeg-frame", first_frame)
        self.assertIn(b"X-Frame-Sequence:", first_frame)
        self.assertIn(b"X-Capture-Timestamp: 123.456000", first_frame)
        self.assertIn(b"X-JPEG-Quality: 80", first_frame)
        self.assertEqual(manager.report_latency(camera.id, 100), 72)
        self.assertIn(b"jpeg-frame", next(second_stream))
        self.assertEqual(len(captures), 1)

        first_stream.close()
        self.assertFalse(captures[0].released.is_set())
        second_stream.close()
        self.assertTrue(captures[0].released.wait(timeout=1.0))
        manager.close()


if __name__ == "__main__":
    unittest.main()

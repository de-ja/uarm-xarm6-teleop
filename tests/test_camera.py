import tempfile
import threading
import time
import unittest
from pathlib import Path

from uarm_xarm6_teleop.camera import (
    CameraCapability,
    CameraCatalog,
    CameraError,
    CameraInfo,
    CameraManager,
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


class CameraManagerTests(unittest.TestCase):
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
        )
        first = manager.subscribe(camera.id)
        second = manager.subscribe(camera.id)
        first_stream = first.iter_mjpeg()
        second_stream = second.iter_mjpeg()

        self.assertIn(b"jpeg-frame", next(first_stream))
        self.assertIn(b"jpeg-frame", next(second_stream))
        self.assertEqual(len(captures), 1)

        first_stream.close()
        self.assertFalse(captures[0].released.is_set())
        second_stream.close()
        self.assertTrue(captures[0].released.wait(timeout=1.0))
        manager.close()


if __name__ == "__main__":
    unittest.main()

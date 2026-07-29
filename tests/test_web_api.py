import unittest
from pathlib import Path

from fastapi import HTTPException
from pydantic import ValidationError

from uarm_xarm6_teleop.camera import CameraInfo, CameraManager
from uarm_xarm6_teleop.config import load_config
from uarm_xarm6_teleop.controller import (
    ControllerEvent,
    TeleopControllerError,
    TeleopSnapshot,
)
from uarm_xarm6_teleop.web.app import StartRequest, TelemetryClients, _invoke, create_app


class StubController:
    def __init__(self):
        self.config = load_config()
        self.current_state = "idle"
        self.stop_calls = 0
        self.closed = False

    def snapshot(self):
        return TeleopSnapshot(
            protocol_version=1,
            timestamp=100.0,
            state=self.current_state,
            mode=None,
            leader_connected=self.current_state != "idle",
            robot_connected=False,
            robot_ip="",
            torque_enabled_ids=(),
            leader_degrees=None,
            target_degrees=None,
            gripper_command=None,
            robot_status=None,
            loop_rate_hz=0.0,
            last_sample_age_ms=None,
            fault=None,
            events=(ControllerEvent(100.0, "info", "test controller"),),
        )

    def connect_leader(self):
        if self.current_state != "idle":
            raise TeleopControllerError("leader already connected")
        self.current_state = "leader_ready"
        return self.snapshot()

    def inspect_robot(self, _robot_ip):
        raise TeleopControllerError("not available in API stub")

    def start(self, _mode, *, confirmation=None):
        self.current_state = "running"
        return self.snapshot()

    def stop(self):
        self.stop_calls += 1
        if self.current_state == "running":
            self.current_state = "stopped"
        return self.snapshot()

    def disconnect(self):
        self.current_state = "idle"
        return self.snapshot()

    def reset_fault(self):
        return self.disconnect()

    def close(self):
        self.closed = True


class WebApiTests(unittest.TestCase):
    def setUp(self):
        self.controller = StubController()
        self.app = create_app(self.controller)

    def test_api_routes_and_built_frontend_are_present(self):
        paths = {route.path for route in self.app.routes}
        self.assertIn("/api/status", paths)
        self.assertIn("/api/cameras", paths)
        self.assertIn("/api/cameras/{camera_id}/stream", paths)
        self.assertIn("/api/leader/connect", paths)
        self.assertIn("/api/teleop/start", paths)
        self.assertIn("/ws/telemetry", paths)
        self.assertIn("", paths)  # StaticFiles mount at the application root.
        frontend = Path(__file__).parents[1] / "src/uarm_xarm6_teleop/web/dist/index.html"
        self.assertIn("U-ARM Operator", frontend.read_text())

    def test_camera_catalog_is_exposed_without_fixed_device_names(self):
        camera = CameraInfo("camera-a", "Workspace camera", "/dev/v4l/by-id/camera-a")

        class StubCatalog:
            def list_cameras(self):
                return (camera,)

            def get(self, camera_id):
                if camera_id != camera.id:
                    raise AssertionError("unexpected camera ID")
                return camera

        manager = CameraManager(catalog=StubCatalog())
        app = create_app(self.controller, camera_manager=manager)
        route = next(route for route in app.routes if route.path == "/api/cameras")

        self.assertEqual(
            route.endpoint(),
            [{"id": "camera-a", "name": "Workspace camera", "device": "/dev/v4l/by-id/camera-a"}],
        )

    def test_controller_conflicts_are_returned_as_409(self):
        self.assertEqual(_invoke(self.controller.connect_leader)["state"], "leader_ready")
        with self.assertRaises(HTTPException) as raised:
            _invoke(self.controller.connect_leader)
        self.assertEqual(raised.exception.status_code, 409)
        self.assertIn("already connected", raised.exception.detail)

    def test_invalid_mode_is_rejected_by_schema(self):
        with self.assertRaises(ValidationError):
            StartRequest.model_validate({"mode": "unsafe"})

    def test_last_websocket_disconnect_requests_stop(self):
        self.controller.current_state = "running"
        clients = TelemetryClients(self.controller)
        clients.connected()
        if clients.disconnected():
            self.controller.stop()
        self.assertEqual(self.controller.stop_calls, 1)


if __name__ == "__main__":
    unittest.main()

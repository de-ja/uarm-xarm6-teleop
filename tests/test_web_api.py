import asyncio
import unittest
from pathlib import Path

from fastapi import HTTPException, Request
from pydantic import ValidationError

from uarm_xarm6_teleop.camera import CameraInfo, CameraManager
from uarm_xarm6_teleop.config import load_config
from uarm_xarm6_teleop.controller import (
    ControllerEvent,
    TeleopControllerError,
    TeleopSnapshot,
)
from uarm_xarm6_teleop.protocol import RuntimeCapabilities
from uarm_xarm6_teleop.web.app import (
    CameraLatencyRequest,
    StartRequest,
    TelemetryClients,
    _invoke,
    create_app,
)


class StubController:
    def __init__(self):
        self.config = load_config()
        self.capabilities = RuntimeCapabilities(
            leader_transport="remote_browser_pairing",
            simulation_available=True,
            physical_available=True,
            camera_streaming=True,
            structured_logging=False,
            video_transport="mjpeg",
            max_robots=1,
        )
        self.current_state = "idle"
        self.stop_calls = 0
        self.closed = False

    def snapshot(self):
        return TeleopSnapshot(
            protocol_version=3,
            session_id="test-session",
            capabilities=self.capabilities,
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
            command_latency_ms=None,
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
        self.assertIn("/api/capabilities", paths)
        self.assertIn("/api/time", paths)
        self.assertIn("/api/cameras", paths)
        self.assertIn("/api/cameras/{camera_id}/stream", paths)
        self.assertIn("/api/cameras/{camera_id}/latency", paths)
        self.assertIn("/api/leader/connect", paths)
        self.assertIn("/api/teleop/start", paths)
        self.assertIn("/ws/telemetry", paths)
        self.assertIn("", paths)  # StaticFiles mount at the application root.
        frontend = Path(__file__).parents[1] / "src/uarm_xarm6_teleop/web/dist/index.html"
        self.assertIn("U-ARM Operator", frontend.read_text())

    def test_capabilities_endpoint_exposes_backend_features(self):
        route = next(route for route in self.app.routes if route.path == "/api/capabilities")

        result = route.endpoint()

        self.assertTrue(result.physical_available)
        self.assertEqual(result.leader_transport, "remote_browser_pairing")
        self.assertEqual(result.max_robots, 1)

    def test_openapi_schema_owns_the_frontend_protocol_contract(self):
        schemas = self.app.openapi()["components"]["schemas"]

        snapshot = schemas["TeleopSnapshot"]
        self.assertIn("session_id", snapshot["required"])
        self.assertIn("capabilities", snapshot["required"])
        self.assertEqual(schemas["RuntimeCapabilities"]["properties"]["max_robots"]["const"], 1)
        self.assertEqual(
            schemas["RuntimeCapabilities"]["properties"]["video_transport"]["const"],
            "mjpeg",
        )

    def test_time_endpoint_returns_a_wall_clock_timestamp(self):
        route = next(route for route in self.app.routes if route.path == "/api/time")

        self.assertGreater(route.endpoint()["timestamp"], 0.0)

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

    def test_camera_latency_feedback_updates_auto_quality(self):
        class StubCameraManager:
            def list_cameras(self):
                return ()

            def report_latency(self, camera_id, latency_ms):
                self.feedback = (camera_id, latency_ms)
                return 64

            def close(self):
                pass

        manager = StubCameraManager()
        app = create_app(self.controller, camera_manager=manager)
        route = next(
            route for route in app.routes if route.path == "/api/cameras/{camera_id}/latency"
        )

        result = route.endpoint("camera-a", CameraLatencyRequest(latency_ms=118.5))

        self.assertEqual(result, {"mode": "auto", "jpeg_quality": 64})
        self.assertEqual(manager.feedback, ("camera-a", 118.5))

    def test_controller_conflicts_are_returned_as_409(self):
        self.assertEqual(_invoke(self.controller.connect_leader)["state"], "leader_ready")
        with self.assertRaises(HTTPException) as raised:
            _invoke(self.controller.connect_leader)
        self.assertEqual(raised.exception.status_code, 409)
        self.assertIn("already connected", raised.exception.detail)

    def test_connect_leader_pairs_from_the_browser_source_address(self):
        class StubPairingFactory:
            host = None

            def pair_browser(self, host):
                self.host = host

        pairing = StubPairingFactory()
        app = create_app(self.controller, browser_leader_factory=pairing)
        route = next(route for route in app.routes if route.path == "/api/leader/connect")
        request = Request(
            {
                "type": "http",
                "method": "POST",
                "path": "/api/leader/connect",
                "headers": [],
                "query_string": b"",
                "client": ("10.42.0.15", 51234),
                "server": ("10.42.0.20", 8000),
                "scheme": "http",
            }
        )

        result = route.endpoint(request)

        self.assertEqual(pairing.host, "10.42.0.15")
        self.assertEqual(result["state"], "leader_ready")

    def test_invalid_mode_is_rejected_by_schema(self):
        self.assertEqual(StartRequest.model_validate({"mode": "simulation"}).mode, "simulation")
        with self.assertRaises(ValidationError):
            StartRequest.model_validate({"mode": "unsafe"})

    def test_last_websocket_disconnect_requests_stop(self):
        self.controller.current_state = "running"
        clients = TelemetryClients(self.controller)
        clients.connected()
        if clients.disconnected():
            self.controller.stop()
        self.assertEqual(self.controller.stop_calls, 1)

    def test_supervision_tracks_reconnecting_clients(self):
        clients = TelemetryClients(self.controller)
        clients.connected()
        self.assertTrue(clients.supervised)
        self.assertTrue(clients.disconnected())
        self.assertFalse(clients.supervised)
        clients.connected()
        self.assertTrue(clients.supervised)

    def test_browser_reconnect_within_grace_period_keeps_motion(self):
        self.controller.current_state = "running"

        async def scenario():
            clients = TelemetryClients(self.controller, grace_seconds=0.01)
            clients.connected()
            self.assertTrue(clients.disconnected())
            clients.schedule_stop()
            await asyncio.sleep(0)
            # The browser returns inside the grace period.
            clients.connected()
            # Wait well past the grace period: motion must survive it.
            await asyncio.sleep(0.2)

        asyncio.run(scenario())
        self.assertEqual(self.controller.stop_calls, 0)

    def test_browser_drop_beyond_grace_period_stops_motion(self):
        self.controller.current_state = "running"

        async def scenario():
            clients = TelemetryClients(self.controller, grace_seconds=0.01)
            clients.connected()
            self.assertTrue(clients.disconnected())
            clients.schedule_stop()
            await asyncio.sleep(0.2)

        asyncio.run(scenario())
        self.assertEqual(self.controller.stop_calls, 1)

    def test_failed_supervision_stop_is_recorded_not_swallowed(self):
        def failing_stop():
            self.controller.stop_calls += 1
            raise RuntimeError("stop failed")

        self.controller.stop = failing_stop

        async def scenario():
            clients = TelemetryClients(self.controller, grace_seconds=0.01)
            clients.connected()
            self.assertTrue(clients.disconnected())
            clients.schedule_stop()
            await asyncio.sleep(0.2)
            return clients

        with self.assertLogs("uarm_xarm6_teleop.web.app", level="ERROR"):
            clients = asyncio.run(scenario())

        self.assertEqual(self.controller.stop_calls, 1)
        self.assertEqual(clients.stop_failure, "stop failed")

    def test_zero_grace_stops_motion_immediately(self):
        self.controller.current_state = "running"

        async def scenario():
            clients = TelemetryClients(self.controller, grace_seconds=0.0)
            clients.connected()
            self.assertTrue(clients.disconnected())
            clients.schedule_stop()
            await asyncio.sleep(0.05)

        asyncio.run(scenario())
        self.assertEqual(self.controller.stop_calls, 1)


if __name__ == "__main__":
    unittest.main()

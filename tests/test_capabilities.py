import unittest
from unittest.mock import patch

from uarm_xarm6_teleop.capabilities import detect_runtime_capabilities


class RuntimeCapabilitiesTests(unittest.TestCase):
    def test_detection_reports_selected_transport_and_installed_extras(self):
        installed = {"mani_skill", "cv2"}

        with patch(
            "uarm_xarm6_teleop.capabilities.find_spec",
            side_effect=lambda name: object() if name in installed else None,
        ):
            capabilities = detect_runtime_capabilities(
                leader_transport="remote_browser_pairing",
                structured_logging=True,
            )

        self.assertEqual(capabilities.leader_transport, "remote_browser_pairing")
        self.assertTrue(capabilities.simulation_available)
        self.assertFalse(capabilities.physical_available)
        self.assertTrue(capabilities.camera_streaming)
        self.assertTrue(capabilities.structured_logging)
        self.assertEqual(capabilities.video_transport, "mjpeg")
        self.assertEqual(capabilities.max_robots, 1)


if __name__ == "__main__":
    unittest.main()

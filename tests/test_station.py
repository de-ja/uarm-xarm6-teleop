import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from uarm_xarm6_teleop.cli.station import (
    NetworkAddress,
    _parse_network_addresses,
    console_urls,
    interactive_station,
)


class StationTests(unittest.TestCase):
    def test_network_address_parser_ignores_loopback_and_link_local(self):
        raw = json.dumps(
            [
                {
                    "ifname": "lo",
                    "addr_info": [{"family": "inet", "local": "127.0.0.1"}],
                },
                {
                    "ifname": "wlp4s0",
                    "addr_info": [{"family": "inet", "local": "10.42.0.20"}],
                },
                {
                    "ifname": "enp3s0",
                    "addr_info": [{"family": "inet", "local": "169.254.10.3"}],
                },
            ]
        )

        self.assertEqual(
            _parse_network_addresses(raw),
            (NetworkAddress("wlp4s0", "10.42.0.20"),),
        )

    def test_console_urls_include_every_active_desktop_address(self):
        addresses = (
            NetworkAddress("enp3s0", "192.168.1.100"),
            NetworkAddress("wlp4s0", "10.42.0.20"),
        )

        self.assertEqual(
            console_urls(8000, addresses),
            ("http://192.168.1.100:8000", "http://10.42.0.20:8000"),
        )

    def test_enter_starts_browser_paired_station_without_asking_for_laptop_ip(self):
        with tempfile.TemporaryDirectory() as directory:
            token_path = Path(directory) / "leader.token"
            token_path.write_text(
                "test-token-with-more-than-thirty-two-characters", encoding="utf-8"
            )
            os.chmod(token_path, 0o600)
            prompts = []

            def answer(prompt):
                prompts.append(prompt)
                return ""

            addresses = (NetworkAddress("wlp4s0", "10.42.0.20"),)
            with (
                patch(
                    "uarm_xarm6_teleop.cli.station.discover_network_addresses",
                    return_value=addresses,
                ),
                patch("uarm_xarm6_teleop.cli.station.run_web") as run_web,
            ):
                interactive_station(
                    token_path=token_path,
                    input_fn=answer,
                    output=lambda _message: None,
                )

            self.assertEqual(len(prompts), 1)
            self.assertNotIn("IP", prompts[0])
            run_web.assert_called_once_with(
                config_path=None,
                host="0.0.0.0",
                port=8000,
                leader_token_file=token_path,
                leader_timeout=0.2,
                browser_pair_leader=True,
                leader_port=8765,
                open_browser=False,
            )


if __name__ == "__main__":
    unittest.main()

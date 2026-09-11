import unittest

from uarm_xarm6_teleop.serial_ports import (
    PortCandidate,
    SerialPortError,
    is_selector,
    parse_selector,
    resolve_serial_port,
)


def candidate(
    device, vid=0x239A, pid=0x800F, serial="AAA", manufacturer="Adafruit", product="QT Py M0"
):
    return PortCandidate(
        device=device,
        vid=vid,
        pid=pid,
        serial_number=serial,
        manufacturer=manufacturer,
        product=product,
    )


LEADER = candidate(
    "/dev/ttyACM0",
    vid=0x1A86,
    pid=0x7523,
    serial="LEADER123",
    manufacturer="QinHeng",
    product="U-ARM",
)
EFLESH = candidate("/dev/ttyACM1", serial="5B5620DE5032434A582E3120FF031835")
SECOND_EFLESH = candidate("/dev/ttyACM2", serial="OTHER456")


class SelectorParsingTests(unittest.TestCase):
    def test_a_path_is_not_a_selector(self):
        self.assertFalse(is_selector("/dev/ttyACM0"))
        self.assertFalse(is_selector("/dev/serial/by-id/usb-Thing-if00"))
        self.assertTrue(is_selector("usb:serial=ABC"))

    def test_criteria_are_parsed_and_lower_cased(self):
        self.assertEqual(
            parse_selector("usb:VID=239a,Serial=ABC"), {"vid": "239a", "serial": "ABC"}
        )

    def test_an_empty_selector_is_rejected(self):
        with self.assertRaisesRegex(SerialPortError, "no criteria"):
            parse_selector("usb:")

    def test_a_clause_without_a_value_is_rejected(self):
        with self.assertRaisesRegex(SerialPortError, "key=value"):
            parse_selector("usb:serial")

    def test_an_unknown_field_is_rejected(self):
        with self.assertRaisesRegex(SerialPortError, "not supported"):
            parse_selector("usb:colour=blue")


class ResolutionTests(unittest.TestCase):
    def setUp(self):
        self.attached = (LEADER, EFLESH, SECOND_EFLESH)

    def test_a_path_passes_through_untouched(self):
        # Explicit paths and by-id links must keep working.
        for spec in ("/dev/ttyACM0", "/dev/serial/by-id/usb-Adafruit_QT_Py_M0_X-if00"):
            self.assertEqual(resolve_serial_port(spec, self.attached), spec)

    def test_a_serial_number_selects_one_physical_unit(self):
        resolved = resolve_serial_port("usb:serial=5B5620DE5032434A582E3120FF031835", self.attached)
        self.assertEqual(resolved, "/dev/ttyACM1")

    def test_serial_matching_ignores_case(self):
        self.assertEqual(resolve_serial_port("usb:serial=leader123", self.attached), "/dev/ttyACM0")

    def test_vid_and_pid_select_a_device_type(self):
        self.assertEqual(
            resolve_serial_port("usb:vid=1a86,pid=7523", self.attached), "/dev/ttyACM0"
        )

    def test_product_matches_a_substring(self):
        self.assertEqual(resolve_serial_port("usb:product=U-ARM", self.attached), "/dev/ttyACM0")

    def test_criteria_are_combined_with_and(self):
        # vid alone is ambiguous across the two Adafruit boards; adding the
        # serial narrows it to one.
        resolved = resolve_serial_port("usb:vid=239a,serial=OTHER456", self.attached)
        self.assertEqual(resolved, "/dev/ttyACM2")

    def test_an_ambiguous_selector_is_an_error_not_a_guess(self):
        # Two Adafruit boards share vid:pid. Picking either could attach the
        # leader to a tactile sensor, so this must fail rather than choose.
        with self.assertRaises(SerialPortError) as raised:
            resolve_serial_port("usb:vid=239a", self.attached)
        message = str(raised.exception)
        self.assertIn("ambiguous", message)
        self.assertIn("/dev/ttyACM1", message)
        self.assertIn("/dev/ttyACM2", message)

    def test_no_match_reports_what_is_attached(self):
        with self.assertRaises(SerialPortError) as raised:
            resolve_serial_port("usb:serial=NOTPRESENT", self.attached)
        message = str(raised.exception)
        self.assertIn("matched no attached", message)
        self.assertIn("LEADER123", message)

    def test_no_match_with_nothing_attached_says_so(self):
        with self.assertRaisesRegex(SerialPortError, "No USB serial devices are attached"):
            resolve_serial_port("usb:serial=ANY", ())

    def test_a_non_hexadecimal_vid_is_rejected(self):
        with self.assertRaisesRegex(SerialPortError, "hexadecimal"):
            resolve_serial_port("usb:vid=zzzz", self.attached)


class DescriptionTests(unittest.TestCase):
    def test_a_serial_number_yields_the_most_specific_selector(self):
        self.assertEqual(EFLESH.selector(), "usb:serial=5B5620DE5032434A582E3120FF031835")

    def test_without_a_serial_number_vid_and_pid_are_used(self):
        anonymous = candidate("/dev/ttyACM9", serial=None)
        self.assertEqual(anonymous.selector(), "usb:vid=239a,pid=800f")

    def test_description_includes_identity_and_serial(self):
        described = EFLESH.describe()
        self.assertIn("/dev/ttyACM1", described)
        self.assertIn("239a:800f", described)
        self.assertIn("Adafruit QT Py M0", described)


if __name__ == "__main__":
    unittest.main()

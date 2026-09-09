"""Resolve serial devices by USB identity rather than by enumeration order.

The U-ARM leader and a USB CDC sensor both appear as ``/dev/ttyACM*`` and the
numbering depends on boot order, so a device node is not a stable way to name
either of them. A selector here describes *which device* by its USB descriptors,
and resolution fails loudly when the answer is not unique.

Refusing to guess is the point. Silently attaching to the wrong device would
mean reading a tactile sensor as servo positions, so an ambiguous selector is an
error rather than a coin flip.
"""

from __future__ import annotations

from dataclasses import dataclass

from serial.tools import list_ports


SELECTOR_PREFIX = "usb:"


class SerialPortError(RuntimeError):
    """Raised when a selector matches no device, or more than one."""


@dataclass(frozen=True)
class PortCandidate:
    """One attached USB serial device and the identity it advertises."""

    device: str
    vid: int | None
    pid: int | None
    serial_number: str | None
    manufacturer: str | None
    product: str | None

    def describe(self) -> str:
        """Return a single-line human description of this device."""
        identity = "unknown"
        if self.vid is not None and self.pid is not None:
            identity = f"{self.vid:04x}:{self.pid:04x}"
        label = " ".join(part for part in (self.manufacturer, self.product) if part)
        suffix = f" serial={self.serial_number}" if self.serial_number else ""
        return f"{self.device}  {identity}  {label or 'unnamed'}{suffix}"

    def selector(self) -> str:
        """Return the most specific selector that identifies this device."""
        if self.serial_number:
            return f"{SELECTOR_PREFIX}serial={self.serial_number}"
        if self.vid is not None and self.pid is not None:
            return f"{SELECTOR_PREFIX}vid={self.vid:04x},pid={self.pid:04x}"
        return self.device


def list_usb_serial_ports() -> tuple[PortCandidate, ...]:
    """Return every attached USB serial device.

    Ports without USB descriptors are excluded. A typical Linux host exposes
    dozens of legacy ``/dev/ttyS*`` nodes that are never the hardware meant
    here, and including them would bury the real devices.

    Returns:
        Candidates ordered by device node.
    """
    candidates = [
        PortCandidate(
            device=port.device,
            vid=port.vid,
            pid=port.pid,
            serial_number=port.serial_number,
            manufacturer=port.manufacturer,
            product=port.product,
        )
        for port in list_ports.comports()
        if port.vid is not None
    ]
    return tuple(sorted(candidates, key=lambda candidate: candidate.device))


def is_selector(spec: str) -> bool:
    """Report whether a configured value is a USB selector rather than a path."""
    return spec.startswith(SELECTOR_PREFIX)


def parse_selector(spec: str) -> dict[str, str]:
    """Parse a ``usb:key=value,...`` selector into its criteria.

    Args:
        spec: Selector string beginning with ``usb:``.

    Returns:
        Lower-cased criteria keyed by field name.

    Raises:
        SerialPortError: If the selector is empty, malformed, or names an
            unsupported field.
    """
    supported = {"vid", "pid", "serial", "manufacturer", "product"}
    body = spec[len(SELECTOR_PREFIX) :].strip()
    if not body:
        raise SerialPortError(f"Selector '{spec}' names no criteria")
    criteria: dict[str, str] = {}
    for clause in body.split(","):
        key, separator, value = clause.partition("=")
        key = key.strip().lower()
        value = value.strip()
        if not separator or not key or not value:
            raise SerialPortError(f"Selector clause '{clause.strip()}' must be key=value")
        if key not in supported:
            raise SerialPortError(
                f"Selector field '{key}' is not supported; use one of "
                + ", ".join(sorted(supported))
            )
        criteria[key] = value
    return criteria


def _matches(candidate: PortCandidate, criteria: dict[str, str]) -> bool:
    for key, value in criteria.items():
        if key in ("vid", "pid"):
            attribute = candidate.vid if key == "vid" else candidate.pid
            try:
                wanted = int(value, 16)
            except ValueError as error:
                raise SerialPortError(f"Selector {key}='{value}' is not hexadecimal") from error
            if attribute != wanted:
                return False
        elif key == "serial":
            if (candidate.serial_number or "").lower() != value.lower():
                return False
        else:
            attribute = getattr(candidate, key) or ""
            if value.lower() not in attribute.lower():
                return False
    return True


def resolve_serial_port(spec: str, candidates: tuple[PortCandidate, ...] | None = None) -> str:
    """Resolve a configured device value to a concrete device node.

    A value that is not a ``usb:`` selector is returned unchanged, so explicit
    paths and ``/dev/serial/by-id`` links keep working.

    Args:
        spec: Device path or ``usb:`` selector from configuration.
        candidates: Attached devices, enumerated from the host when omitted.

    Returns:
        The device node to open.

    Raises:
        SerialPortError: If the selector matches no attached device, or matches
            more than one and is therefore ambiguous.
    """
    if not is_selector(spec):
        return spec
    criteria = parse_selector(spec)
    attached = list_usb_serial_ports() if candidates is None else candidates
    matches = [candidate for candidate in attached if _matches(candidate, criteria)]

    if not matches:
        available = "\n".join(f"  {candidate.describe()}" for candidate in attached)
        raise SerialPortError(
            f"Selector '{spec}' matched no attached USB serial device.\n"
            + (f"Attached:\n{available}" if attached else "No USB serial devices are attached.")
        )
    if len(matches) > 1:
        listed = "\n".join(f"  {candidate.describe()}" for candidate in matches)
        raise SerialPortError(
            f"Selector '{spec}' is ambiguous and matched {len(matches)} devices:\n{listed}\n"
            "Add a discriminator such as serial= to identify exactly one."
        )
    return matches[0].device

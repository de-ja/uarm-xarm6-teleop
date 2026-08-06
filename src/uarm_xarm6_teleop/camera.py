"""Dynamic V4L2 camera discovery and shared browser video streams."""

from __future__ import annotations

import fcntl
import hashlib
import math
import os
import struct
import threading
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

VIDIOC_QUERYCAP = 0x80685600
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
V4L2_CAP_DEVICE_CAPS = 0x80000000


class CameraError(RuntimeError):
    """Raised when a camera cannot be discovered or streamed."""


@dataclass(frozen=True)
class CameraCapability:
    """Describe the identity and capture support reported by a V4L2 node."""

    name: str
    bus_info: str
    can_capture: bool


@dataclass(frozen=True)
class CameraInfo:
    """Describe one stable browser-selectable camera source."""

    id: str
    name: str
    device: str

    def to_dict(self) -> dict[str, str]:
        """Serialize the camera for the HTTP API."""
        return {"id": self.id, "name": self.name, "device": self.device}


def _decode_c_string(value: bytes) -> str:
    return value.split(b"\0", 1)[0].decode(errors="replace").strip()


def query_camera_capability(device: Path) -> CameraCapability:
    """Read V4L2 capabilities without starting camera capture."""
    descriptor = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    try:
        payload = bytearray(104)
        fcntl.ioctl(descriptor, VIDIOC_QUERYCAP, payload, True)
    finally:
        os.close(descriptor)

    _driver, card, bus_info, _version, capabilities, device_caps, *_reserved = struct.unpack(
        "16s32s32sIII3I", payload
    )
    effective = device_caps if capabilities & V4L2_CAP_DEVICE_CAPS else capabilities
    capture_mask = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE
    return CameraCapability(
        name=_decode_c_string(card),
        bus_info=_decode_c_string(bus_info),
        can_capture=bool(effective & capture_mask),
    )


class CameraCatalog:
    """Discover one browser-facing RGB source per physical V4L2 camera.

    Args:
        dev_root: Device-tree root containing V4L2 nodes and stable links.
        capability_reader: Read-only function used to query a device node.
    """

    def __init__(
        self,
        dev_root: Path = Path("/dev"),
        capability_reader: Callable[[Path], CameraCapability] = query_camera_capability,
    ) -> None:
        self.dev_root = dev_root
        self.capability_reader = capability_reader

    @staticmethod
    def _node_sort_key(path: Path) -> tuple[int, str]:
        suffix = path.name.removeprefix("video")
        return (int(suffix) if suffix.isdigit() else 1_000_000, path.name)

    def _capabilities(self) -> dict[Path, CameraCapability]:
        discovered: dict[Path, CameraCapability] = {}
        for node in sorted(self.dev_root.glob("video*"), key=self._node_sort_key):
            try:
                capability = self.capability_reader(node)
            except (OSError, ValueError, struct.error):
                continue
            if capability.can_capture:
                discovered[node.resolve()] = capability
        return discovered

    def list_cameras(self) -> tuple[CameraInfo, ...]:
        """Return deduplicated capture devices, preferring stable symlinks."""
        capabilities = self._capabilities()
        if not capabilities:
            return ()

        selected: list[tuple[Path, Path, CameraCapability]] = []
        selected_nodes: set[Path] = set()
        selected_buses: set[str] = set()

        stable_patterns = (
            self.dev_root / "v4l" / "by-id" / "*-video-index0",
            self.dev_root / "v4l" / "by-path" / "*-video-index0",
        )
        for pattern in stable_patterns:
            for link in sorted(pattern.parent.glob(pattern.name)):
                try:
                    node = link.resolve(strict=True)
                except OSError:
                    continue
                capability = capabilities.get(node)
                if capability is None or node in selected_nodes:
                    continue
                if capability.bus_info and capability.bus_info in selected_buses:
                    continue
                selected.append((link, node, capability))
                selected_nodes.add(node)
                if capability.bus_info:
                    selected_buses.add(capability.bus_info)

        for node, capability in capabilities.items():
            if node in selected_nodes:
                continue
            if capability.bus_info and capability.bus_info in selected_buses:
                continue
            selected.append((node, node, capability))
            selected_nodes.add(node)
            if capability.bus_info:
                selected_buses.add(capability.bus_info)

        cameras = []
        for source, node, capability in selected:
            identity = f"{source.name}\0{capability.bus_info}"
            camera_id = hashlib.sha256(identity.encode()).hexdigest()[:16]
            cameras.append(
                CameraInfo(
                    id=camera_id,
                    name=capability.name or node.name,
                    device=str(source),
                )
            )
        return tuple(cameras)

    def get(self, camera_id: str) -> CameraInfo:
        """Resolve a current camera ID.

        Args:
            camera_id: Stable identifier returned by :meth:`list_cameras`.

        Returns:
            The currently available camera description.

        Raises:
            CameraError: If the device has disappeared or its ID is unknown.
        """
        for camera in self.list_cameras():
            if camera.id == camera_id:
                return camera
        raise CameraError("Camera is no longer available; refresh the camera list")


class _VideoCapture(Protocol):
    def isOpened(self) -> bool: ...

    def read(self) -> tuple[bool, object]: ...

    def release(self) -> None: ...


@dataclass(frozen=True)
class CameraFrame:
    """Store an encoded frame and the metadata used for latency measurement."""

    jpeg: bytes
    captured_at: float
    jpeg_quality: int


class AdaptiveJpegQuality:
    """Favor delivery latency while cautiously recovering image quality.

    Args:
        initial: Initial JPEG quality from 1 through 100.
        minimum: Lowest permitted adaptive quality.
        maximum: Highest permitted adaptive quality.
        target_latency_ms: Desired capture-to-browser latency.

    Raises:
        ValueError: If quality bounds or the latency target are invalid.
    """

    def __init__(
        self,
        initial: int = 80,
        minimum: int = 35,
        maximum: int = 85,
        target_latency_ms: float = 75.0,
    ) -> None:
        if not 1 <= minimum <= initial <= maximum <= 100:
            raise ValueError("JPEG quality must satisfy 1 <= minimum <= initial <= maximum <= 100")
        if target_latency_ms <= 0:
            raise ValueError("Target camera latency must be positive")
        self._quality = initial
        self.minimum = minimum
        self.maximum = maximum
        self.target_latency_ms = target_latency_ms
        self._healthy_reports = 0
        self._lock = threading.Lock()

    @property
    def quality(self) -> int:
        """Return the current thread-safe JPEG quality."""
        with self._lock:
            return self._quality

    def observe(self, latency_ms: float) -> int:
        """Update quality from one browser latency report.

        Args:
            latency_ms: Measured capture-to-browser latency in milliseconds.

        Returns:
            The new JPEG quality.

        Raises:
            ValueError: If latency is negative or non-finite.
        """
        if not math.isfinite(latency_ms) or latency_ms < 0:
            raise ValueError("Camera latency must be a finite, non-negative value")

        with self._lock:
            if latency_ms > self.target_latency_ms:
                reduction = 15 if latency_ms > self.target_latency_ms * 2 else 8
                self._quality = max(self.minimum, self._quality - reduction)
                self._healthy_reports = 0
            elif latency_ms < self.target_latency_ms * 0.6:
                self._healthy_reports += 1
                if self._healthy_reports >= 4:
                    self._quality = min(self.maximum, self._quality + 3)
                    self._healthy_reports = 0
            else:
                self._healthy_reports = 0
            return self._quality


class _CameraSession:
    def __init__(
        self,
        camera: CameraInfo,
        width: int,
        height: int,
        fps: int,
        quality: AdaptiveJpegQuality,
        capture_factory: Callable[[str], _VideoCapture],
        frame_encoder: Callable[[object, int], bytes],
        wall_time: Callable[[], float],
    ) -> None:
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.capture_factory = capture_factory
        self.frame_encoder = frame_encoder
        self.wall_time = wall_time
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frame: CameraFrame | None = None
        self.sequence = 0
        self.error: str | None = None
        self.subscribers = 0
        self.thread = threading.Thread(
            target=self._capture,
            name=f"camera-{camera.id}",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def _capture(self) -> None:
        capture: _VideoCapture | None = None
        try:
            capture = self.capture_factory(self.camera.device)
            if not capture.isOpened():
                raise CameraError(f"Could not open {self.camera.name}")

            failed_reads = 0
            while not self.stop_event.is_set():
                ok, raw_frame = capture.read()
                if not ok:
                    failed_reads += 1
                    if failed_reads >= 3:
                        raise CameraError(f"Video capture stopped for {self.camera.name}")
                    continue
                failed_reads = 0
                captured_at = self.wall_time()
                jpeg_quality = self.quality.quality
                encoded = self.frame_encoder(raw_frame, jpeg_quality)
                with self.condition:
                    self.frame = CameraFrame(encoded, captured_at, jpeg_quality)
                    self.sequence += 1
                    self.condition.notify_all()
        except (CameraError, OSError, RuntimeError, ValueError) as error:
            with self.condition:
                self.error = str(error)
                self.condition.notify_all()
        finally:
            if capture is not None:
                capture.release()

    def wait_until_ready(self, timeout: float = 5.0) -> None:
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.frame is not None or self.error is not None,
                timeout=timeout,
            )
            if not ready:
                raise CameraError(f"Timed out opening {self.camera.name}")
            if self.error is not None:
                raise CameraError(self.error)

    def wait_for_frame(self, sequence: int) -> tuple[int, CameraFrame] | None:
        with self.condition:
            self.condition.wait_for(
                lambda: (
                    self.sequence != sequence or self.error is not None or self.stop_event.is_set()
                ),
                timeout=5.0,
            )
            if self.sequence != sequence and self.frame is not None:
                return self.sequence, self.frame
            if self.error is not None or self.stop_event.is_set():
                return None
            return None

    def stop(self) -> None:
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=2.0)


class CameraSubscription:
    """Stream frames from one shared camera session for one HTTP subscriber."""

    def __init__(self, manager: CameraManager, session: _CameraSession) -> None:
        self.manager = manager
        self.session = session
        self.closed = False

    def iter_mjpeg(self) -> Iterator[bytes]:
        """Yield multipart JPEG records until the session or subscriber closes."""
        sequence = -1
        try:
            while not self.closed:
                current = self.session.wait_for_frame(sequence)
                if current is None:
                    return
                sequence, frame = current
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame.jpeg)}\r\n".encode()
                    + f"X-Frame-Sequence: {sequence}\r\n".encode()
                    + f"X-Capture-Timestamp: {frame.captured_at:.6f}\r\n".encode()
                    + f"X-JPEG-Quality: {frame.jpeg_quality}\r\n\r\n".encode()
                    + frame.jpeg
                    + b"\r\n"
                )
        finally:
            self.close()

    def close(self) -> None:
        """Release this subscriber and stop an otherwise unused capture session."""
        if not self.closed:
            self.closed = True
            self.manager._unsubscribe(self.session)


class CameraManager:
    """Own shared camera capture sessions and adaptive stream quality.

    Args:
        catalog: Camera discovery service.
        width: Requested capture width in pixels.
        height: Requested capture height in pixels.
        fps: Requested capture frame rate.
        jpeg_quality: Initial JPEG quality.
        min_jpeg_quality: Minimum adaptive JPEG quality.
        max_jpeg_quality: Maximum adaptive JPEG quality.
        target_latency_ms: Desired capture-to-browser latency.
        capture_factory: Optional capture factory used for testing or alternate sources.
        frame_encoder: Optional JPEG encoder.
        wall_time: Clock used to timestamp frames for browser latency measurement.
    """

    def __init__(
        self,
        catalog: CameraCatalog | None = None,
        *,
        width: int = 1280,
        height: int = 720,
        fps: int = 15,
        jpeg_quality: int = 80,
        min_jpeg_quality: int = 35,
        max_jpeg_quality: int = 85,
        target_latency_ms: float = 75.0,
        capture_factory: Callable[[str], _VideoCapture] | None = None,
        frame_encoder: Callable[[object, int], bytes] | None = None,
        wall_time: Callable[[], float] = time.time,
    ) -> None:
        self.catalog = catalog or CameraCatalog()
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.min_jpeg_quality = min_jpeg_quality
        self.max_jpeg_quality = max_jpeg_quality
        self.target_latency_ms = target_latency_ms
        self.capture_factory = capture_factory or self._open_capture
        self.frame_encoder = frame_encoder or self._encode_frame
        self.wall_time = wall_time
        self.lock = threading.Lock()
        self.sessions: dict[str, _CameraSession] = {}

    def _open_capture(self, device: str) -> _VideoCapture:
        try:
            import cv2
        except ImportError as error:
            raise CameraError(
                "Camera streaming dependencies are missing; reinstall with `pip install -e '.[web]'`"
            ) from error
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)
        return capture

    @staticmethod
    def _encode_frame(frame: object, quality: int) -> bytes:
        import cv2

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise CameraError("Could not encode camera frame")
        return encoded.tobytes()

    def list_cameras(self) -> tuple[CameraInfo, ...]:
        """Return the cameras currently offered to the operator console."""
        return self.catalog.list_cameras()

    def subscribe(self, camera_id: str) -> CameraSubscription:
        """Subscribe to a shared camera capture session.

        Args:
            camera_id: ID returned by :meth:`list_cameras`.

        Returns:
            A stream subscription that must eventually be closed.

        Raises:
            CameraError: If the camera disappears, cannot open, or produces no frame.
        """
        with self.lock:
            session = self.sessions.get(camera_id)
            if session is None:
                camera = self.catalog.get(camera_id)
                quality = AdaptiveJpegQuality(
                    self.jpeg_quality,
                    self.min_jpeg_quality,
                    self.max_jpeg_quality,
                    self.target_latency_ms,
                )
                session = _CameraSession(
                    camera,
                    self.width,
                    self.height,
                    self.fps,
                    quality,
                    self.capture_factory,
                    self.frame_encoder,
                    self.wall_time,
                )
                self.sessions[camera_id] = session
                session.start()
            session.subscribers += 1

        try:
            session.wait_until_ready()
        except CameraError:
            self._unsubscribe(session)
            raise
        return CameraSubscription(self, session)

    def report_latency(self, camera_id: str, latency_ms: float) -> int:
        """Apply browser latency feedback and return the resulting JPEG quality."""
        with self.lock:
            session = self.sessions.get(camera_id)
        if session is None:
            raise CameraError("Camera is not currently streaming")
        return session.quality.observe(latency_ms)

    def _unsubscribe(self, session: _CameraSession) -> None:
        should_stop = False
        with self.lock:
            session.subscribers = max(0, session.subscribers - 1)
            if session.subscribers == 0 and self.sessions.get(session.camera.id) is session:
                self.sessions.pop(session.camera.id, None)
                should_stop = True
        if should_stop:
            session.stop()

    def close(self) -> None:
        """Stop and remove every active camera capture session."""
        with self.lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
            for session in sessions:
                session.subscribers = 0
        for session in sessions:
            session.stop()

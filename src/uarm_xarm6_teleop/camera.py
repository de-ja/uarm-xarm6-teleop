"""Dynamic V4L2 camera discovery and shared browser video streams."""

from __future__ import annotations

import fcntl
import hashlib
import os
import struct
import threading
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
    name: str
    bus_info: str
    can_capture: bool


@dataclass(frozen=True)
class CameraInfo:
    id: str
    name: str
    device: str

    def to_dict(self) -> dict[str, str]:
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
    """Discover one browser-facing RGB source per physical V4L2 camera."""

    def __init__(
        self,
        dev_root: Path = Path("/dev"),
        capability_reader: Callable[[Path], CameraCapability] = query_camera_capability,
    ):
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
        for camera in self.list_cameras():
            if camera.id == camera_id:
                return camera
        raise CameraError("Camera is no longer available; refresh the camera list")


class VideoCapture(Protocol):
    def isOpened(self) -> bool: ...

    def read(self): ...

    def release(self) -> None: ...


class _CameraSession:
    def __init__(
        self,
        camera: CameraInfo,
        width: int,
        height: int,
        fps: int,
        jpeg_quality: int,
        capture_factory: Callable[[str], VideoCapture],
        frame_encoder: Callable[[object, int], bytes],
    ):
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.capture_factory = capture_factory
        self.frame_encoder = frame_encoder
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frame: bytes | None = None
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
        capture: VideoCapture | None = None
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
                encoded = self.frame_encoder(raw_frame, self.jpeg_quality)
                with self.condition:
                    self.frame = encoded
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

    def wait_for_frame(self, sequence: int) -> tuple[int, bytes] | None:
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence != sequence
                or self.error is not None
                or self.stop_event.is_set(),
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
    def __init__(self, manager: CameraManager, session: _CameraSession):
        self.manager = manager
        self.session = session
        self.closed = False

    def iter_mjpeg(self) -> Iterator[bytes]:
        sequence = -1
        try:
            while not self.closed:
                current = self.session.wait_for_frame(sequence)
                if current is None:
                    return
                sequence, frame = current
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                    + frame
                    + b"\r\n"
                )
        finally:
            self.close()

    def close(self) -> None:
        if not self.closed:
            self.closed = True
            self.manager._unsubscribe(self.session)


class CameraManager:
    def __init__(
        self,
        catalog: CameraCatalog | None = None,
        *,
        width: int = 1280,
        height: int = 720,
        fps: int = 15,
        jpeg_quality: int = 80,
        capture_factory: Callable[[str], VideoCapture] | None = None,
        frame_encoder: Callable[[object, int], bytes] | None = None,
    ):
        self.catalog = catalog or CameraCatalog()
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.capture_factory = capture_factory or self._open_capture
        self.frame_encoder = frame_encoder or self._encode_frame
        self.lock = threading.Lock()
        self.sessions: dict[str, _CameraSession] = {}

    def _open_capture(self, device: str) -> VideoCapture:
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
        return self.catalog.list_cameras()

    def subscribe(self, camera_id: str) -> CameraSubscription:
        with self.lock:
            session = self.sessions.get(camera_id)
            if session is None:
                camera = self.catalog.get(camera_id)
                session = _CameraSession(
                    camera,
                    self.width,
                    self.height,
                    self.fps,
                    self.jpeg_quality,
                    self.capture_factory,
                    self.frame_encoder,
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
        with self.lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
            for session in sessions:
                session.subscribers = 0
        for session in sessions:
            session.stop()

"""FastAPI application that exposes guarded controller transitions."""

from __future__ import annotations

import asyncio
import threading
import time
from collections.abc import Callable
from contextlib import asynccontextmanager
from dataclasses import asdict
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from ..backends.xarm import XArmHardwareError
from ..camera import CameraError, CameraManager
from ..config import TeleopConfig, load_config
from ..controller import TeleopController, TeleopControllerError, TeleopSnapshot
from ..feetech import FeetechError


class RobotRequest(BaseModel):
    robot_ip: str = Field(min_length=1, max_length=255)


class StartRequest(BaseModel):
    mode: Literal["dry_run", "physical"]
    confirmation: str | None = None


class CameraLatencyRequest(BaseModel):
    latency_ms: float = Field(ge=0, le=60_000)


class TelemetryClients:
    """Track browser supervision without coupling it to the hardware loop."""

    def __init__(self, controller: TeleopController):
        self.controller = controller
        self._lock = threading.Lock()
        self._count = 0

    def connected(self) -> None:
        with self._lock:
            self._count += 1

    def disconnected(self) -> bool:
        with self._lock:
            self._count = max(0, self._count - 1)
            return self._count == 0


def _invoke(operation: Callable[[], TeleopSnapshot]) -> dict[str, object]:
    try:
        return operation().to_dict()
    except (TeleopControllerError, FeetechError, XArmHardwareError, ValueError, OSError) as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


def create_app(
    controller: TeleopController | None = None,
    *,
    config: TeleopConfig | None = None,
    camera_manager: CameraManager | None = None,
) -> FastAPI:
    active_controller = controller or TeleopController(config or load_config())
    active_cameras = camera_manager or CameraManager()
    telemetry_clients = TelemetryClients(active_controller)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        yield
        await asyncio.to_thread(active_cameras.close)
        await asyncio.to_thread(active_controller.close)

    app = FastAPI(
        title="U-ARM xArm6 Operator API",
        version="0.2.0",
        lifespan=lifespan,
    )
    app.state.teleop_controller = active_controller
    app.state.camera_manager = active_cameras
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
        allow_credentials=False,
        allow_methods=["GET", "POST"],
        allow_headers=["content-type"],
    )

    @app.get("/api/health")
    def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.get("/api/time")
    def server_time() -> dict[str, float]:
        return {"timestamp": time.time()}

    @app.get("/api/status")
    def status() -> dict[str, object]:
        return active_controller.snapshot().to_dict()

    @app.get("/api/config")
    def get_config() -> dict[str, object]:
        return asdict(active_controller.config)

    @app.get("/api/cameras")
    def cameras() -> list[dict[str, str]]:
        return [camera.to_dict() for camera in active_cameras.list_cameras()]

    @app.get("/api/cameras/{camera_id}/stream")
    def camera_stream(camera_id: str) -> StreamingResponse:
        try:
            subscription = active_cameras.subscribe(camera_id)
        except CameraError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        return StreamingResponse(
            subscription.iter_mjpeg(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, max-age=0"},
        )

    @app.post("/api/cameras/{camera_id}/latency")
    def camera_latency(camera_id: str, request: CameraLatencyRequest) -> dict[str, object]:
        try:
            quality = active_cameras.report_latency(camera_id, request.latency_ms)
        except (CameraError, ValueError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        return {"mode": "auto", "jpeg_quality": quality}

    @app.post("/api/leader/connect")
    def connect_leader() -> dict[str, object]:
        return _invoke(active_controller.connect_leader)

    @app.post("/api/robot/inspect")
    def inspect_robot(request: RobotRequest) -> dict[str, object]:
        return _invoke(lambda: active_controller.inspect_robot(request.robot_ip))

    @app.post("/api/teleop/start")
    def start_teleop(request: StartRequest) -> dict[str, object]:
        return _invoke(
            lambda: active_controller.start(
                request.mode,
                confirmation=request.confirmation,
            )
        )

    @app.post("/api/teleop/stop")
    def stop_teleop() -> dict[str, object]:
        return _invoke(active_controller.stop)

    @app.post("/api/session/disconnect")
    def disconnect() -> dict[str, object]:
        return _invoke(active_controller.disconnect)

    @app.post("/api/fault/reset")
    def reset_fault() -> dict[str, object]:
        return _invoke(active_controller.reset_fault)

    @app.websocket("/ws/telemetry")
    async def telemetry(
        websocket: WebSocket,
        frequency: float = Query(default=10.0, ge=1.0, le=20.0),
    ) -> None:
        await websocket.accept()
        telemetry_clients.connected()
        period = 1.0 / frequency

        async def send_snapshots() -> None:
            while True:
                await websocket.send_json(active_controller.snapshot().to_dict())
                await asyncio.sleep(period)

        async def wait_for_disconnect() -> None:
            while True:
                message = await websocket.receive()
                if message["type"] == "websocket.disconnect":
                    return

        sender = asyncio.create_task(send_snapshots())
        receiver = asyncio.create_task(wait_for_disconnect())
        try:
            completed, _pending = await asyncio.wait(
                {sender, receiver}, return_when=asyncio.FIRST_COMPLETED
            )
            for task in completed:
                await task
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            sender.cancel()
            receiver.cancel()
            await asyncio.gather(sender, receiver, return_exceptions=True)
            if telemetry_clients.disconnected():
                await asyncio.to_thread(active_controller.stop)

    frontend_dist = Path(__file__).resolve().parent / "dist"
    if frontend_dist.is_dir():
        app.mount("/", StaticFiles(directory=frontend_dist, html=True), name="frontend")
    else:

        @app.get("/", include_in_schema=False)
        def frontend_missing() -> JSONResponse:
            return JSONResponse(
                {
                    "message": "Operator frontend is not built.",
                    "docs": "/docs",
                    "health": "/api/health",
                }
            )

    return app


app = create_app()

"""Async JSON Lines client for the TestH743 gantry TCP server."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import secrets
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum, auto
from typing import Any


class ProtocolError(RuntimeError):
    pass


class RemoteCommandError(RuntimeError):
    pass


class ConnectionState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    READY = auto()
    ERROR = auto()


@dataclass(frozen=True)
class DeviceConfig:
    host: str = "192.168.10.111"
    port: int = 5000
    device_id: str = "gantry"
    local_device_id: str = "center"


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    status: str
    message: dict[str, Any]


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _new_id(prefix: str) -> str:
    return f"{prefix}-{secrets.token_hex(8)}"


class DeviceConnection:
    def __init__(self, name: str, config: DeviceConfig):
        self.name = name
        self.config = config
        self.reader: asyncio.StreamReader | None = None
        self.writer: asyncio.StreamWriter | None = None
        self.state = ConnectionState.DISCONNECTED
        self.hello: dict[str, Any] | None = None
        self.last_heartbeat: dict[str, Any] | None = None
        self.last_status: dict[str, Any] | None = None
        self._reader_task: asyncio.Task[None] | None = None
        self._write_lock = asyncio.Lock()
        self._ack_waiters: dict[str, asyncio.Future[dict[str, Any]]] = {}
        self._event_waiters: dict[str, asyncio.Future[dict[str, Any]]] = {}

    async def connect(self, timeout: float = 3.0) -> dict[str, Any]:
        if self.state is ConnectionState.READY and self.hello is not None:
            return self.hello
        await self.close()
        self.state = ConnectionState.CONNECTING
        try:
            self.reader, self.writer = await asyncio.wait_for(
                asyncio.open_connection(self.config.host, self.config.port), timeout
            )
            hello = await asyncio.wait_for(self._read_message(), timeout)
            self._validate_envelope(hello)
            if hello["type"] != "hello":
                raise ProtocolError("the first server message is not hello")
            payload = hello["payload"]
            if payload.get("protocol_version") != 1:
                raise ProtocolError("unsupported protocol_version")
            self.hello = hello
            await self._send_message({
                "type": "heartbeat",
                "device_id": self.config.local_device_id,
                "msg_id": _new_id("center"),
                "timestamp": _timestamp(),
                "payload": {"status": "online"},
            })
            self.state = ConnectionState.READY
            self._reader_task = asyncio.create_task(
                self._reader_loop(), name=f"{self.name}-jsonl-reader"
            )
            return hello
        except Exception:
            await self.close()
            self.state = ConnectionState.ERROR
            raise

    async def command(
        self,
        name: str,
        target: dict[str, Any] | None = None,
        *,
        command_id: str | None = None,
        timeout: float = 3.0,
    ) -> CommandResult:
        if self.state is not ConnectionState.READY:
            await self.connect(timeout)
        command_id = command_id or _new_id("cmd")
        if command_id in self._ack_waiters or command_id in self._event_waiters:
            raise ValueError(f"command_id is already pending: {command_id}")

        loop = asyncio.get_running_loop()
        ack_future = loop.create_future()
        event_future = loop.create_future()
        self._ack_waiters[command_id] = ack_future
        self._event_waiters[command_id] = event_future
        message = {
            "type": "command",
            "device_id": self.config.local_device_id,
            "msg_id": _new_id("center"),
            "timestamp": _timestamp(),
            "payload": {
                "command_id": command_id,
                "name": name,
                "target": target or {},
            },
        }
        try:
            await self._send_message(message)
            ack = await asyncio.wait_for(asyncio.shield(ack_future), timeout)
        except Exception:
            self._ack_waiters.pop(command_id, None)
            self._event_waiters.pop(command_id, None)
            self._discard_future(event_future)
            raise
        finally:
            self._ack_waiters.pop(command_id, None)

        payload = ack["payload"]
        if payload.get("status") != "accepted":
            self._event_waiters.pop(command_id, None)
            self._discard_future(event_future)
            raise RemoteCommandError(payload.get("reason", "command rejected"))
        return CommandResult(command_id, "accepted", ack)

    async def wait_event(
        self, command_id: str, *, timeout: float = 660.0
    ) -> CommandResult:
        future = self._event_waiters.get(command_id)
        if future is None:
            raise ValueError(f"no pending command: {command_id}")
        try:
            event = await asyncio.wait_for(asyncio.shield(future), timeout)
        except TimeoutError:
            future.cancel()
            raise TimeoutError(
                f"{self.name}: ACK accepted for {command_id}, "
                f"but no final event arrived within {timeout}s"
            ) from None
        finally:
            self._event_waiters.pop(command_id, None)
        payload = event["payload"]
        status = payload.get("status")
        if status != "completed":
            raise RemoteCommandError(payload.get("reason", "command failed"))
        return CommandResult(command_id, status, event)

    async def run_command(
        self,
        name: str,
        target: dict[str, Any] | None = None,
        *,
        command_id: str | None = None,
        ack_timeout: float = 3.0,
        completion_timeout: float = 660.0,
    ) -> CommandResult:
        accepted = await self.command(
            name, target, command_id=command_id, timeout=ack_timeout
        )
        return await self.wait_event(
            accepted.command_id, timeout=completion_timeout
        )

    async def _send_message(self, message: dict[str, Any]) -> None:
        if self.writer is None:
            raise ConnectionError(f"{self.name}: not connected")
        encoded = json.dumps(message, separators=(",", ":"), ensure_ascii=False)
        if "\n" in encoded or len(encoded.encode("utf-8")) >= 2048:
            raise ValueError("JSON message exceeds firmware line limit")
        async with self._write_lock:
            self.writer.write(encoded.encode("utf-8") + b"\n")
            await self.writer.drain()

    async def _read_message(self) -> dict[str, Any]:
        if self.reader is None:
            raise ConnectionError(f"{self.name}: not connected")
        data = await self.reader.readline()
        if not data:
            raise ConnectionError(f"{self.name}: peer closed connection")
        if len(data) >= 2048 or not data.endswith(b"\n"):
            raise ProtocolError("invalid JSONL framing")
        try:
            message = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError("invalid UTF-8 JSON message") from exc
        if not isinstance(message, dict):
            raise ProtocolError("message root must be an object")
        return message

    def _validate_envelope(self, message: dict[str, Any]) -> None:
        for field in ("type", "device_id", "msg_id", "timestamp"):
            if not isinstance(message.get(field), str) or not message[field]:
                raise ProtocolError(f"missing or invalid field: {field}")
        if message["device_id"] != self.config.device_id:
            raise ProtocolError("device_id mismatch")
        if not isinstance(message.get("payload"), dict):
            raise ProtocolError("missing or invalid field: payload")

    async def _reader_loop(self) -> None:
        try:
            while True:
                message = await self._read_message()
                self._validate_envelope(message)
                kind = message["type"]
                payload = message["payload"]
                if kind == "heartbeat":
                    self.last_heartbeat = message
                    continue
                if kind == "status":
                    self.last_status = message
                    continue
                if kind not in {"ack", "event"}:
                    raise ProtocolError(f"unexpected message type: {kind}")
                command_id = payload.get("command_id")
                if not isinstance(command_id, str) or not command_id:
                    raise ProtocolError(f"{kind} has no command_id")
                waiters = self._ack_waiters if kind == "ack" else self._event_waiters
                future = waiters.get(command_id)
                if future is not None and not future.done():
                    future.set_result(message)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.state = ConnectionState.ERROR
            self._fail_waiters(exc)

    def _fail_waiters(self, exc: Exception) -> None:
        for future in (*self._ack_waiters.values(), *self._event_waiters.values()):
            if not future.done():
                future.set_exception(exc)
            if not future.cancelled():
                future.exception()
        self._ack_waiters.clear()
        self._event_waiters.clear()

    @staticmethod
    def _discard_future(future: asyncio.Future[Any]) -> None:
        if not future.done():
            future.cancel()
        elif not future.cancelled():
            future.exception()

    async def close(self) -> None:
        task = self._reader_task
        self._reader_task = None
        if task is not None:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        writer = self.writer
        self.reader = None
        self.writer = None
        self.hello = None
        self.state = ConnectionState.DISCONNECTED
        self._fail_waiters(ConnectionError(f"{self.name}: connection closed"))
        if writer is not None:
            writer.close()
            if sys.platform != "win32":
                with contextlib.suppress(ConnectionError, OSError):
                    await writer.wait_closed()


class H743Controller:
    TASKS = {"red_pick", "tag_put", "red_find", "frame_put"}

    def __init__(self, connection: DeviceConnection):
        self.connection = connection

    async def run_mission(self, task: str, *, timeout: float = 660.0) -> CommandResult:
        if task not in self.TASKS:
            raise ValueError(f"unknown task: {task}")
        return await self.connection.run_command(
            "mission_start", {"task": task}, completion_timeout=timeout
        )

    async def set_payload(self, state: str) -> CommandResult:
        if state not in {"empty", "held"}:
            raise ValueError("payload state must be empty or held")
        return await self.connection.run_command(
            "payload_set", {"state": state}, completion_timeout=5.0
        )

    async def mission_cancel(self) -> CommandResult:
        return await self.connection.run_command(
            "mission_cancel", completion_timeout=10.0
        )

    async def axis_move(self, axis: str, delta_pulses: int) -> CommandResult:
        self._validate_axis(axis)
        if not isinstance(delta_pulses, int) or isinstance(delta_pulses, bool):
            raise ValueError("delta_pulses must be an integer")
        if delta_pulses == 0:
            raise ValueError("delta_pulses must be non-zero")
        return await self.connection.run_command(
            "axis_move", {"axis": axis, "delta_pulses": delta_pulses}
        )

    async def axis_stop(self, axis: str) -> CommandResult:
        self._validate_axis(axis)
        return await self.connection.run_command(
            "axis_stop", {"axis": axis}, completion_timeout=10.0
        )

    async def axis_status(self, axis: str | None = None) -> CommandResult:
        if axis is not None:
            self._validate_axis(axis)
        return await self.connection.run_command(
            "axis_status", {} if axis is None else {"axis": axis},
            completion_timeout=5.0,
        )

    async def status_query(self) -> CommandResult:
        return await self.connection.run_command(
            "status_query", completion_timeout=5.0
        )

    async def config_query(self, task: str | None = None) -> CommandResult:
        if task is not None and task not in self.TASKS:
            raise ValueError(f"unknown task: {task}")
        return await self.connection.run_command(
            "config_query", {} if task is None else {"task": task},
            completion_timeout=5.0,
        )

    async def z_set_zero(self) -> CommandResult:
        return await self.connection.run_command(
            "z_set_zero", completion_timeout=5.0
        )

    async def z_clear_fault(self) -> CommandResult:
        return await self.connection.run_command(
            "z_clear_fault", completion_timeout=10.0
        )

    @staticmethod
    def _validate_axis(axis: str) -> None:
        if axis not in {"x", "y", "z"}:
            raise ValueError("axis must be x, y, or z")

    async def emergency_stop(self) -> CommandResult:
        return await self.connection.run_command(
            "emergency_stop", completion_timeout=10.0
        )


async def _run_cli(args: argparse.Namespace) -> None:
    connection = DeviceConnection(
        "gantry", DeviceConfig(args.host, args.port, args.device_id)
    )
    controller = H743Controller(connection)
    try:
        if args.action == "hello":
            print(json.dumps(await connection.connect(), ensure_ascii=False))
        elif args.action in {"mission", "mission_start"}:
            result = await controller.run_mission(args.task, timeout=args.timeout)
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"payload", "payload_set"}:
            result = await controller.set_payload(args.state)
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"mission-cancel", "mission_cancel"}:
            result = await controller.mission_cancel()
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"axis-move", "axis_move"}:
            result = await controller.axis_move(args.axis, args.delta_pulses)
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"axis-stop", "axis_stop"}:
            result = await controller.axis_stop(args.axis)
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"axis-status", "axis_status"}:
            result = await controller.axis_status(args.axis)
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"status", "status_query"}:
            result = await controller.status_query()
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"config", "config_query"}:
            result = await controller.config_query(args.task)
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"z-zero", "z_set_zero"}:
            result = await controller.z_set_zero()
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"z-clear", "z_clear_fault"}:
            result = await controller.z_clear_fault()
            print(json.dumps(result.message, ensure_ascii=False))
        elif args.action in {"emergency-stop", "emergency_stop"}:
            result = await controller.emergency_stop()
            print(json.dumps(result.message, ensure_ascii=False))
    finally:
        await connection.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.10.111")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--device-id", default="gantry")
    parser.add_argument("--timeout", type=float, default=660.0)
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("hello")
    mission = commands.add_parser("mission", aliases=["mission_start"])
    mission.add_argument("task", choices=sorted(H743Controller.TASKS))
    payload = commands.add_parser("payload", aliases=["payload_set"])
    payload.add_argument("state", choices=["empty", "held"])
    commands.add_parser("mission-cancel", aliases=["mission_cancel"])
    axis_move = commands.add_parser("axis-move", aliases=["axis_move"])
    axis_move.add_argument("axis", choices=["x", "y", "z"])
    axis_move.add_argument("delta_pulses", type=int)
    axis_stop = commands.add_parser("axis-stop", aliases=["axis_stop"])
    axis_stop.add_argument("axis", choices=["x", "y", "z"])
    axis_status = commands.add_parser("axis-status", aliases=["axis_status"])
    axis_status.add_argument("axis", nargs="?", choices=["x", "y", "z"])
    commands.add_parser("status", aliases=["status_query"])
    config = commands.add_parser("config", aliases=["config_query"])
    config.add_argument("task", nargs="?", choices=sorted(H743Controller.TASKS))
    commands.add_parser("z-zero", aliases=["z_set_zero"])
    commands.add_parser("z-clear", aliases=["z_clear_fault"])
    commands.add_parser("emergency-stop", aliases=["emergency_stop"])
    return parser


if __name__ == "__main__":
    asyncio.run(_run_cli(_parser().parse_args()))

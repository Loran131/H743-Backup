import asyncio
import json
import unittest
from contextlib import redirect_stderr
from io import StringIO

from h743_client import (
    ConnectionState,
    DeviceConfig,
    DeviceConnection,
    H743Controller,
    ProtocolError,
    RemoteCommandError,
    _parser,
)


def message(kind, payload):
    return {
        "type": kind,
        "device_id": "gantry",
        "msg_id": f"server-{kind}",
        "timestamp": "uptime_ms:1",
        "payload": payload,
    }


class MockH743:
    def __init__(self, *, first_type="hello"):
        self.first_type = first_type
        self.server = None
        self.port = 0
        self.execution_count = 0
        self.completed = set()
        self.requests = []

    async def start(self):
        self.server = await asyncio.start_server(self.handle, "127.0.0.1", 0)
        self.port = self.server.sockets[0].getsockname()[1]

    async def close(self):
        self.server.close()
        await self.server.wait_closed()

    async def handle(self, reader, writer):
        hello = message(self.first_type, {
            "protocol_version": 1,
            "device_type": "gantry",
            "capabilities": ["red_pick", "payload_set"],
        })
        encoded = (json.dumps(hello) + "\n").encode()
        writer.write(encoded[:7])
        await writer.drain()
        writer.write(encoded[7:])
        await writer.drain()
        writer.write((json.dumps(message("status", {"system": {"state": "ready"}})) + "\n").encode())
        await writer.drain()
        try:
            while data := await reader.readline():
                request = json.loads(data)
                self.requests.append(request)
                if request["device_id"] != "center":
                    writer.close()
                    return
                if request["type"] == "heartbeat":
                    continue
                payload = request["payload"]
                command_id = payload["command_id"]
                name = payload["name"]
                if (name == "mission_start" and
                        payload.get("target", {}).get("task") == "frame_put"):
                    ack = message("ack", {
                        "command_id": command_id,
                        "status": "rejected",
                        "reason": "gantry_busy",
                    })
                    writer.write((json.dumps(ack) + "\n").encode())
                    await writer.drain()
                    continue
                ack = message("ack", {"command_id": command_id, "status": "accepted"})
                event = message("event", {"command_id": command_id, "status": "completed"})
                heartbeat = message("heartbeat", {"status": "online"})
                if command_id not in self.completed:
                    self.execution_count += 1
                    self.completed.add(command_id)
                # One write deliberately contains three JSONL frames.
                writer.write("".join(json.dumps(item) + "\n" for item in
                                     (ack, heartbeat, event)).encode())
                await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()


class H743ClientTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.mock = MockH743()
        await self.mock.start()
        self.connection = DeviceConnection(
            "test", DeviceConfig("127.0.0.1", self.mock.port)
        )
        self.controller = H743Controller(self.connection)

    async def asyncTearDown(self):
        await self.connection.close()
        await self.mock.close()

    async def test_split_hello_and_sticky_ack_heartbeat_event(self):
        result = await self.controller.run_mission("red_pick", timeout=1.0)
        self.assertEqual(result.status, "completed")
        self.assertEqual(self.connection.last_heartbeat["payload"]["status"], "online")
        self.assertEqual(self.connection.last_status["payload"]["system"]["state"], "ready")
        commands = [request for request in self.mock.requests
                    if request["type"] == "command"]
        payload = commands[-1]["payload"]
        self.assertEqual(payload["name"], "mission_start")
        self.assertEqual(payload["target"], {"task": "red_pick"})

    async def test_rejected_ack_raises(self):
        with self.assertRaisesRegex(RemoteCommandError, "gantry_busy"):
            await self.controller.run_mission("frame_put", timeout=1.0)

    async def test_duplicate_command_id_is_not_executed_twice(self):
        first = await self.connection.run_command(
            "mission_start", {"task": "red_pick"},
            command_id="same-id", completion_timeout=1.0
        )
        second = await self.connection.run_command(
            "mission_start", {"task": "red_pick"},
            command_id="same-id", completion_timeout=1.0
        )
        self.assertEqual(first.status, "completed")
        self.assertEqual(second.status, "completed")
        self.assertEqual(self.mock.execution_count, 1)

    async def test_first_message_must_be_hello(self):
        await self.connection.close()
        await self.mock.close()
        self.mock = MockH743(first_type="heartbeat")
        await self.mock.start()
        self.connection = DeviceConnection(
            "bad", DeviceConfig("127.0.0.1", self.mock.port)
        )
        with self.assertRaisesRegex(ProtocolError, "first server message"):
            await self.connection.connect()
        self.assertEqual(self.connection.state, ConnectionState.ERROR)

    async def test_v1_axis_and_z_commands(self):
        await self.controller.axis_move("x", 512000)
        await self.controller.axis_stop("z")
        await self.controller.axis_status("y")
        await self.controller.status_query()
        await self.controller.config_query("red_find")
        await self.controller.z_set_zero()
        await self.controller.z_clear_fault()
        commands = [request["payload"] for request in self.mock.requests
                    if request["type"] == "command"]
        self.assertEqual(commands[0]["target"],
                         {"axis": "x", "delta_pulses": 512000})
        self.assertEqual([item["name"] for item in commands], [
            "axis_move", "axis_stop", "axis_status", "status_query",
            "config_query", "z_set_zero", "z_clear_fault",
        ])

    async def test_local_validation_rejects_invalid_axis_and_zero_move(self):
        with self.assertRaises(ValueError):
            await self.controller.axis_move("q", 1)
        with self.assertRaises(ValueError):
            await self.controller.axis_move("x", 0)

    def test_protocol_command_names_are_cli_aliases(self):
        cases = {
            "mission_start red_pick": "mission_start",
            "axis_move x 1": "axis_move",
            "z_set_zero": "z_set_zero",
            "z_clear_fault": "z_clear_fault",
            "emergency_stop": "emergency_stop",
        }
        with redirect_stderr(StringIO()):
            for arguments, expected in cases.items():
                parsed = _parser().parse_args(arguments.split())
                self.assertEqual(parsed.action, expected)


if __name__ == "__main__":
    unittest.main()

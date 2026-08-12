# Python Socket 总控框架

本目录定义比赛总控程序的 Python 网络架构，供后续 AI 或开发者继续实现。

范围：Python 作为 TCP 客户端，同时管理多个设备连接；中心状态机按比赛任务顺序发送指令、等待确认并处理失败。

不包含任何特定硬件、固件或 GPIO 的实现细节。

## 设计原则

1. 用一个 asyncio 事件循环管理全部 TCP 连接，不要为每个 socket 创建阻塞线程。
2. 每个设备名称只对应一条长期 TCP 连接，并由 SocketPool 独占管理。
3. 只有状态机能决定比赛任务的下一状态；网络层只返回结果或抛出错误。
4. 每条影响任务流程的命令必须等待明确响应，并设置超时。
5. 网络错误、超时和意外响应必须进入明确的恢复或失败状态，不能静默忽略。

## 分层

    CompetitionStateMachine
            |  决定任务流转与错误策略
            v
        SocketPool
            |  按设备名提供连接、重连、收发和超时
            v
    DeviceConnection x N
            |  一台设备一条 TCP 连接
            v
        TCP devices

推荐的设备连接状态：

    DISCONNECTED -> CONNECTING -> READY
          ^              |          |
          +--- ERROR ----+----------+

## 文本协议约定

建议使用 UTF-8 文本的“每行一条命令”协议：

    客户端 -> 设备: COMMAND arg1 arg2\n
    设备 -> 客户端: OK detail\n
    设备 -> 客户端: ERR reason\n

- 发送后必须调用 writer.drain()。
- 接收使用 reader.readline()；设备响应必须以换行结束。
- 不要假设一次 recv() 恰好得到一条完整消息。TCP 是字节流，可能拆包或粘包。
- 状态机只接受与当前命令匹配的 OK 响应；ERR 和未知响应均视为失败。

如需二进制协议，使用“固定报文头 + 长度字段 + 载荷 + 校验”的分帧方式，并在 DeviceConnection 内实现；不要把分帧逻辑放入状态机。

## 核心接口

状态机只应依赖如下接口：

    await pool.connect_all()
    reply = await pool.command("device_name", "COMMAND args", timeout=2.0)
    await pool.close_all()

设备地址应放在配置中，业务状态机只能引用设备名：

    DEVICES = {
        "motion": ("192.168.1.10", 5000),
        "sensor": ("192.168.1.11", 5000),
        "actuator": ("192.168.1.12", 5000),
    }

## 可作为起点的实现

    import asyncio
    from dataclasses import dataclass
    from enum import Enum, auto


    class ConnectionState(Enum):
        DISCONNECTED = auto()
        CONNECTING = auto()
        READY = auto()
        ERROR = auto()


    @dataclass(frozen=True)
    class DeviceConfig:
        host: str
        port: int


    class DeviceConnection:
        def __init__(self, name: str, config: DeviceConfig):
            self.name = name
            self.config = config
            self.reader = None
            self.writer = None
            self.state = ConnectionState.DISCONNECTED
            self._lock = asyncio.Lock()

        async def connect(self, timeout: float = 3.0) -> None:
            if self.state is ConnectionState.READY:
                return

            self.state = ConnectionState.CONNECTING
            try:
                self.reader, self.writer = await asyncio.wait_for(
                    asyncio.open_connection(self.config.host, self.config.port),
                    timeout,
                )
                self.state = ConnectionState.READY
            except Exception:
                await self.close()
                self.state = ConnectionState.ERROR
                raise

        async def command(self, text: str, timeout: float = 2.0) -> str:
            # 对同一设备串行收发，保证命令与响应一一对应。
            async with self._lock:
                if self.state is not ConnectionState.READY:
                    await self.connect()

                self.writer.write((text + "\n").encode("utf-8"))
                await self.writer.drain()

                try:
                    data = await asyncio.wait_for(self.reader.readline(), timeout)
                except Exception:
                    await self.close()
                    self.state = ConnectionState.ERROR
                    raise

                if not data:
                    await self.close()
                    self.state = ConnectionState.ERROR
                    raise ConnectionError(f"{self.name}: peer closed connection")

                reply = data.decode("utf-8").strip()
                if reply.startswith("ERR"):
                    raise RuntimeError(f"{self.name}: {reply}")
                if not reply.startswith("OK"):
                    raise RuntimeError(f"{self.name}: unexpected reply: {reply}")
                return reply

        async def close(self) -> None:
            if self.writer is not None:
                self.writer.close()
                await self.writer.wait_closed()
            self.reader = None
            self.writer = None
            self.state = ConnectionState.DISCONNECTED


    class SocketPool:
        def __init__(self, devices: dict[str, DeviceConfig]):
            self.devices = {
                name: DeviceConnection(name, config)
                for name, config in devices.items()
            }

        async def connect_all(self) -> None:
            await asyncio.gather(
                *(connection.connect() for connection in self.devices.values())
            )

        async def command(
            self, device_name: str, text: str, timeout: float = 2.0
        ) -> str:
            return await self.devices[device_name].command(text, timeout)

        async def close_all(self) -> None:
            await asyncio.gather(
                *(connection.close() for connection in self.devices.values()),
                return_exceptions=True,
            )

## 状态机规则

比赛状态应表达为独立枚举状态，例如：

    INIT -> PREPARE -> EXECUTE -> VERIFY -> FINISHED
                                 |
                                 +-> RECOVER -> FAILED

状态机的每个分支遵循固定流程：

1. 向指定设备发送命令。
2. 校验回应是否精确符合预期。
3. 仅成功时进入下一个比赛状态。
4. 超时、断线、ERR 或未知回应时，进入 RECOVER 或 FAILED。

伪代码：

    if state is PREPARE:
        reply = await pool.command("motion", "PREPARE")
        if reply == "OK PREPARED":
            state = EXECUTE
        else:
            state = FAILED

状态机不得在多个协程中同时推进。并行设备动作可以由一个状态统一启动和汇总，之后再由该状态机单点作出状态转换。

## 重连与重试

- DeviceConnection 在断线或超时后关闭旧连接并标记 ERROR。
- 状态机决定是否重试、重试次数和回退步骤；网络层不允许无限重试。
- 重试前要判断命令是否幂等。查询和停止命令通常适合重试；一次性动作需要请求 ID 或先查询设备状态。
- 每个比赛状态都应有总时间预算，防止一个设备永久阻塞整个流程。

## 后续扩展

- 统一日志：时间、比赛状态、设备名、命令、响应、耗时和异常。
- 将地址、超时和重试次数放入独立配置文件。
- 心跳或状态轮询，用于提前发现不可用设备。
- 每台设备的命令白名单，避免状态机发送错误命令。
- 使用模拟 TCP Server 做单元测试，在没有实体设备时验证状态转移。

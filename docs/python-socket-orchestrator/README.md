# TestH743 Python JSONL 客户端

`h743_client.py` 是 STM32H743 龙门架 TCP Server 的 asyncio 参考客户端。它在连接时先校验 hello，
后台持续接收 heartbeat、ACK 和最终 event，并按 `command_id` 分发，因此消息交错或 TCP 拆包/粘包
不会破坏命令对应关系。

中心发出的命令使用 `device_id=center`，固件上报使用 `device_id=gantry`。默认地址为 `192.168.10.111:5000`。

## 命令行使用

```powershell
python h743_client.py hello
python h743_client.py payload held
python h743_client.py mission red_pick
python h743_client.py mission-cancel
python h743_client.py axis-move x 512000
python h743_client.py axis-stop z
python h743_client.py axis-status
python h743_client.py status
python h743_client.py config red_pick
python h743_client.py z-zero
python h743_client.py z-clear
python h743_client.py emergency-stop
```

路由器配置或自定义地址：

```powershell
python h743_client.py --host 192.168.10.111 hello
```

`emergency-stop` 会触发固件的全局 ABORT 锁存，设备重启前不能恢复运动，不要将它当作普通取消使用。

## 代码调用

```python
connection = DeviceConnection("gantry", DeviceConfig("192.168.10.111", 5000))
controller = H743Controller(connection)
try:
    result = await controller.run_mission("red_pick", timeout=660.0)
finally:
    await connection.close()
```

`z-zero` 不会自动寻找机械零点。只能在操作员确认 Z 轴已处于真实机械零位后使用；错误设零会让后续软限位和绝对位置失去安全依据。`z-clear` 只清除 Z 轴故障，不清除全局 ABORT。

`command()` 只等待 accepted ACK；`wait_event()` 等待真实终态；`run_command()` 依次等待两者。
上层比赛状态机只应在 `run_command()` 返回 completed 后推进。

## 模拟测试

```powershell
Set-Location docs/python-socket-orchestrator
python -m unittest -v test_h743_client.py
```

测试覆盖拆分 hello、同一写入中的 ACK/heartbeat/event 多帧、拒绝 ACK、hello 首包约束，以及重复
`command_id` 不重复执行。真实网线恢复、2 秒周期和运动安全必须在板上验证。完整协议见
`../socket控制协议.md`。

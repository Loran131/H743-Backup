# TestH743 龙门架 TCP JSONL 协议 V1

更新时间：2026-08-21

## 1. 连接和身份

- STM32H743 使用 LwIP BSD Socket API，作为 TCP Server 监听 `0.0.0.0:5000`。
- Python 中心端是 TCP Client；固件一次服务一个中心端，断线后关闭旧 socket 并重新 `accept`。
- 编码和分帧为 UTF-8 JSON + `\n`，一行只能有一个 JSON 对象。
- 中心端发出的消息固定使用 `device_id="center"`；H743 发出的消息固定使用 `device_id="gantry"`。

每条消息都有以下信封字段，字符串不能为空：

```json
{"type":"...","device_id":"center","msg_id":"...","timestamp":"...","payload":{}}
```

固件没有 RTC 时，发送的时间戳使用 `uptime_ms:<毫秒>`。`msg_id` 由本次运行时间和自增序号组成。

## 2. 首包和心跳

连接建立后，固件第一条消息一定是 hello，之前不会发送 heartbeat：

```json
{"type":"hello","device_id":"gantry","msg_id":"boot-...","timestamp":"uptime_ms:...","payload":{"protocol_version":1,"device_type":"gantry","capabilities":["mission_start","mission_cancel","axis_move","axis_stop","axis_status","emergency_stop","payload_set","status_query","config_query","z_set_zero","z_clear_fault"],"tasks":["red_pick","red_find","tag_put","frame_put"],"axes":["x","y","z"]}}
```

hello 后立即发送完整 `status`，以后每 1 秒发送一次；固件每 2 秒发送一次心跳：

```json
{"type":"heartbeat","device_id":"gantry","msg_id":"boot-...","timestamp":"uptime_ms:...","payload":{"status":"online","uptime_ms":1234}}
```

中心端心跳超时建议为 7 秒。中心端发来的合法 heartbeat 只更新通信活动，不返回 ACK。

## 3. 命令

中心端命令格式：

```json
{"type":"command","device_id":"center","msg_id":"center-1","timestamp":"2026-08-21T13:00:00Z","payload":{"command_id":"mission-42","name":"mission_start","target":{"task":"red_pick"}}}
```

`command_id` 最长 47 字符，只允许字母、数字、`-_.:`。支持的命令如下：

| name | target | 实际接入 |
|---|---|---|
| `mission_start` | `{"task":"red_pick|red_find|tag_put|frame_put"}` | 启动对应 P7 任务 |
| `mission_cancel` | `{}` | `MissionTask_RequestCancel()`，不锁存 |
| `axis_move` | `{"axis":"x|y|z","delta_pulses":整数}` | 受 owner、单步限幅和软限位保护的相对运动 |
| `axis_stop` | `{"axis":"x|y|z"}` | 指定轴局部停止，不产生全局锁存 |
| `axis_status` | `{}` 或 `{"axis":"x|y|z"}` | 发送完整状态快照 |
| `emergency_stop` | `{}` | `MotionCoordinator_RequestAbort()`，全局锁存到重启 |
| `payload_set` | `{"state":"empty"}` 或 `{"state":"held"}` | `MissionTask_SetPayload()` |
| `status_query` | `{}` | 发送完整状态快照 |
| `config_query` | `{}` 或 `{"task":"..."}` | 发送任务配置状态 |
| `z_set_zero` | `{}` | 人工确认机械零位后，将 Z 轴当前位置设为 0 |
| `z_clear_fault` | `{}` | 启动 Z 轴故障清除并等待恢复终态 |

单次相对运动上限为 X `512000`、Y `10000`、Z `57600` 脉冲。`z_set_zero` 不会寻找机械原点，
只能在操作员确认 Z 轴已位于真实机械零位后发送。网络层不实现运动算法，也不绕过原有所有权、
限位、超时和急停逻辑；`z_clear_fault` 不清除全局 ABORT。

## 4. ACK 和最终事件

命令成功进入业务队列后立即返回 accepted；它不代表动作完成：

```json
{"type":"ack","device_id":"gantry","msg_id":"boot-...","timestamp":"uptime_ms:...","payload":{"command_id":"mission-42","status":"accepted"}}
```

参数错误、任务忙、队列满或不支持时返回 rejected：

```json
{"type":"ack","device_id":"gantry","msg_id":"boot-...","timestamp":"uptime_ms:...","payload":{"command_id":"mission-42","status":"rejected","reason":"BUSY"}}
```

业务任务观察到真实终态后才发送 event：

任务遇到可恢复故障时，固件会保持同一个 `command_id` 为运行态，回到该任务预设位置并重新开始；
该过程不会重复发送 accepted，也不会提前发送 failed。最多任务级重启 10 次，并受 600 秒任务总时限约束。

```json
{"type":"event","device_id":"gantry","msg_id":"boot-...","timestamp":"uptime_ms:...","payload":{"command_id":"mission-42","status":"completed"}}
```

```json
{"type":"event","device_id":"gantry","msg_id":"boot-...","timestamp":"uptime_ms:...","payload":{"command_id":"mission-42","status":"failed","reason":"AXIS_Z:TIMEOUT","detail":3}}
```

断线时未发送成功的最终 event 保留到下一次连接。网络错误消息使用 `type="error"`，例如
`invalid_json`、`missing_required_field`、`device_id_mismatch`、`line_too_long`。

## 5. 去重和并发

- 固件按 `command_id` 保留最近 32 条记录，包括原 ACK、运行状态和最终结果。
- 重复 ID 不会再次执行；运行中重发原 ACK，终态时重发 ACK 和 event。
- 同一 ID 改用另一命令，或改变 `payload_set` 状态，返回 `command_id_conflict`。
- SocketTask 只做拆包、解析和发送。普通业务命令排队后由 `LegacyIoTask` 在已有 CAN mutex 下执行。
- 完整任务运行期间 SocketTask 独立工作，所以心跳和接收不会被运动流程阻塞。
- 所有 socket `send` 只发生在 SocketTask，防止 JSON 行交叉。

## 6. 网络配置

唯一切换点是 `App/Inc/network_config.h` 中的 `NETWORK_ACTIVE_PROFILE`：

| 配置 | STM32 地址 | 掩码 | 默认网关 |
|---|---|---|---|
| `NETWORK_PROFILE_DIRECT_PC` | `192.168.137.10` | `255.255.255.0` | `0.0.0.0` |
| `NETWORK_PROFILE_ROUTER` | `192.168.10.111` | `255.255.255.0` | `0.0.0.0` |

路由器模式若需要跨网段，按现场路由器地址覆盖 `NETWORK_GATEWAY_0..3`，不要假设网关一定是
`192.168.10.1`。电脑直连时给电脑网口配置 `192.168.137.0/24` 内除 `.10` 外的静态地址。

## 7. 构建、烧录和验收

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

产物为 `build/Debug/TestH743.elf` 和 `build/Debug/TestH743.bin`。使用本机 STM32CubeProgrammer
通过 ST-LINK/SWD 烧录；烧录前确认目标为 STM32H743，避免执行整片擦除之外的非预期操作。

电脑端快速检查：

```powershell
ping 192.168.10.111
python docs/python-socket-orchestrator/h743_client.py hello
python docs/python-socket-orchestrator/h743_client.py payload held
python docs/python-socket-orchestrator/h743_client.py mission red_pick
```

板测还必须覆盖：hello 首包、2 秒心跳、半包、粘包、非法/超长 JSON、重复 ID、动作期间心跳、
中心端重启、拔插网线、两套 IP，以及所有任务的真实成功/限位/超时/急停结果。编译和 Python
模拟测试不能替代这些板级安全测试。

## 8. 资源说明

- SocketTask 栈为 4096 字节，并启用 FreeRTOS 栈溢出检查；收发和 JSON 工作缓冲约 1.8 KiB。
- 32 条去重记录、解析 token 和协议静态状态约增加 2.4 KiB BSS。
- 命令队列深度为 4；使用固定数组和固定缓冲，不做随消息增长的动态分配。
- `configTOTAL_HEAP_SIZE` 为 24576 字节。烧录后应观察 SocketTask 栈高水位和剩余堆，再决定是否调栈。

## 9. 中心端接入要求

中心端需支持 JSONL 信封、连接后校验 hello、忽略/记录 heartbeat、按 `command_id` 匹配 ACK 和 event，
并允许三类消息交错到达。状态机只能以 `event.status=completed` 作为动作成功；accepted、超时、断线、
failed 和未知响应都不能推进成功流程。示例 asyncio 实现见 `python-socket-orchestrator/h743_client.py`。

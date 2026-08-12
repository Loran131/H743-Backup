# FDCAN1 与 PD42S1 调试总结

更新时间：2026-08-06

## 1. 当前配置

| 项目 | 当前值 |
|---|---|
| MCU | STM32H743IIT6 |
| CAN 外设 | FDCAN1 |
| FDCAN1_RX | PA11，AF9 |
| FDCAN1_TX | PA12，AF9 |
| UART4 | 未初始化，避免占用 PA11/PA12 |
| 工作模式 | Normal |
| 内部回环开关 | `CAN_INTERNAL_LOOPBACK_TEST = 0U` |
| 帧格式 | Classic CAN，29 位扩展 ID |
| 默认 CAN ID | `0x00001000` |
| 波特率 | 125 kbit/s |
| FDCAN 内核时钟 | 120 MHz |
| 位时序 | Prescaler=48，TSEG1=16，TSEG2=3，SJW=1 |
| RX | RX FIFO0，4 个元素，每个 8 字节 |
| TX | TX FIFO，4 个元素，每个 8 字节 |

## 2. 已解决问题

### 2.1 FDCAN2 已迁移到 FDCAN1

- 原来的 FDCAN2 PB12/PB13 已改为 FDCAN1 PA11/PA12。
- IRQ 已改为 `FDCAN1_IT0_IRQHandler()`。
- 主程序调用 `MX_FDCAN1_Init()`。
- UART4 初始化已停用，运行时没有软件重新配置 PA11/PA12。
- 工程仍由 Keil 编译，没有更新 CMake。

### 2.2 FDCAN1 内部回环验证通过

内部回环模式下已经确认：

- FDCAN1 内核能够工作。
- Message RAM、TX FIFO、RX FIFO 正常。
- 29 位扩展 ID `0x1000` 正常。
- RX 中断/轮询接收链路能够收到回环帧。
- 长命令可以按 8 字节加剩余字节分帧，并重新拼接。

回环统计曾显示：

```text
[CAN] rx: count=3 last_id=0x00001000 type=EXT dlc=0
```

其中 3 帧正好对应一条 14 字节相对位置命令的两帧，加一条版本查询帧。

### 2.3 找到并修复 FDCAN DLC 根因

原代码错误地使用：

```c
g_tx_header.DataLength = len << 16;
```

当前 STM32H7 HAL 要求 `DataLength` 填写未移位 DLC，HAL 在写入 Message RAM 时会自行左移。因此原代码在总线上实际发送的是 DLC=0 的空数据帧。

现已修正为：

```c
g_tx_header.DataLength = len;
```

RX 侧也已改为直接读取未移位的 `rx_header.DataLength`。内部回环确认修正后 RX 长度和数据均正确。

这是此前“CAN 状态看似正常，但电机完全收不到命令”的一个确定软件根因。

### 2.4 PD42S1 命令帧已经核对

版本查询：

```text
C5 01 20 E6 5C
```

相对位置命令：

```text
pos_rel 1 0 50 300 51200
```

完整协议数据为：

```text
C5 01 F3 00 32 01 2C 00 00 C8 00 00 E0 5C
```

实际 CAN 分帧为：

```text
帧 1，DLC=8：C5 01 F3 00 32 01 2C 00
帧 2，DLC=6：00 C8 00 00 E0 5C
```

`F2/F3` 命令缺少的协议保留字节已经补齐，命令总长度为 14 字节。

### 2.5 CAN 诊断能力已经补充

`cans` 当前可以显示：

- Bus-Off、Error Passive、Error Warning。
- TEC、REC、TX FIFO 空闲数量。
- RX 数量、最后 ID、ID 类型和 DLC。
- 恢复次数和发送入队失败次数。
- FDCAN 协议活动状态和最后错误码。
- PA11 实际输入电平。
- `TXFQS`、`TXBRP`、`TXBTO`、`TXBCF` 等 TX 寄存器。

## 3. 尚未解决的问题

### 3.1 H743 连接当前 CAN 模块时，PA11 一直为低

正常模式下捕获到：

```text
[CAN] busoff=0 passive=0 warning=0 TEC=0 REC=0 txfree=3
[CAN] proto: activity=0x00 LEC=7 DLEC=7 PA11_RX=0
[CAN] tx: TXBRP=0x00000001 TXBTO=0x00000000 TXBCF=0x00000000
```

含义：

- `activity=SYNC`：FDCAN 正在等待总线同步。
- `PA11_RX=0`：控制器一直看到 dominant 电平。
- `TXBRP=1`：发送请求仍在等待。
- `TXBTO=0`：帧没有发送完成。
- `TEC=0`：帧还没有真正开始发送，因此不是单纯的“无人 ACK”。

结论：当前直接阻止发送的是 PA11/FDCAN_RX 持续为低，FDCAN 无法观察到 11 个连续 recessive 位并进入总线空闲状态。

### 3.2 断开电机 CANH/CANL 后，模块 CANRX 仍约为 0.35 V

已经做过以下测试：

- 断开电机侧 CANH/CANL。
- 模块 `CANRX` 仍为约 0.35 V。
- `cans` 仍显示 `PA11_RX=0`。
- PA11 临时接到模块高电平端时可以读到 1，说明 PA11 GPIO 输入本身正常。
- PA12/FDCAN_TX 空闲时为高电平。

这表明低电平来自 CAN 模块的 RX 输出、模块供电/模式、模块引脚定义或板级连接，而不是 FDCAN1 软件配置。

### 3.3 F103 正常方案测得约 4.9 V，来源尚不明确

使用能够正常控制电机的 F103 和 `300project` 时，在连接 CANH/CANL 的情况下，测得 CAN 收发器 RX/TX 约为 4.9 V，但模块标注的 VIN 为 3.3 V。

该现象尚未解释。若测量点确实是逻辑 RXD/TXD，并且以模块 GND 为参考，则 3.3 V 供电器件的逻辑脚不应正常输出 4.9 V。可能原因包括：

- 测量参考地不一致或隔离两侧地混用。
- 测量点不是实际逻辑 RXD/TXD。
- 模块内部还有 5 V 电源、升压或隔离电源。
- 电机侧通过错误接线或损坏器件产生回灌。
- 模块引脚标注方向理解错误。
- F103 板上存在额外收发器、电平转换或上拉电路。

在查明来源前，不应将约 4.9 V 的信号直接接入 H743 PA11。

### 3.4 TX FIFO 填满是次生现象

连续发送两次 `pos_rel` 会产生 4 个物理 CAN 帧，正好填满 4 槽 TX FIFO：

```text
txfree=0
```

由于 PA11 一直为低，这些帧无法开始发送，所以 TX FIFO 不会释放。该现象不是命令分帧长度错误，也不是 RX FIFO 数量不足，而是总线始终无法进入 recessive 空闲状态的结果。

## 4. 已排除方向

- 不是 FDCAN1 内核损坏：内部回环正常。
- 不是 Message RAM、TX FIFO 或 RX FIFO 基本配置错误：回环可正确收发。
- 不是 DLC 软件编码问题：已经修复并通过回环验证。
- 不是 PA11/PA12仍被 UART4重新初始化：UART4没有调用。
- 不是 PD42S1相对位置命令缺少保留字节：已经补齐。
- 不是普通 Bus-Off 自锁：当前为 SYNC，TEC=0，Bus-Off=0。
- 不是接收过滤器导致无法发送：发送甚至还没有开始，且过滤器已放宽。

## 5. 下一步排查顺序

### 5.1 单独验证模块逻辑侧

断电后将模块与 H743 和电机全部分开，然后：

1. 只给模块正确供电。
2. CANH/CANL 保持断开。
3. 保证模块 TXD/CANTX 为 recessive 高电平。
4. 以模块 GND 为唯一参考，测量 VIN、TXD、RXD。

普通 CAN 收发器在该条件下应满足：

```text
TXD：高
RXD：高
```

如果 RXD 仍约为 0.35 V，应检查模块的芯片型号、EN/STB/S 引脚、隔离侧供电和模块是否损坏。

### 5.2 核对模块型号和完整引脚定义

需要获得：

- 模块正反面照片。
- 主芯片和隔离芯片丝印。
- VIN、GND、CANRX、CANTX、CANH、CANL 的厂家定义。
- 模块是否为普通收发器、隔离 CAN 模块或 CAN-to-CAN 中继模块。
- 是否存在两侧独立供电和两组 GND。

不能仅根据 `CANRX/CANTX` 名称假定方向。

### 5.3 统一参考地重新测量 F103 正常系统

万用表黑表笔始终接同一个模块 GND，分别测量：

```text
VIN-GND
RXD-GND
TXD-GND
CANH-GND
CANL-GND
```

同时测量 F103 GND、模块 GND和电机 GND之间的电位差。

正常 CAN 总线空闲时通常为：

```text
CANH ≈ 2.5 V
CANL ≈ 2.5 V
RXD  = recessive 高电平
```

### 5.4 检查总线接线和终端

断电后测量 CANH 与 CANL 之间电阻：

- 约 120 ohm：一个终端电阻。
- 约 60 ohm：两个 120 ohm 终端并联。
- 接近 0 ohm：疑似短路或接线错误。

检查 CANH、CANL、GND 是否与电机端定义一致。

### 5.5 模块确认正常后再做 H743 实总线测试

复位 H743 后只发送一条单帧命令，避免立即填满 TX FIFO：

```text
ver 1
cans
```

预期空闲状态首先应满足：

```text
PA11_RX=1
activity=IDLE
```

发送成功并获得 ACK 时应看到 TX 请求完成，`TXBTO` 出现对应位，TX FIFO 恢复空闲。收到电机应答后 `rx count` 应增加。

## 6. 当前代码位置

- 回环开关和 CAN 位时序：`Core/Inc/fdcan.h`
- FDCAN1 初始化、收发、恢复和诊断：`Core/Src/fdcan.c`
- FDCAN1 IRQ：`Core/Src/stm32h7xx_it.c`
- PD42S1 协议和命令组帧：`Core/Src/smd.c`
- 串口命令和 `cans`：`Core/Src/shell.c`

## 7. 当前总判断

软件侧已经确认 FDCAN1 能够生成并回环接收正确的 PD42S1 CAN 帧，最关键的 DLC=0 软件错误也已经修复。

当前剩余阻塞点位于真实物理接口：H743 的 PA11 接入现有 CAN 模块后持续为低，FDCAN 因此停留在同步状态，发送请求无法上总线。下一步重点不是继续修改 CAN 协议或 FIFO，而是确认 CAN 模块的型号、供电、方向、逻辑电平、隔离地和 RXD 低电平来源。

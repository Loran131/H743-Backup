# STM32H743 下载后无法运行问题总结

## 1. 问题现象

本次调试过程中依次出现了以下现象：

1. OpenOCD 能识别 ST-Link 和 STM32H743，但下载时提示：

   ```text
   Error: timed out while waiting for target halted
   Error: ** Unable to reset target **
   ```

2. 修改复位配置后，固件能够成功烧写，但 CPU 停在 `Reset_Handler`，程序没有继续运行。

3. 程序复位运行后，USART1 没有启动提示、回显或心跳输出。

## 2. 原因一：OpenOCD 使用了错误的硬件复位方式

CLion 原来使用：

```text
board/stm32h7x3i_eval.cfg
```

该配置面向 STM32H743I-EVAL 评估板，并包含：

```tcl
reset_config srst_only
```

这会强制 OpenOCD 使用 ST-Link 的硬件 `NRST` 信号复位目标芯片。实际板卡的 NRST 路径未能被 ST-Link 正确控制，因此 SWD 可以识别芯片，但执行 `reset halt` 时超时。

### 定位证据

- ST-Link、目标电压和 SWD DPIDR 均识别正常。
- 硬件复位方式执行 `reset halt` 失败。
- 改用 Cortex-M `SYSRESETREQ` 软件复位后，目标能够立即暂停。

### 解决方法

工程新增了 [openocd.cfg](openocd.cfg)，核心配置如下：

```tcl
source [find interface/stlink.cfg]
transport select swd
source [find target/stm32h7x_dual_bank.cfg]

reset_config none
adapter speed 1000
```

CLion 的 OpenOCD 配置已改为使用工程内的 `openocd.cfg`。

## 3. 原因二：下载后 CPU 仍处于暂停状态

固件烧写成功时，OpenOCD 输出了：

```text
** Programming Finished **
```

但没有执行复位运行。在线读取到的 PC 为 `0x080085b8`，经 ELF 符号解析对应 `Reset_Handler`，说明这不是 HardFault，而是 CPU 停在复位入口等待调试器继续执行。

### 解决方法

将 CLion 配置中的：

```xml
reset-type="INIT"
```

改为：

```xml
reset-type="RUN"
```

也可以在 OpenOCD 中显式执行：

```tcl
reset run
```

## 4. 原因三：Ethernet 初始化失败，程序进入 Error_Handler

解决复位问题后，程序仍然没有串口输出。在线暂停读取到：

```text
PC = 0x08007040
```

该地址位于 `Error_Handler`。在 `Error_Handler` 入口设置硬件断点并读取 LR，得到调用位置：

```text
MX_ETH_Init
Core/Src/eth.c:85
```

失败调用为：

```c
if (HAL_ETH_Init(&heth) != HAL_OK)
{
  Error_Handler();
}
```

STM32H7 的 RMII Ethernet 初始化需要有效的外部 PHY 和 50 MHz `ETH_REF_CLK`。当前硬件没有提供有效的 RMII 参考时钟，导致 `HAL_ETH_Init()` 失败。由于 Ethernet 初始化发生在串口启动提示之前，程序没有机会执行 `printf()` 和主循环。

### 解决方法

当前功能不使用 Ethernet，因此在 `MX_ETH_Init()` 的 CubeMX 用户代码区直接返回，跳过 Ethernet 初始化：

```c
/* USER CODE BEGIN ETH_Init 0 */
/* The board has no active RMII reference clock; keep unused Ethernet off. */
return;
/* USER CODE END ETH_Init 0 */
```

若后续需要 Ethernet，应恢复初始化，并检查 PHY 供电、复位、RMII 引脚以及 PA1 上的 50 MHz REF_CLK。

## 5. 原因四：ST-Link V2 不等于 USB 串口

修复程序运行问题后，在线采样 PC 已落在 `HAL_UART_Receive()` 中，证明主循环正在正常执行。但 Windows 没有识别到任何 COM 设备。

普通 ST-Link V2 只提供 SWD 下载调试功能，不提供虚拟串口。USART1 数据从 MCU 的 PA9/PA10 输出，不能直接在 OpenOCD 控制台中看到。

### USART1 连接方式

使用 3.3V USB-TTL 模块交叉连接：

| USB-TTL | STM32H743 |
|---|---|
| RX | PA9 / USART1_TX |
| TX | PA10 / USART1_RX |
| GND | GND |

串口参数：

```text
115200 baud
8 data bits
1 stop bit
no parity
no flow control
```

不要向 MCU 串口引脚接入 5V 电平。

## 6. 最终验证结果

- OpenOCD 能通过软件复位稳定连接并暂停目标。
- 固件烧写完成且 Flash 校验结果为 `Verified OK`。
- 程序不再进入 `Error_Handler`。
- 在线 PC 采样位于 `HAL_UART_Receive()`，确认主循环正在运行。
- USART1 会发送以下 ASCII 文本：

  ```text
  USART1 ready
  heartbeat
  heartbeat
  ```

- USART1 接收到的字节会立即原样回显。

## 7. 后续排查顺序

遇到“烧写成功但程序无反应”时，建议按以下顺序检查：

1. 确认日志中是否有 `Programming Finished` 和 `Verified OK`。
2. 确认烧写后是否执行了 `reset run`，而不是停在 `Reset_Handler`。
3. 在线暂停读取 PC，并使用 `arm-none-eabi-addr2line` 解析地址。
4. 若停在 `Error_Handler`，在入口设置断点并读取 LR，定位失败的 HAL 初始化函数。
5. 暂时关闭未连接硬件的外设初始化，例如 Ethernet、外部存储器或传感器。
6. 确认串口工具、COM 设备、TX/RX 交叉连接、共地和电平标准。


# STM32H7 以太网与 LwIP：新项目配置指南（供 AI 执行）

本文定义一个 AI 面对全新的 STM32H7 + 外置 PHY 项目时，配置 ETH 和 LwIP 的标准流程。先根据原理图确认硬件，再在 CubeMX 配置，最后检查内存、缓存和链路；不要直接套用其他项目的 PHY 地址、引脚或 RAM 地址。

## 0. 先收集不可假设的硬件信息

在改代码或 CubeMX 配置前，读取原理图、PCB 说明和芯片手册，确认下列项目：

| 项目 | 必须确认的内容 |
| --- | --- |
| MCU | 具体 STM32H7 型号、Flash/RAM 分区与可供 ETH DMA 访问的 RAM |
| PHY | 型号、MDIO/MDC 管脚、PHY 地址绑带、复位脚极性、RMII/MII 接口 |
| 时钟 | PHY 的 50 MHz RMII REF_CLK 来源与方向（MCU 输出或 PHY 输出） |
| 数据线 | TXD0/1、TX_EN、RXD0/1、CRS_DV、MDC、MDIO 的实际 MCU 引脚 |
| 复位 | PHY RESET 是否低有效、上电后需要保持和释放的时间 |
| 网络 | DHCP 或静态 IP；若静态，开发板与电脑须在同一子网 |

常见 PHY（如 LAN8720、LAN8742、DP83848）不能仅凭名称互换：PHY 地址、寄存器定义和复位/时钟要求可能不同。

## 1. CubeMX：配置时钟、ETH 与引脚

1. 在 **System Core > RCC** 与 **Clock Configuration** 中把系统时钟配置到目标频率。
2. 在 **Connectivity > ETH** 启用 Ethernet，接口模式选择与硬件一致的 **RMII** 或 **MII**。现代小型板卡通常是 RMII。
3. 核对 CubeMX 自动分配的每一个 ETH 引脚与原理图完全一致。特别检查 `ETH_REF_CLK`、`ETH_MDIO`、`ETH_MDC`。
4. 配置 PHY 的复位 GPIO：通常为推挽输出；若 PHY 的 RESET 为低有效，上电默认输出应为低电平。
5. 不要手动启用与 ETH 冲突的外设、调试接口或 GPIO 复用。

RMII 的典型信号为：`REF_CLK`、`MDIO`、`MDC`、`CRS_DV`、`RXD0`、`RXD1`、`TX_EN`、`TXD0`、`TXD1`。实际管脚以芯片封装、原理图和 CubeMX pinout 为准。

## 2. CubeMX：启用 LwIP

1. 在 **Middleware and Software Packs > LwIP** 启用 LwIP。
2. 无 RTOS 项目选择轮询模式；使用 FreeRTOS 时按生成器提供的线程模式配置。
3. 初次联调建议关闭 DHCP，设定静态 IPv4，例如：

   | 设备 | IP | 子网掩码 |
   | --- | --- | --- |
   | 开发板 | `192.168.137.10` | `255.255.255.0` |
   | 电脑有线网卡 | `192.168.137.1` | `255.255.255.0` |

   网关在点对点测试时可留空；若接入路由器或需要访问外网，再填写实际网关和 DNS。
4. 选择与 PHY 匹配的驱动。若 CubeMX 没有该 PHY 的专用驱动，使用通用 PHY 接口前，必须实现/核对其 PHY 地址、链路状态读取和速度/双工配置。
5. 生成代码前保存 `.ioc`，并将项目纳入版本控制。

## 3. PHY 复位时序

将复位控制代码放入 CubeMX 生成文件的 `USER CODE` 区域，以避免重新生成时丢失。对于低有效 RESET，通用顺序如下：

```c
/* 在 MX_LWIP_Init() 中、ETH 初始化之前执行；时间以 PHY 手册为准。 */
HAL_Delay(300U);  /* RESET 已由 GPIO 初始状态保持为低 */
HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_SET);
HAL_Delay(1000U); /* 等待 PHY 振荡器、寄存器与自动协商准备完成 */
```

若 RESET 为高有效，电平逻辑反转。上述 300/1000 ms 是保守联调值，不应取代 PHY 手册的最小时间要求。

## 4. 主程序中启动与轮询 LwIP

在 `main()` 完成 GPIO、时钟和其他基础外设初始化后调用：

```c
MX_LWIP_Init();
```

裸机（无 RTOS）项目必须在无限循环中持续调用：

```c
while (1)
{
  MX_LWIP_Process();
  HAL_Delay(1U);
}
```

不要同时调用另一套旧的 `MX_ETH_Init()`，也不要重复创建 ETH 句柄；LwIP 生成的 `ethernetif.c` 应是 ETH 初始化和收发的唯一所有者。

## 5. STM32H7 必做：ETH DMA 内存规划

ETH DMA 只能访问特定 RAM 区域。根据具体 H7 型号、链接脚本和 CubeMX 生成代码，检查以下对象的地址和范围：

- ETH 描述符（RX/TX descriptors）；
- ETH RX 缓冲区与 RX pool；
- LwIP heap；
- 任何自定义 DMA 缓冲区。

这些区域必须同时满足：

1. 位于 ETH DMA 可访问的 RAM；
2. 彼此不重叠；
3. 地址和对齐符合 HAL、链接脚本及缓存行要求；
4. 总大小未超出所在 RAM 区域。

若 `LWIP_RAM_HEAP_POINTER` 是固定地址，必须计算其起点与 heap 长度，并确认它没有覆盖 ETH RX pool。示例检查：

```c
/* 仅示意：地址必须按本项目的 map 文件和生成代码决定。 */
#define LWIP_RAM_HEAP_POINTER 0x30005000
```

不要把 `0x30005000` 当作所有 H7 项目的固定答案。它只在确认前方 ETH 内存池结束地址、该地址属于可用 DMA RAM 后才能使用。

## 6. STM32H7 必做：D-Cache 一致性

先明确工程是否调用了 `SCB_EnableDCache()`：

- **未开启 D-Cache**：不要无条件调用 `SCB_CleanDCache_by_Addr()` 或 `SCB_InvalidateDCache_by_Addr()`。
- **开启 D-Cache**：ETH DMA 的 RX/TX 缓冲须放在不可缓存 MPU 区域，或严格执行缓存维护。

若采用缓存维护，缓存操作范围必须按 Cortex-M7 的 **32 字节缓存行**对齐，且仅在 D-Cache 已开启时执行：

```c
if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
{
  uint32_t start = (uint32_t)buffer & ~31UL;
  uint32_t end = ((uint32_t)buffer + length + 31UL) & ~31UL;

  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}
```

将代码放在 `ethernetif.c` 对应的 `USER CODE` 区域。TX 一般需要在 DMA 读取数据前 Clean，RX 一般需要在 CPU 读取 DMA 写入的数据前 Invalidate；以所用 STM32 HAL/LwIP 生成代码的生命周期为准。

## 7. 首次验证顺序

按以下顺序排查，不能跳过前一项就直接调 Ping：

1. 编译并下载，确认不会进入 HardFault。
2. 用 MDIO 读取 PHY ID 寄存器 2、3；返回非全零、非全 `0xFFFF` 才表示 MDIO 通信正常。
3. 读取 BMSR 两次，确认链接状态与自动协商完成。无链路时优先检查网线、交换机、REF_CLK、PHY 复位与 PHY 地址。
4. 确认 LwIP netif 已 up 且 link up。
5. 电脑先清 ARP 缓存，再连续 Ping：

   ```powershell
   arp -d <开发板IP>
   ping -n 20 <开发板IP>
   ```

6. 若只偶尔回复或一发包就异常，优先复查 DMA 内存重叠与 D-Cache 一致性，不要先归因于网线。

## 8. 重新生成代码后必须复查

每次 CubeMX 重新生成后，AI 应检查：

- `.ioc` 中 ETH 接口、PHY 相关引脚、静态 IP/DHCP 配置未改变；
- PHY 复位时序仍在 `USER CODE` 区；
- `LWIP_RAM_HEAP_POINTER` 及链接脚本中的 DMA 内存布局仍无重叠；
- `ethernetif.c` 中缓存一致性代码仍保留且处于正确回调；
- 裸机主循环仍调用 `MX_LWIP_Process()`；
- 完整重新编译后，重新做 PHY ID、BMSR 和 20 次 Ping 验证。

## AI 执行原则

- 先从硬件和生成代码取得证据，再修改地址、引脚、PHY 地址或缓存策略。
- 所有手写修改放在 CubeMX `USER CODE` 标记之间，并记录修改原因。
- Ethernet 的随机丢包、ARP 不稳定和接收后 HardFault，优先检查 DMA 可访问性、内存重叠和缓存，而不是重复改变 IP。

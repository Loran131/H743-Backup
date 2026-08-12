# STM32H7：从空白工程到 LwIP Socket Echo（CubeMX + FreeRTOS）

本文是面向后续 AI 的执行指南：在一个全新的 STM32H7 + 外置 RMII PHY 工程中，如何建立稳定的 LwIP RTOS 网络，并完成第一个 TCP socket Echo 测试。

本文的已验证目标工程使用 STM32H743、CMSIS-RTOS v2、FreeRTOS、LwIP 2.1.2，以及静态 IPv4 `192.168.137.10/24`。其中 GPIO、PHY 型号/地址、内存地址和链路脚本布局均**不能**直接照抄到其他板子；每一步必须以原理图、CubeMX 配置和 map 文件为准。

## 0. 完成定义

最终应满足以下结果：

1. 固件能由 ST-LINK 正确烧录并运行，不进入 Fault；
2. PC 连续 Ping 板卡 IP 成功；
3. PC 连接板卡 TCP `5000` 端口并收到 Echo 回显；
4. Ethernet DMA 描述符、RX pool 与 LwIP heap 的内存范围不重叠；
5. 所有手工修改位于 CubeMX 的 `USER CODE` 区域，或已同步写入 `.ioc`。

## 1. 先获取硬件事实，禁止猜测

在 CubeMX 或源码修改前，读取原理图、PCB 说明和 PHY 数据手册，确认：

| 项目 | 必须确认的内容 |
| --- | --- |
| MCU | 具体 H7 型号、可供 ETH DMA 访问的 RAM 区域 |
| PHY | 型号、RMII/MII、MDIO 地址、RESET 极性、REF_CLK 来源 |
| 引脚 | `REF_CLK`、`MDC`、`MDIO`、`CRS_DV`、`RXD0/1`、`TX_EN`、`TXD0/1` |
| 时钟 | 50 MHz RMII 参考时钟由 MCU 还是 PHY 提供 |
| 网络 | 使用 DHCP 还是静态 IP；PC 与板卡是否在同一子网 |

若 PHY 没有专用 CubeMX 驱动，通用 PHY 驱动只能读取 IEEE 标准寄存器；仍需验证其 MDIO 地址扫描、链路状态和协商结果与实际 PHY 一致。

## 2. CubeMX 配置顺序

### 2.1 时钟、GPIO 与 ETH

1. 配置系统时钟到项目要求的频率。
2. 在 **Connectivity > ETH** 启用 ETH，接口选择实际硬件的 `RMII` 或 `MII`。
3. 将所有 ETH 引脚逐一与原理图核对。不要仅依赖 CubeMX 自动分配。
4. 配置 PHY RESET GPIO。低有效 RESET 的常见启动顺序为：初始保持低、延时、拉高、再等待 PHY 及协商稳定。
5. 配置并启用 `ETH_IRQn`，优先级设为 `5`，子优先级 `0`。

`ETH_IRQn` 是 RTOS LwIP 版必须项：ETH RX 完成回调会释放 FreeRTOS 信号量，因此中断优先级数值不得小于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`。本工程该值为 `5`。

### 2.2 FreeRTOS

1. 在 **Middleware and Software Packs > FreeRTOS** 启用 FreeRTOS。
2. Interface 选择 **CMSIS v2**。
3. 保持抢占式调度与 `1000 Hz` tick；内存方案使用 `heap_4`。
4. 启用 `configUSE_MALLOC_FAILED_HOOK` 和 `configCHECK_FOR_STACK_OVERFLOW = 2`，并实现两个 hook。
5. 创建：
   - `defaultTask`：`1024 Words`；由 CubeMX 任务入口调用 `MX_LWIP_Init()`。
   - `SocketTask`：`1024 Words`；运行 TCP Echo 服务。
6. 将 `configTOTAL_HEAP_SIZE` 设为至少 `24576 Bytes`。

不要把默认任务保留在 `128 Words`：它会执行 LwIP、ETH 和 PHY 初始化，512 B 很容易溢出。任务堆栈单位是 **Words**，而 CMSIS `osThreadAttr_t.stack_size` 单位是 **Bytes**。

### 2.3 LwIP

1. 在 **Middleware and Software Packs > LwIP** 启用 LwIP。
2. 选择 **with RTOS** 模式；生成结果必须满足：

   ```c
   #define WITH_RTOS 1
   /* opt.h 默认值最终应为： */
   #define NO_SYS       0
   #define LWIP_NETCONN 1
   #define LWIP_SOCKET  1
   ```

3. 初次联调建议关闭 DHCP，设置静态 IPv4。示例：

   | 设备 | IP | 掩码 |
   | --- | --- | --- |
   | 板卡 | `192.168.137.10` | `255.255.255.0` |
   | PC 有线网卡 | `192.168.137.1` | `255.255.255.0` |

4. 初次测试中网关可为 `0.0.0.0`。只有访问其他子网时才设置真实网关和 DNS。
5. 设置线程栈（单位均为 **Bytes**）：
   - `TCPIP_THREAD_STACKSIZE`：`1536`；
   - Ethernet input thread：`1024`；
   - Ethernet link thread：`1024`。

## 3. 第一次生成后必须核对的源码

### 3.1 初始化位置

RTOS 模式下，CubeMX 应在 `StartDefaultTask()` 中执行：

```c
MX_LWIP_Init();
```

此时 `tcpip_init()`、`tcpip_input`、Ethernet input 线程和 Ethernet link 线程由 LwIP/CubeMX 管理。

**不得**在 `main()` 的无限循环中继续调用裸机轮询接口：

```c
MX_LWIP_Process(); /* RTOS 模式下不存在，也不应调用 */
```

### 3.2 ETH 全局中断

若 CubeMX 未生成以下内容，必须补上，并将手工代码置于 `USER CODE` 区域：

```c
/* Core/Src/stm32h7xx_it.c */
#include "lwip.h"

void ETH_IRQHandler(void)
{
  HAL_ETH_IRQHandler(&heth);
}
```

并在 `HAL_ETH_MspInit()` 的 `USER CODE` 区域添加：

```c
HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(ETH_IRQn);
```

在 `HAL_ETH_MspDeInit()` 中对应关闭：

```c
HAL_NVIC_DisableIRQ(ETH_IRQn);
```

缺少这一组代码的典型现象是：链路可能看似建立，但板卡不回应 ARP，因此 Ping 和 TCP 连接全部超时。

### 3.3 PHY RESET 时序

将复位逻辑放进 `LWIP/App/lwip.c` 的 `USER CODE BEGIN IP_ADDRESSES` 区域，保证在 ETH 初始化前运行。例如低有效 RESET：

```c
HAL_Delay(300U);  /* GPIO 初始输出保持 RESET 有效 */
HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_SET);
HAL_Delay(1000U);
```

延时时间必须以 PHY 数据手册为准。重新生成后确认该代码仍在。

## 4. STM32H7 DMA 内存与 D-Cache

这一步优先级高于应用层 TCP。网络随机丢包、ARP 不稳定、首包后卡死，优先检查本节。

### 4.1 用 map 文件计算真实内存范围

编译后检查 map 文件中：

```powershell
Select-String -Path build\Debug\TestH743.map `
  -Pattern 'DMARxDscrTab|DMATxDscrTab|memp_memory_RX_POOL_base|ucHeap'
```

本工程实际测得：

| 对象 | 地址/范围 |
| --- | --- |
| RX descriptors | `0x30000000` 起 |
| TX descriptors | `0x30000080` 起 |
| RX pool | `0x30000100–0x30004A82` |

CubeMX 本次生成的 `LWIP_RAM_HEAP_POINTER = 0x30004000` 与 RX pool 重叠，会在网络负载下破坏内存。实际修复是将下列覆盖放在 `LWIP/Target/lwipopts.h` 的 `USER CODE BEGIN 1` 中：

```c
#undef LWIP_RAM_HEAP_POINTER
#define LWIP_RAM_HEAP_POINTER 0x30005000
```

`0x30005000` 只适用于本项目已验证的 RX pool 大小和链接脚本；换项目时必须重新计算。

### 4.2 D-Cache

1. 若项目未开启 D-Cache，不要无条件插入 Clean/Invalidate。
2. 若开启 D-Cache，ETH DMA buffer 要么位于 MPU 标记为 non-cacheable 的区域，要么在正确生命周期执行缓存维护。
3. 缓存操作必须按 Cortex-M7 的 32 B cache line 向下/向上对齐。
4. TX：DMA 读数据前 Clean；RX：CPU 读 DMA 写入的数据前 Invalidate。

本工程的 RX 回调保留了按 32 B 对齐的 Invalidate 代码，并且只在 D-Cache 已启用时执行。

## 5. SocketTask 与 LwIP 初始化同步

CubeMX 会同时创建 `defaultTask` 与 `SocketTask`。不能假设前者必然先完成 `MX_LWIP_Init()`；用 CMSIS v2 Event Flags 同步：

```c
static osEventFlagsId_t lwipReadyEventHandle;
#define LWIP_READY_FLAG (1U)

/* MX_FREERTOS_Init(): 创建任务前 */
lwipReadyEventHandle = osEventFlagsNew(NULL);
if (lwipReadyEventHandle == NULL) {
  Error_Handler();
}

/* StartDefaultTask(): MX_LWIP_Init() 成功返回后 */
(void)osEventFlagsSet(lwipReadyEventHandle, LWIP_READY_FLAG);

/* StartSocketTask(): */
uint32_t flags = osEventFlagsWait(lwipReadyEventHandle, LWIP_READY_FLAG,
                                  osFlagsWaitAny, osWaitForever);
if ((flags & osFlagsError) != 0U) {
  Error_Handler();
}
```

Event Flags、socket 包含文件、服务函数与同步代码均放入 `Core/Src/freertos.c` 对应的 `USER CODE` 块。

## 6. 最小 TCP Echo 服务

服务监听全部本地地址的 TCP `5000` 端口。使用 `lwip_*` 函数，避免和宿主 C 库的同名 socket 符号冲突。

```c
#include "lwip/sockets.h"
#include "lwip/inet.h"

static void TcpEchoServer(void)
{
  struct sockaddr_in address;
  int listen_fd;

  for (;;) {
    listen_fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
      osDelay(1000U);
      continue;
    }

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = PP_HTONS(5000U);
    address.sin_addr.s_addr = PP_HTONL(INADDR_ANY);

    if ((lwip_bind(listen_fd, (const struct sockaddr *)&address,
                   sizeof(address)) != 0) ||
        (lwip_listen(listen_fd, 2) != 0)) {
      (void)lwip_close(listen_fd);
      osDelay(1000U);
      continue;
    }

    for (;;) {
      int client_fd = lwip_accept(listen_fd, NULL, NULL);
      uint8_t buffer[256];
      int received;

      if (client_fd < 0) {
        break;
      }
      while ((received = lwip_recv(client_fd, buffer, sizeof(buffer), 0)) > 0) {
        /* 生产代码必须处理 lwip_send() 部分发送；测试代码也不应假设一次发完。 */
        if (lwip_send(client_fd, buffer, (size_t)received, 0) <= 0) {
          break;
        }
      }
      (void)lwip_close(client_fd);
    }
    (void)lwip_close(listen_fd);
  }
}
```

实际工程中的实现应循环发送直到全部字节送出，并在 `SocketTask` 入口调用该函数。

## 7. Hook 函数

当 CubeMX 中启用了 hook，禁止保留空实现。至少进入 `Error_Handler()`，避免静默继续执行：

```c
void vApplicationStackOverflowHook(xTaskHandle task, signed char *name)
{
  (void)task;
  (void)name;
  Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
  Error_Handler();
}
```

调试版本可额外通过 UART、LED 或调试器记录任务名、`xPortGetFreeHeapSize()` 和任务 high-water mark。

## 8. 构建、烧录与验收

### 8.1 构建

```powershell
cmake --build --preset Debug
```

应先修复所有网络相关编译错误。未使用的旧测试函数警告不阻止本次验证，但后续应清理。

### 8.2 ST-LINK 烧录

```powershell
& 'D:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD mode=UR freq=4000 reset=HWrst `
  -w 'D:\Code\stm32\TestH743\build\Debug\TestH743.elf' -v -rst
```

`mode=UR`（Under Reset）适合目标程序正在运行、普通 Hot Plug 连接偶发超时时的恢复场景。

### 8.3 PC 验证

先清除 ARP 并验证 ICMP：

```powershell
arp -d 192.168.137.10
ping -n 20 192.168.137.10
```

再验证 TCP Echo：

```powershell
python -c "import socket; s=socket.create_connection(('192.168.137.10',5000), timeout=5); s.sendall(b'hello stm32'); print(s.recv(256)); s.close()"
```

预期输出：

```text
b'hello stm32'
```

## 9. 故障定位顺序

| 现象 | 首先检查 |
| --- | --- |
| 不能连接 ST-LINK | 使用 `mode=UR reset=HWrst`；确认供电和 NRST |
| 进入 Fault | CFSR/HFSR、任务栈、FreeRTOS heap、非法中断优先级 |
| 无 ARP/Ping | PHY RESET、REF_CLK、MDIO/PHY ID、`ETH_IRQHandler`、ETH IRQ 优先级 |
| Ping 正常但 TCP 超时 | `SocketTask` 是否等待 LwIP ready、是否执行到 `lwip_listen()`、端口是否正确 |
| 随机丢包或首包后卡死 | RX pool/LwIP heap/DMA descriptors 是否重叠、D-Cache 一致性 |
| 仅大数据量失败 | TCP/IP、ETH input、SocketTask 栈，以及 pbuf/TCP 缓冲配置 |

## 10. 每次 CubeMX 重新生成后的复查清单

1. `.ioc` 中 ETH、PHY RESET、静态 IP、FreeRTOS 任务栈与 heap 设置未被改变；
2. `ETH_IRQn` 仍启用，优先级仍为 `5`；
3. `ETH_IRQHandler()` 仍调用 `HAL_ETH_IRQHandler(&heth)`；
4. `main()` 中没有恢复裸机 `MX_LWIP_Process()`；
5. PHY RESET、LwIP heap 覆盖、socket 服务和初始化同步代码仍在 `USER CODE` 区域；
6. 重新生成后重新查看 map，确认 DMA 内存没有重叠；
7. 完整构建、ST-LINK 烧录、20 次 Ping 和 TCP Echo 均通过。

若上述任一步缺少证据，不要直接把网络故障归因于 socket 代码。

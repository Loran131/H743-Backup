# TestH743 旧工程功能迁移状态

更新时间：2026-08-12

## 当前结果

`TestH743` 继续作为 FreeRTOS、LwIP 和 Ethernet 的主工程。旧工程的业务模块已放入
`App/Inc` 和 `App/Src`，没有复制旧工程的 `main.c`、GPIO、时钟、MPU、Ethernet 或
中断文件覆盖新工程。

已接入构建和运行：

- USART1 Shell，115200 8N1，中断接收；
- USART3 C552，460800 8N1，DMA1 Stream0 循环接收；
- FDCAN2，PB12/PB13，125 kbit/s；
- UART4，PA11/PA12，115200 8N1，保留给 Z 轴；
- I2C2 EEPROM，PH4/PH5；
- QSPI，PF6-PF9、PB2、PB6；
- 三个板载 LED；
- FreeRTOS 的 LegacyIoTask、ShellTask、MonitorTask；
- 依赖关键任务心跳的 IWDG；
- 原有 LwIP 和 TCP 5000 服务。
- X/Y 控制层：固定地址 1/2、脉冲坐标、软限位、非阻塞反馈、广播双轴停止；
- 人工触发的 X/Y 无限位回零和人工设零入口，启动及故障恢复均不会自动回零。

明确不进入当前构建：按键、蜂鸣器、TIM3 测试步进、旧 `axis/servo/stepper`。
历史源码仍保留在旧工程中。

## 关键约束

- FDCAN 使用 25 MHz HSE 作为内核时钟，nominal prescaler 为 10，
  与 TSEG1=16、TSEG2=3 组合得到 125 kbit/s；CPU 仍保持 480 MHz。
- USART3 DMA 缓冲位于链接段 `.dma_buffer`，当前地址为 `0x24000000`，不能移回
  DTCM。D-Cache 当前未启用；若以后启用，必须增加 DMA cache 一致性处理或配置
  non-cacheable MPU 区域。
- Ethernet 地址保持不变：RX descriptor `0x30000000`、TX descriptor
  `0x30000080`、RX pool `0x30000100`、LwIP heap `0x30005000`。
- USART1、USART3、DMA1 Stream0、FDCAN2 和 ETH IRQ 均使用优先级 5，符合当前
  FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`。
- IWDG 约 2 秒超时。只有 DefaultTask、LegacyIoTask 和 ShellTask 都开始运行且
  最近 1 秒内更新过心跳时才启动和喂狗。
- `TestH743.ioc` 尚未同步本轮新增外设。板测通过前不要用 CubeMX 重新生成源码；
  当前以源码、链接脚本和 CMake 清单为准。

## 尚未完成

- UART4 的 17 字节 Z 轴命令/应答驱动尚未实现。目前只是硬件链路已经初始化。
- 协议声明的 Z 轴 SPEED 范围是 1 到 100000 Hz，计划中的 1 MHz 超出上限 10 倍。
  在 Z 轴 MCU 协议和硬件能力确认前，不设置 1 MHz 默认速度。
- Shell 当前保留旧的低层 SMD 命令，尚未整理为 `x_rel/y_rel/x_home/y_home` 等轴接口。
- QSPI 只保留原初始化，尚无独立 Flash 读写自检。
- X/Y 视觉映射参数接口已经预留，但 `center_x/center_y` 的轴映射、方向和
  pixels/mm 尚未标定；默认状态明确为不可用于视觉闭环。
- X/Y 当前坐标只在 RAM 中维护。上电后必须人工无限位回零成功或人工确认当前位置为零，
  控制层才允许受软限位保护的运动。

## 板上联调顺序

1. 不接电机和 C552，上电确认无持续复位，串口出现 Shell 提示和 IWDG 启动提示。
2. 连接网线，确认静态 IP ping 和 TCP 5000 原控制服务正常。
3. 连接 C552，连续执行 `C552`，确认 `valid` 增长、DMA 错误不增长、链路 ONLINE。
4. 连接 FDCAN 收发器和单个电机，执行 `cans`、`canprobe 1` 和只读查询。
5. 低速、短距离验证 X/Y 相对运动、停止、堵转标志和无限位回零；再逐步提高速度。
6. 验证 EEPROM 上电自检，并做独立的写入、掉电、读回测试。
7. 用逻辑分析仪检查 UART4 的 115200 8N1 电平和 TX/RX 交叉接线；协议层完成后再发
   17 字节样例帧。
8. 最后进行长时间并发测试：TCP、C552 50 Hz、CAN 运动和 Shell 同时运行，观察
   看门狗复位、C552 DMA 计数和 CAN recovery 计数。

## 构建验证

独立构建目录：`build/migration-peripherals`

构建结果：成功。FLASH 197788 B，DTCMRAM 46064 B，AXI RAM 512 B，D2 RAM
19075 B。链接 map 已验证上述 DMA 和 Ethernet 地址。

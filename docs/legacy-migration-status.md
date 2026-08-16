# TestH743 旧工程功能迁移状态

更新时间：2026-08-16

## 当前结果

`TestH743` 继续作为 FreeRTOS、LwIP 和 Ethernet 的主工程。旧工程的业务模块已放入
`App/Inc` 和 `App/Src`，没有复制旧工程的 `main.c`、GPIO、时钟、MPU、Ethernet 或
中断文件覆盖新工程。

已接入构建和运行：

- USART1 Shell，115200 8N1，中断接收；
- USART3 C552，460800 8N1，DMA1 Stream0 循环接收；
- FDCAN2，PB12/PB13，125 kbit/s；
- USART6，PC6/PC7，9600 8N1，保留给 Z 轴；
- USART6 使用 DMA1 Stream1 循环接收和 Stream2 发送；Z 轴 17 字节命令/应答、
  CRC16/MODBUS、异步状态和超时处理已接入；
- Z 轴位置层使用人工找零后执行 `z_zero` 建立坐标，正方向远离原点，软件范围
  `0..576000` 脉冲（机械行程约 36 mm），默认运动频率 90000 Hz；坐标暂不持久化，
  每次上电必须重新人工找零；运行期故障恢复保留故障前坐标，但不会自动重发未确认动作；
- `UNREFERENCED` 状态允许使用 `z_move` 正负相对点动寻找人工零点，点动不累计未知
  坐标且保持未引用；自动任务仍必须等待 `z_zero` 建立有效坐标；对外 Shell 不再提供
  `z_abs`，内部标定/任务仍使用受软件限位保护的绝对运动接口；
- 实机方向已确认：F103 协议 `DIR=0` 朝机械零点，`DIR=1` 远离零点；H743 逻辑
  正脉冲映射到 `DIR=1`，负脉冲映射到 `DIR=0`；
- P3 K230_2 XZ 红色标定已实现：等待红色模式 `APPLIED` 后，从当前安全姿态执行 X/Z
  正负中心差分，拟合 2x2 像素/脉冲矩阵；人工对正后采集参考中心，并在 EEPROM
  地址 64～119 保存带 generation 和 CRC32 的独立记录；
- I2C2 EEPROM，PH4/PH5；
- QSPI，PF6-PF9、PB2、PB6；
- 三个板载 LED；
- FreeRTOS 的 LegacyIoTask、ShellTask、MonitorTask；
- 依赖关键任务心跳的 IWDG；
- 原有 LwIP 和 TCP 5000 服务。
- X/Y 控制层：固定地址 1/2、脉冲坐标、软限位、非阻塞反馈、双轴停止；
- 保留人工触发的 X/Y 无限位回零和人工设零入口；启动流程仅在两台驱动器均确认在线后
  自动回零一次，故障恢复不会自动回零或续跑。
- P2 第一版 XY 视觉标定：复用上电回零坐标、中心差分采样、2x2 像素/脉冲矩阵拟合，以及人工
  对正后的参考像素采集。2026-08-13 已完成首次板上标定和人工对正；标定结果使用带
  magic、结构版本、长度、generation 和 CRC32 的 56 字节记录保存到 M24C02，并在启动时自动加载。
- XY 自动视觉对正第一版：使用 P2 的脉冲/像素逆矩阵，每 470 ms 最多决策一次，X/Y 分轴
  串行修正并等待 `IDLE`；带像素死区、比例增益、单步脉冲限幅、软限位、视觉超时和总时限。
- P4 K230_2 红色 XZ 中心对正第一版：复用 P3 逆矩阵和参考中心，等待红色模式 `APPLIED`
  后按 X、Z 分轴串行执行离散 P 控制；Shell 使用 `p4_start`、`p4_status`、`p4_abort`。
- P5 三轴统一安全与资源仲裁：`motion_coordinator` 统一管理 MANUAL、P2、P3、XY ALIGN、
  P4 和后续 MISSION 的运动所有权；自动任务、标定和 Shell 手动运动互斥。C552 必需设备
  掩码随 owner 动态切换，任务结束后恢复空闲掩码。
- P6 公共任务子流程已接入：任务名称固定为 `red_pick`、`tag_put`、`red_find`、
  `frame_put`；Observe、AlignXZ/XY、BlindMoveY、Tof3Descend、Grip、Record/ReturnPose、
  SafeRetreat 和 AbortAll 均由任务线程轮询，统一持有 `MOTION_OWNER_MISSION`。
- P7 四任务状态机已接入：四条状态链由 `mission_task` 编排，任务启动后跨子流程持续持有
  `MOTION_OWNER_MISSION`；任务级取消、子流程失败、全局锁存和 10 分钟总时限均会终止当前任务，
  并保留结构化失败来源、原因和 detail。Shell 只投递启动/取消事件，状态切换由 LegacyIoTask 完成。
- ObserveRedFront 已取得匹配的红色模式 `APPLIED` 后，紧随其后的 AlignXZ 复用该事务状态并
  等待新的 K230_2 样本，不再重复发送相同模式命令。若期间命令状态已被其他事务覆盖，AlignXZ
  仍会执行完整模式切换和 `APPLIED` 等待。
- AlignXZ 按任务保持 K230_2 识别模式：`tag_put` 使用 AprilTag，`red_pick`/`red_find`
  使用红色方块；独立 P4 对正仍默认使用红色方块。
- 2026-08-16 实机完整任务结果：`red_pick`、`red_find`、`tag_put` 均已从
  `mission_start` 运行至 `COMPLETE`；`red_pick`/`red_find` 的 BlindMoveY 使用 TOF2，
  `tag_put` 使用 TOF1。`frame_put` 尚未完成整条任务链验证，仍是当前 P7 主线阻塞项。
- 停止权限分为三级：`p2/p3/p4/align_abort` 和 `sf_abort` 只取消本地流程且不锁存；
  子流程最终失败终止当前任务并保留失败码；只有全局 `abort` 绕过普通 owner 锁，冻结夹爪、
  停止 X/Y/Z 并锁存。UART/DMA 回调只发布事件，不直接切换任务状态。

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
- K230 数据新鲜度门限为 150 ms；P2 仍只接受 `VALID=1`、`STALE=0`、达到 `READY`
  且 `sample_seq` 变化的新样本。
- P4 初始参数为 470 ms 决策周期、0.5 比例增益、像素死区 `+-3`、连续 3 个独立样本
  稳定、X/Z 单次限幅 24000/6400 脉冲、Z 10000 Hz、视觉超时 500 ms、总时限 60 s、
  最多 50 轮修正；Z 目标始终受实际软件范围 `0..576000` 约束。X/Z 使用同一个缩放因子
  同时满足两轴单步上限和剩余行程，保持逆矩阵算出的二维修正方向；禁止分别裁剪两个分量。
- P5 的夹爪停止语义是冻结当前状态并拒绝后续夹爪命令，不主动开爪。当前 C552 协议没有
  物理停止 PWM 命令；若任务要求夹爪硬件断 PWM，必须先扩展 C552 协议。
- 只有全局 `abort` 对 PD42S1 发送双轴 `STOP_NOW`；X、Y 间隔 10 ms 发送并分别等待应答，
  不清除 `position_valid`、零点偏移或驱动器状态。普通 X/Y/Z 单轴故障不产生全局锁存，只
  终止使用该轴的自动任务并释放 owner，也不停止无关轴。只有全局 `abort` 保持到设备重启。
  代码中已删除 X/Y
  `CLEAR_STATE (0xFB)` 的协议常量、控制 API、Shell 命令和自动恢复路径。
- `xyz_snapshot` 仅在 X/Y/Z 三个坐标均有效、无故障且均为 `IDLE` 时返回快照；任一坐标
  无效时整份快照失败，禁止保存部分有效姿态。
- P7 不为未板测运动参数提供默认值。每个任务分别配置目标中心、Y 盲走参数和按需的 TOF3
  下降参数；所有任务通过预检后先按 Z、X、Y 顺序移动到固定预设位置
  `(X=512000, Y=0, Z=77600)`。`tag_put` 和 `frame_put` 还要求单独配置有效的安全撤离位姿；
  固定预设位置不替代该配置。任务活动期间配置不可修改。
- C552 当前没有夹持物检测字段；P7 的 `PRECHECK_HELD` 使用最近一次 GripClose/GripOpen
  `APPLIED` 后维护的逻辑状态。上电默认 `EMPTY`，若设备带物上电，必须先执行
  `mission_payload held`。此状态不是力传感器或物体存在检测。
- 所有 P6/P7 子流程统一具有 10 次尝试预算，失败或 BUSY 后间隔 50 ms 再次尝试。AlignXZ
  和 Tof3Descend 的 Z 瞬时故障另有独立的 10 次自动恢复预算，每次恢复严格执行 F103
  `CLEAR_FAULT` 应答和 `QUERY_STATUS` 健康确认，再等待 50 ms 重试，且不消耗子流程尝试。
  `sf_status` 和 `mission_status` 分开显示 `attempt` 与 `z_recovery`；
  持续故障达到 10 次后仍终止任务，不绕过软限位或坐标有效性检查。

## 尚未完成

- USART6 Z 轴协议已实现 `MOTOR`、`STOP`、`CLEAR_FAULT` 和 `QUERY_STATUS`，并已在三条完整
  任务链中完成实际收发和运动验证；急停、持续故障恢复、长行程超时及长时间稳定性仍需专项板测。
  Shell `z_clear` 只启动 Z 轴 `CLEAR_FAULT -> QUERY_STATUS` 恢复事务，不访问 X/Y CAN。
- P7 的 `red_pick`、`red_find`、`tag_put` 已完成实机完整流程；`frame_put` 尚未通过。
  任务目标中心、Y 轴 `pulses_per_mm`、盲走截止距离、TOF3 最大下降量/方向和安全位姿由板测配置；
  四任务配置和安全位姿保存在
  EEPROM `128..239`，使用 magic/version/generation/CRC32 校验并在上电时自动加载，不使用未经
  测量的运动默认值。逻辑持物状态不持久化，带物上电仍需执行 `mission_payload held`。
  抓取后的 `VERIFY` 仅确认逻辑持物状态，不执行已作废的视觉计数验证，
  避免被夹持物遮挡相机造成必然误判。
- Z 轴 SPEED 协议范围是 1 到 100000 Hz，当前推荐及默认速度为 90000 Hz。
- Shell 当前保留旧的低层 SMD 命令，尚未整理为 `x_rel/y_rel/x_home/y_home` 等轴接口。
- QSPI 只保留原初始化，尚无独立 Flash 读写自检。
- X/Y 视觉映射已完成一次有效板测，获得 2x2 像素/脉冲矩阵和人工对正参考像素；参数持久化
  和第一版自动对正代码已实现，但尚未完成 EEPROM 掉电恢复、矩阵重复性和闭环板测。
- X/Y 当前坐标只在 RAM 中维护。上电后必须人工无限位回零成功或人工确认当前位置为零，
  控制层才允许受软限位保护的运动。
- C552 接收已升级到 V3.1：62 字节周期快照、TOF3、五设备健康掩码，以及带
  `CMD_SEQ` 的 K230 模式和夹爪命令回执。当前默认必需设备为 K230_1；只有匹配事务的
  最终 `APPLIED` 回执表示下行命令执行完成。
- X/Y 无限位回零曾按 `ORIGIN_BREAK -> CLEAR_STATE -> MOTOR_ENABLE` 流程板测通过；为彻底
  禁止 X/Y `CLEAR_STATE`，当前已改为 `ORIGIN_BREAK -> MOTOR_ENABLE`。上电后仍先等待地址 1
  和 2 各返回一帧合法应答，两轴都在线才按 X 后 Y 自动回零；新流程必须重新板测，任一轴
  无应答或回零失败都会终止启动流程。
- X/Y 驱动器堵转保护当前按调试要求关闭：回零前后均发送
  `FCT_SET_CLOG_PRO=0`，普通位置运动期间不重新开启；H743 仍会处理驱动器主动上报的
  `MOTOR_STA_STALL`。关闭驱动器保护后必须依靠软件限位、运动超时和人工急停控制风险。
- X/Y 机械运动结束后 `BUSY` 长时间不释放的问题已通过放宽位移差完成判据解决：目标附近
  连续 3 次位置反馈进入容差后，控制层直接释放为 `IDLE`，并以 `completion=TOLERANCE`
  记录；不向仍在目标附近追零的驱动器额外发送停止命令。
- BlindMoveY 提交 Y 轴命令时若恰逢后台状态查询，`XY_RESULT_BUSY` 按瞬时总线占用处理：
  不消耗 10 次动作尝试预算，50 ms 后重新提交。该结果值为 3，不表示脉冲或软限位错误。
- P3 已取得实机标定结果。P4 已完成实机闭环，最终连续 3 个样本稳定，12 轮修正后进入
  `COMPLETE`；完成位置约为 X=1392117、Z=114750 脉冲。
- P4 首轮板测发现分别将 X/Z 限幅为 12000/640 会严重改变逆矩阵修正比例，使其中一个像素
  分量反向增大；已改为二维向量同比例缩放。`p4_status` 分开显示最新 `latest` 像素和本轮
  实际参与计算的 `decision` 像素，并显示 `raw`、`scale` 和最终 `step`，避免混合时刻误判。
  为缩短大误差对正时间，后续将保守限幅提高到 X/Z 24000/6400 脉冲；两者仍显著低于
  P3 实机标定步长 51200/57600 脉冲。

## P2 第一版视觉标定

1. 确认 K230 已切换到目标识别模式，目标在整个标定运动范围内持续可见。
2. 执行 `p2_start 1 51200 12800`。参数依次为 K230 编号、X 标定步长、Y 标定步长，
   单位均为脉冲；默认值与该命令相同。
3. P2 不会再次回零；启动时要求两轴已经由上电回零建立坐标、均为 `IDLE`。程序直接移动
   到安全基准位置，然后依次采集 X+/X-/Y+/Y-。每个位置只接受
   `sample_seq` 变化的新样本，并要求 5 个样本的中心像素极差不超过 4 像素。
4. 用 `p2_status` 查看进度。进入 `MANUAL_ALIGN` 表示 2x2 矩阵有效，此时可用
   `x_move`/`y_move` 人工将末端与目标对齐。
5. 人工移动结束后执行 `p2_ref`。请求会先进入 `WAIT_REFERENCE_IDLE`，允许控制层等待
   已知的 `BUSY` 释放延迟；两轴均为 `IDLE` 后才开始采集稳定参考像素。状态变为
   `COMPLETE` 且 `valid=1` 表示第一版标定完成。
6. `p2_abort` 可随时终止自动阶段；自动阶段会立即停止两轴。参考采集成功后自动保存 EEPROM；
   `p2_save`、`p2_load`、`p2_reset` 可人工保存、重载或清除。第一版仍不执行自动视觉闭环。

### 2026-08-13 首次板测结果

- 命令参数：K230_1，X 步长 51200 脉冲，Y 步长 12800 脉冲；
- 完成状态：`COMPLETE`，`fault=NONE`，`valid=1`；
- 人工对正参考像素：`center_x=255`，`center_y=182`；
- 像素/脉冲矩阵（行依次为 `center_x`、`center_y`，列依次为 X、Y 轴脉冲）：

```text
J = [[ 0.000000, -0.002851 ],
     [ 0.000097,  0.000117 ]]
```

- 脉冲/像素逆矩阵：

```text
J^-1 = [[ 420.809600, 2147.483647 ],
        [-350.643872,    0.000000 ]]
```

该结果确认相机像素轴与机械轴并非一一同名对应：X 轴运动主要改变 `center_y`，Y 轴运动
主要改变 `center_x`。上述数值目前只代表本次标定结果；接入自动对正前需要重复标定并做
正反向小步移动验证，统计矩阵离散程度，同时确认参考像素在相机安装不变时是否稳定。

下一步顺序：EEPROM 掉电恢复验证 -> 用 `align_start` 低风险板测第一版闭环，并通过
`align_status`、`xy_status` 观察每次修正和 `BUSY` 完成来源 -> 根据结果调整增益、死区和步长限幅。

## 板上联调顺序

1. 不接电机和 C552，上电确认无持续复位，串口出现 Shell 提示和 IWDG 启动提示。
2. 连接网线，确认静态 IP ping 和 TCP 5000 原控制服务正常。
3. 连接 C552，连续执行 `C552`，确认 `valid` 增长、DMA 错误不增长、链路 ONLINE。
4. 连接 FDCAN 收发器和单个电机，执行 `cans`、`canprobe 1` 和只读查询。
5. 低速、短距离验证 X/Y 相对运动、停止、堵转标志和无限位回零；再逐步提高速度。
   X/Y 闭环在目标附近可能持续低速追零并保持 `MOVING`。当前两轴位置反馈连续 3 次进入
   目标 `+-1024` 脉冲后仅由控制层释放为 `IDLE`，不发送 `0xFC`；`xy_status` 以
   `completion=TOLERANCE` 和 `releases=.../Tn` 记录该结束路径，并显示各类主动停止计数。
6. 验证 EEPROM 上电自检，并做独立的写入、掉电、读回测试。
7. 用逻辑分析仪检查 USART6 的 9600 8N1 电平和 TX/RX 交叉接线；使用 `z_move`、
   `z_stop` 和 `z_status` 验证 17 字节命令、应答及异常计数。
   Z 轴故障恢复必须依次取得 `CLEAR_FAULT=COMPLETE` 和 `QUERY_STATUS=COMPLETE`；仅恢复 UART
   DMA 不会解除 FAULT。所有 Z 轴故障均保留坐标参考，包括运动事务应答超时和软件坐标越界；
   恢复确认后继续使用故障前坐标，不自动重发未确认的相对运动。若超时期间发生了未确认位移，
   坐标误差由后续 KP 闭环修正。`z_status` 显示恢复事务、控制器原始 STATUS、
   累计故障、确认恢复次数和最近故障。
8. 最后进行长时间并发测试：TCP、C552 50 Hz、CAN 运动和 Shell 同时运行，观察
   看门狗复位、C552 DMA 计数和 CAN recovery 计数。

## P4 红色 XZ 中心对正板测

1. 确认 `p3_status` 显示 `valid=1`，X/Z 已建立坐标且位于行程中部，K230_2 红色目标可见。
2. 执行 `p4_start`，立即用 `p4_status` 确认经过 `WAIT_RED_MODE` 并收到新的 K230_2 样本。
3. 首轮只允许较小像素误差，逐次查看 `error`、`step`、`pos` 和 `target`；确认 X/Z 修正
   均使下一份新样本的像素误差减小。方向相反时立即执行 `p4_abort`，先复核 P3 矩阵。
4. 确认每轮严格按 X 完成、Z 完成、等待新 `sample_seq` 的顺序运行，没有复用旧误差累加。
5. 在参考中心附近确认连续 3 个新样本进入 `+-3` 像素后状态变为 `COMPLETE`。
6. 分别测试遮挡目标超过 500 ms、接近 X/Z 软件限位、运行中执行 `p4_abort`；均应停止
   后续修正；`p4_abort` 应同时请求停止仍在运动的 X 和 Z，但不产生全局锁存。

## 构建验证

当前构建目录：`build/Debug`

2026-08-14 增加 P7 EEPROM 参数持久化、Z 坐标可信度恢复规则，并移除普通轴故障全局锁、
跨轴停止和 X/Y `CLEAR_STATE` 后 clean build 结果：成功。FLASH 283324 B，DTCMRAM 47840 B，
AXI RAM 593 B，
D2 RAM 19075 B。`mission_task.c`、`mission_subflow.c`、`xz_vision_align.c` 和
`motion_coordinator.c` 已参与编译并链接到最终 ELF。

2026-08-16 在 USART6 迁移、任务预设位、统一重试、TOF映射和安全撤离顺序更新后的最新
CMake Debug 构建成功：FLASH 284268 B / 2 MB，输出 `build/Debug/TestH743.elf`。
本轮文档更新未改固件，因此没有重复构建或烧录。

## P6 公共子流程板测

先执行 `sf_status` 确认空闲。所有 `<task>` 参数均从 `red_pick`、`tag_put`、`red_find`、
`frame_put` 中选择。

1. 依次执行 `sf_observe <task> red|tag|frame`。确认模式命令达到 `APPLIED` 后才接受新样本；
   空白 K230 数据段不算目标，连续 3 个新样本中心波动不超过 4 px 才完成，最多尝试 10 次。
2. 使用 `sf_align <task> xz|xy <target_x> <target_y>` 验证同一标定矩阵可加载不同任务中心；
   `sf_status` 应显示显式目标像素，对正故障后局部停止并重试，总尝试次数为 10，间隔 50 ms。
3. 使用 `sf_blind_y <task> tof1|tof2 <stop_mm> <pulses_per_mm> <-1|1>`。确认每次尝试只在
   起步前读取一次 TOF，运动期间不再按 TOF 修正；`red_pick`/`red_find` 只允许 TOF2，
   `tag_put` 只允许 TOF1；X/Y 轴故障立即终止子流程，不发送清状态命令；仅 Z 轴故障允许经
   F103 确认清故障后从新样本重新计算。
4. 使用 `sf_descend <task> 30 <max_pulses> <-1|1>`。确认 Z 分段下降，TOF3 首个独立有效样本
   `<=30 mm` 时立即停轴，并在静止状态累计 10 个独立样本后完成；任一样本 `>30 mm` 清零计数
   并恢复分段下降，最大下降量和软限位均有效。
5. 使用 `sf_grip <task> open|close`，确认 `ACCEPTED` 不完成流程，只有 `APPLIED` 才完成；
   失败最多尝试 10 次。
6. 三轴有效且空闲时执行 `sf_record <task>`，移动后执行 `sf_return <task>`。先用
   `sf_safe_set <x> <y> <z>` 设置限位内安全位姿，再执行 `sf_retreat <task>`；SafeRetreat
   对 `tag_put` 按 Y、Z、X 顺序，其他任务按 Z、X、Y 顺序；当前任一坐标无效时拒绝启动。
7. 运行任一子流程时执行 `sf_abort`，确认只取消当前子流程、不产生全局锁存；再运行并执行
   `abort`，确认 X/Y/Z 停止、夹爪冻结且重启前不能重新取得 owner。

## P7 四任务状态机板测

截至 2026-08-16 的实机结果：

| 任务 | 完整流程 | 测距依赖 | 备注 |
|---|---|---|---|
| `red_pick` | 通过，到达 `COMPLETE` | TOF2 | 抓取后返回记录位姿，逻辑载荷为 `HELD` |
| `red_find` | 通过，到达 `COMPLETE` | TOF2 | 配置独立于 `red_pick` |
| `tag_put` | 通过，到达 `COMPLETE` | TOF1 | AprilTag 模式；开爪后安全撤离顺序为 Y、Z、X |
| `frame_put` | 未通过/待继续板测 | TOF3 | 下视 AlignXY、TOF3 下降及完整撤离链仍需验证 |

上述“通过”表示正常条件下单条完整任务链通过，不等同于 EEPROM 掉电恢复、故障注入、全局
`abort`、重复运行和长时间耐久项目全部通过。

P7 四任务参数与安全位姿会自动保存到 EEPROM；配置命令返回 `persisted` 且 `mission_status`
显示 `storage=VALID` 后，掉电重启应自动恢复。每次上电仍应先按 P6 顺序单独验证对应子流程，
再启动完整任务。任务运行期间 `motion_status` 应始终显示 `owner=MISSION`，包括
`FRONT_*_READY`、`STABLE` 和子流程切换
间隙；此时所有会改变 P6 状态的 `sf_*` 命令和 Shell 手动运动都必须被拒绝。

1. 为抓取任务配置目标和 Y 盲走：
   `mission_align_set red_pick <x> <y>`、
   `mission_blind_set red_pick <stop_mm> <pulses_per_mm> <-1|1>`。
   `red_find` 使用相同命令独立配置，不自动继承 `red_pick` 参数。
   BlindMoveY 只在起步前读取一次 TOF；若距离大于 `stop_mm`，一次性请求
   `(TOF-stop_mm)*pulses_per_mm*direction` 脉冲，运动中不会按 TOF 实时停止。
2. 为 Tag 放置配置目标和 Y 盲走，并选择是否启用 Z 下降：
   `mission_align_set tag_put <x> <y>`、
   `mission_blind_set tag_put <stop_mm> <pulses_per_mm> <-1|1>`、
   `mission_z_set tag_put off`，或
   `mission_z_set tag_put <stop_mm> <max_pulses> <-1|1>`。
3. 为红框放置配置下视目标和必需的 TOF3 下降：
   `mission_align_set frame_put <x> <y>`、
   `mission_z_set frame_put <stop_mm> <max_pulses> <-1|1>`。
4. 放置任务前执行 `sf_safe_set <x> <y> <z>` 设置限位内安全位姿。正常抓取成功会将逻辑状态
   更新为 `HELD`；若带物上电，用 `mission_payload held` 显式恢复，放置成功后应变为 `EMPTY`。
5. 用 `mission_status [task]` 检查当前状态和指定任务配置。缺少配置的 bit detail 为：
   `1=目标中心`、`2=Y盲走`、`4=Z下降`、`8=安全位姿`。随后执行
   `mission_start red_pick|tag_put|red_find|frame_put`。四个任务通过 PRECHECK/PRECHECK_HELD 后
   均应先经过 `PRESET_POSE`，按 Z、X、Y 顺序到达 `Z=77600`、`X=512000`、`Y=0`。
6. red_pick/red_find 应依次经过 PRESET_POSE、RED_OBSERVE、FRONT_RED_READY、ALIGN_XZ、STABLE、RECORD_XYZ、
   BLIND_Y、GRIP_CLOSE、RETURN_RECORDED_POSE、VERIFY、COMPLETE。VERIFY 只检查逻辑 `HELD`，
   不读取被物块遮挡的相机。
7. tag_put 应经过 PRESET_POSE、TAG_OBSERVE、FRONT_TAG_READY、ALIGN_XZ、STABLE、BLIND_Y、
   OPTIONAL_Z_DROP、GRIP_OPEN、SAFE_RETREAT、COMPLETE；`mission_z_set tag_put off` 时
   OPTIONAL_Z_DROP 不发 Z 运动。
8. frame_put 应经过 PRESET_POSE、FRAME_OBSERVE、DOWN_FRAME_READY、ALIGN_XY、STABLE、TOF3_Z_DESCEND、
   GRIP_OPEN、SAFE_RETREAT、COMPLETE。`tag_put` 的 SafeRetreat 按 Y、Z、X 顺序，先收回 Y；
   其他任务的 SafeRetreat 和所有任务的 PRESET_POSE 仍按 Z、X、Y 顺序。
9. 各状态分别注入视觉超时、C552 命令失败和轴故障，确认任务进入 FAULT 并保留失败码；执行
   `mission_abort` 应局部停止并进入 `FAULT/CANCELLED`，不产生全局锁存。另测全局 `abort`，
   确认任务终止且重启设备前不能重新启动。
10. 在 AlignXZ 或 Tof3Descend 中注入可恢复的 Z `CONTROLLER_REJECTED`，确认
    `z_recovery` 递增、`attempt` 不减少剩余预算，并在 Z 恢复 IDLE 后继续当前子流程。持续注入
    10 次时应以 `AXIS_Z:RETRY_EXHAUSTED` 终止；软限位必须立即终止且不得进入恢复循环。

## P5 统一仲裁板测

1. 空闲执行 `motion_status`，确认 `owner=NONE`、`latch=NONE`，再执行 `xyz_snapshot`；只有
   X/Y/Z 坐标均有效且空闲时才应成功。
2. 分别启动 P2、P3、`align_start` 和 P4，在运行中尝试 `x_move`、`y_move`、`z_move`、
   `grip` 和另一自动任务，确认均因 owner 冲突被拒绝；任务完成/故障后 owner 自动释放。
3. P2/P3 进入 `MANUAL_ALIGN` 时确认 owner 已释放，人工对正可运动；执行 `p2_ref`/`p3_ref`
   后标定重新取得 owner，直到 COMPLETE/FAULT。
4. 任一轴运动中执行全局 `abort`，确认命令不等待普通 CAN mutex，X/Y/Z 同时进入停止流程，
   自动任务终止，`motion_status` 显示 `latch=ABORT` 且夹爪被冻结。
5. 手动 Z 运动中注入 Z 故障，确认 X/Y 不收到 `STOP_NOW`，`owner` 自动释放且 `latch=NONE`；
   `z_clear` 确认恢复后应能再次执行 `z_move`。在 P2/P3/独立 Align/P4 中注入 X、Y 或 Z 故障，
   确认只停止任务实际参与且仍在运动的轴，当前任务终止且不产生全局锁存。MISSION 中 X/Y
   故障直接结束当前子流程，不发送清状态命令；AlignXZ
   和 Tof3Descend 仅对 Z 故障执行 F103 确认恢复，不自动续跑未确认的旧动作。
6. ABORT 后尝试 `grip` 和重新取得 owner 均应被拒绝；`motion_status` 的锁存保持到设备重启。
   夹爪保持 ABORT 前状态，不应自动开爪。

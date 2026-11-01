# 超级电容移植说明

本模块按用户提供的 `superpower_task.c/.h` 和《超级电容文件.docx》移植，仅实现该资料中的
`0x61` 控制及新版反馈。已移除此前额外加入的旧版 `0x210/0x211` 兼容分支。

## 工程内分工

| 文件 | 职责 |
| --- | --- |
| `Core/Src/freertos.c` 用户代码区 | 定义 `SUPERPOWER` 静态线程，256 字栈，`osPriorityAboveNormal`；初始化成功后创建 |
| `app_supercap.cpp` | `superpower_task()` 入口、裁判数据读取、每 1 ms 发送和状态查询 |
| `2_Device/Supercap/dvc_supercap.cpp` | 按文档打包/解析 8 字节 CAN 帧、有效数据缓存、反馈在线检测 |
| `supercap_config.h` | CAN 口、反馈 ID、发送周期、超时和充电限制比例 |

应用内不调用 `osThreadCreate()`，主函数不再调用超电初始化。
`MX_FREERTOS_Init()` 中先调用 `supercapInit()` 注册设备，成功才创建线程。
所有线程添加均在 `USER CODE` 区域，不修改 `.ioc`、FDCAN 配置或 HAL 库。

CAN 发送复用 `bspFdcanSend()`，反馈复用 `bspFdcanAddObserver()`，与电机共享本工程的
FDCAN 中断分发。没有复制外部 `can_service` 或另写 HAL 接收回调。
裁判系统在底盘板接收；超电直接使用本板 `referee.GetPowerSupplySnapshot()`
获取功率、缓冲能量及其新鲜度。

## 与提供任务的对应

- 默认请求 `enableDCDC=1`，`systemRestart=0`、`clearError=0`。
- `useNewFeedbackMessage=1`，`enableActiveChargingLimit=1`。
- `activeChargingLimitRatio=229`，229/255 约为 90%，留出回收能量的余量。
- `refereePowerLimit` 来自本工程裁判 0x0201 的底盘功率上限。
- `refereeEnergyBuffer` 来自本工程裁判 0x0202 的缓冲能量。
- 循环通过现有 CAN 接口发送，然后 `osDelay(1)`。

按文档注意事项，功率和缓冲能量各自过滤零、负数、NaN、无穷及配置范围外的值，沿用
各自上次有效值。没有首组完整有效数据时不发送，避免把初始化的零写给超电。
裁判整体离线或任一对应报文过期时，使用缓存有效值发送 DCDC 失能。

有效且新鲜的裁判数据就绪后即可发使能，不把超电在线作为首次发送使能的前提。
超电反馈离线仍继续发送控制帧；底盘预算查询返回 false，不能使用旧预算。
这是为了保持原任务的控制发送方式，避免需要使能后才反馈的设备无法启动。

## 配置与使用

默认 CAN1，参考旧工程底盘的 `CHASSIS_CAN=1`。
`SUPERCAP_FEEDBACK_CAN_ID` 当前为 `0x800`，仅是待填写标记，不是真实反馈 ID。
未填写时 `supercapInit()` 返回 false，FreeRTOS 不创建超电线程。
确认反馈 ID 后修改宏并重新编译，线程将由 FreeRTOS 自动创建，不用再在 main 里调用。

运行时可调用：

```cpp
supercapSetEnabled(false); // 撤销 DCDC 许可，例如机器人禁用或急停。
supercapSetEnabled(true);  // 恢复许可，仍须满足裁判数据有效和新鲜。
const auto feedback = supercapGetFeedback(); // C++：直接读取 SupercapFeedback。
const auto status = supercapGetStatus();    // C/C++：查询应用状态。
```

许可请求保持不变时，裁判通信恢复后会恢复对应使能状态。
发送成功表示入硬件 FIFO，不表示超电已经执行。
`SupercapFeedback` 直接包含 `last_rx_tick`、`rx_count` 和 `online`；读取时重新判定超时。
应用层只保留运行状态和使能请求，不重复缓存设备反馈，也不另建快照结构。
CAN 收发诊断可查询现有 `bspFdcanGetStats()`。

## 帧格式

控制帧 ID 固定为 `0x61`，标准经典 CAN、DLC 8：

| 字节 | 内容 |
| --- | --- |
| 0 | bit0 使能、bit1 重启、bit5 清错、bit6 主动充电限制、bit7 新版反馈，其余为零 |
| 1–2 | 裁判功率上限，小端 uint16，W |
| 3–4 | 裁判缓冲能量，小端 uint16，J |
| 5 | 充电限制比例，默认 229 |
| 6–7 | 保留，清零 |

代码显式打包字节，避免直接 memcpy 位域结构体造成编译器布局依赖。

反馈按照文档解析：byte0 状态字；byte1–2 底盘功率、byte3–4 裁判侧功率，均为小端
uint16 后按 `(raw-16384)/64` 转 float；byte5–6 底盘允许功率 W；byte7 能量原始值。
保留功率的小数和负值；状态位定义和能量量纲未提供，因此只保留原始值。
总线/ID/帧类型/长度不符不更新反馈。默认反馈超时为 100 ms。

## 使用现有功率算法

正常功率控制使用超电返回的 `chassisPowerLimit`，不是发给超电的裁判上限：

```cpp
float watts = 0.0f;
if (supercapGetChassisPowerLimit(&watts)) {
    // 提供给已有 Algorithm/Power 模块作为本轮硬上限。
    // 输入电流用 A，角速度用与模型辨识一致的 rad/s。
} else {
    // 由底盘执行停止或经过验证的供电降级策略。
}
```

保留现有 `Algorithm/Power`，本次未新增另一套功率算法，也未编写具体机器人的底盘任务。
功率算法使用 `powerControllerSetPowerLimit()` 接收超电硬上限，姿态倍率和裕度只能降低预算。
预算不可用时传入 0，本轮输出零电流；旧名称 `powerControllerSetRefereeLimit()` 行为相同。
算法会复核最终输出的模型功率，应用还应处理 `BudgetInfeasible` 状态。

## 验证

`python tests/run_host_tests.py` 执行算法、遥控器和计时等主机回归，不包含超电测试。
本次主固件和本地调参固件均编译通过；临时超电检查位于 `build/snapshot_cleanup`，
覆盖 tick 为零、超时边界、节拍回绕、反馈复位、控制帧字节、无效预算拒绝和裁判失联失能。
`tests` 目录保持原样。以上检查不替代实际反馈 ID 确认和上板验证。

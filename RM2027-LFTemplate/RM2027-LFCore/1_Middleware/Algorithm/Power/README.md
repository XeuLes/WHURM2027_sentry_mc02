# 四参数功率控制

模型为 `P = k0 + k3*I*w + k4*I^2 + k5*w^2`。删除原来的 `k1*I`、`k2*w` 两个零项，
保留其余四个标定值与字段名。没有新的采样数据，因此本次没有重新拟合数值。
电流使用 A，角速度使用 rad/s，轴和正方向必须与辨识时一致。
若用整车总功率拟合单电机共用模型，常数项的回归特征应为参与的电机数量，不能将整车
常数损耗直接重复加到每台电机。模型要求 k0/k4/k5 非负，所有系数有限，以保证凸性。

## 调用约定

调用方在进入功率控制之前已经完成电流限幅。本模块直接使用输入电流，不再截断。
`current_limit` 和 `powerControllerSetCurrentLimit()` 仅检查输入是否满足调用方约定；
超出范围、非有限输入或溢出时清零输出并标记 `InvalidInput`。

超电返回的允许功率使用 `powerControllerSetPowerLimit()` 设置。旧名称
`powerControllerSetRefereeLimit()` 保留，行为一致。它不读取超电或裁判系统，也不发 CAN，
数据选择和电机发送由应用负责。

移动姿态默认倍率改为 1；所有姿态倍率与 margin 在应用时限制为 0~1，不能放大超电硬上限。
预算为零时输出零电流，不沿用旧预算，也不再初始化成 45 W。

```cpp
float watts = 0.0f;
const bool available = supercapGetChassisPowerLimit(&watts);
power_control::powerControllerSetPowerLimit(&ctrl, available ? watts : 0.0f);
// motors[] 的电流此前已经完成限幅。
power_control::powerControllerUpdate(&ctrl, motors, count);
```

## 两种方法

`POWER_CONTROL_USE_POWER_DECAY=0` 为统一电流衰减：一次缓存遍历同时得到每台模型和
总功率方程的 A/B/C，解 `A*eta^2+B*eta+C=budget`。所有电流统一乘 eta，包括负功率电机，
保持原方法的电流比例。可行区间限制为 eta 在 0~1；无可行点时返回该区间的最小功率点。

`POWER_CONTROL_USE_POWER_DECAY=1` 为逐电机功率分配：负功率电机直接输出输入电流。
正功率电机先计算同方向、幅值不增加区间内的最低可达功率，保留这些最低功率与负功率后，
再按比例分配可调部分，逐台反解电流。factor 表示可调功率的保留比例，不再表示总功率比例。

原来的二次反解已经包含已知速度和常数项，问题不是漏掉这两项，而是分配的目标功率
可能没有可行电流解。现在显式考虑最低可达值，不将不可达的分配值作为最终功率。

通用求根接口在 `alg_basic.hpp` 的 `basic_math` 命名空间：

```cpp
const auto roots = basic_math::solveQuadratic(a, b, c - budget);
if (roots.status == basic_math::QuadraticStatus::OneRealRoot
    || roots.status == basic_math::QuadraticStatus::TwoRealRoots) {
    // roots.lower <= roots.upper；单根时二者相等。
    const float right_root = roots.upper;
}
```

接口求解 `a*x^2+b*x+c=0`，采用归一化系数和 q 形式避免判别式溢出与相近数相减。
返回状态区分单根、双根、无实根、恒等式、非法输入和数值范围错误；仅单根/双根状态的
根有效。精确 `a==0` 时处理一次或常数方程。power 取右根、限制到最小功率点与 1 之间，
并保留预算不可达时返回最小功率点的策略。
正常路径采用解析解。最终按真实 output[] 重新计算 consume_power；仅在浮点舍入导致
超预算时，沿可行方向执行最多 24 次有界修正。未实测 Cortex-M7 上的最坏执行时间。

## 状态

| 状态 | 含义 |
| --- | --- |
| Ready | 无需衰减 |
| Limited | 已衰减，最终输出的模型总功率不超过预算 |
| Disabled | 零预算，输出零电流；模型常数/速度损耗仍可能非零 |
| BudgetInfeasible | 允许电流范围内无法满足预算，输出最小模型功率点 |
| InvalidInput | 输入不符合约定或数值运算异常，输出清零 |

`budget_feasible` 表示最终模型总功率是否不超过预算；`minimum_power` 给出当前方法的最低值。
应用需要处理预算不可达状态，不能将它理解为已完成限功率。上述均为模型估计，不代表
真实母线功率一定精确等于估计值。仍需验证系数、回馈能量能否被电源实际吸收等条件。

通用求根的边界回归位于 `tests/core_math_test.cpp`，通过 `python tests/run_host_tests.py`
运行。功率控制的临时数值验证放在 build 目录，两种模式分别验证边界和 50000 组随机输入，
basic 与两种 power 模式均完成 Cortex-M7 目标编译；尚未做上板时延或真实功率验证。

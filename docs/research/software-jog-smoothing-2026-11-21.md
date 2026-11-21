# 机械臂抖动的软件改善方案

日期：2026-11-21。软件基线为 [`f8c31e8`](https://github.com/ExplodingUFO/UnitreeR17aController/commit/f8c31e85a96617038f72f59a03d7864316f5db2b)。测试条件为 macOS、右臂、底座向上立装、无外接负载。本轮核对官方源码、厂商控制文档和论文，没有修改运行算法、增益或连接真机。

建议先做**短时曲线采样与统一的限 jerk 轨迹整形**，同时核对最终发出的位置、速度和时间间隔。低通用于已经确认的信号噪声；输入整形和陷波用于已经识别的共振。现有信息不足以把本次抖动定为 PID、结构共振或某一个滤波频率问题。

## 1. 当前软件已有与尚缺的处理

| 部分           | 当前实现                                                             | 适用改进                                                                                                                                                                                                                                                                                                                                                   |
| -------------- | -------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 连续关节点动   | 速度、加速度限幅后积分位置，未维护显式加速度状态和 jerk 上限         | 启动、松键和换向沿用位置/速度/加速度状态，限制加速度变化率。[源码](https://github.com/ExplodingUFO/UnitreeR17aController/blob/f8c31e85a96617038f72f59a03d7864316f5db2b/src-tauri/src/arm.rs#L1147-L1162)                                                                                                                                                   |
| 固定步长与收纳 | 步长使用三次曲线，收纳使用五次曲线                                   | 收纳曲线的端点速度、加速度为零，但这不等于所有点动都有统一 jerk 上限。[源码](https://github.com/ExplodingUFO/UnitreeR17aController/blob/f8c31e85a96617038f72f59a03d7864316f5db2b/src-tauri/src/arm.rs#L1225-L1234)                                                                                                                                         |
| 末端点动       | 按参考姿态计算 Jacobian，检查最终 FK、速度和加速度                   | 平滑输入速度后仍检查七轴动态和 TCP 路径，保留选定旋转中心。[源码](https://github.com/ExplodingUFO/UnitreeR17aController/blob/f8c31e85a96617038f72f59a03d7864316f5db2b/src-tauri/src/kinematics.rs#L355-L376)                                                                                                                                               |
| 发送与日志     | 控制和发送目标周期均为 10 ms；发送合并为最新目标；诊断约每秒汇总一次 | 补实际发送帧序号、时间与目标曲线，检查跳帧和位置/速度一致性。[发送](https://github.com/ExplodingUFO/UnitreeR17aController/blob/f8c31e85a96617038f72f59a03d7864316f5db2b/src-tauri/src/protocol.rs#L758-L785)、[诊断](https://github.com/ExplodingUFO/UnitreeR17aController/blob/f8c31e85a96617038f72f59a03d7864316f5db2b/src-tauri/src/arm.rs#L1461-L1482) |

前端每 100 ms 的 `jog_update` 用于延长输入有效期，不直接生成每个电机位置目标；运动生成仍在后端独立运行。[前端续期](https://github.com/ExplodingUFO/UnitreeR17aController/blob/f8c31e85a96617038f72f59a03d7864316f5db2b/src/api.ts#L619-L646)。两个线程的实际节拍、帧合并是否造成值得处理的目标跳步，需要发送侧曲线验证，不能仅凭线程结构判断本次振动原因。

## 2. 官方控制实现提供的依据

宇树遥操作维护单一的 `q_cmd/v_cmd/a_cmd` 状态，通过位置误差和速度阻尼生成加速度，再约束 jerk；当 jerk 与速度/加速度硬界冲突时，优先硬界。最终 `dq` 由最后实际发出的位置差和实际命令间隔计算。它比单独匹配电机 `kp/kd` 多了一层轨迹处理。[轨迹状态](https://github.com/unitreerobotics/R1-7A/blob/e2faf3ec17f589db2139c51b020592a4a05f836b/teleop/r1arm_controller_base.py#L300-L359)、[最终下发](https://github.com/unitreerobotics/R1-7A/blob/e2faf3ec17f589db2139c51b020592a4a05f836b/teleop/r1arm_real_controller.py#L531-L561)。

Franka 将命令平滑与导数限幅分开，并要求先设计平滑的轨迹生成器。滤波不能代替运动学限制。其默认 100 Hz 低通运行在 1 kHz 控制接口；本软件名义 100 Hz，截止频率必须按实际采样和延迟重新设计。[Franka Signal Processing](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html#signal-processing)。

MoveIt Servo 对比了三种处理：Butterworth 低通可偏离笛卡尔路径，且不显式限制加速度与 jerk；加速度限制不保证 jerk；Ruckig 平滑也可能改变方向或过冲。因此，直接在 IK 后逐轴滤波不适合作为保持 TCP 旋转中心的默认补丁。[MoveIt Signal Smoothing](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html#signal-smoothing)。

UR 的伺服文档指出，高增益叠加噪声或低频目标更新会引发振动。这支持先检查指令质量再调增益，但其 gain、lookahead 数值不能直接用于宇树。[UR servoj](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-scriptmanual/G5/servoj_qavt0-008lookahead_time.htm)。

## 3. Ruckig：在线生成有界 jerk 的参考轨迹

**文献支持：**Ruckig 支持非零初始及目标速度、加速度，按速度、加速度、jerk 约束生成多自由度同步轨迹；目标仍须满足可达条件。其分段常值 jerk 使加速度连续，并不要求 jerk 连续；时间最优针对运动学约束，不包含机械臂力矩、碰撞或结构柔性模型。[RSS 2021 原论文，§III–IV](https://www.roboticsproceedings.org/rss17/p015.pdf)

Position 接口指定目标位置、速度、加速度；Velocity 接口求目标速度、加速度，论文明确其忽略目标位置及速度界，不能依赖它自动完成关节位置保护或速度限幅。[原论文，§V](https://www.roboticsproceedings.org/rss17/p015.pdf#page=6)

在线使用需保留并逐周期传递输出的完整位置、速度、加速度；输入或目标改变时重算。持续把移动信号当目标会产生追赶滞后，上游预测式 Tracking 接口属于 Pro。[官方仓库：更新循环与 Tracking](https://github.com/pantor/ruckig#tracking-interface)

单目标状态转移可本地计算；Community 的中间路点功能依赖云 API，Pro 才提供本地路点求解。不能将两类接口的实时性混为一谈。[官方仓库：路点限制](https://github.com/pantor/ruckig#intermediate-waypoints)

**适用判断（推断）：**可研究点动启动、松键、换向时的参考连续性，切换时需要沿用完整运动状态。有界 jerk 限制的是参考轨迹的运动学变化，不能据此保证结构残振消失。

## 4. 纯 Rust 实现与评估范围

| 原仓库                                                      | 版本、实质维护与许可                                                                                                                                                                                                                                                             | 功能及适用判断                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| ----------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [petrikosk/rsruckig](https://github.com/petrikosk/rsruckig) | [3.0.1](https://crates.io/crates/rsruckig/3.0.1)，2026-08-29 发布；同日[修复跟踪器跨轨迹末端采样并补回归测试](https://github.com/petrikosk/rsruckig/commit/36559078b64542bd750ad710fce59f017834e25a)；MIT。                                                                      | [库清单](https://github.com/petrikosk/rsruckig/blob/36559078b64542bd750ad710fce59f017834e25a/lib/Cargo.toml)和源码显示核心为纯 Rust，无 C++/FFI 构建；[泛型 DOF 与同步选项](https://docs.rs/rsruckig/3.0.1/rsruckig/ruckig/struct.Ruckig.html)可表达 7DOF，[速度模式示例](https://github.com/petrikosk/rsruckig/blob/36559078b64542bd750ad710fce59f017834e25a/samples/src/example_velocity_1dof.rs)确实存在。列为优先评估候选，尚未验证本机 7DOF 行为。 |
| [marcbone/s_curve](https://github.com/marcbone/s_curve)     | [0.1.7](https://crates.io/crates/s_curve/0.1.7)，2023-05-02 发布；最近算法修复为 [2023-02-05 反向运动修复](https://github.com/marcbone/s_curve/commit/ef39ed13ace97543c9082f5aafda5b7b64ea6da1)；[MIT 或 Apache-2.0](https://github.com/marcbone/s_curve/blob/main/Cargo.toml)。 | 核心纯 Rust、无运行依赖；[接口](https://docs.rs/s_curve/0.1.7/s_curve/struct.SCurveStartConditions.html)仅单轴起终位置、速度，没有初末加速度、多轴同步或独立在线速度接口。可作离线曲线参考，不宜承担本项目点动。                                                                                                                                                                                                                                        |

注意上游官方 Rust 接口使用 [`cxx`/`cxx-build`](https://github.com/pantor/ruckig/blob/main/Cargo.toml)，不满足纯 Rust 要求。rsruckig [自行声明](https://github.com/petrikosk/rsruckig#development)位置限制、Tracking 和本地路点是独立扩展；它们不等于官方 Pro 实现，路点求解也不保证时间最优。上游论文的大规模随机验证不能转移为移植库的验证结论。

**评估建议：**先离线核对 7DOF、非零初始加速度、持续改目标、松键和反向、限位及错误返回，并测 macOS 下重规划耗时；与固定上游版本对照后，再决定是否引入。本次未编译或运行候选库。

## 5. ZV/ZVD 与 notch：先识别结构模态

Singer、Seering 的 [1988 年原始报告](https://dspace.mit.edu/entities/publication/a4b1d339-171a-4641-80c4-85d8c48f166b)研究通过改变输入减少残振；[原发明说明，§0020–0022](https://data.epo.org/publication-server/rest/v1.2/publication-dates/19961023/patents/EP0433375NWB1/document.html)给出指令与脉冲序列卷积、增加残振导数约束以提高稳健性的原理。[MAL 厂商文档，§1.4.2](https://www.malinc.com/wp-content/uploads/2014/12/VCNC_Help_File.pdf#page=19)明确整形需要模态频率和阻尼，比较如下：

| 方法       | 所需信息与代价                                                                                                                                                                                                                 |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| ZV         | 需要模态频率、阻尼；附加完成时间约半个振动周期，对频率误差较敏感。                                                                                                                                                             |
| ZVD        | 同样需要模态参数；对频率偏差更稳健，附加完成时间约一个振动周期，轨迹失真更大。                                                                                                                                                 |
| notch 陷波 | 需要共振峰中心、宽度及所需衰减，并检查相位；窄带滤波会改变相位，中心频率须接近实际共振。[Rockwell 控制文档，p.66](https://literature.rockwellautomation.com/idc/groups/literature/documents/at/motion-at005_-en-p.pdf#page=66) |

[Beckhoff](https://infosys.beckhoff.com/content/1033/ax5000_function_doku_hw2/3245596043.html)明确共振频率可能随位置、时间变化。**本项目推断：**姿态、负载或安装条件改变后，应重新核对频率和阻尼；目前没有模态数据，不能指定滤波频率。ZV/ZVD 的时间代价会影响点动停稳与换向；驱动内部 notch 的效果也不能直接套用于主机目标滤波。滤波选择与效果取决于实测频谱、衰减和相位。

## 6. 建议实施顺序

以下是结合当前实现的工程判断，不是已经完成的控制改动。

| 顺序 | 处理                                                                                                                      | 如何判断有效                                                                                              |
| ---- | ------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| 1    | 增加 10–20 秒短时原始采样：七轴目标/反馈位置、目标/反馈速度、加速度估计、前馈与反馈扭矩、反馈年龄、控制和发送时间、帧序号 | 分清目标本身不连续、跟随振荡、反馈噪声与中途取消；分别观察启动、匀速、松键和静止保持。                    |
| 2    | 评估统一的在线限 jerk 轨迹处理；七轴状态和时间使用同一套规则，停止时沿用当前状态制动                                      | 在周期抖动、反向、短步长及取消测试中，最终位置差分与下发速度一致，速度/加速度上限保持有效；停止距离可测。 |
| 3    | 对末端输入的速度做渐入渐出，并复查最终七轴动态、TCP 漂移与姿态误差                                                        | 在相同姿态、工具偏移和速度下，对比平滑前后；不能以降低抖动换来未知的旋转中心漂移。                        |
| 4    | 指令稳定后，若静止保持仍振荡，再逐轴核对增益、速度反馈质量、零位及重力补偿                                                | 对比同一固定目标下的跟随误差、振荡幅度和扭矩；不直接套其他机器人参数。                                    |
| 5    | 有明确噪声或共振证据时再选择低通、ZV/ZVD 或 notch                                                                         | 同时检查抑制幅度、附加延迟、停止响应，以及不同姿态/负载下是否仍有效。                                     |

采样通过有界队列交给后台批量写入 SQLite，记录原始数据和派生数据，避免在控制线程逐帧格式化长文本、打印或同步写磁盘。每秒诊断继续保留，用于总览；频谱分析使用带原始时间戳的短时曲线。主机写调用时刻不是设备实际执行时刻，USB 缓冲也应纳入解释。

保持纯 Rust 时，rsruckig 是值得验证的候选。范围仅为单轴定速点动时，也可以独立实现较小的限 jerk 速度生成器，但要完整处理制动、反向、极短步长和约束冲突，不能只给加速度简单套一次低通。选择哪一种应由离线对照结果决定，可以保留当前架构和运动学实现。

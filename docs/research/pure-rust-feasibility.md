# R1-7a 纯 Rust 后端可行性与替换工作评估

日期：2026-11-09，Asia/Shanghai。范围：资料、源码核验与工程估算。本轮未编写控制器、协议实现或验证程序，未创建 Cargo 项目，未连接机械臂。

选型更新（2026-11-09）：采用 serialport + nalgebra + urdf-rs 实现当前七轴的局部点动，Caliper 作为现成库备选。本文保留早期的库比较和人工工时估算；一天内的开发安排见[开发计划](day-one-plan.md)。当前在 macOS arm64 进行软件测试，真机测试单独记录。

本评估中的“纯 Rust”指机械臂后端、通信实现和运动学不依赖官方 C++ SDK、C++ FFI 或 ROS。Tauri 的系统 WebView、操作系统串口调用，以及前端 TypeScript/Three.js 不属于被替换的机器人 SDK。

用户已确定：桌面程序所在电脑直连；三端均为目标，验证后决定首发；六维连续/步长点动，基座/工具坐标；构型连续；本机 API；服务随桌面启停；单一控制权；不为无窗口模式、局域网访问或未知设备预留框架。

## 上一轮综合判断（当前优先路线见上方更新）

**纯 Rust 在库和协议资料层面可行，建议作为当前优先验证路线。** 初始候选组合是 `serialport 4.10.1`、`k 0.32.0` 及其兼容数学类型；本轮不锁依赖、不启动实现。最大的项目工作是把这些基础能力组成有界、连续、状态可信的控制循环，而不是寻找串口 API。

现成库覆盖了操作系统串口、URDF、FK、公开 Jacobian 与七轴数值 IK，但没有在本次核查候选中找到可直接替代独立 R1-7a SDK 并完成本需求的纯 Rust 成品。协议和当前功能的 SDK 替换与单平台联调粗估 7–14 人日；其中明确因 Rust 新增的迁移/等价对照约 3–6 人日。笛卡尔控制另估，不能把它算作全由换语言导致的成本。

库方面的主要保留意见是 k 稳定版较旧；硬件方面的主要未知是三端真实 6 Mbps；控制方面的主要未知是当前七轴模型在限位/奇异附近的连续性与计算时延。三项均需进入实现验证后才能转为已支持结论。

## 串口库评估结论

当前优先候选为 **`serialport 4.10.1` 加同步专用控制线程**。该库已分别实现 Windows DCB、macOS `IOSSIOSPEED`、常见 Linux 架构的 `termios2/BOTHER`。因此不必为当前官方 SDK 缺少 macOS 非标准波特率路径而先自行封装系统 FFI；仍需验证厂商转接器和驱动的真实 6 Mbps 能力。[平台与构建说明](https://github.com/serialport/serialport-rs/blob/main/README.md)、[参数实现](https://docs.rs/serialport/4.10.1/src/serialport/posix/termios.rs.html#77-128)、[Windows 参数实现](https://docs.rs/crate/serialport/4.10.1/source/src/windows/dcb.rs)

| 候选 | 已核实版本 | 本项目判断 |
|---|---|---|
| serialport | 4.10.1，2026-09-08 发布 | 首选，现有同步 API 和 OS 独占能力适合单设备控制者 |
| serial2 | 0.2.39，2026-10-26 发布 | 可用对照，独立读/写超时与依赖较小，但 Unix 默认打开缺少同等进程间独占 |
| tokio-serial | 5.5.0，2026-06-15 发布 | 不作为首版首选；不会增加底层硬件速率能力，且要单独处理异步超时与取消 |

版本与许可来自发布元数据及专项报告。`serialport` 是 MPL-2.0，`serial2` 为 BSD-2-Clause 或 Apache-2.0；上层 tokio-serial 为 MIT，不改变其 serialport 依赖的许可。serialport 的 Linux glibc 默认依赖 libudev；可关闭该 feature，但串口枚举信息会减少，不能据此把“纯 Rust 后端”写成“没有任何系统库”。[serialport 元数据](https://crates.io/api/v1/crates/serialport)、[serial2 元数据](https://crates.io/api/v1/crates/serial2)、[tokio-serial 依赖](https://crates.io/api/v1/crates/tokio-serial/5.5.0/dependencies)

超时和关闭的两个重要差异：同步库的单次 I/O 超时不是整个协议事务的截止时间；tokio-serial 的 `set_timeout` 不执行同步串口超时设置，需要异步 deadline。`flush` 也不是机械臂执行确认，不应将其写成停机完成条件。[serialport API](https://docs.rs/serialport/4.10.1/serialport/trait.SerialPort.html)、[tokio-serial 发布实现](https://docs.rs/tokio-serial/5.5.0/src/tokio_serial/lib.rs.html)

## 运动学库评估结论

**`k 0.32.0` 是首个值得验证的通用七轴候选。** 它提供 URDF 建链、FK、公开的 `k::jacobian()`，以及 `JacobianIkSolver` 的数值 IK；冗余分支对 7 个关节和 6 维目标使用 SVD/伪逆。不是仅能调用内部 IK 的黑盒，也不要求 OPW 六轴几何。[公开 Jacobian](https://docs.rs/k/0.32.0/k/fn.jacobian.html)、[求解器 API](https://docs.rs/k/0.32.0/k/struct.JacobianIkSolver.html)、[冗余求解实现](https://github.com/openrr/k/blob/v0.32.0/src/ik.rs#L221-L372)

| 能力 | k 已提供 | 集成时仍需处理 |
|---|---|---|
| FK 与 6×7 Jacobian | 串链变换和公开 Jacobian 函数 | 当前关节名称/顺序、基座与 TCP 对齐 |
| 完整位姿 IK | 读取模型当前关节状态作为 seed，支持七轴冗余分支 | 可达性、连续性、迭代预算与结果验收 |
| 关节位置限位 | 从 URDF lower/upper 构造位置范围，迭代中 clamp | 速度/加速度、限位裕量，以及反馈与目标的误差约束 |
| 冗余偏好 | nullspace callback 和示例 helper | 适用于本臂的构型偏好与权重核对 |
| TCP | 可用固定节点或末端目标变换表达 | 实际工具变换，以及 Jacobian 作用点的一致性 |
| 近奇异行为 | 可获取 Jacobian 并进行数值分解 | 速度缩放、阻尼策略与停止/恢复规则 |

IK 的当前状态 seed 只是局部求解起点，不保证不会构型跳变。内置位置 clamp 不约束速度或加速度，固定 SVD 截断也不是自适应阻尼或奇异降速控制。正常 Result 错误路径有恢复逻辑，但内部仍有 `unwrap`；输入有限值、维数、参数与输出验收要由控制模块处理。[求解及恢复实现](https://github.com/openrr/k/blob/v0.32.0/src/ik.rs)、[Chain API](https://docs.rs/k/0.32.0/k/struct.Chain.html)

**维护性限制必须一起接受或另行解决。** k 当前稳定版发布于 2024-09-10，主分支本次可见的最近提交为 2026-02-09 的开发依赖更新。其 manifest 要求 `nalgebra ^0.30`、`urdf-rs ^0.9`，与调查时最新 0.35/0.10 不是可互换的类型。首版可优先使用 `k::nalgebra`；确需直接读取模型元数据时，再使用兼容的 `urdf-rs 0.9`，避免同时引入两套不同版本数学类型。[已发布 manifest](https://docs.rs/crate/k/0.32.0/source/Cargo.toml)、[发布元数据](https://crates.io/api/v1/crates/k)、[本次可见提交](https://github.com/openrr/k/commit/244364955475f84715680d51fa95bfa050bacb60)

许可也按实际依赖版本记录：k 与 urdf-rs 0.9 为 Apache-2.0，k 所用 nalgebra 0.30.1 为 BSD-3-Clause；不能套用最新版 nalgebra 的许可。k 核心不要求 C++ 求解器，`kiss3d` 属于开发示例依赖；plain URDF 读取不需要 ROS/xacro 外部工具路径。[兼容版本元数据](https://crates.io/api/v1/crates/nalgebra/0.30.1)、[k manifest](https://docs.rs/crate/k/0.32.0/source/Cargo.toml)

另外两个候选应区分用途：

- [`rs-opw-kinematics 3.0.0`](https://docs.rs/rs-opw-kinematics/3.0.0/rs_opw_kinematics/)面向特定 OPW 机构，核心关节类型为六元数组，并有 5DOF 模式；不作为本机通用 7R 主候选。
- [`rsruckig 3.0.1`](https://docs.rs/crate/rsruckig/3.0.1)是纯 Rust 轨迹生成移植，可处理速度/加速度/jerk 约束，但不能替代 TCP 路径和 IK。它不是首版必选；名称为 `ruckig` 的另一个 crate 使用 C++ 桥接，不能混称纯 Rust。

最小候选组合因而是 `serialport + k`，用 k 的数学类型与传递 URDF 依赖完成模型接入。`crc`、直接 urdf-rs 或轨迹生成库只在选定的当前实现确实需要时加入，不同时搭建多套实现。

## 官方 SDK 实际要替换什么

基准为官方 [`r1-7a_sdk` commit 0803d033b530fcfa0e6fa9236de68c2773fc9402](https://github.com/unitreerobotics/r1-7a_sdk/commit/0803d033b530fcfa0e6fa9236de68c2773fc9402)。现有库与该 SDK 的关系是“串口工具＋本机协议实现”，而不是替换一套包含运动规划的完整机器人系统。

| 能力 | 现有官方实现 | Rust 方案的工作 |
|---|---|---|
| 打开与配置串口 | `SerialPort`，平台串口分支 | 使用现成串口库，核对所需参数及关闭语义 |
| 命令编码、反馈解码 | `MotorProtocol` | 按实际实现迁移字段、定点缩放、符号、减速比与范围处理 |
| CRC 与字节流恢复 | 特定 CRC 输入顺序、不同收发覆盖区间、定长反馈包缓冲 | 建立协议专用编解码和恢复逻辑，以官方输出与实际反馈作对照 |
| 七关节总线与标定 | `ArmMotorBus` | 关节 ID 映射、方向/比例/零偏、反馈聚合、命令顺序与收发节奏 |
| 电机状态/停机/清错 | 总线和例程中的模式/超时处理 | 为当前调试功能实现对应行为，并补集成层的新鲜度和错误结果 |
| 笛卡尔点动 | 官方该版本未提供 | 本项目新增，与是否采用 Rust 无关 |

命令/反馈格式、缩放与 CRC 的直接依据为 [MotorProtocol.cpp](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp)；总线、映射与收发节奏依据为 [arm_motor_bus.cpp](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp)。

SDK 的实际主链是 `ArmMotorBus::step → MotorProtocol::buildControlPacket → SerialPort::write`，没有经过单电机 `MotorController` 包装。首版替换当前所需总线功能即可，不需要复刻整个公开 C++ 类层次或全部命令行示例。[总线发送入口](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L275-L308)

源码体量提供一个粗略边界：`arm_motor_bus.cpp` 约 381 行，`MotorProtocol.cpp` 约 487 行，`SerialPort.cpp` 约 528 行，均包含注释和辅助逻辑。串口平台实现大部分由 crate 承担；这些数字不等于需要重写的 Rust 行数，也不代表验证容易。

## 协议迁移中不能省掉的核对

1. **数值与标定**：保留真实公式与减速比、整数宽度、舍入及钳制语义。SDK 的注释不能替代实际表达式；界面和运动学统一采用 URDF 关节坐标，映射只执行一次。
2. **CRC**：官方实现按四字节分组逆序送入 CRC，收发计算区间不同。通用 `crc32fast::hash` 是 IEEE CRC32，不能直接当作协议等价实现；[`crc`](https://docs.rs/crc/3.4.0/crc/)支持选择或配置算法，但仍需正确整理输入并验证相同输出。[官方 CRC 实现](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L373-L395)、[crc32fast 的算法说明](https://docs.rs/crc32fast/latest/crc32fast/)
3. **字节流**：串口一次读取可能是半包、多包或带噪声的数据；按协议组帧、校验并恢复同步。官方代码已有固定长度缓冲处理，Rust 仍需协议层承担这部分。[官方组帧逻辑](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/SerialPort.cpp#L482-L528)
4. **半双工节奏**：当前 SDK 在每个关节写入后留有 200 μs 间隔；移植需要理解实际发送与反馈时序，不把它简化成连续写七个包，也不直接承诺官网标称频率。[step 实现](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L275-L308)
5. **新鲜度与失败结果**：SDK 的 `feedbackReady` 记录收到过反馈，不能证明现在仍然有效。集成层需要逐关节收包时间、会话清理、反馈过期和错误统计。[反馈处理](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L69-L94)

校准样例有左右臂差异；真实工具 TCP 与用户设备零位仍需配置和核对。[右臂标定](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/config/joint_calib_right.txt)、[左臂标定](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/config/joint_calib_left.txt)

## 看起来相关、但不能直接替代的项目

- 官方 [`unitree_sdk2_rs`](https://github.com/unitreerobotics/unitree_sdk2_rs)的 README 明确是通过 `cxx` 包装 SDK2，使用 DDS。它既不是这条 IM6014 串口链，也不满足本评估的无 C++ 后端条件。
- [`motorcom 1.1.1`](https://docs.rs/motorcom/1.1.1/motorcom/)声明的 Unitree 支持是 GO-M8010-6、4 Mbps。当前文档没有给出 IM6014/R1-7a 支持证据，不能按“支持宇树”直接复用。
- 官方 [Unitree Motor Assistant](https://github.com/unitreerobotics/Unitree-Motor-Assistant)确实列出 IM6014，但当前 README 定位为 Windows 调试程序，分发 Qt 程序；它不是文档中可直接依赖的跨平台 Rust 机械臂 crate。

以上是对这些特定项目的排除，不表示已经证明全网不存在可用实现。

## 笛卡尔点动是独立的一项新增工作

URDF/FK/Jacobian/数值 IK 是算法基础。当前需求还包含基座/工具速度变换、真实 TCP、连续解、七轴冗余偏好、速度与加速度约束、奇异处降速、不可达退出、固定步长运动段，以及停止和反馈过期行为。

静态位姿 IK 能否收敛，与连续点动在控制周期内是否稳定，是两种不同的验收。即使使用现成 IK，也需要验证迭代上限、最坏计算时间、失败时关节状态，以及关节解连续性。该工作在 C++ 官方 SDK 方案中同样存在。[已核实的官方接口边界](r1-7a-sdk.md)

## 推荐的首版组织方式

在 Tauri Rust 后端内完成本机 HTTP/WebSocket 与控制业务。用一个控制模块拥有设备、实测状态、控制权和运动循环；网络入口提交意图、订阅快照。串口 I/O 与控制计算放在受控的专用线程，使用有界数据交接，避免 WebView、网络请求或普通 async 定时任务决定电机节奏。

按当前职责，可用 `arm`、`protocol`、`kinematics`、`server` 几个模块组织；它们属于同一个应用，不建立插件系统、通用传输框架或额外 workspace 包。这里仅描述职责，不创建这些代码文件。

关闭桌面时，停止接收新运动、结束当前运动、收回控制权并结束后端。具体设备退出动作须经实机确认，进程结束不能代替设备状态证明。

## 工作量估算

以下是工程判断，未经过本项目原型验证，不能作为交付承诺。按一名熟悉 Rust、串口及基本机器人学的工程师估算；假设机械臂、厂商转接器与目标电脑可用，固件/协议与所读 SDK 一致。

| 工作 | 粗估人日 | 与语言选择的关系 |
|---|---:|---|
| 协议、CRC、物理量与标定迁移 | 2–4 | Rust 替换 SDK 的主要新增工作 |
| 总线收发、线程与反馈聚合、错误及退出 | 2–4 | Rust 集成工作；产品新鲜度/退出约束在 C++ 方案也需要 |
| 官方数据对照与协议等价验证 | 1–2 | Rust 替换新增，验证计划尚未实施 |
| 一个平台的七关节反馈与受控运动联调 | 2–4 | 两种语言都需要实机验证 |
| **当前功能所需 SDK 替换与单平台联调小计** | **7–14** | 不包括笛卡尔控制、完整 UI/3D 或完整 API 工作 |
| 另外两个平台的正常驱动/打包/串口验证 | 3–8 | 假定转接器驱动可满足要求；遇硬件/驱动限制时无可靠上界 |
| 六维连续/步长点动与约束、模型及实机验证 | 8–15 | C++ 与 Rust 都要补充，不应全计为 Rust 替换成本 |

其中较明确的 Rust 特有新增是协议/标定迁移及等价验证，合计约 3–6 人日；完整接入与稳定联调的时间更长。表中分项也不是完整控制器的总排期，最终 UI、接口范围与库适配结果会影响集成时间。数字假设可以采用已核查的兼容运动学版本；若必须维护 k 分支或替换求解器，需按实际缺口重新估算。

## 不写代码阶段可形成的结论边界

可以判断：库是否公开了所需能力、平台 API 是否有实现路径、协议替换范围、需要自己承担的连续控制和验证工作。

不能据此判断：具体 USB 转接器在三端是否能持续 6 Mbps、七关节反馈时延与抖动、真实 URDF/TCP 误差，以及七轴连续点动的最坏时延和停止表现。这些是实现阶段的证据门槛，不在本轮运行验证。

## 资料与核验记录

- [串口库专项调查](rust-serial-libraries.md)与[运动学库专项调查](rust-kinematics-libraries.md)记录库版本、能力与一手来源。
- 官方 SDK 使用已存在的 `r1-7a-sdk-research` full 图索引，generation `2026-11-09T06:25:34Z`，本轮状态 ready。Tier 2 查询读取 `step`、协议编码/解码、CRC、标定函数和反馈对象；`step` 下游追踪返回 11 个结果，无后续页。
- 本轮已对依赖的源码与配置路径执行 `check_index_coverage`。`SerialPort.cpp:33` 的解析遗漏已直接补读；标定及 LICENSE 为 not_tracked，已直接读取。图索引的“无记录缺口”不是完整性证明。
- 根目录 [LICENSE](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/LICENSE)为 BSD-3-Clause；若迁移官方实现，保留对应版权和许可信息。

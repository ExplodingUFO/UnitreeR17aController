# R1-7a SDK：公开接口与直连平台核验

核验日期：2026-11-09（Asia/Shanghai）。仅采用官方 `unitreerobotics/r1-7a_sdk`、其文件及官方手册；尚未连接机械臂，未发送命令。本文是技术调查记录，不构成正式方案决策。

- 固定 commit：[`0803d033b530fcfa0e6fa9236de68c2773fc9402`](https://github.com/unitreerobotics/r1-7a_sdk/commit/0803d033b530fcfa0e6fa9236de68c2773fc9402)，提交时间 2026-11-03 08:12:50 UTC，提交说明 `fix (urdf): update urdf`。
- `VERSION` 仍为 `0.1.0`，其日期 2026-09-17 不能替代 commit 身份。[VERSION L1–9](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/VERSION#L1-L9)
- 本报告在项目实现前完成，使用隔离的临时目录核查源码与模型。临时目录不是本项目的运行或构建依赖；后续实现结果见[实施状态](../implementation-status.md)。

## 结论与范围

README 明确称其为 IM6014 serial bus 的低层 SDK。`--goto --q` 示例接收七个关节角，而 README 的 “target pose” 措辞不能证明 TCP 位姿 API；控制量 `q/dq/tau` 在 URDF 关节坐标下，经 `ArmMotorBus` 转换。[README L1–7、L61–80](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/README.md#L61-L80)

**在该 commit 的公开头文件和例程中，没有可直接调用的末端笛卡尔点动、TCP 速度/twist、TCP 位姿/MoveL/Cartesian servo 接口，也没有 IK、FK 或 Jacobian 求解器。** 已逐一核对 `include/arm_motor_bus.h` 和底层四个头文件，并用图符号搜索、`step()` 双向调用追踪、源码关键词检查交叉验证。结论仅限本仓库固定版本，不外推为厂商所有产品或未公开软件均不具备这些能力。

## 有限公开接口清单与语义

| 层次 | 实际公开接口 | 能力边界与一级来源 |
|---|---|---|
| 七关节命令/反馈 | `ArmJointCmd{q,dq,tau,kp,kd,mode,timed_out}`；`ArmJointState{q,dq,tau_est,mode,timeout,error_code}` | rad、rad/s、Nm，均为关节量；没有 xyz、姿态、TCP frame 或 twist 字段。[arm_motor_bus.h L8–46](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/include/arm_motor_bus.h#L8-L46) |
| 七关节总线 | `open/close/isOpen`、`setMotorId`、关节标定加载/设置/读取、`step`、`stop/clearFaults/resetMotors`、`states/motorStates`、`feedbackReady/allFeedbackReady` | `step(array<ArmJointCmd,7>)` 发七个关节命令；`states()` 是已标定的实测关节反馈快照。[L53–108](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/include/arm_motor_bus.h#L53-L108) |
| 单电机控制包装 | `initialize/disconnect/isConnected`、`setControlCommand`、异步控制频率/启停、`stopMotor/startMotor/zeroTorqueMode/setTorque/setDampingMode/setSpeed/setPosition/setMixedControl`、清错/复位、反馈回调/查询 | 每个控制调用均接收 `motor_id`；`setSpeed` 是该电机角速度控制，`setPosition` 是该电机角位置控制，`setMixedControl` 是该电机位置/速度/增益/前馈扭矩组合。[MotorController.h L32–77](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/include/MotorController.h#L32-L77)、[实现 L205–227](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorController.cpp#L205-L227) |
| 协议/串口 | `MotorProtocol` 提供控制/清错/复位包、反馈解析、错误/警告描述、减速比；`SerialPort` 提供打开/关闭/读写/flush/异步接收 | 均未提供机械臂运动学或 TCP 运动命令。[MotorProtocol.h L40–103](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/include/MotorProtocol.h#L40-L103)、[SerialPort.h L20–36](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/include/SerialPort.h#L20-L36) |
| 低层例程 | `--foc/--stop/--zero-torque/--clear/--reset/--home/--goto/--goto-hold`；`--q F0..F6` | `--goto` 对每个关节使用 `q_cmd=q_meas+clamp(q_target-q_meas,±lead)`，再发送位置 FOC；不保证 TCP 直线路径。[test_arm_lowlevel.cpp L45–70](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/test_arm_lowlevel.cpp#L45-L70)、[L373–424](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/test_arm_lowlevel.cpp#L373-L424) |
| 运动 demo | 例程内部 `MotionDemo::moveJoints(targets,...)` | 接收 `(joint,q)` 列表，对关节目标做余弦插值，并生成关节 `dq`；是例程内部逻辑，并非笛卡尔 SDK API。[demo_arm_motion.cpp L279–338](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/demo_arm_motion.cpp#L279-L338) |

`ArmMotorBus::step()` 的实际链为：七关节命令 → `urdfToMotorAngle/urdfToMotorVec` 标定换算 → `MotorProtocol::buildControlPacket` → `SerialPort::write`。它直接组合协议和串口对象，未经过 `MotorController` 包装，也没有求解 IK/FK。每关节发送后包含 200 μs sleep，因此循环频率不能由 UI 刷新率或某个 `--rate` 参数直接推导。[arm_motor_bus.cpp L50–54](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L50-L54)、[L275–308](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L275-L308)

首版 3D 可从 `states()[i].q` 驱动各关节节点，通过 URDF 关节树合成模型变换；这属于本地模型变换计算，不是调用了官方 FK API。反馈字段没有主机接收时间戳，`feedbackReady` 只表示该关节曾收到有效包；新鲜度须由集成层补充。[反馈更新 L69–94](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L69-L94)、[states/readiness L347–380](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L347-L380)

若后续需要笛卡尔点动，需要另行实现/引入几何模型与 TCP 定义、FK/Jacobian、姿态目标 IK 或 twist 到关节速度的求解，以及持续关节指令生成、限位/奇异位形/不可达处理；仅接上 `setSpeed` 或给 UI 增加 XYZ 按钮无法补齐这条链。这是根据上述接口边界得出的实现缺口。

该版本例程也存在 README/帮助文字与实际默认值差异：`test_arm_lowlevel` 的 `Options` 为 50 Hz、`/dev/ttyUSB0`；demo 为 100 Hz、`/dev/ttyUSB3`；两者解析器均要求显式选 `-r/-l`。接入时宜显式传端口、手性和频率。[test Options L26–38](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/test_arm_lowlevel.cpp#L26-L38)、[test 解析 L206–213](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/test_arm_lowlevel.cpp#L206-L213)、[demo Options L36–48](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/demo_arm_motion.cpp#L36-L48)、[demo 解析 L133–140](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/demo_arm_motion.cpp#L133-L140)

## macOS / Windows：直连电脑的关键事实

| 项目 | 已验证事实 | 对方案的含义 |
|---|---|---|
| 官方列明的平台 | Ubuntu 20.04+、aarch64/x86_64、GCC 9.4.0。[README L4–16](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/README.md#L4-L16) | 不能把 UI 跨平台等同于官方完整底层跨平台承诺。 |
| 库并非只有闭源二进制 | 顶层 `add_subdirectory`；底层从 `MotorController.cpp`、`MotorProtocol.cpp`、`SerialPort.cpp` 编译共享库。[顶层 CMake L16–18](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/CMakeLists.txt#L16-L18)、[底层 CMake L10–14](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/CMakeLists.txt#L10-L14) | 可以研究本机重新编译或移植串口后端，无需改成 Linux 网关。 |
| 随附二进制 | [`libmotor_controller.so.0.1.0`](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/lib/libmotor_controller.so.0.1.0) 本地 `file` 检查为 Linux x86-64 ELF 共享库；SHA-256 `f05b39dbdd2d22dcc0067d44ccedabeb3a92b120dc2adf971107ba3505131280`。 | 该文件不能直接作为 macOS dylib、Windows DLL 或 ARM64 库使用。 |
| Windows 源码路径 | `_WIN32` 使用 `CreateFileA` 打开 COM 口；`DCB.BaudRate=baudrate`，8N1，设置超时；读写使用 `ReadFile/WriteFile`。[SerialPort.cpp L53–69、L123–158](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/SerialPort.cpp#L123-L158)、[L260–321](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/SerialPort.cpp#L260-L321) | 已有实质性 Windows 后端，不应写成“仅 Linux 源码”；但本次没有 Windows 构建和厂商转接器实机结果。 |
| Windows DLL 导出 | 提供 `__declspec(dllexport/dllimport)`，CMake 定义共享库宏并在 Windows 链接 `ws2_32`。[导出头 L7–18](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/include/motor_controller_export.h#L7-L18)、[底层 CMake L22–31](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/CMakeLists.txt#L22-L31) | 说明源码有 DLL 构建安排；不证明已发布 Windows SDK 成品或稳定 C ABI。 |
| macOS 编译问题，已复现 | 当前 Darwin arm64 上 `clang++ -std=c++17 -fsyntax-only -I thirdparty/motor_controller/include thirdparty/motor_controller/src/SerialPort.cpp` 失败，L176–187 的 `B460800/B500000/.../B4000000` 等 12 个符号未定义。[SerialPort.cpp L167–191](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/SerialPort.cpp#L167-L191) | 官方该版本串口源码不能在本机原样编译；这是编译检查，未涉及实机能力判断。 |
| macOS 波特率问题 | 默认 6,000,000 不在 POSIX switch 中；非 Linux 分支对 `non_standard_baud` 直接抛出 `Non-standard baudrate requires Linux termios2 support`。[README L46](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/README.md#L46)、[SerialPort.cpp L215–248](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/SerialPort.cpp#L215-L248) | 仅修复未定义常量仍不够；需要 macOS 的串口配置实现，并在实际转接器/驱动上验证 6 Mbps。 |

## 串口协议：源码可读，但接口仍是单电机物理量

- 控制包 20 字节，头 `FE EE`；含 ID、mode、timeout、扭矩、角速度、角位置、Kp/Kd 及 CRC32。输出单位为 Nm、rad/s、rad；其 `speed` 不能解释为 TCP 的 m/s。[MotorProtocol.h L46–70](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/include/MotorProtocol.h#L46-L70)、[MotorProtocol.cpp L52–150](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L52-L150)
- 反馈包 26 字节，头 `FC EE`；含位置、速度、扭矩、温度、电压、mode/timeout、错误码和警告码；不含 TCP 坐标。解析会验证长度、包头与 CRC。[MotorProtocol.h L14–27](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/include/MotorProtocol.h#L14-L27)、[MotorProtocol.cpp L261–359](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L261-L359)
- CRC 是源码内实现的查表算法，初值 `FFFFFFFF`、每四字节逆序处理，并非可直接假定为任意语言标准库默认 CRC32；发送与反馈的 CRC 覆盖区间不同。[L145–148](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L145-L148)、[L303–308](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L303-L308)、[L362–394](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L362-L394)
- 电机 ID 0–1、2–4、5–6 的减速比分别为 32、38/3、16。移植时需要保留序列化比例转换。[L399–410](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L399-L410)

## URDF、mesh、标定资源

- [官方 URDF 压缩包](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/urdf/URDF_R1-7A.tar.gz)，SHA-256 `dca2cea68150891b5312108dd7c448103d0f0d67690fde0918cea23b41b65e25`。已解包并解析 XML：右臂为 `R1_Robotic_Arm_R_V1.2-1023/.../urdf/R1_Robotic_Arm.urdf`；左臂为 `R1_Robotic_Arm_L_V1.0-1027/.../urdf/R1_Robotic_Arm_L_V1.0-1027.urdf`。两者均有 `base_link`、`Link1`–`Link7`、7 个 revolute 关节、inertial/visual/collision 元素、各 link STL 和关节 limit；左包还含 meshes 目录内一份 URDF。
- 左右模型存在实际差异，例如 J6 右臂为 ±0.785 rad、左臂为 ±0.524 rad；不能假设仅镜像一份模型即可。右臂 URDF 使用相对 STL 文件名，左臂主要 URDF 使用 `package://.../meshes/...`，加载器需解析不同资源路径。未在这些 URDF 中发现单独命名的 TCP/tool 固定关节；`Link7` 到实际工具中心的变换仍需确认。
- 压缩包内容在 GitHub 中不能逐行引用：上述资源结论的可复核来源是固定 commit 下的压缩包、SHA-256 和内部路径，不能伪造仓库中不存在的解压文件网页行号。
- [STEP 资源](https://github.com/unitreerobotics/r1-7a_sdk/tree/0803d033b530fcfa0e6fa9236de68c2773fc9402/step) 为 `R1-7A机械臂_简化模型_0928.STEP`，它不能替代 URDF 关节树。
- [右臂标定 L1–9](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/config/joint_calib_right.txt#L1-L9)、[左臂标定 L1–9](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/config/joint_calib_left.txt#L1-L9) 包含七关节 `dir/scale/offset`，文件公式为 `urdf = dir * scale * motor + offset`。这些是仓库样例，尚未证明适配用户实际机械臂的零位。
- README 列出 `calibrate_joints`，但当前 [CMake L20–33](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/CMakeLists.txt#L20-L33) 已注释该目标；默认仅构建 `test_arm_lowlevel`、`demo_arm_motion`，不能承诺按 README 构建后即得到标定工具。

## 图索引与源码核查证据

- 项目 `r1-7a-sdk-research`，full 索引，generation/indexed_at `2026-11-09T06:25:34Z`，Tier 2。`index_status` 确认 ready。图名称检索 `.*(Arm|Cartesian|Kinematic|[Ii]nverse|[Ff]orward|[Jj]og|[Tt]wist|[Pp]ose|Jacobian|IK|FK|TCP).*` 返回 17/17、无更多页；结果均为关节总线相关符号/模块，未发现运动学或 Cartesian 接口。
- 对 `ArmMotorBus`、`step`、`MotorController::setSpeed`、`MotionDemo::moveJoints` 使用 `get_code_snippet`；`trace_path(step,both,depth=3)` 返回 13 个下游、17 个上游，无未读取分页。协议打包和串口写入的图边标为 heuristic，已以 `step()` 实际源码直接核实，不将调用图当作完整性证明。
- 全部五个公开头文件直接阅读；对 `include/`、`src/`、`thirdparty/motor_controller/include/`、`thirdparty/motor_controller/src/` 验证 `cartesian/kinematic/inverse/forward/jacobian/jog/twist/pose/MoveL/MoveJ/servo/IK/FK/TCP` 等文本边界，命中内容是关节例程与普通 pose 用语。
- `check_index_coverage` 对上述四个源码 scope 及本记录依赖的 21 个精确路径执行，所有结果分页均已返回。三个 partial 为 `MotorController.h:36`（析构声明）、`SerialPort.h:25`（析构声明）、`SerialPort.cpp:33`（Windows 构造初始化），均直接补读；其余已索引依赖文件没有记录缺口、freshness 为 metadata_match。
- `.so`、VERSION、左右标定文件、URDF 压缩包和 STEP 的 freshness 为 `not_tracked`；已直接读取/检查文件类型、校验和、解包 XML 或目录清单，不将其称作被图完整覆盖。索引未记录缺口只是一种尽力而为的信号；接口缺失结论结合全部公开头文件与有界源码验证得出。

## 实现缺口与未知事项

1. 笛卡尔求解/伺服链需要补充，不能列为官方现成功能。公开接口是 C++ 类和 STL 类型，未见稳定 C ABI；若由 Tauri 的 Rust 进程直接调用，需另做 FFI 包装，或通过独立 C++ 服务隔离。`ArmMotorBus::open` 在标定加载失败时会调用 `std::exit(1)`，嵌入桌面进程也需处理这一行为。[arm_motor_bus.cpp L118–127](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/src/arm_motor_bus.cpp#L118-L127)
2. Windows：源码分支存在；是否可原样用目标编译器构建、厂商 USB 转接器驱动能否 6 Mbps 稳定全关节收发，均未实测。
3. macOS：已证实原样构建存在编译错误和非 Linux 自定义波特率路径缺口；可行修复方案及转接器驱动能力仍需专项验证。无需以 Linux 远程网关作为跨平台直连方案的替代。
4. 首版“实测关节驱动 3D 模型”需要保留实测与目标值的区别、确认左右型号及标定，并解决 mesh 资源路径；它不要求已经具有 IK 或笛卡尔伺服。
5. 官方手册连接说明、手册 50 Hz/700 Hz 声明另见主代理的 `official-manual.md`；本记录不将标称频率转为跨平台实机承诺。

# 运动补偿开源仓库调研（2026-11-21）

本页的代码调查以 `799f759` 为基线。后续实现了可开关的平滑摩擦补偿和逐轴阻尼/速度前馈调试，配置与采样字段见[接口说明](../implementation-contract.md#59-伺服调试配置)。

建议优先验证运动时的逐轴伺服响应和摩擦补偿，再按测量结果选择完整动力学或共振抑制。可以保持纯 Rust：摩擦前馈与编码补偿适合直接实现小型计算函数，轨迹生成和陷波已有 Rust 库。现成库提供算法，关节摩擦、阻尼和模态参数仍需本体数据。

用户确认：已使用 `799f759` 或更新版本，运动明显抖动，静止保持基本稳定。安装条件延续此前信息：macOS、右臂、基座向上、无外接负载；此前关节点动与末端点动均抖动，低速更明显。本轮核对当前代码、官方实现与开源算法，只新增研究文档和忽略目录内的分析结果，没有改变运行参数或连接设备。

**优先验证：逐轴运动跟随、低速摩擦、速度前馈；测得稳定窄带峰后，再选择陷波或输入整形。** 这是实验顺序，不是已确认的根因。当前只有修复前的完整采样，尚缺新版发生抖动时的记录。

## 当前实现与采样证据

- 当前发送目标已按实际发送时刻取样并连续积分，关节和末端共用导数限制。上一轮 2.31% 指标描述离线指令一致性，不是实际机械振动幅度。
- 当前力矩前馈为指令姿态下的 URDF 重力项，逐轴限制 ±8 Nm；`kp=60/kd=10` 为统一值。当前正常发送路径没有摩擦补偿或完整的惯性/科里奥利前馈，也没有前馈扭矩变率限制。[arm.rs](../../src-tauri/src/arm.rs)、[发送取样](../../src-tauri/src/reference.rs)、[报文编码](../../src-tauri/src/protocol.rs)、[重力模型](../../src-tauri/src/kinematics.rs)。
- 官方遥操作同样发送目标速度，并在指令姿态计算重力前馈。因此，保留速度前馈符合官方路径；是否需要调整其权重，须用对照实验决定。[官方发送路径](https://github.com/unitreerobotics/R1-7A/blob/main/teleop/r1arm_real_controller.py#L531-L561)、[官方重力前馈](https://github.com/unitreerobotics/R1-7A/blob/main/teleop/r1arm_real_controller.py#L321-L336)。

按本项目固定的 SDK 协议与默认标定，速度步距为 `2π/(64r)`，扭矩步距为 `r/2560`：

| 关节 | 减速比 r | 速度步距 rad/s | 扭矩步距 Nm |
| --- | --- | --- | --- |
| J1–J2 | 32 | 0.003068 | 0.012500 |
| J3–J5 | 38/3 | 0.007751 | 0.004948 |
| J6–J7 | 16 | 0.006136 | 0.006250 |

来源：[官方编码与增益换算](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/thirdparty/motor_controller/src/MotorProtocol.cpp#L74-L126)。当前 SDK 默认分支为 `92f3251`，其 motor_controller 改为预编译库；上述可读源码以本项目固定提交为准。[当前 SDK 说明](https://github.com/unitreerobotics/r1-7a_sdk/blob/92f32514c6a9089c580ddf331eeb26075fcc5609/README.md)。

对旧采样 2.5–6.5 s 的 376 个发送批次重新计数：J2 的速度编码改变 5 次，J4 改变 0 次。速度量化确实存在，但这份记录不支持把持续抖动全部归因于反复跳档。旧记录还显示反馈在主机上分组到达，缺少设备采样时间；不宜按接收时刻直接差分，再据此选定陷波频率。

## 可直接采用的 Rust 库与控制参考

| 仓库 | 已核对的实现 | 对本项目的用途 | 采用限制 |
| --- | --- | --- | --- |
| [petrikosk/rsruckig](https://github.com/petrikosk/rsruckig) | Rust 轨迹生成；源码清单 3.0.1，MIT；本次 HEAD `3655907`，2026-08-29 | 与现有限 jerk 轨迹做离线对照，或承担路径进度的平滑计算 | 不是摩擦补偿器；七轴各自平滑后仍要验证 TCP 路径和旋转中心 |
| [korken89/biquad-rs](https://github.com/korken89/biquad-rs) | Rust、no_std；源码清单 0.6.0，MIT OR Apache-2.0；有 `Type::Notch` | 已确认窄带振动后的陷波实现 | 需要可靠采样率、中心频率和 Q；不自动辨识机械共振 |
| [frankarobotics/libfranka](https://github.com/frankarobotics/libfranka) | C++、Apache-2.0；阻抗示例有科里奥利前馈、逐轴阻尼和扭矩变率限制 | 借鉴前馈与扭矩变化约束的处理方式，在 Rust 中实现 | 示例是另一台机器人；其 1 ms 周期、增益和限值不能直接照搬 |

源码依据：[rsruckig README](https://github.com/petrikosk/rsruckig/blob/36559078b64542bd750ad710fce59f017834e25a/README.md)、[Rust 依赖清单](https://github.com/petrikosk/rsruckig/blob/36559078b64542bd750ad710fce59f017834e25a/lib/Cargo.toml)、[MIT](https://github.com/petrikosk/rsruckig/blob/36559078b64542bd750ad710fce59f017834e25a/LICENSE.txt)、[biquad 清单](https://github.com/korken89/biquad-rs/blob/master/Cargo.toml)、[陷波实现](https://github.com/korken89/biquad-rs/blob/master/src/coefficients.rs#L192)、[libfranka 阻抗示例](https://github.com/frankarobotics/libfranka/blob/e88d08f64cbf842b8f53c0b551963684064038d6/examples/joint_impedance_control.cpp#L180-L207)、[扭矩变率与周期定义](https://github.com/frankarobotics/libfranka/blob/e88d08f64cbf842b8f53c0b551963684064038d6/include/franka/rate_limiting.h#L20-L55)。源码清单版本不代表本轮已验证 crates.io 发布或完成本项目集成。

MoveIt Servo 的官方文档还说明，Butterworth 及 Ruckig 平滑可能改变笛卡尔方向；Ruckig 在转角处可能出现绕行。当前软件已有 TCP 路径约束，新增滤波或轨迹生成器仍应经过同一检查。[MoveIt Servo 信号平滑](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html#signal-smoothing)。

## 补偿与实验顺序

### 先做平滑摩擦前馈

简单基线可取 `τ_f(v)=F_c tanh(v/ε)+Bv`，与重力前馈相加。这里 `τ_f` 是克服摩擦的补偿量，其方向应与阻碍运动的摩擦力矩相反。七个关节及正反方向应分别标定；不直接套用仓库中的平滑尺度或别的机器人的参数。

参考速度可避免原始反馈在零速附近反复切换补偿符号；补偿必须平滑过零、限制幅值与变化率。若该基线在独立记录中仍不能解释低速或换向残差，再拟合 Stribeck；带内部状态的 LuGre 需要更多数据与状态管理。下面 BIRDy 的源码可以作为模型与离线辨识参照。

### 用对照实验区分前馈与阻尼问题

同一姿态、轴、方向和速度下，每次只改变一个因素：先比较运动速度前馈权重，再逐轴评估阻尼；观察跟随误差、位置残差、停止距离与可见抖动。统一增益与官方一致不等于适合每个姿态和关节，静止稳定也不能单独证明运动阻尼合适。

扭矩变率限制可参考 libfranka 的处理方式，但应使用实际发送间隔及本机械臂的限值。当前 ±8 Nm 是附加前馈通道的限制，不是电机总输出扭矩的上限。新增补偿不改变使能、停止、失联和收纳条件。

### 量化残差补偿是一个可离线核验的候选

若驱动的等效混合控制为 `τ=Kp(q_d−q)+Kd(v_d−v)+τ_ff`，可在真实编码后的 `v_wire` 上计算：

`Δτ_quant = Kd_wire · (v_ref − v_wire)`

将该残差加入扭矩前馈，可使 `τ_ff_wire + Kd_wire·v_wire` 更接近期望的等效前馈。这里必须使用实际编码后的 Kd、标定与扭矩量化，重新检查前馈幅值；速度和扭矩要在同一批次配对。单独平滑其中一个通道会破坏补偿关系。此式是根据控制模型的推导，不是已验证的 R1-7a 减抖功能；固件的通道语义、内部限幅和真实效果仍需对照。

旧记录的静态编码计算能减小等效前馈误差，但没有重新模拟机械臂响应；J4 未跳档这一事实也使其优先级低于运动跟随与摩擦辨识。本轮不据此修改默认速度前馈。

### 再按测量选择动力学或共振补偿

若误差主要随加速度和姿态变化，补齐惯量模型，再考虑 `M(q)q̈+C(q,q̇)q̇+g(q)`。当前模型运行数据包含质量/质心，尚未建立完整惯性矩阵与科里奥利计算，不能把现有重力函数直接当作完整逆动力学。

若存在稳定窄带峰，才选择陷波或输入整形，并检查不同姿态下的频率变化和停止收尾。当前记录的反馈接收分组也需要纳入频谱可信度判断。

建议下一份记录在同一姿态下分别包含静止保持、低速点动、常用速度、同速反向及停止，各场景保留独立采样。先看最终编码目标是否仍有不规则变化，再比较实测位置、速度和估计扭矩。拟合摩擦时，附加前馈命令不能代替总驱动力矩；反馈扭矩的标定和含义也需核对。补偿参数用一部分记录拟合，再在未参与拟合的方向、速度或姿态验证。

实现可集中在现有前馈计算和最终编码处：计算函数无 I/O，采样以固定大小数据提交到现有非等待队列，SQLite 仍由后台写入。第一阶段以可开关的平滑摩擦补偿、前馈对照和离线回放为范围。

## 摩擦、动力学及输入整形仓库详查

| 优先级与仓库 | 对应现象与可验证条件 | 已确认的用途 | 与纯 Rust 的关系 |
| --- | --- | --- | --- |
| 1. [TUM-ICS/BIRDy](https://github.com/TUM-ICS/BIRDy) | 低速、换向附近速度波动；同姿态、相近速度幅值的正反向力矩残差有可重复差异 | 刚体动力学及线性摩擦参数辨识；另有 Stribeck/LuGre 模型计算 | 可移植回归、平滑摩擦及状态模型；没有本次核实到的 Rust 实现 |
| 2. [nepfaff/robot_payload_id](https://github.com/nepfaff/robot_payload_id) | 残差随加速度、姿态或多轴运动明显变化，且有同步的关节位置、速度、加速度和总驱动力矩数据 | 带物理约束的惯性、反射惯量及动摩擦辨识；也有固定机械臂参数后的负载辨识 | Python/Drake 离线工具；可借用辨识流程，运行端只实现选定模型 |
| 3. [Klipper3d/klipper](https://github.com/Klipper3d/klipper) | 加减速或换向后出现近似固定频率的衰减振动，频率及其姿态依赖可测 | 面向机械共振的命令输入整形、加速度频谱分析与参数选择 | 可在 Rust 重写脉冲卷积思路；现有实现是 Python/C，集成于打印机运动系统 |

## 版本、日期与许可证

“最近提交”取默认分支 HEAD 的 **committer date（UTC）**，不使用仓库 `updated_at`、`pushed_at` 或文件更新时间。默认分支通过 GitHub 插件元数据核对；分支 HEAD 和日期通过 GitHub 官方只读 API 补齐。三个日期均不晚于 2026-11-21。

| 仓库 | 默认分支 | 本次 HEAD 与最近提交时间（UTC） | 运行语言/环境 | 源码许可证 |
| --- | --- | --- | --- | --- |
| BIRDy | `main` | [`4d9802f`](https://github.com/TUM-ICS/BIRDy/commit/4d9802f1d692815792dfb5652a3d19f1fcdaec02)，2024-05-09 17:43:00；[分支 API](https://api.github.com/repos/TUM-ICS/BIRDy/branches/main) | MATLAB；部分路径可用 C/MEX；物理一致性约束求解另依赖 CVX 等 | [MIT](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/LICENSE) |
| robot_payload_id | `main` | [`c52e31c`](https://github.com/nepfaff/robot_payload_id/commit/c52e31cf26c83b33aee5e56f805e1d4d710fd549)，2026-05-20 04:57:53；[分支 API](https://api.github.com/repos/nepfaff/robot_payload_id/branches/main) | Python 3.10、Drake、NumPy/SciPy 等；所查清单限定 Drake `<=1.36.0` | [MIT](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/LICENSE) |
| Klipper | `master` | [`c9559fa`](https://github.com/Klipper3d/klipper/commit/c9559fa04ec6b11fb1a9a1610903546319fcf310)，2026-11-20 15:45:15；[分支 API](https://api.github.com/repos/Klipper3d/klipper/branches/master) | Python 校准/配置，C 运动执行代码 | [GPLv3](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/COPYING)，所查整形源文件也明确标注 GPLv3 |

环境依据：[BIRDy README](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/README.md)、[robot_payload_id pyproject.toml](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/pyproject.toml)、[Klipper shaper_defs.py](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/klippy/extras/shaper_defs.py)。这些依赖声明不代表已在当前 macOS 或本项目工具链上验证可运行。

## 1. BIRDy：先参考线性摩擦辨识，再考虑非线性模型

**实际实现。** `IDIM_LS_identification.m` 从处理后的 `q/q̇/q̈/τ` 形成回归矩阵和力矩向量，实际包含 OLS、WLS、IRLS、TLS 以及物理一致性版本的分支，并计算矩阵条件数和最小奇异值。`LeastSquares.m` 有反斜杠、伪逆及 CVX/半定约束求解路径。因此这里确有参数辨识算法，而不只是动力学模型或 README 中的方法列表。[IDIM_LS_identification.m](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/Benchmark/Robot_Identification_Algorithms/IDIM_LS/IDIM_LS_identification.m)、[LeastSquares.m](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/Benchmark/Robot_Identification_Algorithms/Utils/LeastSquares.m)。

**输入、输出与标定。** 需要机器人几何/DH 描述、重力方向、模型初值及按时间对齐的数据。真实数据路径读取 `Qm/Taum`，可用位置差分得到速度和加速度，并处理滤波、裁边和降采样；力矩通道还涉及 `Ktau` 和 `Ktau_sign` 的换算。输出是可辨识的基础参数向量 `Beta_LS`、条件数及最小奇异值，并非每个物理参数都必然独立可辨识。仓库另有 Fourier 激励轨迹及约束优化代码；其用途是改善辨识条件，不能把激励轨迹生成当作减振轨迹整形。[getFilteredData.m](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/Benchmark/Robot_Identification_Algorithms/Utils/getFilteredData.m)、[generateExcitationTrajectory.m](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/Benchmark/Robot_Data_Generation/Trajectory_Data_Generation/generateExcitationTrajectory.m)、[模型与使用说明](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/README.md)。

**对低速现象有价值的模型。** `computeFrictionModel.m` 实现无摩擦、黏性、库仑、黏性+库仑、加偏置、Stribeck 和带状态的 LuGre 计算。其线性摩擦组合可概括为：

$$
\tau_f(v)=Bv+F_c\tanh(100v)+\tau_0.
$$

Stribeck 分支把库仑幅值换为 `Fc + (Fs−Fc) exp(−|v/Vs|^Es)`；LuGre 分支按 `Z ← Z + Ż·dt` 更新内部状态。这里的 `100` 是该源码采用的平滑尺度，不是本机械臂已经标定出的数值。[computeFrictionModel.m](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/Benchmark/Robot_Data_Generation/Dynamic_Model_Generation/Utils/computeFrictionModel.m)。

**关键边界。** `getStandardParameterVector.m` 明确把可与惯性参数同时线性辨识的摩擦项限定为库仑和黏性项。Stribeck/LuGre 分支的标准辨识向量没有纳入完整的 `Fs/Vs/Es/Σ0/Σ1/Σ2/Z0`。所以可以确认它们的模型计算存在，不能据此声称该线性辨识流程会自动求出完整 Stribeck/LuGre 参数。[getStandardParameterVector.m](https://github.com/TUM-ICS/BIRDy/blob/4d9802f1d692815792dfb5652a3d19f1fcdaec02/Utils/SymbolicModelData/getStandardParameterVector.m#L30)。

**采用判断。** 对当前背景，最值得先借鉴的是分方向低速数据、平滑库仑+黏性基线、回归条件数和独立轨迹验证；若基线残差仍呈现明确的速度衰减或历史依赖，再研究 Stribeck/LuGre。计算模型和小规模回归可用 Rust 实现；MATLAB、符号生成及 SDP 环节更适合作为离线参考。这是刚体与摩擦建模路线，不是专用于柔性共振的补偿器。

## 2. robot_payload_id：有完整动力学辨识，但不能只看“payload”名字

**实际实现。** 数值回归器通过 Drake 逆动力学和自动微分构造仿射模型；除每连杆的 10 个惯性参数，还可加入转子惯量或反射惯量（二者互斥）、黏性摩擦、动态干摩擦。SDP 求解器实现二次残差目标、可选正则化、伪惯性矩阵正定约束，以及摩擦和附加惯量非负约束。[data_matrix_numeric.py](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/data/data_matrix_numeric.py#L173)、[inertial_param_sdp.py](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/optimization/inertial_param_sdp.py#L60)。

辨识关系可简写为：

$$
\tau=W(q,\dot q,\ddot q)\theta+w_0,\qquad
\min_{\theta}\|W\theta-(\tau_{\rm meas}-w_0)\|^2+\lambda R(\theta).
$$

`w0` 是未作为待估变量的模型贡献，不能在仅辨识负载时忽略。回归器还有基于 SVD 的基础参数映射，用来去掉不可辨识方向。脚本确实将实测力矩减去 `w0` 后再求解，而非只用期望轨迹预测出的力矩做真实数据辨识。[回归及基础参数映射](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/data/data_matrix_numeric.py#L292)、[脚本中的实测力矩处理](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/scripts/solve_inertial_param_sdp.py#L674)。

**输入、输出与标定。** 数据结构明确要求 `joint_positions/velocities/accelerations/torques` 为 `(T,N)`，`sample_times_s` 为 `(T,)`；可从位置估算导数。还需要可计算逆动力学的机械臂模型、重力与坐标定义、可信的力矩数据和有足够激励的数据。输出包含求解状态、参数变量，脚本可保存参数字典 `.npy`。README 推荐优化激励轨迹及细调数据处理；支持先辨识无负载机械臂，再固定其参数辨识末端负载的流程。因此“所有机械臂动力学已知”是 payload-only 路线的前提，不是它全部辨识方法的前提。[JointData](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/utils/dataclasses.py#L28)、[激励与两阶段辨识说明](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/README.md)。

**与当前低速问题有关的限制。** 动态干摩擦项采用 `Fc·sign(q̇)`，并在 `|q̇|<0.001` 时置零；对转动关节通常对应 rad/s。所核实的这条回归路径没有静摩擦、Stribeck 或 LuGre 状态，不能把结果称为低速黏滑的完整辨识。`--payload_only` 优先于摩擦/反射惯量开关，只估末端连杆的 10 个惯性参数；无载背景下应先关注机械臂/摩擦辨识是否有数据依据，而不是直接使用 payload-only。[干摩擦阈值](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/data/data_matrix_numeric.py#L279)、[payload-only 参数选择](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/optimization/inertial_param_sdp.py#L142)。

**实现边界与 Rust。** 所查顶层求解脚本直接选择单连杆或 7 关节 iiwa 模型，不是传入任意 URDF 就完成任意机械臂适配的入口。其数据滤波调用 `sosfiltfilt`，是前后向、离线零相位处理；不能把它当作相同效果的因果在线滤波器。可借鉴“离线辨识→留出轨迹验证→导出参数→Rust 运行时计算”的分工；Drake/自动微分/SDP 无需因此被放进发送循环。这是刚体与驱动参数辨识，不是柔性模态辨识或运动整形。[固定模型入口](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/scripts/solve_inertial_param_sdp.py#L546)、[filtering.py](https://github.com/nepfaff/robot_payload_id/blob/c52e31cf26c83b33aee5e56f805e1d4d710fd549/robot_payload_id/utils/filtering.py#L11)。

## 3. Klipper：确认是机械共振后再参考输入整形

**实际实现。** `shaper_defs.py` 有 ZV、ZVD、MZV、EI、2-hump EI、3-hump EI 的脉冲定义；`kin_shaper.c` 将脉冲归一化并与运动位置轨迹卷积。其对象是命令对机械共振的激励，不是摩擦前馈或在线负载辨识。官方说明明确把输入整形用于减少打印机方向突变引起的 ringing。[shaper_defs.py](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/klippy/extras/shaper_defs.py)、[kin_shaper.c](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/klippy/chelper/kin_shaper.c)、[官方 Resonance Compensation](https://www.klipper3d.org/Resonance_Compensation.html)。

**输入、输出与标定。** 已知共振频率和选定阻尼比时，脉冲定义输出幅值 `A` 和时刻 `T`，执行端输出整形后的运动轨迹。自动校准读取时间戳与三轴加速度，使用 Welch PSD，比较候选频率和整形器的预计残余振动及平滑代价。它不是精确辨识出阻尼比：源码默认值为 `0.1`，并在若干测试阻尼比上评估结果。官方另有加速度计测量和离线数据处理说明。[shaper_calibrate.py](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/klippy/extras/shaper_calibrate.py#L138)、[官方 Measuring Resonances](https://www.klipper3d.org/Measuring_Resonances.html)。

用其 ZVD 定义即可说明可移植部分：令 `d=√(1−ζ²)`、`K=exp(−ζπ/d)`，则

$$
A=\frac{[1,2K,K^2]}{(1+K)^2},\quad
T=\left[0,\frac{1}{2f_nd},\frac{1}{f_nd}\right],\quad
u_s(t)=\sum_i A_i u(t-T_i).
$$

公式对应源码的脉冲定义及执行时归一化。因果实现需要历史缓冲和运动结束后的收尾；Klipper 还把脉冲按时间重心平移，使匀速运动保持对齐，并在执行端访问前后轨迹，因此依赖前瞻。其已平移方案不能直接视为零延迟在线滤波。[ZVD 定义](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/klippy/extras/shaper_defs.py#L30)、[归一化、平移与卷积](https://github.com/Klipper3d/klipper/blob/c9559fa04ec6b11fb1a9a1610903546319fcf310/klippy/chelper/kin_shaper.c#L30)。

**采用判断。** 只有在数据中找到可重复的模态频率，且能接受轨迹时序和形状变化时，才有较强参考价值。机械臂姿态改变可能改变模态及轴间耦合，需验证同一组参数的适用范围；打印机各轴参数不能直接移到 7 轴机械臂。现有 jerk 约束不等于已经消除了某个模态的激励，而单凭“低速更抖”也不足以选定整形频率。若后续采用整形，应对最终输出重新检查关节限值、速度/加速度/jerk、TCP 路径和收尾行为。可在纯 Rust 实现有限脉冲卷积；所查源码本身为 GPLv3 的 Python/C，实现复用需要按其许可证处理。

官方网页没有在本次可见内容中给出明确的更新日期，故其资料时间记为“2026-11-21 访问，网页更新日期不确定”；算法核对以表中固定提交的源码为准。

## 按测量结果选择方法

| 若现有记录能确认 | 值得优先借鉴 | 仍需补齐的关键证据 |
| --- | --- | --- |
| 低速正反向残差可重复，换向最明显，未见稳定窄带峰 | BIRDy 的线性摩擦回归、Stribeck/LuGre 模型及平滑过零方式 | 按姿态/方向/速度分组的残差、可信的总关节驱动力矩口径、过零附近行为；仅有附加前馈命令不能当作实测总力矩 |
| 加速度相关残差或多轴耦合突出，且同步力矩/状态数据质量足够 | robot_payload_id 的仿射回归、基础参数选择和物理约束 | 激励覆盖、导数质量、重力坐标、独立轨迹预测误差；不要仅看训练残差 |
| 加减速后有稳定的衰减振动，模态及姿态适用范围可测 | Klipper 的 ZV/ZVD/MZV 等整形和残余振动/平滑权衡 | 频率、阻尼范围、时延预算、输出路径和约束是否仍满足 |

以上是模型所需输入及源码行为的比较；这些仓库尚未接入本项目。BIRDy、robot_payload_id 和 Klipper 提供辨识或建模参照；可直接评估的 Rust 库是前文的 rsruckig 与 biquad。

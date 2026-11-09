# R1-7a 直接使用 nalgebra 的局部七轴运动学评估

调研日期：2026-11-09。本文只做在线官方 API、发布资料及定点源码核查和数学推导，没有编写或运行实现、探针、Cargo 命令、编译、示例或硬件操作。当前设备不可访问，后续软件验收环境为 Darwin arm64；跨平台是设计约束，不能据此声称其他平台已通过构建或真机验收。

## 结论与适用条件

**直接采用 nalgebra 加 urdf-rs，足以支撑当前固定 7R 串链的小范围连续与步长点动，且适合维护优先、高内聚、少模块的目标。** 可统一为一个受限的局部加权阻尼最小二乘（DLS）速度步：连续点动持续提供局部速度，步长点动生成一个邻近目标并通过同一个步进器逐步接近。首版不需要任意全局 goto-pose、解析七轴 IK、肘部单独调节、完整约束优化器或零空间姿态策略。这是结合本次需求作出的工程判断，不是上游对 R1-7a 的验证结论。

真正自写的范围有限：选定七关节链及校验、FK 与 TCP 几何 Jacobian、点动坐标语义、加权 DLS、关节增量及动态限幅、奇异与失败退出、seed 和模式状态管理。矩阵分解、四元数和刚体变换交给 nalgebra。**节省的是机器人框架和依赖集成负担，项目仍须承担模型、坐标约定及闭环行为的验证责任。** Modern Robotics 原作者明确说明数值 IK 对初值邻近性有要求，并指出前一关节解适合用作相邻目标的初值；这支持局部延续，不能推出全局可达性或全局收敛性。[原作者数值 IK 教程（上）](https://modernrobotics.northwestern.edu/nu-gm-book-resource/6-2-numerical-inverse-kinematics-part-1-of-2/)

本评估把“固定 7R、左右 URDF 限位不同、实际 TCP 待配置”作为主任务已核验的项目背景。没有重新探索本地代码结构、核对本地 URDF 数值或判定现有控制链路是否具备这些能力；这些具体事实引用主任务已有的官方材料证据，不代表本调查做过实机验证。

## 当前版本与依赖：可直接组合，但未做整仓解析验证

| 核查对象 | 当前一手证据 | 对本项目的含义 |
| --- | --- | --- |
| nalgebra | docs.rs 当前发布为 **0.35.0，2026-05-24**；发布清单为 Rust 2024、`rust-version = 1.89.0` | 后续采用该版本至少需要 Rust 1.89。不能沿用旧版 1.87 的要求，也不能假定本仓库工具链已满足。 |
| urdf-rs | 当前发布为 **0.10.0，2026-09-25**；官方 tag 清单为 Rust 2021，未声明包级 `rust-version` | 未声明 MSRV 不等于任何旧 Rust 都能编译；仍须检查实际工具链与传递依赖。 |
| 两者的直接依赖关系 | urdf-rs 0.10.0 的依赖为 XML、serde、regex、thiserror 等，**没有 nalgebra** | 两者之间不存在必须对齐的 nalgebra 次版本，也没有跨版本 `Isometry3` 类型交换问题。解析后的标量、三元组由项目转换为自己的 nalgebra 类型。 |

来源：[nalgebra 发布列表](https://docs.rs/crate/nalgebra/0.35.0)、[nalgebra 发布清单](https://docs.rs/crate/nalgebra/latest/source/Cargo.toml)、[nalgebra 0.35 更新说明](https://docs.rs/crate/nalgebra/latest/source/CHANGELOG.md)、[urdf-rs 当前发布](https://docs.rs/crate/urdf-rs/0.10.0)、[urdf-rs v0.10.0 官方清单](https://raw.githubusercontent.com/openrr/urdf-rs/v0.10.0/Cargo.toml)。

因此，“nalgebra 0.35.0 + urdf-rs 0.10.0”在发布清单层面没有直接版本冲突；这不是已完成 Cargo 解析、编译或整仓依赖一致性验证。现有其他依赖若暴露旧版 nalgebra 类型，仍可能产生重复版本与类型边界问题，本轮没有证据判断是否存在。后续可把内部数学类型统一在一个 nalgebra 版本上，URDF 边界只取数据，不增加兼容多版本的抽象层。

urdf-rs v0.10.0 的 GitHub 发布正文只有简短的发布记录，不能当成完整迁移指南。当前包说明补充了 URDF 1.1 的 `quat_xyzw` 转 RPY、1.2 的 acceleration/deceleration/jerk，以及 1.2 缺失 limit 可表示为无穷大等行为。**解析器接受的模型不等于本控制器可直接使用的模型**：本项目仍应要求七个受控转轴具有经确认的有限位置、速度及所需动态约束；缺失的动态参数应由可信设备配置补齐，不能把 infinity 当成允许无限运动。[v0.10.0 发布记录](https://github.com/openrr/urdf-rs/releases/tag/v0.10.0)、[当前包说明](https://docs.rs/crate/urdf-rs/0.10.0)、[v0.10.0 JointLimit 定义与版本注释](https://raw.githubusercontent.com/openrr/urdf-rs/v0.10.0/src/deserialize.rs)

调研中部分 docs.rs `/latest/` 页面缓存仍显示 urdf-rs 0.9.0；本文版本结论以 0.10.0 发布页及官方 v0.10.0 tag 为准，没有把旧 `JointLimit` 页面当作新版本 API。

## nalgebra 已提供什么

| 需求 | 当前 API 与边界 | 建议用途 |
| --- | --- | --- |
| 固定 6×7 Jacobian、6×6 任务空间矩阵 | `SMatrix<T, R, C>` 是静态列主序矩阵；可表达 `SMatrix<f64, 6, 7>`、`SMatrix<f64, 6, 6>`；元素索引不受常用小尺寸别名限制 | 固定尺寸足够，不必为七轴引入动态维度框架。注意从外部数组装配时的行列顺序。 |
| FK、TCP 刚体变换 | `Isometry3<f64>` 使用平移与单位四元数；通用 `Isometry` 提供 `from_parts`、`inverse`、`transform_point`、`transform_vector` 与复合运算 | 点、方向和变换分别使用相应操作，不能用向量变换代替点变换。`Isometry3::new` 的第二参数是轴角向量，**不是 RPY**。 |
| 旋转、局部姿态误差 | `UnitQuaternion` 提供 `from_euler_angles`、`from_axis_angle`、`from_scaled_axis`、`scaled_axis`、`inverse`、`slerp` 等 | URDF RPY 明确转换；点动增量使用轴角/四元数。旋转误差用相对旋转的 `scaled_axis`，不对 Euler 三个数直接相减。 |
| 奇异值与伪逆 | `SVD::try_new(matrix, compute_u, compute_v, eps, max_niter)` 返回 `Option`；有 `singular_values`、`rank`、`solve`、`pseudo_inverse` | 有限迭代预算下检测奇异与失败；`max_niter = 0` 表示无迭代上限，不适合作为受限伺服策略。`solve`/`pseudo_inverse` 的 epsilon 是截断阈值，**不是 DLS 阻尼**。 |
| DLS 的 6×6 正定求解 | `Cholesky::new`/矩阵 `.cholesky()` 返回 `Option`；成功后 `.solve` 解线性方程 | 对正阻尼构造的正定系统求解，无须显式求逆。分解成功后仍校验输出有限性与残差。 |
| QR 备选 | `QR::new` 可分解一般矩阵，但当前 `QR::solve` 定义于方阵 `QR<T,D,D>`，返回 `Option` | 可以解 6×6 方程；不能把 6×7 Jacobian 的 `.qr().solve(...)` 当成现成欠定最小范数 IK。首版不用为此另写 QR 算法。 |

来源分别为：[SMatrix](https://docs.rs/nalgebra/0.35.0/nalgebra/base/type.SMatrix.html)、[Isometry3](https://docs.rs/nalgebra/latest/nalgebra/geometry/type.Isometry3.html)、[Isometry](https://docs.rs/nalgebra/latest/nalgebra/geometry/struct.Isometry.html)、[UnitQuaternion](https://docs.rs/nalgebra/latest/nalgebra/geometry/type.UnitQuaternion.html)、[SVD](https://docs.rs/nalgebra/latest/nalgebra/linalg/struct.SVD.html)、[Cholesky](https://docs.rs/nalgebra/latest/nalgebra/linalg/struct.Cholesky.html)、[QR](https://docs.rs/nalgebra/0.35.0/nalgebra/linalg/struct.QR.html)。

以上是文档与类型签名核查，没有用探针验证具体表达式能否在本仓库编译。nalgebra 官方提供纯 Rust 分解路径，本任务不需要为这些小矩阵添加 LAPACK/BLAS 或平台 FFI；这有利于保持数学核心的平台无关性，但不能替代 Darwin arm64 及其他目标的实际验证。[官方分解说明](https://www.nalgebra.rs/docs/user_guide/decompositions_and_lapack/)

## 有限的自写算法与坐标契约

### 1. 模型与 FK：固定七转轴，保留全部固定变换

约定 \({}^{B}T_T=(R,p)\) 把 TCP 坐标转换到基座 B。从明确的 base 到明确的 flange/TCP，按父子链接选出唯一串链，确认恰有七个受控 revolute 关节，并建立设备关节顺序与 URDF 名称的一一映射。不能把 XML 中 joint 的排列顺序当运动链顺序。固定关节可以折叠到相邻变换中，但不能遗漏。对本任务不支持的 mimic、闭链或其他关节类型应明确拒绝，不扩展成通用机器人加载器。

对每个关节保存父系到关节原点的固定变换 \(O_i\) 及关节局部轴 \(a_i\)，按顺序计算：

\[
{}^BT_T(q)=T_{B,0}\left[\prod_{i=1}^{7}O_i\,\mathrm{Rot}(a_i,q_i)\right]T_{7,T}.
\]

其中固定连接都已计入相关因子；RPY 旋转明确为 \(R_z(yaw)R_y(pitch)R_x(roll)\)，关节运动在该 origin 后复合，不能交换顺序。URDF 原点和 TCP 平移用米，关节角与轴角用弧度，UI 的毫米/度只在输入输出边界转换。轴需检查有限、非零和单位化规则。以上是本项目应采用并验证的串链递推；Modern Robotics 原作者的 FK/速度运动学建立在明确的刚体变换及关节轴约定上。[原作者 Space Jacobian 教程](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-1-1-space-jacobian/)、[nalgebra RPY API 约定](https://docs.rs/nalgebra/latest/nalgebra/geometry/type.UnitQuaternion.html#method.from_euler_angles)

左右臂分别读取实际七轴 limits，不通过“把角度取反”复制另一侧；机械零位、编码器方向和硬件序号映射另须确认。TCP 是 flange 到实际工具的**完整平移和旋转配置**，不能默认末关节原点就是接触/工作点。只有当实际 TCP 已确认与所选 URDF 末端重合时，单位变换才是有效配置。

### 2. geometric Jacobian：参考点必须是实际 TCP

在同一次 FK 遍历保存每轴原点 \(o_i\) 与单位轴 \(z_i\)，两者都表达在基座；对 TCP 位置 \(p\)，采用行顺序 **[TCP 线速度；角速度]**：

\[
J_i=\begin{bmatrix}z_i\times(p-o_i)\\z_i\end{bmatrix},\qquad
\begin{bmatrix}\dot p\\\omega_B\end{bmatrix}=J(q)\dot q.
\]

这是逐轴刚体旋转速度关系的直接推导，只需七列叉乘，不必维护第二套 DH 模型。**这里的基座表达 geometric Jacobian 不是直接照搬 Modern Robotics 的 space-twist Jacobian**：原作者常用 [角；线] 排序，space twist 的线性部分也不是一般情况下的 TCP \(\dot p\)。照抄矩阵或六维向量时必须显式转换，不能只换行顺序。[原作者 Space Jacobian](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-1-1-space-jacobian/)、[Body Jacobian](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-1-2-body-jacobian/)

若把同一个 TCP 点的速度分量改用工具轴表示，则使用 \(\operatorname{diag}(R^T,R^T)J\)；这里仅改变表达轴，不移动参考点。若从 flange Jacobian 改成 TCP Jacobian，还必须补上杠杆臂：\(v_T=v_F+\omega\times(p_T-p_F)\)。涉及不同点或真正空间/body twist 的变换时使用与其定义匹配的 adjoint，不能统一以“乘一个旋转”处理。

### 3. 基座/工具坐标点动：先规定操作语义

首版建议 XYZ 平移保持当前目标姿态；姿态点动围绕 TCP 本身旋转、保持当前目标 TCP 位置。以一步开始时的 \((R,p)\) 为例：

| 命令 | TCP 目标增量 |
| --- | --- |
| 基座 XYZ | \(p'=p+\delta p_B\)，\(R'=R\) |
| 工具 XYZ | \(p'=p+R\delta p_T\)，\(R'=R\) |
| 基座轴姿态 | \(R'=\exp([\delta\theta_B]_\times)R\)，\(p'=p\) |
| 工具轴姿态 | \(R'=R\exp([\delta\theta_T]_\times)\)，\(p'=p\) |

这是本项目点动语义的建议，不是对既有 UI 的核验。基座姿态点动若直接用一个纯旋转 Isometry 左乘整个 TCP 位姿，会同时绕基座原点转动位置，偏离上述语义。连续工具点动在每个控制步使用当时工具朝向转换速度；一次步长命令则在接受命令时固定该次目标，不能每周期重新叠加完整步长。SE(3) 左右乘法意义可对照原作者的空间/body 运动解释。[原作者位姿与数值 IK 教程](https://modernrobotics.northwestern.edu/nu-gm-book-resource/6-2-numerical-inverse-kinematics-part-2-of-2/)

### 4. 统一局部加权 DLS 步：矩阵小，策略仍须验收

令 \(v_d=[v_{TCP,B};\omega_B]\)。连续点动提供有界速度，可叠加有界跟踪反馈；步长点动由固定邻近目标生成反馈速度，再调用同一求解步。对步长误差可用：

\[
e_p=p_d-p,\qquad e_R=\mathrm{Log}(R_dR^T)^\vee.
\]

相对四元数的 `scaled_axis` 可提供该旋转向量。位置和角度各自设置增益、速度上限与停止容差，不用 Euler 差，也不把四元数四个分量差当三维角误差。接近 \(\pi\) 的分支歧义不属于首版小范围点动契约，应拒绝该步或重新确认局部目标。

为避免米、毫米、弧度、度的选择改变阻尼与奇异判断，定义正的任务速度尺度 \(v_0,\omega_0\) 和关节速度尺度 \(d_i\)：

\[
S=\operatorname{diag}(1/v_0,1/v_0,1/v_0,1/\omega_0,1/\omega_0,1/\omega_0),\quad
D=\operatorname{diag}(d_1,\ldots,d_7),\quad A=SJD,\quad b=Sv_d.
\]

求解归一化目标：

\[
\min_{\dot q}\;\|S(J\dot q-v_d)\|^2+\lambda^2\|D^{-1}\dot q\|^2,
\qquad
\dot q=D A^T(AA^T+\lambda^2I_6)^{-1}b.
\]

这给出了任务与关节加权的明确含义，而不是向六维向量随意添加权重。实际用 Cholesky 解 \((AA^T+\lambda^2I)y=b\)，再回乘，不显式求逆。\(\lambda>0\) 时该矩阵在精确算术下正定；并不意味着模型正确、速度足够小或任务必达。Pinocchio 官方教程给出了相同结构的阻尼求解，并提醒固定阻尼并非总是合适；本文写 \(\lambda^2\)，其示例直接把 `damp` 加到对角线，参数数值不能混用。[Pinocchio 官方 CLIK 教程](https://gepetto.github.io/doc/pinocchio/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html)

用 \(J\) 处理上述小误差属于**局部运动反馈**，不能宣称是任意有限姿态残差的精确 Newton Jacobian。若改成完整 \(\log(T^{-1}T_d)\) 的六维 SE(3) 残差，则必须实现并验证相应 log 微分/Jlog 与 frame、符号约定；`Isometry3` 和 `UnitQuaternion` 并不会自动提供这一机器人任务算法。当前契约可通过小步、关节增量上限、实际 FK 误差复查及失败退出收敛范围，不必为首版实现通用 SE(3) 优化器。Pinocchio 示例中的 `Jlog6` 正是不能漏抄的差别。[Pinocchio 官方残差 Jacobian 说明](https://gepetto.github.io/doc/pinocchio/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html)

### 5. 约束、奇异与动态退出：可保守停止，无须完整优化器

以下属于自写且必须验证的有限策略，不是 nalgebra 自动提供的控制功能：

- **真实时间与增量**：使用有效控制间隔 \(\Delta t\)，检查时间戳、过期/乱序反馈和间隔异常；限制每轴 \(|\Delta q_i|\)，以 \(q_{next}=q+\alpha\dot q\Delta t\) 形成局部候选。控制周期超界时停止该次推进，不能用异常大的 dt 积分。
- **关节位置和速度**：在各侧真实 limits 内留裕量，计算共同比例 \(0\le\alpha\le1\) 使下一候选合法。到达限位且方向向外时允许保守停止；不要求在限位上继续找替代构型。逐轴裁剪会改变 TCP 速度方向，若采用必须重算任务误差，不能把裁剪前的解当成仍满足目标。单步范围小也不能替代关节增量检查。
- **动态限幅**：核对实际发送的关节速度和加/减速度，含按住启动、松手、正反切换和停止。只保证 q-next 不越界不能保证真实设备刹得住；位置裕量应考虑已存在速度、可用制动减速度、测量和通信延迟。\(\dot q_i^2/(2a_{brake,i})\) 只是恒定减速度下的距离估算，不是该设备的实测保证。平滑/限幅后的最终命令仍须重新校验全部约束和任务误差。
- **奇异降速与停止**：对归一化 \(A\) 做有预算的 SVD，根据最小奇异值或条件数设置降速、硬停止、恢复迟滞；阈值依赖尺度、工具、硬件和控制周期，不能照抄别的机器人常数。阻尼限制逆解放大，不自动保证有用的 TCP 运动。检查衰减后任务速度残差与步长进展；初版可保守停止，不承诺自动穿越或逃离奇异区。
- **失联与数值失败**：反馈过期、命令源丢失、模式取消、NaN/Inf、分解失败、超迭代预算或异常增量，均不接受新推进结果，并走已定义的控制器停止/保持与 watchdog 路径。应用侧零速度或“保持最后位置”不等于实物已停止，具体退出动作须经设备链路验证。

MoveIt 官方把奇异降速、限位、平滑与控制输入分开处理，并提供逼近奇异、奇异停止、关节边界等状态；它也明确运行所需的控制器和快速准确的关节反馈。这支持上述职责拆分，**不表示本方案已具备 MoveIt 的碰撞检查或安全保证**。[MoveIt Realtime Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)、[官方奇异/限位状态与缩放 API](https://moveit.picknik.ai/main/api/html/namespacemoveit__servo.html)

### 6. seed、正反点动与模式互斥

seed 从当前有效实测关节状态开始；前一个**已接受**的局部参考可用于延续，但须检查相对实测的偏差，不能沿着尚未执行的 IK 迭代独自漂移。对于有限位 7R，不为追求角度“接近”随意加减 \(2\pi\)。小步积分和关节增量验收能约束局部连续性，不能保证全程避开奇异或获得任意冗余构型。

正/反方向切换沿当前有效 q 和减速状态继续，不切换成另一个随机 seed；同轴同时正反输入应取消/拒绝，不能由消息先后顺序偶然决定。正向再反向点动也不保证七轴构型精确回到原值：冗余解、阻尼、数值积分和约束可能产生路径差异，必须检查跳变与残差，不能把“完全可逆”列为未经证实的性质。

关节点动、Cartesian 连续点动、Cartesian 步长点动具有唯一命令所有者；模式、左右臂、坐标系或 TCP 配置切换时，取消旧目标和按住状态，确认停止/重同步，并丢弃旧模式晚到的计算结果。FK 显示始终由实测 q 更新，本身不应成为另一个发命令模式。此状态仲裁是项目设计建议；原作者“前一解作邻近初值”的建议并不覆盖设备状态过期、模式竞争等工程问题。[原作者局部初值延续依据](https://modernrobotics.northwestern.edu/nu-gm-book-resource/6-2-numerical-inverse-kinematics-part-1-of-2/)

### 7. 不需要 elbow 项，也不能把阻尼补项误称严格零空间

没有自定义 elbow 需求时，初版取上述最小加权速度步即可，不添加自动回中、姿态偏好或额外调肘参数。若以后确有任务要增加次目标，必须先解决以下数学问题，而不是直接套公式。

在归一化坐标下令 \(A^\#_\lambda=A^T(AA^T+\lambda^2I)^{-1}\)。由直接代数推导：

\[
A(I-A^\#_\lambda A)
=\lambda^2(AA^T+\lambda^2I)^{-1}A\ne0.
\]

因此把 \((I-A^\#_\lambda A)z\) 加到主任务，会向任务空间泄漏，不能声称 TCP 不动。严格局部任务优先需要实际零空间基，或在秩处理明确时用 Moore–Penrose 投影 \(P=I-A^\dagger A\)，并验收 \(AP\) 的数值残差；物理关节速度次项相应为 \(DPz\)。小奇异值截断、秩变化和后续逐轴限幅仍会破坏“完全不影响任务”的直觉。

nalgebra 6×7 SVD 的 `v_t` 为薄矩阵 6×7，不包含一个可直接取出的“第七右奇异向量”；不能按完整 7×7 V 编号读取。投影可从保留的行空间构造，但本次 MVP 无需为此增加算法。分解维度来自当前 API；零空间泄漏结论为上式独立推导。[SVD 字段及维度](https://docs.rs/nalgebra/latest/nalgebra/linalg/struct.SVD.html)

## 数学与软件验收负担：必须有独立证据

下表是后续实现的验收设计，本轮没有运行这些检查，也没有产出通过记录。具体容差、采样域、速度、奇异阈值、时限和停止距离应由设备要求与实测约束确定，本文不虚构数值。

| 验收层 | 必需证据 | 能发现什么／还不能证明什么 |
| --- | --- | --- |
| 模型装配与左右侧 | 七轴名称/顺序、实际两侧 limits、各 origin/axis、base、flange、TCP 配置的可追溯清单；错误、缺失或非有限配置被拒绝 | 防错链、错序、左右限位复用。没有实物标定，不能证明编码器零偏和模型尺寸吻合。 |
| 已知姿态 FK 对照 | 零位及多个非零、非对称构型的独立位置/朝向基准；来自设备方确认数据、独立手算/几何推导或独立可信参考；逐段及最终 TCP 都对照 | 可查乘法顺序、方向、单位和 TCP 偏置。仅使用同一 FK 生成“期望值”没有独立性；零位一例也不足够。零位可只作为数学样本，不要求实物到该位姿。 |
| Jacobian 有限差分 | 每侧、多个合法 q 与非零 TCP 偏置下逐列对照；中心差分的平移列为 \([p(q+he_i)-p(q-he_i)]/(2h)\)，角列为 \(\mathrm{Log}(R(q+he_i)R(q-he_i)^T)^\vee/(2h)\) | 与基座表达 geometric J 的定义匹配。比较一组 h，兼顾截断与舍入误差；边界用合法单侧差分。差分依赖同一 FK，能查 J 实现，却不能独立证明 FK 或 URDF 正确。 |
| 坐标与姿态语义 | 在非单位朝向与非零 TCP 下分别检查基座/工具 XYZ、姿态步进；旋转 TCP 时位置应按约定保持；四元数正负号等价、近零角与局部分支均覆盖 | 查点/frame 混用、左/右乘错误、Euler 差误用。还需检验非主轴组合，防止只在对齐坐标下“碰巧正确”。 |
| DLS 与局部连续性 | 归一化、单位等价输入、阻尼/截断区别、线性系统残差；每步 q 增量、最终命令和 FK 任务残差；正反输入、两种点动及模式切换 | 验证统一局部步，不以求解器“返回成功”为充分条件。没有真实时序和执行器反馈，不能证明端到端跟踪表现。 |
| 近奇异／限位 | 接近秩亏、各侧各关节上下界、多个约束同时活跃、保持姿态的平移、保持位置的旋转；降速/停止及迟滞退出可观察 | 阻尼成功但目标方向丢失也必须报告。停止在合法边界是可接受的首版结果，无须求出沿边界继续走的解。 |
| 不可达／无进展 | 远端或限位阻塞目标、超出局部契约的步长；迭代/时间有界，返回原因且不提交失败候选 | **局部迭代失败不能证明几何上全局不可达**；对用户应报告“局部无法推进/本步未到达”，而不是无依据宣称全局无解。 |
| 数值与链路异常 | 非有限 q/目标、SVD 或 Cholesky 失败、时间异常、反馈过期、命令松开/失联、旧模式结果晚到；从既有速度状态触发退出 | 验证软件拒绝继续推进与停止请求路径；物理制动、掉线保持和停止距离必须另取真机证据。 |

FK→IK→FK roundtrip 只能作为补充一致性检查：FK 和 IK 若共享错误 axis、TCP 或单位，也可能互相吻合；七轴冗余下也不该把“回到相同 q”作为唯一成功条件。应同时保留**独立已知姿态 FK 基准 + 有限差分 Jacobian + 约束/失败行为**三类证据。Modern Robotics 对 Jacobian 的定义和分离位置/角度停止条件提供数学参照；具体测试组合为本评估的工程建议。[原作者 Jacobian 教程](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-1-1-space-jacobian/)、[原作者数值 IK 教程（下）](https://modernrobotics.northwestern.edu/nu-gm-book-resource/6-2-numerical-inverse-kinematics-part-2-of-2/)

验证负担的排序是：**FK/Jacobian 公式本身较有限；坐标、尺度、左右模型与 TCP 的一致性更需要独立对照；动态限幅、异常退出与真实闭环最依赖设备证据。** 不需要把这一有限问题升级成通用 IK 研究，但不能因为矩阵只有 6×7 就略过后两类工作。

## 维护边界与“一天纯 Agent MVP”的可信交付

建议让一个内聚的运动学模块拥有：固定模型/TCP 与单位契约、一次 FK 及 Jacobian 计算、统一局部 DLS 步、局部目标和失败原因。既有控制器继续拥有设备 I/O、反馈新鲜度、命令所有权、动态停止和 watchdog；两者通过明确的“当前关节状态、局部命令、有效候选/停止原因”交互。这里描述职责，不要求创建新的文件或框架。没有必要为七轴再拆矩阵适配、Jacobian 插件、solver 工厂、机器人基类、动力学或全局规划层。

纯数学部分保持平台无关、使用普通 f64 与纯 Rust 依赖；设备和 UI 的平台差异留在既有边界。首次验收只针对当前 Darwin arm64，后续逐平台记录构建和行为证据，不为“跨平台”预建插件体系。

**若后续获准实施，一天可争取的是 Darwin arm64 上可审查的局部点动软件候选**：固定七轴模型与配置 TCP、共享 DLS 的两种点动、明确的限幅/失败输出，以及上述离线数学和状态验证记录。前提包括模型及独立基准可得、工具链可用、现有控制接口足够明确；这只是有条件交付边界，不是精确工期或通过承诺。本轮的实际交付仍只有本文。

用户的一日期望是“单平台真机端到端 MVP”。**设备当前不可访问，所以现阶段不能完成或宣称该验收通过。** 必须由后续真实设备证据补齐：

- 编码器序号、角度方向与零位；左右臂实际限位和安装变换；真实 TCP 位置与朝向。
- 实际反馈/发送周期、时延和抖动；加载工具后的速度/加减速及制动能力；底层控制模式对位置/速度指令的真实响应。
- 小范围连续与步长 XYZ/姿态点动的实物方向、轨迹和稳定性；接近限位/奇异时的实际降速停止。
- 松手、切模式、失联与 watchdog 的端到端退出，以及从运动状态开始的停止时间/距离和驱动保持行为。

多 Agent 能并行审查公式、单位契约、差分设计与失败路径，不能产生上述设备观测。Darwin arm64 离线软件通过、模型数学一致、仿真通过、真机端到端通过应分别记录；任何前三者都不能替代最后一项。也不承诺未经测量的周期、吞吐、精度或停止距离。MoveIt 官方对控制器和关节反馈的前置要求同样说明，运动学算法只是端到端伺服链路的一部分。[MoveIt 新机器人前置条件](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html#setup-on-a-new-robot)

**有条件建议：选择直接 nalgebra 路径，先把统一局部点动步及其验证契约做完整；只有需求明确扩展到任意目标、多分支搜索或任务层级时，再重新评估额外求解能力。** 当前不引入通用全局 IK、完整约束优化器和非必要 nullspace 策略，有利于把维护面集中在实际需要且可逐项验收的有限算法上。

# 纯 Rust 运动学库能力调查

调查日期：2026-11-09（Asia/Shanghai）。对象：随桌面应用启停的本机 Tauri Rust 后端，首版连续点动、步长 XYZ 与姿态操作；无 ROS、独立常驻服务或未来扩展前提。

后续选型更新：本报告保留 k 等库的 API/版本考据，最新优先建议已改为当前 nalgebra + urdf-rs 的有限实现；活跃替代库和一天 Agent 预算见[最新评估](day-one-agent-plan.md)。这里的 k 初始候选不表示项目已决定采用。

本报告仅查阅官方上游 README、特定函数源码、docs.rs 和 crates.io 元数据。没有创建 Cargo 项目、编写运动学探针或实现、编译/运行示例、连接硬件，也没有本机依赖解析或性能实测。左右臂各有 7 个 revolute、两侧 limits 不同、官方 URDF 无具名 TCP，均作为任务给定事实使用，本次未重新解包核查。

## 结论与选型边界

`k + 与其兼容的 urdf-rs/nalgebra` 已提供值得继续评估的运动学基础：URDF 建链、FK、**调用者可直接取得的 6×N Jacobian**、从当前关节状态开始的局部数值 IK，以及 7 关节对位置/姿态 6 维目标的冗余求解分支。它不要求机械臂属于某个 6 轴解析几何族。[公开 Jacobian][k-jac-api]、[求解实现][k-ik-src]

这足以减少首版的模型、变换与单点 IK 工作，但还不能证明这台 7DOF 机械臂能够稳定连续运动。输入到目标的积分、TCP/基座坐标约定、构型连续、奇异降速/停止、关节速度与加速度限制、发送节拍和异常停机仍要由应用完成。**IK 库不能称为完整 servo**；求得一个满足容差的关节向量，与按时、连续、有界地执行运动是不同的交付能力。这是根据公开接口边界作出的工程判断，无实测背书。

`rs-opw-kinematics` 可作为纯 Rust 解析 IK 的对照，但不是这份任意 7 轴 URDF 的直接替代。它的 OPW 假设是 ortho-parallel basis 与 spherical wrist，公开 `Joints = [f64; 6]`；另有固定第六轴/忽略工具轴滚转的 5DOF 模式。因此“只支持 6 轴”不够准确，应表述为“面向特定 OPW 几何的 5/6 轴，非通用 7DOF 数值 IK”。[上游说明][opw-readme]、[3.0.0 关节类型与接口源码][opw-trait-src]

## 版本、许可与维护证据

下表的“当前”指调查当日 crates.io 的 `max_stable_version`；发布日期取对应 version 元数据，不取搜索抓取时间。兼容旧版本单列，不能把最新 README 的能力套到旧依赖上。

| 组件 | 当前稳定版 / 发布时间（UTC 日期） | 本报告涉及的兼容版本 | 许可元数据与维护证据 |
| --- | --- | --- | --- |
| `k` | 0.32.0 / 2024-09-10 | 0.32.0 | Apache-2.0。[crate 元数据][k-meta]。仓库未归档；本次看到主分支最新提交为 2026-02-09 的开发依赖 rand 更新，不能把它解释为 IK 算法近期改进。[仓库 API][k-repo-meta]、[具体提交][k-maintenance] |
| `urdf-rs` | 0.10.0 / 2026-09-25 | **0.9.0 / 2024-09-06**，满足 k 的 `^0.9` | 两版 Apache-2.0；新版本发布是维护证据，但 0.10 的 URDF 1.1/1.2 支持不能归入 0.9。[当前元数据][urdf-meta]、[0.9 元数据][urdf09-meta]、[0.10 文档][urdf10-api] |
| `nalgebra` | 0.35.0 / 2026-05-24 | **0.30.1 / 2022-01-09**，满足 k 的 `^0.30` | **0.30.1 为 BSD-3-Clause，0.35.0 为 Apache-2.0**，不要混记许可。当前版声明 Rust 1.89.0；这不是 k 所用旧版的 MSRV。[当前元数据][na-meta]、[0.30.1 元数据][na030-meta] |
| `rs-opw-kinematics` | 3.0.0 / 2026-09-10 | 3.0.0，仅对照 | BSD-3-Clause；当前版本发布和 README 的 3.0 迁移说明提供维护证据。[元数据][opw-meta]、[README][opw-readme] |
| `parry3d` | 0.31.1 / 2026-09-18 | 无首版引入建议 | Apache-2.0；Rust 几何/碰撞库，不是 IK。[元数据][parry-meta]、[上游 README][parry-readme] |
| `rsruckig` / `ruckig` | 3.0.1 / 2026-08-29；0.19.4 / 2026-07-15 | 无首版必选建议 | 均为 MIT，但前者是 Rust 移植，后者 Rust crate 使用 C++ 桥接。不能只凭 Ruckig 名称判断纯 Rust。[移植版元数据][rsruckig-meta]、[原版 crate 元数据][ruckig-meta]、[原版依赖][ruckig-deps] |

维护证据只能支持“仍有发布/提交”，不能替代可达域覆盖、收敛成功率、控制频率、数值稳定性或长期支持承诺。尤其 k 的最新发布距离调查日已超过两年。

## k 0.32.0：公开 API 与真实行为

| 需要核对的能力 | 调用者可用的接口 / 已确认行为 | 限制与首版含义 |
| --- | --- | --- |
| 读取 URDF | `Chain::<f64>::from_urdf_file(path)`；也可用兼容版本 `urdf_rs::read_file` / `read_from_string` 后 `Chain::from(&robot)`。[转换源码][k-urdf-src] | 模型解析不等于控制器关节顺序、零位、正方向已经对齐。需按 joint name 建立映射。 |
| FK | `Chain::set_joint_positions(&q)`；`SerialChain::end_transform() -> Isometry3<T>`；`Chain::update_transforms()` 和节点 `world_transform()`。[SerialChain API][k-chain-api]、[实现][k-chain-src] | `end_transform` 从 identity 连乘所选串链的 local transforms；不要把任意截出的子链结果自动当作完整模型的 world pose。 |
| Jacobian 对外可访问性 | **顶层公开函数** `k::jacobian(&SerialChain<T>) -> nalgebra::DMatrix<T>`，非仅 solver 私有能力。源码构造 6 行、`arm.dof()` 列，上三行为线速度，下三行为角速度；转轴列使用 `a × (p_end - p_joint)` 与 `a`。[API][k-jac-api]、[具体实现][k-jac-src]、[导出][k-lib-src] | 7 个可动关节对应 6×7，固定关节不占列。返回末端节点处的几何 Jacobian；不会同时返回奇异阈值、降速建议或 TCP 点偏移补偿。 |
| 完整 6 维位姿 IK / 7DOF | `InverseKinematicsSolver::solve(&arm, &Isometry3)` 默认约束 XYZ 与三个姿态误差分量。`Constraints` 默认 6 项全开；`available_dof > required_dof` 时走 SVD/伪逆分支，故 **7 > 6 在实现中有明确支持**。[trait API][k-ik-api]、[solver 源码][k-ik-src] | 这里“支持”是接口与算法维数成立，不是目标总可达或 7DOF 全局搜索成功。不要为凑方阵而忽略第七轴；也不能把姿态三个误差分量当成独立 Euler 角插值。 |
| 当前解 seed | 接口没有独立 `seed` 参数；`solve` 读取传入 arm 的 `joint_positions()` 作为初值，并直接修改该 arm。[源码][k-ik-src] | 调用前用当前可信反馈或上一已接受解设定模型状态；如何选择、何时重新与反馈同步属于应用策略。每次回零 seed 会损害构型延续。 |
| 关节位置限位 | `Joint::limits: Option<Range<T>>`；URDF 转换读取 lower/upper；`set_joint_positions` 可返回 `OutOfLimitError`，IK 迭代实际使用 `set_joint_positions_clamped`。[Joint 源码][k-joint-src]、[URDF 转换][k-urdf-src] | 两臂各用本侧 limits。转换中 **upper == lower 被当成无限位 `None`**，不能依赖它表示“锁死关节”；本次不推断给定 URDF 有此情况。 |
| clamp 的失败语义 | `set_joint_positions_clamped(&q)` 返回 `()`，逐关节截到范围内；夹紧本身不报 `OutOfLimitError`。IK 在夹紧后计算位姿残差；用尽迭代通常得到 `NotConvergedError`，而不是“触限”专用错误。[Chain 实现][k-chain-src]、[solver 实现][k-ik-src] | 不能把 `Ok` 解释为没发生夹紧，也不能只依据错误类型判断是否触限。调用层仍需限位裕量、增量/速度和最终 FK 残差检查。 |
| 收敛参数与误差 | 公开字段 `allowable_target_distance`、`allowable_target_angle`、`jacobian_multiplier`、`num_max_try`；`solve_with_constraints` 可减少任务维数或忽略指定 joint。[solver API][k-solver-api]、[Constraints API][k-constraints-api] | 对本需求应保持完整姿态约束。默认是距离 0.001、角度 0.005、乘数 0.5、最多 10 次；URDF SI 单位下约为 1 mm、0.286°。这是默认容差，不是实测精度。 |
| 冗余自由度偏置 | 公开 `set_nullspace_function(Box<dyn Fn(&[T]) -> Vec<T> + Send + Sync>)`、`clear_nullspace_function`，及 `create_reference_positions_nullspace_function`。[API][k-null-api]、[实现][k-ik-src] | 可以传入二级任务向量，库将其投影到零空间；没有现成的“固定肘形、最小关节跳变、避限位”完整策略。helper 被上游称为示例；其符号与权重需要核对，不能仅凭名称认为正权重必然吸引到参考位形。 |
| TCP / 末端 frame | 可按 `find` / `find_link` 定位末端，`SerialChain::from_end` / `from_end_to_root` 选串链；`NodeBuilder` 的固定关节与 origin/translation/rotation 能表达附加刚体 frame。[串链 API][k-chain-api]、[NodeBuilder API][k-node-api] | solver 没有 `tcp_offset` 参数。可以建模固定 TCP 节点，或在调用层把 TCP 目标换算成选定末端目标；这是模型/坐标接入工作，不能从没有 TCP 名称的 URDF 自动推断实际工具。 |

### 收敛、恢复与数值风险的具体边界

在 `solve_one_loop_with_constraints` 中，冗余分支使用硬编码 **EPS = 0.0001** 的 SVD 求解；有 nullspace callback 时使用伪逆。非冗余方阵分支用 LU，不能求解时返回 `InverseMatrixError`。`jacobian_multiplier` 是每次位置更新的固定乘数，**不是**公开可调的阻尼最小二乘参数；没有自适应阻尼、线搜索或笛卡尔速度缩放接口。[特定实现][k-ik-src]

成功条件是选中位置误差向量的范数与姿态误差向量的范数分别小于阈值。姿态差用四元数 `rotation_to(...).scaled_axis()`，不是逐项 Euler 角相减。`NotConvergedError` 携带 `num_tried`、`position_diff`、`rotation_diff`；自由度不足有 `PreconditionError`。这些能解释单次求解结果，不能提供连续控制的稳定性或每周期完成时限保证。[solver 实现][k-ik-src]、[错误 API][k-errors-api]

公开 `solve_with_constraints` 在内部返回 `Err` 时会尝试恢复调用前的关节位置；这是 **正常 Result 错误路径** 的恢复语义，恢复本身也使用可失败的 checked setter。不要扩大成任意失败都具有事务保证：冗余 SVD/伪逆处有 `unwrap`，`num_max_try = 0` 会走到没有最后残差的 `unwrap`，callback 也要求正确维数。另一个不同的行为是 `Chain::set_joint_positions` 逐关节写入，中途遇错前面的关节可能已经改变；clamped setter 用 `zip`，也不会像 checked setter 那样验证输入长度。输入有限值、维数、参数有效性与输出验收仍需调用层负责。[内部迭代][k-ik-src]、[公开错误恢复与默认参数][k-restore-src]、[Chain 源码][k-chain-src]

### 基座、工具坐标与 TCP 的表达

以下是接入建议，不是已验证的项目实现。令 B 为选定基座、E 为库中末端节点、C 为 TCP，固定工具变换为 `E_T_C`：

- FK：`B_T_C(q) = B_T_E(q) · E_T_C`。
- TCP 目标求解：先换算 `B_T_E_target = B_T_C_target · inverse(E_T_C)`，再交给以 E 为末端的 IK；或者把 C 建为固定节点并以 C 作为串链末端。
- 有安装变换时：`W_T_C = W_T_B · B_T_E(q) · E_T_C`。`Chain::set_origin` 能设置首节点 origin，但不能替调用者统一所有输入坐标约定。[origin/串链实现][k-chain-src]

基座 XYZ 增量与工具 XYZ 增量需要先变到同一个解算坐标系；姿态增量也要定义旋转轴所属 frame 和旋转中心。内部可使用同版本 nalgebra 的 `Isometry3`、`UnitQuaternion`，界面显示的姿态格式不应直接决定数值积分方式。[nalgebra 几何 API][na030-api]

**需要特别验证截链坐标的一致性。** 静态源码显示 `end_transform()` 连乘所选节点的 local transforms，而 Jacobian 中关节位置/轴来自 `world_transform()`；`from_end_to_root` 只选择节点，没有另行归一化所有 world frame。若截断点之外仍有非 identity 父变换，不能假定这些量自然一致。建议使解算串链与 world 使用同一明确基准，再验证任意截链/非 identity 安装的行为；本次没有实测这个组合。[FK/截链实现][k-chain-src]、[Jacobian 实现][k-jac-src]

若用“换算 E 目标”的方式，公开 Jacobian 仍针对 E 点；TCP 有平移偏移时，奇异/速度分析也必须使用 C 点的正确 Jacobian。只给目标乘上 offset 不会自动改变 `k::jacobian` 的末端点。这是由几何 Jacobian 公式推导的调用层要求。[Jacobian 实现][k-jac-src]

## 依赖统一与纯 Rust 边界

`k 0.32.0` 的已发布 manifest 声明 `nalgebra = "0.30"`、`simba = "0.7"`、`urdf-rs = "0.9"`；默认 features 为空。`k::nalgebra` 是公开 re-export，`Isometry3`、`Translation3`、`UnitQuaternion` 等也被顶层导出。[已发布 manifest][k-manifest]、[re-export 源码][k-lib-src]

推荐继续评估的最小依赖组合是：

| 用途 | 建议组合（仅建议，无 Cargo 文件变更） | 原因 |
| --- | --- | --- |
| 只需要建链、FK、IK、Jacobian | `k 0.32.0`；数学类型优先使用 `k::nalgebra` | k 已传递依赖 URDF parser，避免单独引入最新 nalgebra 造成两套类型。 |
| 需要直接检查 URDF joints/limits 或从字符串载入 | 上述组合加直接 `urdf-rs 0.9`（本次核验版本 0.9.0） | 与 `Chain::from(&urdf_rs::Robot)` 接受的依赖版本对齐，也保留原始 velocity/effort 元数据供应用检查。 |
| 项目必须直接依赖 nalgebra | 与 k 对齐到 `0.30`（本次核验 0.30.1） | 类型边界保持单一；不能把 `0.35` 的 `Isometry3` 直接交给 k。 |

`urdf-rs 0.9` 的 `JointLimit` 还公开 `effort`、`velocity`，但 k 的关节转换仅用 lower/upper 构造位置范围；解析到这些字段不表示 IK 会执行速度或力矩限制。[JointLimit API][urdf09-limit-api]、[具体转换][k-urdf-src]

Cargo 对 0.x 的不同 minor 不视作兼容升级；`^0.30` 不接受 `0.35`，`^0.9` 不接受 `0.10`。同时加入最新版通常会解析成两份 crate，`Robot`/`Isometry3` 等类型不同，不能靠名字相同直接传递。这里依据已发布依赖声明与 Cargo 版本规则作出判断，**没有运行 cargo tree 或尝试解析本项目**。[k 依赖元数据][k-deps]、[Cargo 官方版本规则][cargo-semver]

纯 Rust 路径的判断也要分层：

- `k` 核心与 nalgebra 标准矩阵/几何路径是 Rust 源码；已发布 k 普通依赖没有 C++ 桥接或 native solver。`kiss3d` 是 dev-dependency，不是使用 k 的必需运行依赖。本次只核对发布声明和相关源码，没有完整递归解析/构建所有平台依赖。[manifest][k-manifest]、[nalgebra 0.30.1 依赖元数据][na030-deps]
- `urdf-rs 0.9` 普通依赖为 Rust XML/serde/regex 等，没有要求安装 urdfdom；但工具函数的 **xacro 分支会执行 `rosrun`/`xacro`，package 路径展开可能执行 rospack/ros2**。直接使用 plain URDF 的 `read_file` / `read_from_string` 不需要走这些分支。本项目不应把便捷的外部工具路径误记为 Rust FK/IK 的必要条件。[0.9 依赖][urdf09-deps]、[外部命令具体源码][urdf-utils-src]
- `rs-opw-kinematics 3.0.0` 上游明确写明是 Rust 实现而非 C++ binding；它当前直接使用 **glam 0.32.1**，不再是 nalgebra 公共 API。默认 features 包含 filesystem、collisions、stroke_planning，visualization 为可选，不能用其截图推断必需 GUI/native 运行环境。[README][opw-readme]、[3.0.0 manifest][opw-manifest]、[依赖元数据][opw-deps]
- Parry 是 Rust 几何/碰撞工具，不负责 FK、IK 或 servo。当前 `parry3d 0.31.1` 的数学依赖已含 glamx，不能套用旧版本“必然和 k 共用 nalgebra”的印象。[README][parry-readme]、[依赖元数据][parry-deps]
- 原版 Ruckig 是 C++ 实现；名为 `ruckig 0.19.4` 的 Rust crate 依赖 `cxx` 和 build dependency `cxx-build`，不符合“核心求解无 C++ FFI”的条件。`rsruckig 3.0.1` 是独立 Rust 移植，已发布普通依赖为 arrayvec、num-traits、thiserror；可辅助带速度/加速度/jerk 约束的轨迹生成，但不能代替 URDF IK、奇异判断或笛卡尔路径验证。本次不主张为首版立即加入。[原版依赖][ruckig-deps]、[移植版文档][rsruckig-doc]、[移植版依赖][rsruckig-deps]

## rs-opw-kinematics 的有用对照与不适用之处

它公开提供 `Kinematics::forward`、`inverse`、`inverse_continuing(pose, previous)`、`constraints` 与显式 5DOF 方法；`inverse_continuing` 能按 previous 处理解的延续，并对特定腕部奇异情形处理 J4/J6。它还有公开 Jacobian、base/tool 包装和路径规划能力。因此不能简单说它“没有连续性或坐标功能”。但其固定六元关节接口与 OPW 几何假设没有因此变成通用七轴模型；其 URDF 读取器也明确要求该几何族。[3.0.0 trait][opw-trait-api]、[参数 API][opw-parameters-api]、[Jacobian API][opw-jac-api]、[README 中工具/基座和 URDF 假设][opw-readme]

对这台左右各 7R 的机械臂，它更适合作为“解析 IK 有明确几何前提”的参照，而非现阶段主候选。`inverse_continuing` 的上一解选择和腕部解析处理，也不等于对任意机械臂的关节跳变限制、奇异附近速度控制或硬件闭环保证。

## 首版现成库能承担什么、还需自行完成什么

下表是基于上述 API 边界的工作分配建议，不表示已经决定实施方案。

| 首版目标 | 现成能力 | 应用层缺口 |
| --- | --- | --- |
| 关节反馈 → 末端位姿 | URDF 建链、FK、刚体变换 | 左/右命名与顺序映射、反馈时效、零位/符号/单位、TCP 定义；两侧使用各自 limits。 |
| 步长 XYZ + 姿态 | 局部 6D 位姿 IK 与当前状态 seed | 步长在所选 frame 的含义；从当前目标/当前反馈哪一个累计；中间位姿与执行时间的生成。只解终点再做关节插值，不保证 TCP 沿直线或指定姿态路径移动。 |
| 按住连续点动 | 可对连续目标重复 FK/IK | 固定时间基准、速度目标积分、输入续租/超时、松开停止与取消、目标去积累、后台线程生命周期；桌面应用退出需结束本机控制活动。 |
| 基座 / 工具 frame | nalgebra 变换、固定节点、origin 与末端选择 | 统一 frame、旋转中心及 TCP 标定；IK 与 Jacobian 均使用一致的坐标与末端点。 |
| 构型连续 | 当前关节 seed、nullspace callback | 第七轴/肘形偏好、关节角差与每步增量门限、避免 wrap/分支跳跃、候选拒绝后的保持/停止语义。seed 是局部偏好，不是连续性证明。 |
| 奇异降速 / 停止 | 公开 6×7 Jacobian，可交给 nalgebra SVD | 对线/角单位尺度进行定义，选择奇异度指标和阈值、带滞回的降速/停止、可恢复条件；k 的固定 SVD 截断不是这套策略。 |
| 触限、不可达、求解失败 | 位置 clamp、残差容差、Result 错误与正常错误恢复 | 限位裕量、有限值与最终 FK 校验、单次时间预算、禁止把失败中间态直接下发、明确保持/受控停止策略。 |
| 有界且平滑地下发 | 运动学库返回/修改关节位置 | q/dq/ddq/必要的 jerk 约束、轨迹节拍与反馈同步、硬件已有停止接口的接入；单纯 clamp 位置不能约束动态运动。碰撞检查若纳入范围，还需要几何配置与策略，URDF 限位不承担它。 |

## 缺口、未知与后续验证条件

1. **模型接入未验证**：左右独立关节映射、基座、E 与 TCP 的具体变换和实机零位关系；不能从“URDF 能解析”推断正确。特别检查非 identity 安装、截链与附加 TCP 后的 FK/Jacobian 一致性。
2. **收敛与连续性未验证**：在实际 7R 几何、两侧不同限位下，当前解 seed 对附近 6D 目标的成功率、临限位停滞、肘形漂移、近奇异姿态的输出增量。上游冗余分支是支持证据，不是本机测试结果。
3. **动态指标未验证**：单周期耗时分布、内存分配、可达控制频率、取消/停机延迟、系统负载下的时序。源码使用动态矩阵/向量和迭代，不能在未测前承诺实时性。
4. **依赖与工具链未验证**：候选版本在目标桌面平台、现有 Tauri 依赖中的解析结果、重复依赖与构建结果。当前版本与兼容旧版本已区分，本报告不生成 lockfile 或强制升级。
5. **仍需配置或验证的参数**：实际 TCP 变换、速度/步长数值、最大关节增量、奇异指标尺度与阈值，以及失败后的具体硬件退出动作。已记录的点动语义以项目决策票为准，包括基座/工具坐标、米/弧度内部单位和绕当前 TCP 旋转；本调查不重新打开这些选择，也不以库默认值替代设备验证。

建议将 `k 0.32.0 + 同版本数学类型 + 按需 urdf-rs 0.9` 保留为最小优先评估组合，是否采用取决于上述验证结果。

## 一级来源索引

所有来源于 2026-11-09 查阅。k 关键行为固定到已发布的 v0.32.0；OPW 关键类型固定到 docs.rs 3.0.0。主分支 README 只作为调查日说明，版本能力以已发布 API/manifest 为准。

[k-meta]: https://crates.io/api/v1/crates/k
[k-repo-meta]: https://api.github.com/repos/openrr/k
[k-maintenance]: https://github.com/openrr/k/commit/244364955475f84715680d51fa95bfa050bacb60
[k-manifest]: https://docs.rs/crate/k/0.32.0/source/Cargo.toml
[k-deps]: https://crates.io/api/v1/crates/k/0.32.0/dependencies
[k-lib-src]: https://github.com/openrr/k/blob/v0.32.0/src/lib.rs
[k-ik-api]: https://docs.rs/k/0.32.0/k/trait.InverseKinematicsSolver.html
[k-solver-api]: https://docs.rs/k/0.32.0/k/struct.JacobianIkSolver.html
[k-constraints-api]: https://docs.rs/k/0.32.0/k/struct.Constraints.html
[k-ik-src]: https://github.com/openrr/k/blob/v0.32.0/src/ik.rs#L221-L372
[k-restore-src]: https://github.com/openrr/k/blob/v0.32.0/src/ik.rs#L465-L486
[k-null-api]: https://docs.rs/k/0.32.0/k/struct.JacobianIkSolver.html#method.set_nullspace_function
[k-errors-api]: https://docs.rs/k/0.32.0/k/enum.Error.html
[k-jac-api]: https://docs.rs/k/0.32.0/k/fn.jacobian.html
[k-jac-src]: https://github.com/openrr/k/blob/v0.32.0/src/funcs.rs#L8-L57
[k-joint-src]: https://github.com/openrr/k/blob/v0.32.0/src/joint/joint.rs#L104-L157
[k-chain-api]: https://docs.rs/k/0.32.0/k/struct.SerialChain.html
[k-chain-src]: https://github.com/openrr/k/blob/v0.32.0/src/chain.rs
[k-node-api]: https://docs.rs/k/0.32.0/k/node/struct.NodeBuilder.html
[k-urdf-src]: https://github.com/openrr/k/blob/v0.32.0/src/urdf.rs#L228-L340
[urdf-meta]: https://crates.io/api/v1/crates/urdf-rs
[urdf09-meta]: https://crates.io/api/v1/crates/urdf-rs/0.9.0
[urdf09-deps]: https://crates.io/api/v1/crates/urdf-rs/0.9.0/dependencies
[urdf09-limit-api]: https://docs.rs/urdf-rs/0.9.0/urdf_rs/struct.JointLimit.html
[urdf10-api]: https://docs.rs/urdf-rs/0.10.0/urdf_rs/
[urdf-utils-src]: https://github.com/openrr/urdf-rs/blob/v0.9.0/src/utils.rs
[na-meta]: https://crates.io/api/v1/crates/nalgebra
[na030-meta]: https://crates.io/api/v1/crates/nalgebra/0.30.1
[na030-deps]: https://crates.io/api/v1/crates/nalgebra/0.30.1/dependencies
[na030-api]: https://docs.rs/nalgebra/0.30.1/nalgebra/
[cargo-semver]: https://doc.rust-lang.org/cargo/reference/specifying-dependencies.html#version-requirement-syntax
[opw-meta]: https://crates.io/api/v1/crates/rs-opw-kinematics
[opw-readme]: https://github.com/bourumir-wyngs/rs-opw-kinematics/blob/main/README.md
[opw-trait-src]: https://docs.rs/rs-opw-kinematics/3.0.0/src/rs_opw_kinematics/kinematic_traits.rs.html
[opw-trait-api]: https://docs.rs/rs-opw-kinematics/3.0.0/rs_opw_kinematics/kinematic_traits/trait.Kinematics.html
[opw-parameters-api]: https://docs.rs/rs-opw-kinematics/3.0.0/rs_opw_kinematics/parameters/opw_kinematics/struct.Parameters.html
[opw-jac-api]: https://docs.rs/rs-opw-kinematics/3.0.0/rs_opw_kinematics/jacobian/struct.Jacobian.html
[opw-manifest]: https://docs.rs/crate/rs-opw-kinematics/3.0.0/source/Cargo.toml
[opw-deps]: https://crates.io/api/v1/crates/rs-opw-kinematics/3.0.0/dependencies
[parry-meta]: https://crates.io/api/v1/crates/parry3d
[parry-readme]: https://github.com/dimforge/parry/blob/master/README.md
[parry-deps]: https://crates.io/api/v1/crates/parry3d/0.31.1/dependencies
[rsruckig-meta]: https://crates.io/api/v1/crates/rsruckig
[rsruckig-doc]: https://docs.rs/crate/rsruckig/3.0.1
[rsruckig-deps]: https://crates.io/api/v1/crates/rsruckig/3.0.1/dependencies
[ruckig-meta]: https://crates.io/api/v1/crates/ruckig
[ruckig-deps]: https://crates.io/api/v1/crates/ruckig/0.19.4/dependencies

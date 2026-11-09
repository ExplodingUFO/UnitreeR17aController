# 更活跃的纯 Rust 七轴运动学候选：维护与接入风险

调查截至：**2026-11-09，Asia/Shanghai**。公开元数据时间均保留 UTC；提交查询上界为 `2026-11-09T15:59:59Z`。本轮只做文档研究，未写实现、Cargo 项目或探针，未编译、运行示例、解析本项目依赖或连接硬件。未 clone、索引或结构探索新仓库；只读取官方 README、公开 API、相关函数源码、发布依赖和 GitHub 元数据。

目标是**单平台真机端到端 MVP**，当前只能在 **Darwin arm64 / Mac 做软件验收**，机械臂和转接器不可访问。因此下文“可评估接入”只针对软件 MVP；真机协议、反馈/停止、硬件端到端均未验收。其它平台的运行验证另算，维护性与可移植代码边界仍是要求。开发时间按一天估算，具体安排和 nalgebra 方案另文记录。

## 结论

本次范围内，**Caliper 的小型 crates 组合最值得作为现成库复用候选，Ferromotion 次之；没有足够证据把任一项直接升级为“成熟、持续维护、低风险的一天替换方案”**。这不是声称不存在其它库，而是对下列六项初筛、三项重点候选的有限结论。

Caliper 的优势不是功能清单，而是已发布的明确 7DOF IK 回归测试、公开 6×N Jacobian、较小的可独立引用模块和一个成功的核心 CI 提交。弱点是 GitHub 仓库创建于 2026-06-22、crate 仅有一次 0.2.1 发布记录、单一公开 crate owner，最新代码提交停在 2026-07-31。它比 k 的发布新，却不能被描述成截至 11 月仍持续迭代。[仓库元数据][cal-repo]、[发布元数据][cal-ik-meta]、[7DOF 测试][cal-ik-source]、[核心 CI][cal-ci-run]

Ferromotion 在 2026 年 7–9 月密集发布，主分支更新到 2026-10-01，公开接口完整；但 `ferromotion-core` 自身已是很大的综合库，常规依赖包含两套矩阵库及约束求解器，并且最近主分支 CI 的结论是 failure。**失败属于发布之后的 main，不能据此断言 0.73.0 不可用；也不能仅凭 badge 宣称已验证可靠。**[发布元数据][ferro-meta]、[具体提交][ferro-commit]、[普通依赖][ferro-deps]、[CI 运行][ferro-ci-run]

因此，当前倾向的“固定 7R、nalgebra + urdf-rs 局部 DLS”并未明显漏掉一个在本次证据中同时更成熟、更轻、更持续维护的通用替代。这个判断**不等于已证明自行实现可在一天内可靠完成**；现成库路线中应优先比较 Caliper 的独立 crates，而不是先接入 Caliper umbrella 或 Ferromotion 的整个生态。

## 筛选与维护证据口径

保留完整能力候选时，至少核对：URDF 到任意长度关节模型、完整位置与姿态数值 IK、公开 Jacobian，以及普通依赖中的 native/FFI 边界。七轴证据区分为“专门测试”“通用维数 + 上游七轴例子”“仅 README 泛称支持”。本次没有用下载量或 star 证明质量。

维护证据分开记录 **发布历史、仓库年龄、代码提交、公开 owner、CI 定义、CI 实际运行**。`pushed_at` 可能只是其它引用更新，不视作更新了求解器；有 workflow 不等于运行成功；main 的测试不等于已经验收 crates.io 发布包在本项目的行为。GitHub 创建日期也不等于作者开始开发的日期。

## 三项重点候选

| 候选 | 本次核对版本、发布日期 | 通用 7DOF / URDF / 公开 Jacobian / IK | 对当前软件 MVP 的判断 |
| --- | --- | --- | --- |
| **Caliper 小型 crates** | `caliper-model`、`caliper-kinematics`、`caliper-ik` **0.2.1**，均为 2026-07-31；见下表精确时间 | 动态 `Model.ndof`；`Model::from_urdf`；`jacobian` 返回 6×ndof；`ik` 显式接受 seed。已发布源码有 `ik_redundant_7dof`、读取 `redundant7.urdf` 并校验完整位姿。[API][cal-ik-api]、[测试][cal-ik-source] | **现成库首选评估对象**。有实质测试证据且模块可裁小，但单次发布、单 owner、三个月无代码更新仍是维护风险。 |
| **ferromotion-core** | **0.73.0**，2026-09-15T12:36:05.764777Z | `from_urdf_str(xml, base, tip)`、动态 `Robot`、`Robot::jacobian`、`solve_ik(..., q0, opts)`；README 明示 7DoF ElRobot URDF 示例。[载入 API][ferro-load]、[Robot API][ferro-robot]、[IK API][ferro-ik]、[README][ferro-readme] | **功能成立，接入风险较高**。近期主分支 CI 失败；默认 pose-only IK 的限位/成功语义要额外处理；不宜以“新、纯 Rust”省略评估。 |
| **Optima / OptimaIK** | `optima` **0.0.4**，2022-10-18T18:32:32.319543Z | README 明示 Sawyer 7DOF、Panda/Gen3；公开 FK、Jacobian、静态/运动 IK 和 URDF 相关模型流程。[README][opt-readme]、[Jacobian API][opt-jac]、[IK API][opt-ik] | **不推荐替换 k**。版本与核心提交更旧；旧数学/URDF 依赖、多层资产/优化配置，普通依赖还带 PyO3，不能视作轻量纯 Rust 运行闭包。 |

### 精确版本、年龄、owner 与提交

| 项目 | 发布历史 / 许可 / 工具链 | 仓库年龄与最近代码提交（本次可见） | 公开维护主体 |
| --- | --- | --- | --- |
| Caliper | model 0.2.1：**2026-07-31T16:27:27.486972Z**；kinematics：**16:27:37.490162Z**；ik：**16:41:37.955702Z**；umbrella `caliper-robotics`：**18:12:50.331744Z**。四项各只有这一条发布记录；Apache-2.0，声明 Rust **1.89**。[model][cal-model-meta]、[kinematics][cal-kin-meta]、[ik][cal-ik-meta]、[umbrella][cal-umbrella-meta] | GitHub 创建 **2026-06-22T14:22:01Z**；最新提交 **400bccc0830632a70a8e385d1bab624e31f1da69**，**2026-07-31T18:44:34Z**，release 文案整理；不是近期求解器改进。[仓库][cal-repo]、[提交][cal-commit] | `caliper-ik` owner 为 **msannikov03 / Mikhail Sannikov**；GitHub contributors 返回该一个账号（127 commits），不能据此推断背后所有参与者。[owners][cal-owners]、[contributors][cal-contributors] |
| Ferromotion | core 最初发布 **2026-07-12T15:54:44.933531Z**；当前 **0.73.0 / 2026-09-15T12:36:05.764777Z**，前几版包括 0.72.0（09-06）、0.71.0（09-04）、0.68.0（09-01）。MIT OR Apache-2.0，声明 Rust **1.89**；密集 0.x 发布尚不是长期稳定 API 的证据。[元数据][ferro-meta] | GitHub 创建 **2026-07-12T15:51:34Z**；最近默认分支提交 **5daca8aac1b513a18c54792383d021cf1d27af56**，**2026-10-01T19:17:20Z**，关于 adhesion/sparse cones 的仿真验证。`pushed_at = 2026-11-07` 不应当成 11 月求解器更新。[仓库][ferro-repo]、[提交][ferro-commit] | core owner 为 **dcharlot-physicalai-bmi / David Charlot**。本次 contributors 请求被 GitHub 限流，**未确认更广的维护者覆盖**，不从机构名称推断团队规模。[owners][ferro-owners] |
| Optima | crate 初发 **2022-03-13**，当前 **0.0.4 / 2022-10-18T18:32:32.319543Z**；发布元数据许可 MIT OR Apache-2.0。[元数据][opt-meta] | 真正核心 `djrakita/optima` 创建 **2022-03-13T16:53:46Z**；最近提交 **d9b233eeb67e16d8fc6df2cc5fea4ee64ad4dab6**，**2023-08-07T22:35:54Z**。crate repository 指向的聚合仓库 `optima_toolbox` 有 2024-02-19 提交，**不要把它误记成核心较新维护**。[核心仓库][opt-repo]、[核心提交][opt-commit]、[聚合仓库][opt-aggregate] | crate owner 为 **djrakita / Danny Rakita**；本次核心 contributors 返回该一个账号。源码仓库 GitHub license 检测为 MIT，与 crate 双许可元数据不同；真要引入应查发布包许可文件，不能擅自合并解释。[owners][opt-owners]、[核心仓库][opt-repo] |

以上是观察到的单 owner 风险，不表示作者不可靠；也不能把新仓库、版本号高或功能数量当作成熟性的替代证据。

## Caliper：已发布 API 比 README 更有说服力

实际 Rust 接口为：

- `Model::from_urdf(&Path) -> Result<Model, CompileError>`；`frame_id(name)`、`ndof`、`joint_names`、`limits` 可供调用者选择真实末端与映射关节。`tip_frame()` 是最后登记的 link frame 默认值，**不应代替给定机器人 TCP 定义**。[Model API][cal-model-api]
- `caliper_kinematics::jacobian(&model, q, frame, JacFrame) -> (Se3, DMatrix<f64>)`，公开返回 **6×ndof、[linear; angular]**；非末端祖先的关节列为零。不是只有 solver 内部能取得 Jacobian。[具体 API][cal-jac-api]
- `caliper_ik::ik(&model, frame, &target, seed, &IkOpts) -> IkResult`。`IkOpts` 有位置/姿态容差、最大迭代、restarts、阻尼与 `step_clamp`；`IkResult` 有 `success`、`q`、`residual`、`iters`、`restarts_used`。[IK][cal-ik-api]、[选项][cal-opts]、[结果][cal-result]
- 实现使用 body-frame SE(3) 误差与对应 body Jacobian；阻尼正常方程有针对宽 Jacobian 的 floor。**7DOF 测试执行 50 个确定性构型的 FK→IK→FK 检查**，目标与 seed 都是 7 维，并要求完整位姿误差小于 1e-8。这是上游测试代码和 CI 证据，**不是本项目复现结果，也不是全可达域的成功率**。[0.2.1 源码，特别是 290–315 行][cal-ik-source]

限位与连续性不能只看 feature 名称。实现会先 clamp seed、每次更新后 clamp 关节；第一轮用传入 seed，后续重启从 limits 内随机采样，默认 `restarts = 8`。成功便返回该轮解。对连续点动，这是可能改变构型的实际行为；“有 seed”不代表不会跳解。可以通过公开参数限制重启次数，但邻近解验收、实际关节限位裕量、每步变化和控制时序仍需调用层负责。失败返回 `success = false` 和候选 q，不是硬件保持动作。[源码 62–89、166–183 行][cal-ik-source]

**测试/CI 质量证据：** workflow 在 macos-14 做 fmt、clippy、workspace tests、文档与 benchmark 编译；在 ubuntu 做 workspace tests；Python oracle job 安装 Pinocchio/NumPy 并执行 oracle tests。**2026-07-31T18:44:40Z 创建的核心 ci run 30656371909，head 为上述 400bccc，结论 success**。但本次没有逐 job 日志，因此不把 README 声称的全部交叉验证残差当作独立复核结论；且这不覆盖本项目 Tauri 集成。[workflow][cal-ci]、[具体成功运行][cal-ci-run]

此外，2026-10-03 和 2026-11-03 的 `pairing-watch` 运行结论为 failure，**不是上述核心 ci**；本次未取得失败原因和影响范围，不能据此说核心 Rust tests 失败，也不能把没有新提交的运行当作代码仍有新维护。[Actions 元数据][cal-actions]

**最小功能依赖组合：** 对显式使用 Model、Se3、FK/J 和 IK 的代码，优先评估 `caliper-model + caliper-spatial + caliper-kinematics + caliper-ik` **同为 0.2.1**，需要直接构造数学对象时配 `nalgebra 0.35`。ik 普通依赖仅为另三个 Caliper 组件与 nalgebra；kinematics 为 model/spatial/nalgebra；model 为 spatial/nalgebra/thiserror/urdf-rs；spatial 普通依赖仅 nalgebra。[ik 依赖][cal-ik-deps]、[kinematics][cal-kin-deps]、[model][cal-model-deps]、[spatial][cal-spatial-deps]

**注意没有完全摆脱旧 URDF 依赖：caliper-model 仍要求 urdf-rs `^0.9`。** 改善的是 nalgebra 升到 `^0.35`，以及较新的运动学代码与 API；不是所有依赖都换新。另一方面，umbrella `caliper-robotics` 即使默认 features 为空，仍有 dynamics、graph、HAL、motion、planning、trajopt 等 **12 个非 optional 普通依赖**；关闭默认 features 不能移除这些。MuJoCo 是另一个 optional dependency，库的轻量数学路径不要求它。[model 依赖][cal-model-deps]、[umbrella 依赖][cal-umbrella-deps]

**纯 Rust 结论的范围：** 上述小组合的已发布普通依赖及相关 Rust 数学实现未见必需 C++ solver、PyO3、MuJoCo 或 BLAS FFI。README 中 Python、Studio 和 MuJoCo 的运行需要不能转嫁到这个组合上。未进行目标平台完整依赖闭包构建或 binary 审计，因此尚不是 Mac arm64 软件验收通过的结论。

## Ferromotion：接口齐全，但便利 IK 仍需读失败与限位语义

最小 Rust 调用面实际不大：`from_urdf_str(xml, base, tip) -> Result<Robot, String>`，`Robot::fk(q) -> Iso`，`Robot::jacobian(q) -> DMatrix<f64>`，`solve_ik(&robot, &target, q0, &IkOptions) -> IkResult`。模型以 `Vec<Joint>` 和 `ee_offset` 表达任意长度串链与末端固定变换；Jacobian 是 world-frame 6×N、[linear; angular]。这些是公开接口，不是仅内部支持。[载入][ferro-load]、[Robot][ferro-robot]、[IK][ferro-ik]

七轴证据强度低于 Caliper 的专门回归：通用 N 维实现和上游明确的 **7DoF ElRobot URDF 例子** 支持七轴能力；本次核对的 lib 中 Jacobian/IK 单测使用 planar 3R，URDF loader 的所读测试也不是专门的七轴 IK 回归。**没有据此断言整个项目不存在其它七轴测试。**[README][ferro-readme]、[已发布 lib 源码][ferro-source]、[URDF 源码][ferro-urdf-source]

两项接入风险来自具体实现：

1. `solve_ik` 只是把一个 `PoseCost` 交给通用 `solve`；该函数的更新没有按 `Joint.limits` clamp。URDF loader 确实保存 limits/effort/velocity，库也有 `JointLimitCost`，但它明确定义为 **soft hinge penalty**，并非硬限位保证，且便利 IK 没有自动加入它。不能从“支持 limits”推断默认求解结果总在范围内。[solve_ik/solve 实现][ferro-source]、[Joint API][ferro-joint]、[JointLimitCost API][ferro-limit-cost]
2. `IkOptions.tol` 控制迭代退出，但结果 `converged` 在所核对的 0.73.0 实现中使用固定 `error < 1e-4`，并非直接使用 opts.tol；默认 opts.tol 是 1e-10。调用者需要依据目标位姿重新验收，而不能仅看 converged。`IkResult` 实为 `SolveResult` 的 alias，公开 q/error/iters/converged。[具体源码][ferro-source]、[结果 API][ferro-result]

core 的**非 optional 普通依赖**是 `nalgebra ^0.35`（std）、`urdf-rs ^0.9`、`faer ^0.24`（default-features=false，仅 std）和 `clarabel ^0.11`（默认 features）。GPU 的 wgpu 等是 optional，不是本任务必要条件。Clarabel 0.11.1 的默认 feature 是 serde，BLAS/LAPACK 属于另外开启的 SDP 路径；不能仅因“有优化器”就误判必需 C++/BLAS，也不能把 README 的“只有 nalgebra/urdf-rs”当成完整发布依赖。[core 依赖][ferro-deps]、[Clarabel 元数据][clarabel-meta]

**最小直接依赖**是 `ferromotion-core 0.73.0`，需要显式构造 Iso/数学对象时加兼容 `nalgebra 0.35`；不需要引入 umbrella、控制、可视化、GPU 或 Python。问题在于 core 自身不可按 kinematics/IK 功能裁分；其官方 0.73.0 文档称约 150 modules，许多领域与本 MVP 无关。这不是笼统认定功能多就难用，而是单个依赖的审阅、版本变化和排错范围确实更宽。[core API 文档][ferro-api]

**CI 证据：** workflow 包括 Ubuntu workspace test、wasm build、all-features clippy、文档构建；最近 core 所在 workspace 的 CI run **36913181213**，创建 **2026-10-01T19:17:26Z**，head **5daca8a…**，结论 **failure**。job/log 请求被 GitHub 限流，未取得具体原因，不能断言是 IK 测试失败、编译错误或环境问题；也没有当前 Darwin arm64 检查通过的记录。[workflow][ferro-ci]、[具体运行][ferro-ci-run]

对一天内的软件开发，Ferromotion 的优势是现成 API，风险是扩大审阅面、默认 IK 的额外验收责任、失败的最近 CI 与较短维护历史。本次不据此推荐覆盖现有选择。

## Optima：完整但不解决“更活跃、更轻”的问题

`RobotKinematicsModule::compute_jacobian(...) -> Result<DMatrix<f64>, OptimaError>` 是真实公开 API；`OptimaIK::new_static_ik` / `solve_static_ik` 以及 motion IK 同样存在。静态求解支持 initial_condition、max_num_tries 等，但要准备 robot set、几何场景、优化器、goal/spec 与变量容器。`RobotModelModule::new(robot_name)` 的名字指向 `optima_assets/optima_robots` 中的文件夹，读取其中 URDF，不是直接接收任意 URDF 字符串的轻量入口，模型接入需要适配其资产体系。[Jacobian API][opt-jac]、[IK API][opt-ik]、[模型 API][opt-model]

发布普通依赖包括 **nalgebra ^0.30.1、urdf-rs ^0.6.2**，没有缓解用户担心的版本年龄。`pyo3 ^0.16.2` 且 feature 为 `extension-module` 是 **非 optional** 普通依赖，意味着 CPython ABI/build 环境边界不能靠 default-features=false 去掉；这与“核心算法本身用 Rust”是两个问题。Bevy 可选且默认打开，NLopt 为 optional；不用 NLopt 可以避开那条 native 优化器路径，但仍不能称已获得轻量无 native 依赖的纯 Rust 运行闭包。[已发布依赖][opt-deps]

最近核心提交在 2023 年，核心 GitHub Actions API 本次返回无运行记录，未核实到可依赖的当前测试结果。故既不因其较长年龄就认定更成熟，也不认为它符合本次“更活跃替代 k”的目标；无需为它进行更多集成调查。[核心提交][opt-commit]、[Actions][opt-actions]

## 其余三项为什么未进入完整候选

| 项目 | 已核对事实 | 筛除理由 |
| --- | --- | --- |
| **Kinetix** | `kinetix-core` / `kinetix-urdf` **0.1.0**，分别发布于 **2026-06-30T12:30:01.217366Z / 12:30:03.156534Z**。仓库创建 2026-06-25；当前可见提交 `eff0510cc9252cb9da1ba2599cfd7ac7d039e240`，2026-06-30T12:27:32Z。[core 元数据][kin-meta]、[urdf 元数据][kin-urdf-meta]、[提交][kin-commit] | 上游 README 明确说目前没有 IK，把 kinematics/Jacobian API 列在 roadmap；虽然动力学 benchmark 有 7DoF，也不能据此声称有七轴 IK。仅 RevoluteZ/PrismaticZ/Fixed 的当前模型覆盖也须留意。适合关注，不适合填补本次即用 IK。[README][kin-readme] |
| **featherstone** | **0.1.0 / 2026-03-22T21:58:34.545434Z**，Apache-2.0，单 owner starswallow。最近提交 `5e3f1b24be9ca8fbbaff12ec2dfd4278f63377d7`，**2026-03-23T05:12:54Z**，只是 README badge；Actions 未见运行记录。[元数据][feather-meta]、[owner][feather-owners]、[提交][feather-commit]、[Actions][feather-actions] | 动态树、URDF、公开 `body_jacobian -> DMatrix<f32>` 存在，但 0.1.0 全部公开项目中未见通用位姿 IK solver。Jacobian 排序还是 **[angular; linear]**，与前两库相反。普通依赖 nalgebra ^0.33，默认 URDF feature 开启 urdf-rs ^0.8；选择它仍要自行实现 IK，并增加 f32/约定适配，未体现一天内收益。[Jacobian API][feather-jac]、[公开项目][feather-all]、[依赖][feather-deps] |
| **nu-jliu/modern-robotics-rs** | GitHub 创建 **2025-06-05**；最近提交 `d402e540d594be875bfb607035f90e3eae02d298`，**2025-12-11T01:44:06Z**，有成功的 Rust CI 运行记录。源码 manifest 包名是 **modern-robotics / 0.1.2**、MIT、nalgebra 0.34.0；本次该包名的 crates.io 查询 404，不能把源码版本当已发布包。[仓库][modern-repo]、[提交][modern-commit]、[manifest][modern-manifest] | 公开的是 screw axes、FK、Newton–Raphson IK 与 6×N Jacobian 等数学函数；所读 README/manifest 未给出 URDF loader。它是算法参考，不是已经闭合“URDF + limits + 七轴位姿 IK”的直接替代。另一个 `modern_robotics_rust` crate 指向 **TomtheCoder2 的不同仓库**，本次特意不混用其 docs.rs 来证明 nu-jliu 版本。[README][modern-readme]、[同名不同来源的元数据][modern-other] |

对“未见 CI/未见 API”的结论，范围仅限本次公开 Actions/API/README 检查，不断言作者从未测试或仓库绝无其它验证。

## 推荐范围与未验收事项

**库复用路线的比较顺序：Caliper 小组合 → Ferromotion core；Optima 不再投入一天预算。** Caliper 有足够具体的公开 API 和七轴测试证据，值得成为固定 7R 局部 DLS 的对照，而不是因为是新库就一概排除；但本次证据不足以要求项目为它改换路线。

与“自行完成固定 7R 的局部 FK/Jacobian/DLS”相比，节省的是哪些工作，可以限定为：

| 候选 | 可复用、从而省去自行实现的部分 | 仍要支付的接入工作 | 够不够当首版依赖 |
| --- | --- | --- | --- |
| Caliper 小组合 | URDF 编译成模型与 frame、FK、body/world Jacobian、带阻尼/步长夹紧/位置限位的局部 IK；还有专门的七轴回归可参考。即覆盖本题的三个主要数值内核及模型转换。 | 对齐 Se3/frame/关节顺序与 TCP，控制重启以避免构型切换；实际模型的软件验收、连续目标、输出门限、错误/停止状态仍需项目负责。 | **有条件的首版数学内核备选**，证据足以进入软件验证；单次发布、单 owner 和停更时间不足以支持无条件采用。使用独立 crates，避免 umbrella。 |
| Ferromotion core | URDF base→tip 选链、FK/Jacobian、LM 与 seed；能省下数值算法代码。 | 默认 IK 没有硬限位，converged 不能代替目标容差验收；仍需补策略/包装，并排查最近 CI 与发布包是否相关。不可裁开的 core 和额外依赖扩大审阅范围。 | **本次不作为首选**。比 Caliper 更多的功能不等于本 MVP 节省更多接入工作；净收益尚未证实。 |
| Optima | 现成 FK/Jacobian/IK 及通用机器人模型。 | 资产预处理、robot set/scene/optimizer 体系、旧版本和 PyO3 环境；同时没缓解近期维护与旧依赖问题。 | **本次不推荐**；没有证据显示这些额外接入工作比复用局部数学内核更合算。 |

本次没有构建或集成，因此**不把上述节省折算成小时，也不承诺哪条路线能在一天的开发时间内验收**。它们不会省去本项目的软件/真机验收责任；nalgebra 自写路线的具体设计与一天计划由并行评估承担。

两项新候选都改善 nalgebra 版本，但都仍使用 urdf-rs 0.9；它们没有自动消除“维护者少、0.x API、URDF 与本机定义接合、构型连续/奇异策略/控制节拍”的责任。现成 IK 或库中 resolved-rate 相关函数都不等于本项目完整 servo，更不等于真机 MVP 已完成。

后续如进行软件验收，需要验证实际左右 7R URDF、各侧 limits、具名末端与显式 TCP、FK/Jacobian 数值一致性、附近 6D 目标的成功与失败行为、关节增量连续、奇异与限位响应、Tauri 启停/取消和 Mac arm64 的运行表现。其它平台此时可保留便携的接口与依赖选择，运行验收单列；硬件反馈、协议时序和停止行为始终列为 **真机未验收**。本轮没有执行这些检查，也不替主代理展开一天 MVP 计划。

## 一级来源

链接固定版本的 docs.rs 与 crates.io 对应版本依赖；README/workflow 为调查当日读取的默认分支。CI 成败来自实际运行元数据，不从 badge 推断。

[cal-repo]: https://api.github.com/repos/msannikov03/caliper
[cal-commit]: https://github.com/msannikov03/caliper/commit/400bccc0830632a70a8e385d1bab624e31f1da69
[cal-ik-meta]: https://crates.io/api/v1/crates/caliper-ik
[cal-model-meta]: https://crates.io/api/v1/crates/caliper-model
[cal-kin-meta]: https://crates.io/api/v1/crates/caliper-kinematics
[cal-umbrella-meta]: https://crates.io/api/v1/crates/caliper-robotics
[cal-owners]: https://crates.io/api/v1/crates/caliper-ik/owners
[cal-contributors]: https://api.github.com/repos/msannikov03/caliper/contributors?per_page=5
[cal-model-api]: https://docs.rs/caliper-model/0.2.1/caliper_model/struct.Model.html
[cal-ik-api]: https://docs.rs/caliper-ik/0.2.1/caliper_ik/fn.ik.html
[cal-jac-api]: https://docs.rs/caliper-kinematics/0.2.1/caliper_kinematics/fn.jacobian.html
[cal-opts]: https://docs.rs/caliper-ik/0.2.1/caliper_ik/struct.IkOpts.html
[cal-result]: https://docs.rs/caliper-ik/0.2.1/caliper_ik/struct.IkResult.html
[cal-ik-source]: https://docs.rs/caliper-ik/0.2.1/src/caliper_ik/lib.rs.html
[cal-ci]: https://github.com/msannikov03/caliper/blob/main/.github/workflows/ci.yml
[cal-ci-run]: https://github.com/msannikov03/caliper/actions/runs/30656371909
[cal-actions]: https://api.github.com/repos/msannikov03/caliper/actions/runs?per_page=2
[cal-ik-deps]: https://crates.io/api/v1/crates/caliper-ik/0.2.1/dependencies
[cal-kin-deps]: https://crates.io/api/v1/crates/caliper-kinematics/0.2.1/dependencies
[cal-model-deps]: https://crates.io/api/v1/crates/caliper-model/0.2.1/dependencies
[cal-spatial-deps]: https://crates.io/api/v1/crates/caliper-spatial/0.2.1/dependencies
[cal-umbrella-deps]: https://crates.io/api/v1/crates/caliper-robotics/0.2.1/dependencies
[ferro-meta]: https://crates.io/api/v1/crates/ferromotion-core
[ferro-repo]: https://api.github.com/repos/dcharlot-physicalai-bmi/ferromotion
[ferro-commit]: https://github.com/dcharlot-physicalai-bmi/ferromotion/commit/5daca8aac1b513a18c54792383d021cf1d27af56
[ferro-owners]: https://crates.io/api/v1/crates/ferromotion-core/owners
[ferro-readme]: https://github.com/dcharlot-physicalai-bmi/ferromotion/blob/main/README.md
[ferro-api]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/
[ferro-load]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/fn.from_urdf_str.html
[ferro-robot]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/struct.Robot.html
[ferro-ik]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/fn.solve_ik.html
[ferro-source]: https://docs.rs/ferromotion-core/0.73.0/src/ferromotion_core/lib.rs.html
[ferro-urdf-source]: https://docs.rs/ferromotion-core/0.73.0/src/ferromotion_core/urdf.rs.html
[ferro-result]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/type.IkResult.html
[ferro-joint]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/struct.Joint.html
[ferro-limit-cost]: https://docs.rs/ferromotion-core/0.73.0/ferromotion_core/struct.JointLimitCost.html
[ferro-deps]: https://crates.io/api/v1/crates/ferromotion-core/0.73.0/dependencies
[ferro-ci]: https://github.com/dcharlot-physicalai-bmi/ferromotion/blob/main/.github/workflows/ci.yml
[ferro-ci-run]: https://github.com/dcharlot-physicalai-bmi/ferromotion/actions/runs/36913181213
[clarabel-meta]: https://crates.io/api/v1/crates/clarabel/0.11.1
[opt-meta]: https://crates.io/api/v1/crates/optima
[opt-repo]: https://api.github.com/repos/djrakita/optima
[opt-aggregate]: https://api.github.com/repos/djrakita/optima_toolbox
[opt-commit]: https://github.com/djrakita/optima/commit/d9b233eeb67e16d8fc6df2cc5fea4ee64ad4dab6
[opt-owners]: https://crates.io/api/v1/crates/optima/owners
[opt-readme]: https://github.com/djrakita/optima_toolbox/blob/main/README.md
[opt-jac]: https://docs.rs/optima/0.0.4/optima/robot_modules/robot_kinematics_module/struct.RobotKinematicsModule.html#method.compute_jacobian
[opt-ik]: https://docs.rs/optima/0.0.4/optima/inverse_kinematics/struct.OptimaIK.html
[opt-model]: https://docs.rs/optima/0.0.4/optima/robot_modules/robot_model_module/struct.RobotModelModule.html
[opt-deps]: https://crates.io/api/v1/crates/optima/0.0.4/dependencies
[opt-actions]: https://api.github.com/repos/djrakita/optima/actions/runs?per_page=2
[kin-meta]: https://crates.io/api/v1/crates/kinetix-core
[kin-urdf-meta]: https://crates.io/api/v1/crates/kinetix-urdf
[kin-commit]: https://github.com/Robot-Exp-Platform/kinetix/commit/eff0510cc9252cb9da1ba2599cfd7ac7d039e240
[kin-readme]: https://github.com/Robot-Exp-Platform/kinetix/blob/main/README.md
[feather-meta]: https://crates.io/api/v1/crates/featherstone
[feather-owners]: https://crates.io/api/v1/crates/featherstone/owners
[feather-commit]: https://github.com/softmata/featherstone/commit/5e3f1b24be9ca8fbbaff12ec2dfd4278f63377d7
[feather-actions]: https://api.github.com/repos/softmata/featherstone/actions/runs?per_page=2
[feather-jac]: https://docs.rs/featherstone/0.1.0/featherstone/kinematics/fn.body_jacobian.html
[feather-all]: https://docs.rs/featherstone/0.1.0/featherstone/all.html
[feather-deps]: https://crates.io/api/v1/crates/featherstone/0.1.0/dependencies
[modern-repo]: https://api.github.com/repos/nu-jliu/modern-robotics-rs
[modern-commit]: https://github.com/nu-jliu/modern-robotics-rs/commit/d402e540d594be875bfb607035f90e3eae02d298
[modern-manifest]: https://github.com/nu-jliu/modern-robotics-rs/blob/master/Cargo.toml
[modern-readme]: https://github.com/nu-jliu/modern-robotics-rs/blob/master/README.md
[modern-other]: https://crates.io/api/v1/crates/modern_robotics_rust

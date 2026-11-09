# R1-7A model resources

来源：Unitree 官方 `unitreerobotics/r1-7a_sdk`，SDK VERSION 0.1.0，固定 commit `0803d033b530fcfa0e6fa9236de68c2773fc9402` 的 `urdf/URDF_R1-7A.tar.gz`。压缩包 SHA-256 为 `dca2cea68150891b5312108dd7c448103d0f0d67690fde0918cea23b41b65e25`。完整来源路径、原始/归一化 URDF hash、逐 STL hash 与七轴名称/位置限位见 `manifest.json`。

右臂来自 R_V1.2-1023 的 `urdf/R1_Robotic_Arm.urdf`；左臂来自 L_V1.0-1027 的 `urdf/R1_Robotic_Arm_L_V1.0-1027.urdf`，不是左包 `meshes/` 目录的另一份 URDF。左右各自的 8 个 STL 直接保留原始二进制。仅将 URDF 的 mesh filename 改为 `meshes/文件名`；未修改 joint/link 的 origin、axis、limit、inertial 或几何。两侧使用各自模型，不以镜像替代。

`UNITREE-LICENSE` 保留 SDK 根目录的 BSD-3-Clause 版权及许可原文；两侧包内 package.xml 均声明 BSD，压缩包未包含另一份独立许可证文本。未执行包内脚本、launch 或构建文件，未展开 XML 外部实体。

前端 publicDir 为 resources 时，入口为 `/models/right/robot.urdf` 和 `/models/left/robot.urdf`。后端编译内嵌这两份 URDF，base 为 `base_link`，flange 为 `Link7`，TCP 是 Link7→工具的完整刚体变换。米、弧度、秒；四元数 xyzw。

## 数值实现与软件检查

`src-tauri/src/kinematics.rs` 提供固定 7R 的 FK、TCP geometric Jacobian 和无状态局部加权 DLS。按父子链遍历，保留固定关节变换，拒绝非串链、mimic、非有限参数及不支持的关节类型。Jacobian 的行顺序是基座表达的 TCP 线速度、角速度。

局部位姿反馈增益为 4/s，单次目标离参考 TCP 不超过 0.1 m / 0.35 rad；dt 为 1 μs 到 100 ms，实际单关节增量取配置与 0.1 rad 中较小者。MotionLimits 各正参数要求有限且在 [1e-9, 1000]。关节速度取配置与 URDF 限制中较小者。

加权 SVD 最多 64 次迭代，用归一化最小奇异值调整正阻尼；6×6 Cholesky 求解后校验残差。约束用共同缩放区间，包括位置、速度、加速度、参考对实测偏差、增量，以及按配置加速度计算的单步制动距离。最终用实际 FK 增量再次验证 TCP 速度上限、任务方向及位姿误差进展。阻塞、严重方向残差、无法满足加速度的反向等返回 String 错误，调用者应停止/重同步，不能提交失败候选。`speed_scale` 是相对原请求的已实现速度投影（[0,1]），极小非零请求无可分辨进展时为 0；零请求成功保持时为 1。无 Result 的 `pose_error` 对非法输入返回六个正无穷分量，不产生 NaN；对外输入须先校验。

独立基准见 `fk-reference.json`。生成器 `verify_fk_reference.py` 仅用 Python 标准库标量 4×4 矩阵，分别手录官方左右臂参数，不调用生产 Rust FK、nalgebra 或 URDF 解析器。包含两侧零位、两个非对称构型及非零 TCP 变换，可运行 `python3 resources/models/verify_fk_reference.py` 重现。

2026-11-09 在 macOS arm64 上，以独立临时最小 crate（nalgebra 0.35.0、urdf-rs 0.10.0、serde/serde_json）运行 14 项测试全部通过：独立矩阵 FK、带固定原点和乱序 XML 的解析几何基准、两侧多构型/两种步长的 Jacobian 中心差分、基座/工具平移旋转、xyzw/四元数符号/微小角、连续与位姿步一致性、两侧多步邻近目标收敛、动态约束实际激活、上下限与参考偏差、奇异可行/不可行方向、无效数值与模型拒绝。还核对了原始 URDF 到发布 URDF 仅有 mesh filename 变化，全部 16 STL 的 hash、二进制长度及模型引用存在性。

这是离线软件与模型数学验证。真实编码器映射、TCP、负载下动态限制、制动行为和真机方向未验收；模型制动距离不是设备实测结果。未在本模块任务中运行整仓 Cargo 构建或改动共享构建配置。

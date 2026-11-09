# 第三方版权与许可

本项目新增代码采用根目录的 [BSD-3-Clause 许可证](LICENSE)。以下上游材料保留各自的版权与许可；项目许可证不改变第三方材料的授权。

## Unitree R1-7a SDK

- 上游：[unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk)。
- 采用的版本：commit `0803d033b530fcfa0e6fa9236de68c2773fc9402`。
- 原始许可：[该版本的 LICENSE](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/LICENSE)，BSD-3-Clause。
- 原始版权声明：`Copyright (c) 2026 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")`。

原始许可证全文保存在 [resources/models/UNITREE-LICENSE](resources/models/UNITREE-LICENSE) 和 [resources/calibration/UNITREE-LICENSE](resources/calibration/UNITREE-LICENSE)。分发这些材料时应保留版权、条件及免责声明；二进制分发也应附带相应声明。

本仓库使用或参考了以下材料：

| 本地路径 | 来源与处理 |
|---|---|
| `resources/models/{left,right}/robot.urdf` 与 `meshes/` | 来自 SDK 的 `urdf/URDF_R1-7A.tar.gz`；STL 保持原始二进制，URDF 仅归一化 mesh 文件路径。版本、路径和 SHA-256 见 [manifest.json](resources/models/manifest.json)。 |
| `resources/calibration/joint_calib_{left,right}.txt` | SDK `config/` 中的官方示例标定文件，保留原始内容；不是对用户本体的标定结果。 |
| `src-tauri/src/protocol.rs` | Rust 协议实现参考官方电机报文、CRC、缩放与标定约定。 |
| `resources/calibration/protocol-golden.json` 与生成器 | 使用上述 SDK 固定版本的原始 `MotorProtocol.cpp` 离线生成协议对照；SDK 原始实现不作为本项目运行时依赖。复现方式见 [说明](resources/calibration/README.md)。 |

宇树及其贡献者没有为本项目背书。本项目不是宇树官方软件。

## 软件依赖

通过 npm 和 Cargo 获取的依赖继续适用各自的许可证；本仓库不包含 `node_modules/` 或 Cargo 构建目录。版本由 [package-lock.json](package-lock.json) 与 [Cargo.lock](src-tauri/Cargo.lock) 固定。

前端直接运行依赖的许可信息：

| 依赖 | 许可证 |
|---|---|
| `@tauri-apps/api` | Apache-2.0 OR MIT |
| `react`、`react-dom` | MIT |
| `three` | MIT |
| `urdf-loader` | Apache-2.0 |

制作或再分发应用二进制时，还需遵守实际打包依赖各自的版权及许可要求。

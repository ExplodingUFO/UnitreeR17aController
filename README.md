# R1-7a 控制器

这是一个通过 **vibe coding、由 AI Agent 辅助开发的个人自用实验项目**，用于在电脑上连接、调试宇树 R1-7a 机械臂。它不是宇树官方软件，也未获得宇树官方背书。

**真机尚未验收。** 目前完成的是 macOS arm64 上的软件 MVP、离线测试及界面检查。真实串口与 USB 转接器、用户本体标定、运动跟随、停止和失联行为均未完成真机验证；Windows/Linux 的构建和设备验证也未完成。模型显示、数值测试和软件测试通过不代表实机已经验证。

软件侧已实现 Tauri 2 桌面应用、Rust 控制后端、本机 HTTP/WebSocket、七轴关节与末端点动、官方模型 3D 和 SQLite 结构化事件日志。测试、构建及验收记录见[实施状态](docs/implementation-status.md)。

![无设备时的工作台，3D 显示参考姿态](docs/screenshots/resizable-panels-expanded.jpg)

## 许可与来源

本项目新增代码采用 **[BSD-3-Clause](LICENSE)**，允许商用，需遵守许可证条款。“个人自用”描述本项目的开发目的，不构成非商业使用限制。

官方模型、示例标定及协议参考来自 [Unitree R1-7a SDK](https://github.com/unitreerobotics/r1-7a_sdk)，保留宇树原始版权声明和 BSD-3-Clause 许可。具体来源、固定版本和第三方许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 运行与构建

在 macOS arm64 上准备 Node.js/npm、Rust 工具链和 Xcode Command Line Tools，从仓库根目录运行：

```bash
npm ci
npm run tauri dev
```

开发命令同时启动 Vite 与 Tauri 后端；单独运行 `npm run dev` 只启动前端开发服务器。构建生产应用：

```bash
npm run tauri -- build --bundles app
```

未指定额外 Cargo target 或输出目录时，预期应用位置为：

```text
src-tauri/target/release/bundle/macos/R1-7a Controller.app
```

可用 `open "src-tauri/target/release/bundle/macos/R1-7a Controller.app"` 启动。实际产物与检查结果以[实施状态](docs/implementation-status.md)为准。

软件检查：

```bash
npm run format:check
npm run build
cargo test --manifest-path src-tauri/Cargo.toml --lib --locked
cargo clippy --manifest-path src-tauri/Cargo.toml --all-targets --locked -- -D warnings
```

## 使用与边界

工作台集中显示连接、七轴、3D、点动和日志，目标布局为 1280×720 与 1024×768 无页面级滚动。先选择左右臂和串口，显式取得控制、连接并使能；关节与末端动作互斥，末端点动还要求有效 TCP 配置。宿主停止可取消任一客户端的当前动作，保留控制权且不自动失能。

面板之间的分隔线可以拖动：调整左右栏宽度、3D/末端坐标区的高度和日志高度。双击分隔线恢复该项默认尺寸，调整结果自动记忆。分隔线聚焦后可用方向键微调，Shift 加方向键加大步幅，Enter 还原。空间不足时关节表和点动列表在各自区域内滚动。

无设备时关节实测数据和计算 TCP 为 `null`，3D 显示参考姿态；生产界面仅显示当前状态、数据来源、操作和必要错误原因。本机接口默认监听 `127.0.0.1:8177`，使用会话认证与控制租约；凭据不进入日志。SQLite 日志由专用后台线程写入，有界队列满或数据库故障不会让控制线程等待磁盘。

当前交付环境为 macOS arm64。机械臂和转接器不可访问，真机尚未验收；协议默认波特率 6 Mbps，仍需验证实际转接器和本体的通信、标定、运动、停止及失联表现。Windows/Linux 的构建、运行与真机验证均未验收。

- [首版规格](docs/spec.md)：功能、线程、接口、退出与验收边界。
- [实施接口](docs/implementation-contract.md)：模块与数据契约。
- [架构决策](docs/adr/0001-controller-architecture.md)：集中 Rust 后端、局部数值计算与异步 SQLite 日志。
- [历史原型说明](docs/prototype.md)：保留原交互评审记录，与当前产品及验收证据分开。

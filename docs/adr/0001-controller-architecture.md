---
status: accepted
date: 2026-11-09
---

# 在 Tauri 内集中运行纯 Rust 控制后端

首版需要电脑直连 R1-7a、本机服务和桌面工作台，用户要求纯 Rust 评估后定稿，并优先维护集中、职责清楚的实现。决定采用 Tauri 2 内的 Rust 后端，不另起 C++ 服务；串口使用 serialport，模型解析使用 urdf-rs，以 nalgebra 实现当前固定七轴的 FK、TCP Jacobian 和局部加权 DLS。该方案已用于当前 macOS arm64 软件 MVP。详细版本与验证门槛见[规格](../spec.md)，公开接口见[实施接口](../implementation-contract.md)，测试、构建及打包证据见[实施状态](../implementation-status.md)。

官方 SDK 提供底层串口和标定，不能直接提供笛卡尔点动。采用 C++ 仍要补运动控制，并增加本项目的跨语言与进程维护面。k 的旧数学依赖需要额外兼容；Caliper 有七轴与 CI 依据，但维护历史较短，其通用求解行为仍需适配连续点动。首版选择自己掌握有限算法与失败行为，代价是必须承担独立 FK、Jacobian 和约束验证，不能把更少依赖当成正确性的证据。依据见[SDK 调查](../research/r1-7a-sdk.md)、[活跃库调查](../research/active-rust-kinematics.md)与[nalgebra 评估](../research/nalgebra-direct-assessment.md)。

控制状态、目标和最终提交集中在 arm，protocol 处理当前设备报文，kinematics 只计算显式输入的候选，server 处理本机 HTTP/WebSocket，journal 独立处理 SQLite 事件持久化。桌面一般操作复用本机接口，Tauri 只保留会话引导、宿主停止和生命周期协调。宿主停止与外部命令调用同一控制模块，既不抢占控制权，也不自动失能。

SQLite 日志是当前产品要求，采用 rusqlite 0.41.1（bundled）和 crossbeam-channel。控制线程仅执行有界字段拷贝、try_send 和原子统计读取；初始打开、建表、WAL、批量事务、有限 busy_timeout、失败退避、保留最近 100000 条及 checkpoint 均在专用 writer 线程。队列饱和丢弃并计数，数据库失败由后台重试，不反向阻塞控制。只读 recent 查询按 id 降序且最多 500 条，由 server 的 spawn_blocking 执行；关闭协调有超时，Drop 不阻塞。日志只保存结构化事件，不保存逐帧遥测或 token/lease。

正式前端保持紧凑单页，在 1280×720 与 1024×768 中同时呈现连接、七轴、3D、点动与日志，具体布局证据见实施状态。生产 UI 只显示状态、数据来源、操作和必要错误原因，不放原型标识、验收免责声明或解释段落。无设备时实测 q/dq/tau、反馈年龄和计算 TCP 为 null，3D 为明确的参考姿态；原型演示数据不进入产品后端。

当前交付环境为 macOS arm64，软件测试与最终产物证据由实施状态维护。Windows/Linux 构建、运行及设备验证均未验收。机械臂和 USB 转接器尚不可访问；协议默认 6 Mbps，实际通信、标定、跟随、停止与失联行为仍需真机验收。

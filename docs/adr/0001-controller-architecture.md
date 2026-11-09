---
status: accepted
date: 2026-11-09
---

# 在 Tauri 内集中运行纯 Rust 控制后端

控制器需要电脑直连 R1-7a、桌面工作台和本机 API。后端运行在 Tauri 2 应用内，串口使用 serialport，模型解析使用 urdf-rs，以 nalgebra 实现七轴 FK、TCP Jacobian 和局部加权 DLS。详细版本见[规格](../spec.md)，公开接口见[接口说明](../implementation-contract.md)，构建与测试结果见[测试记录](../implementation-status.md)。

官方 SDK 提供底层串口和标定，笛卡尔点动需要另外实现。使用纯 Rust 后端可以省去 C++ 绑定和独立服务进程。k 使用较旧的数学依赖，需要额外兼容；Caliper 有七轴测试和 CI，但维护历史较短，通用求解行为仍需适配连续点动。因此采用 nalgebra 实现当前模型所需的局部算法，并通过独立 FK 基准、Jacobian 差分和约束测试检查结果。选型依据见[SDK 调查](../research/r1-7a-sdk.md)、[运动学库比较](../research/active-rust-kinematics.md)与[nalgebra 评估](../research/nalgebra-direct-assessment.md)。

arm 管理控制状态、目标和输出，protocol 处理设备报文，kinematics 计算运动候选，server 提供本机 HTTP/WebSocket，journal 写入 SQLite 日志。桌面操作复用本机接口，Tauri 提供会话信息、桌面停止命令和应用启停。停止按钮调用同一控制模块，控制权和使能状态保持不变。

SQLite 日志是当前产品要求，采用 rusqlite 0.41.1（bundled）和 crossbeam-channel。控制线程仅执行有界字段拷贝、try_send 和原子统计读取；初始打开、建表、WAL、批量事务、有限 busy_timeout、失败退避、保留最近 100000 条及 checkpoint 均在专用 writer 线程。队列饱和丢弃并计数，数据库失败由后台重试，不反向阻塞控制。只读 recent 查询按 id 降序且最多 500 条，由 server 的 spawn_blocking 执行；关闭协调有超时，Drop 不阻塞。日志只保存结构化事件，不保存逐帧遥测或 token/lease。

前端采用单页三栏布局，在 1280×720 与 1024×768 中同时显示连接、关节、3D、点动与日志。各面板尺寸可以拖动调整。无设备时，q/dq/tau、反馈时间和计算 TCP 为 null，3D 显示参考姿态。

当前已在 macOS arm64 完成构建和软件测试。Windows、Linux 及真机测试项目见[测试记录](../implementation-status.md)。

# R1-7a 官方手册核验

核验日期：2026-11-09。通过浏览器阅读官方正文；直接网页抓取未能读取动态页面。本文记录事实，不替代最新 SDK 源码或实机测试。

## 官方资源与型号边界

- [用户手册入口](https://support.unitree.com/home/zh/R1-7a/R1%20Robotic%20Arm)。本项目指独立 R1-7a 七轴机械臂。
- [获取 SDK 与开发资源](https://support.unitree.com/home/zh/R1-7a/get_SDK)，页面更新时间 2026-10-12：直接链接 [r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk)、其 `urdf/` 目录以及 STEP 压缩包。
- [关于 R1-7a](https://support.unitree.com/home/zh/R1-7a/about%20_R1-A7)，页面更新时间 2026-09-23：参数表列出七轴、用户控制系统 Ubuntu、位置与力控接口。这不代表存在笛卡尔运动 API。
- 人形机器人 R1 / R1-A7 的 DDS SDK 示例不能直接当成该独立机械臂的接口。

## 连接与运动接口

[使用及 SDK 部署](https://support.unitree.com/home/zh/R1-7a/Usage_and%20SDK%20Deployment)，页面更新时间 2026-09-28：

- 通讯采用 485，通过厂商 USB 转接器连接用户电脑；示例主控为 Linux。
- SDK 目录包括源码、头文件、底层 485 通讯库、关节标定文件及 URDF。
- `test_arm_lowlevel` 展示保持、回零、指定七个关节角度、停止、清错等操作。`--goto --q` 的七个值单位为弧度，使用 URDF 关节坐标。
- 手册说明示例的关节运动使用限步长逼近，允许替换运动算法。该说明不足以把 `--goto` 当作 TCP 直线运动。
- 手册示例 `--rate` 默认为 50 Hz，并标称机械臂最多支持 700 Hz。这是手册声明，尚未验证特定电脑、转接器、OS 或全关节反馈下的持续吞吐和抖动。最新 SDK 例程频率应另核对。
- 文档介绍电机状态和超时保护，但未证明 mode 0、零力矩与稳定保持具有相同物理效果；断连后的具体停机策略须结合负载、安装方式及实机确认。

## 可确认与不能确认

官方确实给出了低层 SDK、URDF 和连接说明。已读手册未展示 TCP 位姿、Cartesian twist、MoveL 或笛卡尔点动 API。后续[SDK 源码核验](r1-7a-sdk.md)确认固定 commit 的公开接口也没有这些能力，具体范围与覆盖证据见该记录。

官方页面没有证明 Windows/macOS 完整直连受支持。跨平台 UI 与跨平台底层总线支持必须分别验收。

本文未连接机械臂，未发送任何电机命令。官方 700 Hz 标称值不作为本项目承诺。

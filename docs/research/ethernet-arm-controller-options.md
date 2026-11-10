# R1-7a 网口控制盒选型

检查日期：2026-11-10。

按现有 Rust 控制器和尽快投入使用的目标，优先推荐 NanoPi NEO3 Plus 加原装 USB 转 RS-485。控制服务放在机械臂旁运行，电脑只通过以太网操作。需要专用 MCU 执行七轴通信时，考虑 STM32H7 或 CH32V307。

## 三种方案

| 方案                            | 网口                   | 机械臂接口                | 单块主板参考价             | 软件工作                               |
| ------------------------------- | ---------------------- | ------------------------- | -------------------------- | -------------------------------------- |
| NanoPi NEO3 Plus，1 GB、无 eMMC | 原生千兆网口           | USB 接原装 RS-485 转接器  | US$26，FriendlyELEC        | 复用 Rust 控制后端，调整启动与网络接口 |
| NUCLEO-H753ZI                   | 10/100M PHY、RJ45      | UART 接高速 RS-485 收发器 | US$40.81，DigiKey          | 编写板端通信、调度和控制固件           |
| WCH CH32V307V-EVT-R1            | 内置 PHY 提供 10M 网口 | UART 接高速 RS-485 收发器 | 约 US$12.48，LCSC 单件报价 | 编写板端固件，运动学可保留在电脑       |

以上按主板零售价比较。存储、电源、外壳和 RS-485 接口另计；Linux 路线需要原装转接器。来源：[FriendlyELEC 商店](https://www.friendlyelec.com/index.php?path=69&product_id=316&route=product/product)、[DigiKey H753ZI](https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-H753ZI/21348937)、[LCSC CH32V307 板](https://www.lcsc.com/product-detail/Development-Boards-Kits_WCH-Jiangsu-Qin-Heng-CH32V307V-EVT-R1_C2943980.html)。LCSC 价格通过商品页面读取，单件栏为 US$12.4816。

## Linux 路线

NanoPi NEO3 Plus 提供原生千兆网口、USB 3.0 Host，厂家提供 Debian 13 和 Ubuntu 24.04 系统；1 GB 版本可作为无桌面的控制服务主机候选。[官方规格与系统说明](https://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO3_Plus)

建议连接：电脑／P4 面板 → 以太网 → NanoPi 控制服务 → 原装 USB 转接器 → RS-485 → 机械臂。USB 留在机械臂侧，电脑不再打开串口。

协议、运动学和后台 SQLite 日志可以作为板端服务的迁移基础。当前 [lib.rs](../../src-tauri/src/lib.rs#L97) 在 Tauri setup 中创建控制线程、日志和本机服务，需要增加独立运行入口，并调整设备地址、会话与桌面退出操作。电脑保留界面和 3D；七轴收发、动作执行和停止在控制盒本地处理。这是代码复用方案，板端构建和实机通信仍需验证。

## STM32H7 路线

STM32H743 和 H753 的普通 USART/UART 规格最高 12.5 Mbit/s，覆盖 6 Mbaud；具体时钟和分频按板卡设置，不将该数值套用于 LPUART。[H743 数据手册](https://www.st.com.cn/resource/en/datasheet/stm32h743ii.pdf)、[H753 数据手册](https://www.st.com/resource/en/datasheet/stm32h753zi.pdf)

NUCLEO-H743ZI2 的板载 PHY 和 RJ45 可由 UM2407 确认。当前采购优先参考在售的 NUCLEO-H753ZI；旧 H743ZI2 的经销页面已列为 Obsolete，不能仅凭旧开发板名称下单。[UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-board-stmicroelectronics.pdf)、[H753ZI 官方页](https://www.st.com/en/evaluation-tools/nucleo-h753zi.html)、[H743ZI2 状态](https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-H743ZI2/10130892)

MCU 本地执行定时轮询和命令超时处理，网络传输整组七轴目标与反馈。运动学可以保留在电脑，板端负责控制执行。需要把串口收发、DMA、网络任务和动作状态作为固件实现，工作量高于复用 Linux 后端。

## CH32V307 路线

WCH 应用手册列出串口最高 9 Mbps，规格覆盖 6 Mbaud，仍需选择合适的外设时钟和分频。厂家评估板带网口和调试接口；其内置 PHY 为 10M。芯片 MAC 的能力与整板实际网口速率分别评价。[CH32V307 数据手册](https://www.wch.cn/downloads/CH32V307DS0_PDF.html)、[应用手册](https://www.wch.cn/downloads/CH32FV2x_V3xRM_PDF.html)、[厂家评估板说明](https://www.wch.cn/uploads/file/20220228/1646012640528497.pdf)

这是三者中已取得报价的低板价选项，适合以硬件成本为主要目标的定制通信盒。固件需要另行实现，现有桌面程序不能直接烧录到 MCU。

## RS-485 与触摸屏

MCU 路线还需支持至少 6 Mbps 的 RS-485 收发器和收发方向控制。SN65HVD75 是可比较的器件：3.3 V、半双工、20 Mbps。[TI 规格](https://www.ti.com/product/SN65HVD75)

现有 P4 可继续作为网络触摸面板，显示状态并提交操作；控制盒统一管理控制权。该分工不要求 P4 原生 UART 承担 6 Mbps 总线。

最终联调以完整七轴反馈周期、控制周期抖动和停止响应为依据。目前结论来自厂商规格、零售页面和当前代码分析，三款板的机械臂链路尚未完成测试。

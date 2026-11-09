# Rust 串口库现状与本项目选择

核查日期：2026-11-09。以下依据官方仓库、docs.rs 发布源码/API 和 crates.io 元数据；**没有实测**，未创建 Cargo 项目、编写验证程序、编译/运行示例或连接串口。仅研究串口传输层；“自定义 RS-485、6 Mbps、7 关节数据流”作为已给定背景，不据此推断完整 SDK 协议。

## 结论

- **可用，优先选择：`serialport` + 一个同步专用控制线程。** 三个桌面系统已有本机串口实现，高波特率所需的 macOS `IOSSIOSPEED`、常见 Linux 架构的 arbitrary baud、Windows DCB 路径均已存在。无需因“6 Mbps 不在标准枚举中”先写 FFI 补丁。[平台说明][sp-readme]、[波特率实现][sp-termios]、[Windows DCB][sp-dcb]
- **可用备选：`serial2`。** API 较小、独立读写超时、依赖较少、宽松双许可；但 Unix 默认打开没有 `TIOCEXCL`/`flock`，公开 API 也没有 `serialport` 那样的独占设置。对本项目“单一控制权”，这项差异比能否传入 `u32` 更关键。[API][s2-port]、[Unix 发布源码][s2-unix]
- **可用但暂不优先：`tokio-serial`；不建议直接使用 `mio-serial`。** 前者适合已确定需要异步串口所有者任务的场景，后者是就绪事件层。它们沿用 `serialport`，不会提升硬件波特率能力；会增加 reactor、取消、超时与关闭语义的维护工作。这是针对当前单设备桌面应用的工程判断。[依赖关系][deps-ts]、[Mio API][mio-api]
- **待硬件验证：厂商 USB 转接器在 Win/mac/Linux 上的真实 6 Mbps、收发方向切换、持续数据完整性、时延及退出行为。** 库接受参数、OS 设置成功甚至读回相同数值，都不能替代线路/设备验证。若转接器根本不暴露普通 TTY/COM，而要求私有 USB 协议，本文结论不覆盖该接口。[API 的硬件限制说明][sp-trait]

## 当前版本、许可与维护证据

发布日期均为 crates.io `created_at` 的 **UTC 日期**；是发布记录，不是硬件认证。各版本均为本次查询时的最新未撤回稳定版。

| 库 | 当前版本 / 发布日期 | 许可 | 可观察到的维护证据与限制 |
| --- | --- | --- | --- |
| serialport | **4.10.1 / 2026-09-08** | MPL-2.0 | 2026 年还发布过 4.9.0；4.10.0 已 yanked。README 明确招募维护者，尤其 Windows；CI 覆盖主要目标的构建及部分无需硬件的测试。[发布元数据][v-sp]、[README][sp-readme] |
| tokio-serial | **5.5.0 / 2026-06-15** | MIT | 前版 5.4.5 发布于 2024-12-31；README 明确自动构建主要证明可编译，串口集成测试需硬件。[发布元数据][v-ts]、[README][ts-readme] |
| mio-serial | **5.0.7 / 2026-06-15** | MIT | 前版 5.0.6 发布于 2024-12-31；当前 crate 文档仍提示 Windows 支持主要未经作者充分测试。[发布元数据][v-mio]、[crate 文档][mio-root] |
| serial2 | **0.2.39 / 2026-10-26** | BSD-2-Clause **或** Apache-2.0 | 之前还有 0.2.38（2026-07-31）、0.2.37（2026-05-11）；有持续发布证据，没有据此推定三平台 6 Mbps 已验证。[发布元数据][v-s2] |

`tokio-serial` / `mio-serial` 上层为 MIT，依赖中的 `serialport` 仍是 MPL-2.0；不能把整个依赖集合概括为 MIT。以上只记录许可事实，不扩展作发行合规结论。[依赖][deps-ts]、[依赖][deps-mio]、[serialport 元数据][v-sp]

## 三平台与 6 Mbps：能力边界

| 路径 | Windows | macOS | Linux |
| --- | --- | --- | --- |
| serialport | 原生 `COMPort`；`u32` 写入 `DCB.BaudRate`，经 `SetCommState` 应用。[源码][sp-dcb] | 原生 `TTYPort`；设置 termios 后调用 `IOSSIOSPEED`，**已经内置**。[源码][sp-termios] | x86_64、aarch64 等使用 `termios2`、`TCGETS2` / `TCSETS2`、`BOTHER` 与数值速度；**已经支持非标准波特率**。powerpc/powerpc64 是另一条标准常量路径，不应概括为所有架构相同。[源码][sp-termios] |
| tokio-serial / mio-serial | 有 Windows 实现；底层参数能力来自 serialport，需考虑上表的维护证据限制。[Tokio 源码][ts-src] | Unix 非阻塞串口封装；不另造 IOSSIOSPEED 实现。[Mio 源码][mio-src] | 同左，包裹底层串口，不改变驱动能力。[Mio 源码][mio-src] |
| serial2 | 有 Win32 实现、DCB 配置与 overlapped I/O。[发布源码][s2-win] | 使用 `IOSSIOSPEED`；支持自定义波特率。[Unix 源码][s2-unix] | 常见非 PowerPC 架构采用 termios2/BOTHER 路径；README 宣称除 Solaris/Illumos 外支持自定义波特率，具体仍受目标/驱动约束。[Unix 源码][s2-unix]、[README][s2-readme] |

`6_000_000` 能放进 `u32` 只是表示范围；DCB 允许填写实际速率，也不表示 USB 芯片、驱动、RS-485 收发器和固件均支持。尤其 serialport 在 macOS 的 `baud_rate()` 返回对象保存的请求值，不能把该 getter 当作测得的线路速度。[DCB 官方说明][win-dcb]、[macOS getter 源码][sp-tty]

若采用 8N1，每字节至少占 10 个线路位，6 Mbps 的理想串行字节上限是 **600,000 B/s**。这是算术上限，不是有效负载或控制周期承诺；7 关节帧长、请求/应答、USB 分包及可能的方向切换仍需另算。本文没有确认该自定义 485 链路的方向控制方式；普通 RTS/CTS 流控或通用 RS-485 配置不能自动等同其收发协议。

## 数据、超时与生命周期语义

### 8N1、流控与部分 I/O

三组库都能表达 8 数据位、无校验、1 停止位以及关闭流控。serialport 显式设置 `DataBits::Eight`、`Parity::None`、`StopBits::One`、`FlowControl::None`；异步库复用这些类型。serial2 的 `set_raw()` 即设置上述模式，直接以 `u32` 打开也采用该配置。实际选择须匹配设备协议；对二进制数据流不应误开 XON/XOFF。[Builder][sp-builder]、[serial2 Settings][s2-settings]、[Windows 流控定义][win-dcb]

`read()` 返回的是已有字节片段，`write()` 可能只接受部分输入，均不提供 7 关节帧边界。应由唯一控制者累计接收字节、按协议验帧，并跟踪已发送偏移。`read_exact` / `write_all` 会多次 I/O；中途出错时可能已有数据被消费/发送，不能把整个动作当作未发生后直接重发。[Rust Read][std-read]、[Rust Write][std-write]、[serial2 read_exact/write_all][s2-port]

### 超时

| 库 | 已核实语义 | 对控制循环的含义 |
| --- | --- | --- |
| serialport | 一个 `Duration` 配置；Unix 在 read/write 前等待 fd 就绪，Windows 设置 `COMMTIMEOUTS` 的读写常数。默认 0 ms；精度受 OS 约束。[TTY 源码][sp-tty]、[COM 源码][sp-com]、[README][sp-readme] | 显式配置有限超时。Unix 的就绪等待不是驱动 syscall 的强制抢占，也不是整帧 deadline。 |
| serial2 | 可分别设置读/写超时；文档明确作用于单次 `read` / `write`，`read_exact` / `write_all` 会重复调用。[API][s2-port] | 比单一读写超时方便，但仍需维护事务总截止时间。 |
| tokio-serial / mio-serial | `timeout()` 返回零；`set_timeout()` 接受参数后直接成功，**没有设置超时**。无就绪数据使用 `WouldBlock` / `Pending`。[Tokio 源码][ts-src]、[Mio 源码][mio-src] | 使用异步 deadline/取消设计；不能沿用同步 Builder timeout 的直觉。`tokio::time::timeout` 也无法抢占不让出执行权的阻塞调用。[Tokio timeout][tokio-timeout] |

### 独占、clone、flush 与关闭

| 项目 | serialport | serial2 | tokio-serial / mio-serial |
| --- | --- | --- | --- |
| 独占 | Unix 默认独占，使用 `TIOCEXCL` + advisory `flock`；Windows `CreateFileW` 的 share mode 为 0。[TTY][sp-tty]、[COM][sp-com] | 没有同等公开独占开关；Unix `open` 未施加上述锁。不能仅靠 `&self` 并发安全或同进程单例防止外部进程打开。[Unix][s2-unix] | 提供 Unix `set_exclusive`，底层沿用 serialport；不替代应用自己的控制权管理。[API][ts-api] |
| `try_clone` | 支持共享同一端口的句柄；文档警告每对象缓存设置可能冲突。[trait][sp-trait] | 支持；文档提示多线程读写可能交错，通常只应有一个读者/一个写者。[API][s2-port] | 两者的 `SerialStream::try_clone` 都明确返回错误，不应设计成克隆异步端口。[Tokio][ts-src]、[Mio][mio-src] |
| `flush` | Unix `tcdrain`；Windows `FlushFileBuffers`。清除接收/发送缓冲使用另外的 `clear`。[TTY][sp-tty]、[COM][sp-com] | 同样通过 `tcdrain` / `FlushFileBuffers` 等待输出；丢弃使用 `discard_*`。[Unix][s2-unix]、[Windows][s2-win]、[API][s2-port] | Tokio Unix `poll_flush` 直接调用底层 flush；Windows 委托 Tokio 的管道句柄封装。不能假定三平台异步 flush 都有相同 drain 保证。[源码][ts-src] |
| 关闭 | RAII Drop 关闭本对象 fd/HANDLE，不自动执行设备停止协议；源码没有先做显式输出 drain。[README][sp-readme]、[TTY][sp-tty]、[COM][sp-com] | Drop；另有消费 `self` 的 `close()` 可报告 OS 关闭错误。文档明确它不额外执行清理协议。[API][s2-port] | 释放所有者/拆分半部与底层句柄。Unix `poll_shutdown` 实现仅 flush 后成功，**并不关闭端口**；Windows 委托另一实现，不能把 `shutdown().await` 视作统一关闭确认。[源码][ts-src] |

特别注意三点：

- **flush 不是清空输入，也不是设备已收包/执行的 ACK。** 它涉及驱动发送缓冲；USB、总线及对端处理结果仍要按协议确认。[Windows API][win-flush]、[clear 说明][sp-trait]
- **有限 I/O timeout 不等于 flush 有确定上限。** serialport 的 Unix flush 只对 `EINTR` 重试检查超时，不能截断单次 `tcdrain`；Windows flush 调用也不是简单受该读写超时包围。不要默认每帧 flush 或在退出路径无条件无限等 drain。[TTY][sp-tty]、[COM][sp-com]
- **clone 不是第二个控制者。** 本项目优先完全不克隆端口。serialport 的 Unix 关闭函数还会尽力执行 `TIOCNXCL`；有 clone 时，不能假定先 Drop 一个对象后仍完整保留原有内核独占保护。这是关闭源码带来的推论。[TTY close 源码][sp-tty]

## “纯 Rust”与本机依赖

这里可实现“Rust 控制器不依赖厂商 C++ 串口层”，不能称为完全没有 OS FFI、系统库或驱动。

- **serialport：** Linux glibc 默认启用 `libudev`，构建需要开发文件和 `pkg-config`，运行需要对应动态库。禁用 default features 可移除这项依赖，串口枚举仍可用但信息更少，且可能返回不存在的端口。macOS 使用系统 CoreFoundation/IOKit 等接口；Windows 使用系统 Win32 API。[README][sp-readme]、[发布依赖元数据][deps-sp]
- **tokio-serial / mio-serial：** 当前默认 feature 为空，给 serialport 关闭 default features；可通过 `libudev` feature 再开启。还会带入 Tokio/Mio 等 Rust 依赖。Cargo feature 会合并，若其他依赖启用 serialport 默认项，不能仅凭这两项的默认配置断言最终无 libudev。[Tokio 依赖][deps-ts]、[Mio 依赖][deps-mio]、[Cargo feature 规则][cargo-features]
- **serial2：** 核心 Unix 依赖 `libc` / `cfg-if`，Windows 依赖 `windows-sys`；发布依赖里没有 libudev 或厂商 C/C++ 库。仍使用系统 libc/Win32 与设备驱动；枚举接口返回路径，不等同 serialport 的 USB 元数据接口。[依赖][deps-s2]、[API][s2-port]

## 针对本工程的最小维护决策

**建议采用 serialport，先由一个同步专用控制线程独占端口、协议收发状态和控制状态。** Tauri 2 的 Rust 后端本机 server 可以使用异步运行时，串口循环无需因此改为 async；通过有界命令通道提交操作、通过状态快照反馈 UI，避免前端调用并发直接操作串口。这是基于本次已核实 API 的设计建议，不是已实现或已验证架构。

生命周期由同一个桌面应用管理：启动时建立后端与控制者；退出时停止接单、执行已确认的设备停止流程、让控制循环退出并回收线程/句柄。应把有限读写等待、整帧截止时间、错误后的部分发送状态、flush 是否必要一并纳入退出设计；现有文档不足以保证故障驱动下的硬截止时间。端口 OS 独占、应用单实例和 server 控制权各解决一层冲突，不能相互替代。

本轮不建议为单串口增加独立 Mio reactor、同步/异步双套实现、通用插件传输层，或为未确认的 6 Mbps 问题预写平台补丁；也不预留 headless、LAN 服务或插件扩展。保持本机 server 随桌面启停、单一控制权和集中控制代码。若以后确有多设备并发/取消需求，再评估 tokio-serial；若明确更重视较少依赖与宽松许可，可选 serial2，但需先落实进程间独占方案。

**仍待后续获得硬件测试授权后验证：** 精确 USB 芯片/VID:PID、驱动与 OS 版本；6 Mbps 设置后的真实收发与长时间完整性；7 关节实际周期及丢帧/错帧；方向切换；超时/部分写入；占用冲突、热拔插、退出重开与 RTS/DTR 行为。本文未执行这些检查，也没有据此判定任何组合已达标。

[v-sp]: https://crates.io/api/v1/crates/serialport
[v-ts]: https://crates.io/api/v1/crates/tokio-serial
[v-mio]: https://crates.io/api/v1/crates/mio-serial
[v-s2]: https://crates.io/api/v1/crates/serial2
[deps-sp]: https://crates.io/api/v1/crates/serialport/4.10.1/dependencies
[deps-ts]: https://crates.io/api/v1/crates/tokio-serial/5.5.0/dependencies
[deps-mio]: https://crates.io/api/v1/crates/mio-serial/5.0.7/dependencies
[deps-s2]: https://crates.io/api/v1/crates/serial2/0.2.39/dependencies
[sp-readme]: https://github.com/serialport/serialport-rs/blob/main/README.md
[sp-trait]: https://docs.rs/serialport/4.10.1/serialport/trait.SerialPort.html
[sp-builder]: https://docs.rs/serialport/4.10.1/serialport/struct.SerialPortBuilder.html
[sp-termios]: https://docs.rs/serialport/4.10.1/src/serialport/posix/termios.rs.html#77-128
[sp-tty]: https://docs.rs/serialport/4.10.1/src/serialport/posix/tty.rs.html
[sp-dcb]: https://docs.rs/crate/serialport/4.10.1/source/src/windows/dcb.rs
[sp-com]: https://docs.rs/crate/serialport/4.10.1/source/src/windows/com.rs
[ts-readme]: https://github.com/berkowski/tokio-serial/blob/master/README.md
[ts-api]: https://docs.rs/tokio-serial/5.5.0/tokio_serial/struct.SerialStream.html#method.set_exclusive
[ts-src]: https://docs.rs/tokio-serial/5.5.0/src/tokio_serial/lib.rs.html
[mio-root]: https://docs.rs/mio-serial/5.0.7/mio_serial/
[mio-api]: https://docs.rs/mio-serial/5.0.7/mio_serial/struct.SerialStream.html
[mio-src]: https://docs.rs/mio-serial/5.0.7/src/mio_serial/lib.rs.html
[s2-readme]: https://github.com/de-vri-es/serial2-rs/blob/main/README.md
[s2-port]: https://docs.rs/serial2/0.2.39/serial2/struct.SerialPort.html
[s2-settings]: https://docs.rs/serial2/0.2.39/serial2/struct.Settings.html#method.set_raw
[s2-unix]: https://docs.rs/serial2/0.2.39/src/serial2/sys/unix/mod.rs.html
[s2-win]: https://docs.rs/crate/serial2/0.2.39/source/src/sys/windows/mod.rs
[std-read]: https://doc.rust-lang.org/std/io/trait.Read.html#method.read
[std-write]: https://doc.rust-lang.org/std/io/trait.Write.html#tymethod.write
[win-dcb]: https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-dcb
[win-flush]: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-flushfilebuffers
[tokio-timeout]: https://docs.rs/tokio/1.54.1/tokio/time/fn.timeout.html
[cargo-features]: https://doc.rust-lang.org/cargo/reference/features.html#feature-unification

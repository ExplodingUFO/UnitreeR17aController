# ESP32-P4 平台核查

检查日期：2026-11-10。

适用板型由用户补充为 JC1060P470C_I_W_Y；现有软件基线为 ESP-IDF 5.5.4 + LVGL，固件配置选择 pre-v3、最低 v1.0。这是编译目标，尚不代表读取了实物芯片修订号。

1. **UART 官方上限是 5,000,000 baud，不能按官方规格直接承担要求 6,000,000 baud 的机械臂通信。** Datasheet §4.2.2.1 和 ESP-IDF v5.5.4、v6.1 的 `SOC_UART_BITRATE_MAX` 均为 5 Mbaud。v5.5.4 的 `uart_set_baudrate()` 没有直接检查此上限，而是检查底层分频能否配置；因此配置成功不等于官方支持或实机通过。按其默认 80 MHz 时钟、1/16 分频公式，请求 6 Mbaud 会得到约 **6,009,390 baud（+0.1565%）**，仍超出规格。需要另选支持 6 Mbaud 的外接串行通信通道；仅增加 RS-485 收发器不能消除原生 UART 的速率限制。

   来源：[官方 datasheet，第 68 页](https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf#page=68)、[v5.5.4 soc_caps.h](https://github.com/espressif/esp-idf/blob/v5.5.4/components/soc/esp32p4/include/soc/soc_caps.h)、[v6.1 soc_caps.h](https://github.com/espressif/esp-idf/blob/v6.1/components/soc/esp32p4/include/soc/soc_caps.h)、[uart_set_baudrate 检查](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_uart/src/uart.c#L373)、[P4 分频实现](https://github.com/espressif/esp-idf/blob/v5.5.4/components/hal/esp32p4/include/hal/uart_ll.h#L508)、[UART 时钟选择](https://github.com/espressif/esp-idf/blob/v5.5.4/components/soc/esp32p4/include/soc/clk_tree_defs.h#L264)。

2. **ESP-IDF 的 Ethernet 和 LVGL 接入已有现成支持。** P4 的官方 Ethernet 驱动支持内部 EMAC、外部 PHY、RMII 和 TCP/IP 栈接入；`esp_lvgl_port` 当前发布版 2.9.0 支持 LVGL 8/9、显示及触摸输入，官方 P4 BSP 也有触摸与显示示例。具体面板、触摸、PHY 与引脚仍采用本板配置。与电脑的 Ethernet、显示和触摸接口不需要因为机械臂的 6 Mbaud 要求而更换。

   来源：[P4 Ethernet 官方指南](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32p4/api-reference/network/esp_eth.html)、[esp_lvgl_port 2.9.0](https://components.espressif.com/components/espressif/esp_lvgl_port/versions/2.9.0/readme)、[官方 P4 BSP 5.2.3](https://components.espressif.com/components/espressif/esp32_p4_function_ev_board/versions/5.2.3/readme)。

3. **esp-hal 的 P4 支持限定芯片修订 v3.0 及以上，不宜作为当前 pre-v3 目标的最快路线。** 这已包含在正式发布的 **esp-hal 1.2.0**，并非仅存在于 main；检查时 main 仍标注 v3+。两者的 UART 配置还明确拒绝超过 5,000,000 baud。另一路 `esp-idf-svc` / `esp-idf-hal` 基于 ESP-IDF，不能把 esp-hal 的 v3+ 限制直接套给它；其修订兼容性需按 ESP-IDF 配置确定，也不能解决 UART 官方速率上限。当前继续使用 ESP-IDF + LVGL 最直接。

   来源：[esp-hal 1.2.0 release](https://github.com/esp-rs/esp-hal/releases/tag/esp-hal-v1.2.0)、[main 修订声明，检查时快照](https://github.com/esp-rs/esp-hal/blob/e045405e6f036a7d8b2ef22bc877457dc3e6eba2/esp-hal/Cargo.toml#L258)、[1.2.0 UART 上限检查](https://github.com/esp-rs/esp-hal/blob/esp-hal-v1.2.0/esp-hal/src/uart/mod.rs#L415)、[main UART 检查](https://github.com/esp-rs/esp-hal/blob/e045405e6f036a7d8b2ef22bc877457dc3e6eba2/esp-hal/src/uart/mod.rs#L415)、[esp-idf-hal 的 P4 目标说明](https://github.com/esp-rs/esp-idf/blob/0a4d972bfa33e21bb2d858f4c7291fb41d5984d8/esp-idf-hal/README.md)、[ESP-IDF 修订配置说明](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32p4/api-reference/kconfig-reference.html#config-esp32p4-selects-rev-less-v3)。

## 与当前桌面控制器的分工

建议连接为：电脑 → 以太网 → P4 → 6 Mbaud 通信模块 → RS-485 → 机械臂。机械臂接口和默认参数见[官方连接说明](https://support.unitree.com/home/zh/R1-7a/Usage_and%20SDK%20Deployment)与[SDK 默认参数](https://github.com/unitreerobotics/r1-7a_sdk/blob/0803d033b530fcfa0e6fa9236de68c2773fc9402/README.md#usage)。网线替换电脑侧的 USB 串口连接，设备侧保留原有总线。

| 部分         | 建议处理内容                                                       |
| ------------ | ------------------------------------------------------------------ |
| 电脑         | 点动与运动学计算、3D 显示、SQLite 日志、本机 HTTP/WebSocket 服务   |
| P4           | 以太网通信、触摸屏、设备侧通信调度、反馈采集、命令有效期和本地停止 |
| 外接通信模块 | 支持 6 Mbaud 的串行通道与 RS-485 电气接口                          |

当前后端通过 [Bus::open](../../src-tauri/src/protocol.rs#L357) 打开系统串口，[Bus::exchange](../../src-tauri/src/protocol.rs#L390) 发送单关节命令并等待对应反馈。[Core::poll](../../src-tauri/src/arm.rs#L1153) 每次轮询一个关节，循环处理七轴。网络版本应改为整组目标和状态传输，由 P4 在设备侧完成逐关节通信，避免让每次串口事务等待网络往返。

网络帧需要命令序号和有效期，反馈需要保留采集顺序与数据年龄。P4 本地处理命令过期和停止，重新连接后由操作者重新发起动作。屏幕刷新与通信分别调度。

按此分工，屏幕先提供连接信息、关节状态、故障和停止操作，点动计算依赖电脑连接。若要求脱离电脑独立点动，需要将对应控制计算放到 P4；独立末端点动还包括运动学与模型参数。

当前板级配置为 7 寸 1024×600、JD9165 显示、GT911 触摸和 IP101 RMII 网口。新的控制固件可以采用相同驱动组合，桌面程序继续使用 Rust 和 Tauri。下一项需要确定的是支持 6 Mbaud 的外接通信通道，以及它与该板的连接方式。

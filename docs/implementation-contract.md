# API 接入说明

本文对应当前桌面软件的实现，说明请求、返回值、状态订阅和动作确认方式。更新日期：2026-11-23。机器可读定义见 [OpenAPI 3.1](openapi.json)。

## 1. 连接与认证

| 项目           | 当前行为                                               |
| -------------- | ------------------------------------------------------ |
| HTTP 地址      | `http://127.0.0.1:8177`                                |
| WebSocket 地址 | `ws://127.0.0.1:8177/v1/events`                        |
| 服务生命周期   | 随桌面应用启动和退出                                   |
| 访问范围       | 当前仅监听本机回环地址                                 |
| HTTP 认证      | `Authorization: Bearer <token>`                        |
| POST 格式      | `Content-Type: application/json`，请求体最多 2 MiB     |
| 状态与数值     | JSON；长度用米，角度用弧度；`_ms` 用毫秒，`_us` 用微秒 |

应用每次启动生成新的 `token`。外部程序从桌面顶部“接口”页复制令牌；当前没有通过 HTTP 获取令牌的接口。令牌用于访问应用，`lease_id` 用于持有控制权，两者用途不同。

浏览器 Origin 当前只接受 `tauri://localhost`、`http://tauri.localhost`、`https://tauri.localhost`、`http://127.0.0.1:1420`、`http://localhost:1420`。原生程序可以不发送 Origin。

## 2. 接口总表

| 方法与路径                              | 输入                          | 成功输出                           | 含义                     |
| --------------------------------------- | ----------------------------- | ---------------------------------- | ------------------------ |
| `GET /v1/ports`                         | 无                            | `200 PortInfo[]`                   | 枚举本机串口             |
| `GET /v1/state`                         | 无                            | `200 Snapshot`                     | 读取最近一次状态快照     |
| `POST /v1/control`                      | `ControlRequest`              | `200 ControlReply`                 | 取得、续期或释放控制权   |
| `POST /v1/commands`                     | `CommandRequest`              | `202 {request_id,status:"queued"}` | 提交设备或动作请求       |
| `GET /v1/events`                        | WebSocket 升级后发送认证首帧  | `101`，随后推送 `StateEvent`       | 订阅完整状态快照         |
| `GET /v1/logs?limit=50`                 | 可选条数                      | `200 LogRecord[]`                  | 查询 SQLite 中最近的日志 |
| `GET /v1/logs/filters`                   | 无                            | `200 {levels,sources,codes}`       | 读取历史日志的筛选选项   |
| `GET /v1/logs/page`                     | 分页、搜索与记录范围          | `200 LogPage`                      | 查询持久化日志历史       |
| `GET /v1/traces`                        | 可选 `limit`                  | `200 TraceSummary[]`               | 查询最近采样             |
| `POST /v1/traces`                       | `{duration_ms?:number\|null}` | `200 TraceSummary`                 | 开始短时采样             |
| `POST /v1/traces/stop`                  | 无                            | `200 TraceSummary\|null`           | 结束当前采样             |
| `GET /v1/traces/{id}`                   | 可选 `download/attachment`    | `200 TraceRecording`               | 读取或下载采样 JSON      |
| `POST /v1/calibrations/preview`         | 幅度、速度、方向              | `200 CalibrationPreview`           | 生成只读运动预览         |
| `GET /v1/calibrations?limit=20`         | 可选条数                      | `200 CalibrationReport[]`          | 查询最近校准报告         |
| `GET /v1/calibrations/{id}?points=3000` | 报告 ID、绘图点数             | `200 CalibrationReport`            | 读取校准报告及绘图曲线   |
| `GET /v1/teachings`                     | 无                            | `200 TeachingSummary[]`            | 查询已保存示教轨迹       |
| `GET /v1/teachings/{id}`                | 可选 `download=true`          | `200 TeachingTrajectory`           | 读取或下载完整示教 JSON  |
| `POST /v1/teachings/{id}/preview`       | `TeachingPlaybackOptions`     | `200 TeachingPreview`              | 只读规划接近与回放路径   |

普通 HTTP 返回值直接使用表中的结构，没有统一的 `data` 包装。只有 WebSocket 状态消息使用 `{type:"state",data:Snapshot}`。

## 3. 推荐调用顺序

1. 取得应用令牌，连接 WebSocket 并发送认证首帧。
2. 读取首个状态；只读程序从此持续接收状态即可。
3. 调用 `acquire` 取得控制权，保存返回的 `lease_id` 和 `generation`。
4. 枚举串口，发送 `connect`，等待连接确认。
5. 如需伺服调试，连接确认后发送完整的 `configure_servo`，等待接受并读取新版本；使用自定义工具时发送 `configure_tcp`，否则使用默认 Link7 法兰。
6. 发送 `enable`，等待使能确认，再发送 `step` 或 `jog_start`。
7. 操作期间独立续期控制权；连续点动还需发送 `jog_update`。
8. 结束动作时发送 `stop`。需要断开时，先返回有支撑的收纳位，或明确确认机械臂已支撑后释放保持扭矩，再断开并释放控制权。

每个会改变状态的步骤都等待确认，再从最新状态读取 `generation`。调用方不自行递增该值。关节点动不要求设置 TCP。

采样使用应用令牌即可，不需要上述控制权或运动步骤。已有端口连接时可直接开始采样，记录当前运行状态。

## 4. 控制权输入与输出

同一时间只有一个客户端持有控制权；其他客户端可以读取状态和日志。控制权有效期为 **2500 ms**，建议每 **800 ms** 续期。

| 字段        | 类型           | 输入要求                                                          |
| ----------- | -------------- | ----------------------------------------------------------------- |
| `action`    | string         | `acquire`、`renew`、`release`                                     |
| `client_id` | string         | 非空；建议使用可辨认的名称                                        |
| `lease_id`  | string 或 null | `acquire` 可省略；`renew`、`release` 必须传入取得控制权时返回的值 |

取得控制权：

```json
{ "action": "acquire", "client_id": "example-service" }
```

```json
{
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "expires_in_ms": 2500,
  "generation": 2
}
```

续期与释放：

```json
{
  "action": "renew",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111"
}
```

```json
{
  "action": "release",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111"
}
```

续期返回当前 `lease_id`、重新计时的 `expires_in_ms:2500` 和当前 `generation`。释放返回全部三个字段，其中 `lease_id:null`、`expires_in_ms:0`。

已有有效控制权时，再次 `acquire` 返回 `409 owned`，即使 `client_id` 相同。交接方式是原客户端 `release`，新客户端再 `acquire`。释放或过期会取消当前动作，不自动失能；待确认的使能会被取消。

`POST /v1/control` 等待控制线程处理后才返回。排队截止时间为 300 ms，接口等待上限为 350 ms；超时后先读取状态确认控制者，再决定下一步。

## 5. 命令输入

### 5.1 公共字段

所有命令都提交到 `POST /v1/commands`，包含以下字段。操作参数与公共字段处于同一层。

| 字段         | 类型          | 要求                                |
| ------------ | ------------- | ----------------------------------- |
| `request_id` | string        | 非空；每个新请求使用新值，建议 UUID |
| `client_id`  | string        | 当前持有控制权的客户端标识          |
| `lease_id`   | string        | 当前控制权凭据                      |
| `generation` | 非负整数，u64 | 最新状态或控制权响应中的值          |
| `kind`       | string        | 下表中的命令名                      |

`generation` 会随控制权变化、连接、配置、动作结束或取消等状态变化而更新。收到 `stale_context` 后先读取状态，重新确认操作意图，不直接重发旧动作。独立录制开始/结束只观察和保存位置，不因 generation 变化被拒绝，控制权仍需有效。

### 5.2 操作参数与确认条件

| `kind`             | 附加输入                                                       | 确认方式                                                                                    |
| ------------------ | -------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `connect`          | `port:string`，`side:"left"\|"right"`                          | 请求被接受后，等待 `phase:"disabled"`，七轴反馈齐全且新鲜                                   |
| `disconnect`       | `allow_release?:boolean`                                       | `phase:"disconnected"`、`port:null`                                                         |
| `enable`           | 无                                                             | 该请求 `completed/enabled`，`enabled:true`、`phase:"ready"`                                 |
| `disable`          | `allow_release?:boolean`                                       | 请求被接受且 `enabled:false`；若已有故障，`phase` 仍可为 `fault`                            |
| `clear_fault`      | 无                                                             | 该请求 `completed/cleared`，`error:null`、`enabled:false`                                   |
| `configure_tcp`    | `tcp:Pose`                                                     | 请求被接受，`tcp_configured:true`，`tcp_transform` 与归一化后的输入一致                     |
| `configure_servo`  | `config:ServoTuning`（完整七轴配置）                           | `accepted/configure_servo`，快照配置及版本更新；不产生独立 completed                        |
| `configure_motion` | `config:MotionTuning`                                          | `accepted/configure_motion`，快照运动设置与 generation 更新                                 |
| `calibrate_start`  | `settings:CalibrationSettings`                                 | 接受后跟踪 `calibration`；结束返回 completed、cancelled 或 failed，code 为 calibration_done |
| `calibrate_apply`  | `calibration_id:string`                                        | `accepted/calibrate_apply`，配置及版本与推荐一致                                            |
| `teach_mode`       | `enabled:boolean,transition_s?:number`                        | 请求被接受后，按 `teaching_mode` 确认拖动过渡；恢复保持后该字段为 null                     |
| `teach_start`      | `name?:string`                                                | `accepted/teach_start`，`teaching_recording.state:"recording"`；当前动作、模式与 generation 不变 |
| `teach_finish`     | `recording_request_id?:string`                                                             | `completed/teaching_saved`，`teaching_recording:null`、`latest_teaching` 更新；不取消软件动作 |
| `teach_replay`     | `trajectory_id:string,speed_scale?:number,smoothing_s?:number` | 准备后入队，接受后跟踪 `teaching`；正常结束为 `completed/replayed` 并保持末点               |
| `jog_start`        | `space,axis,direction,speed,frame?,ttl_ms?,input_seq`          | 请求被接受且 `action.request_id` 等于本次 `request_id`                                      |
| `jog_update`       | `action_id,input_seq,ttl_ms?`                                  | 成功更新不产生 `accepted` 记录；按当前动作与后续状态判断                                    |
| `step`             | `space,axis,direction,amount,speed,frame?`                     | 该请求 `completed/completed`                                                                |
| `move_tcp`         | `tcp:Pose,duration_s:number`                                  | 该请求 `completed/tcp_reached`，含 `measured_tcp` 和 `actual_duration_s`                        |
| `return_to_pose`   | `joints:[number;7],speed,disconnect_after?:boolean`            | 该请求 `completed/returned`（继续保持）或 `completed/parked`（已断开）                      |
| `stop`             | 无                                                             | 原动作收到 `cancelled`、`action:null`，等待 `stopping` 结束                                 |

`connect.port` 使用枚举返回的 `name`，非空。串口当前使用 6,000,000 波特率。更换串口或左右臂前先断开连接；切换左右臂会清除已设置的 TCP。

连接被接受表示串口已经打开，连接确认仍需反馈。使能进入 `enabling` 后，最多等待一秒，收齐七轴在使能请求之后的新鲜反馈，并确认设备模式、错误与超时状态。清错也需要反馈确认，完成后不会自动使能。

`disconnect` 与 `disable` 的 `allow_release` 默认 `false`。设备已使能或正在使能时，必须由调用方明确传 `true`；否则异步返回 `rejected/support_required`，保持当前状态。该字段表示操作意图，软件不检测机械支撑。`clear_fault` 在已使能或正在使能时返回 `rejected/disable_first`。

### 5.3 坐标、单位与范围

| 点动类型 | `space` / `axis`                | `speed` 单位 | `amount` 单位 |
| -------- | ------------------------------- | ------------ | ------------- |
| 关节     | `joint`，0..6 对应 J1..J7       | rad/s        | rad           |
| 末端平移 | `cartesian`，0..2 对应 X/Y/Z    | m/s          | m             |
| 末端旋转 | `cartesian`，3..5 对应 Rx/Ry/Rz | rad/s        | rad           |

`speed`、`amount` 必须为正数，方向由 `direction:-1|1` 指定。`frame` 默认 `base`：`base` 沿固定基座轴运动，`tool` 沿当前 TCP 轴运动。关节点动建议传 `base`，该字段不影响关节轴。

平移保持姿态；旋转围绕 TCP。点动一次请求只操作一个轴；收纳返回为七轴关节插值。当前没有绝对笛卡尔位姿移动、轨迹批量下发或原始电机控制接口。

`Pose` 的格式如下，`rotation` 是 **x、y、z、w 顺序的四元数**：

```json
{ "translation": [0.0, 0.0, 0.02], "rotation": [0.0, 0.0, 0.0, 1.0] }
```

`configure_tcp.tcp` 表示 **Link7 → TCP** 的工具变换，不是运动目标。平移为有限米数；非零有限四元数由后端归一化。取得控制权后可离线设置；运动、减速停止或待确认使能时不能设置。配置保存在本次应用内存中。

### 5.4 完整请求示例

下列标识与 `generation` 为示例值，实际调用使用本次会话返回的数据。全部命令的完整输入见 [OpenAPI](openapi.json) 的 `POST /v1/commands` 示例。

连接右臂：

```json
{
  "request_id": "req-connect-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 2,
  "kind": "connect",
  "port": "/dev/cu.usbserial-001",
  "side": "right"
}
```

沿基座 X 正向移动 5 mm，速度 10 mm/s：

```json
{
  "request_id": "req-step-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 5,
  "kind": "step",
  "space": "cartesian",
  "axis": 0,
  "direction": 1,
  "amount": 0.005,
  "speed": 0.01,
  "frame": "base"
}
```

J1 正向连续点动，速度 0.1 rad/s：

```json
{
  "request_id": "req-jog-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 5,
  "kind": "jog_start",
  "space": "joint",
  "axis": 0,
  "direction": 1,
  "speed": 0.1,
  "frame": "base",
  "ttl_ms": 350,
  "input_seq": 0
}
```

保持同一动作：

```json
{
  "request_id": "req-jog-update-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 5,
  "kind": "jog_update",
  "action_id": "req-jog-001",
  "input_seq": 1,
  "ttl_ms": 350
}
```

### 5.5 连续点动与固定步长

连续点动的 `request_id` 同时是 `action_id`。建议设置 `ttl_ms:350`，每 100 ms 发送一次 `jog_update`，并独立续期控制权。`ttl_ms` 为可表示的正整数毫秒，省略时默认 150；有效期从后端首次收到请求开始计算，排队不会延长有效期。

每次更新使用新的 `request_id`，保持相同的 `action_id`、动作期间有效的 `generation`，并递增 `input_seq`。更新只延长当前动作，不改变轴、方向或速度。后端只保留最新一条待处理更新，中间更新可能被合并。

停止发送更新后，动作在输入过期时取消。主动松开按钮时同时发送 `stop`，缩短等待。过期或已结束的动作不能通过迟到的更新恢复。WebSocket 断开不会直接释放控制权；点动输入有效期与控制权有效期分别计时。

固定步长一次执行一个动作，不将后续点击排成运动队列。`progress` 按时间估计，达到 1 不表示已完成；以 `completed` 结果为准。完成前会同时检查实测值与参考值，并在减速结束后报告结果。

### 5.5.1 基座 TCP 目标

`move_tcp.tcp` 是当前配置工具点在基座坐标系中的绝对目标。平移单位为米，旋转为单位 XYZW 四元数。`duration_s` 是有限正数，表示请求的最短用时。控制器按同一进度插值平移与旋转，并应用现有运动约束。动作需要已使能、完整新鲜反馈和空闲控制器；控制权续期规则与其他动作相同。执行中 `action.mode` 为 `move_tcp`，`space` 为 `cartesian`，`axis` 为 `null`。

请求接受后，按 `request_id` 读取 `results` 中的最新记录。到位时控制器先确认静止参考已发送，再以更新的七轴反馈计算 `measured_tcp`。完成容差为平移 1 mm、旋转 1°。实际用时 `actual_duration_s` 从开始执行计到结果记录。取消或失败同样记录实际用时，并在反馈可用时附带取消或失败时的实测 TCP。发送 `stop` 后按原请求查找 `cancelled`，等到 `phase:"ready"` 后使用最新 `generation` 和实测位姿提交下一目标。TCP 路径约束只检查运动学路径，工作区避障由调用方负责。

### 5.6 收纳返回与断开

先在桌面记录有支撑的收纳姿态，或由调用方保存实测七轴位置，再提交 `return_to_pose`。`joints` 按 J1..J7 排列，单位 rad；`speed` 为有限正数，执行时受当前运动设置与模型关节速度约束。请求需要控制权、已使能状态、完整新鲜反馈，且没有其他动作。

七轴按共同的五次多项式生成返回目标，起止速度和加速度为零；控制参考再经过关节速度、加速度和 jerk 限制，按实际控制间隔积分。回程是关节插值，不提供碰撞规划。界面按左右臂与串口保存收纳位，没有预置角度。

`disconnect_after` 默认 `false`，在参考轨迹到位、指令减速结束后产生 `completed/returned` 并继续使能保持。设为 `true` 时，还需七轴实测位置在目标的 0.02 rad（约 1.15°）内，随后失能、关闭串口并产生 `completed/parked`。两种返回均不使用实测 `dq` 判断到位；小幅位置波动在容差内即可。实际位置尚未到位时继续保持连接与使能，不按预计用时报告失败。时间进度到达 1 不能单独触发断开。

返回时 `action.mode` 为 `return`、`space` 为 `joint`、`axis` 为 `null`。原有点动仍返回数值轴索引。接口调用方需要处理这个新增动作类型。使能状态下的旧 `disconnect`/`disable` 请求若未传 `allow_release:true`，现在会被拒绝；该变更自 2026-11-21 起生效。

中途停止、控制权释放/过期、跟随偏差超限或到位超时，会取消返回且不再执行计划中的断开。收纳返回的到位超时与跟随偏差处理保留连接和使能；设备故障、反馈丢失或实际通信中断仍按原故障流程处理。软件不检测支撑，收纳位必须由使用者按实际支撑位置记录。突然断电后的保持需要机械支撑或制动机构。

明确确认支撑后，也可以直接提交下面的断开请求；`generation` 和凭据取自当前会话：

```json
{
  "request_id": "req-disconnect-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 6,
  "kind": "disconnect",
  "allow_release": true
}
```

### 5.7 参考导数与停止

关节、末端点动与收纳返回共用 `MotionTuning`：`joint_speed` 默认 1 rad/s、`acceleration` 默认 0.60 rad/s²、`jerk` 默认 6 rad/s³，均为有限正数。关节速度取设置与 URDF 各轴速度的较小值。控制器先求满足导数窗口的最终速度，再按实际周期积分 `q_ref`；`ddq_ref` 和 `jerk_ref` 分别从相邻速度和加速度求得。

末端求解把导数窗口直接纳入 Jacobian 的有界二次规划，路径修正用于保持所选 TCP 的运动方向和旋转中心。速度按请求、当前姿态和有效关节速度求解；不另设固定的 15 mm/s、0.15 rad/s 上限，也不通过微小 FK 残差结束动作。

空闲时可提交 `{"kind":"configure_motion","config":{"joint_speed":1,"acceleration":0.6,"jerk":6}}`，公共命令字段与其他请求相同。配置需要控制权，接受后更新 generation；当前值见 `Snapshot.motion_tuning`。校准使用预览中的 `program.motion`，结束或取消后恢复开始前的运动设置。

关节步长与收纳使用起止速度、加速度为零的五次轨迹，时长同时考虑速度、加速度与 jerk。末端步长通过当前 Jacobian 估计关节位移，用于安排导数受限的参考时长；执行仍由同一局部求解器推进。预计时长不是普通步长的失败期限，到位并完成减速后才报告完成。

停止权限、租约与 generation 校验、原请求的 cancelled 结果保持原有语义。正常停止按 jerk 限制减速，可能需要更长时间；`action:null` 后仍须等待 `stopping` 结束。停止求解惩罚关节速度本身，使冗余方向的速度也减至零，连续性由导数硬约束维持。笛卡尔路径无法满足制动约束时，写入 `control/stop_path` 警告并转为受限关节制动。故障、失能和关闭继续优先处理。

离散参考的导数限制不保证实际 TX 节拍或机械臂物理运动的 jerk。运动跟踪的目标位置增益为 60，默认阻尼为 10，重力前馈来自 URDF。普通位置保持从使能时的实测位置开始。设某轴发送目标与实测位置的偏差绝对值为 `e`（度），该轴的保持增益目标为 `60 × (1 − 0.5 / (1 + e / 2))`。这是软件初值，两个方向使用同一条连续曲线。每次使能的首个有效目标直接采用本次保持增益。等待使能确认期间，各轴按自己的新鲜反馈调节刚度，缺少反馈的轴沿用上次发送的系数。拖动示教的过渡系数乘在该增益上。运动、校准、重放及停止减速时恢复跟踪增益。模式衔接和保持增益变化均从上个成功发送批次逐步过渡，10 ms 周期内系数最多变化 0.05。短时录制的 `transmit.kp` 记录七轴实际发送的增益，重力前馈与逐轴阻尼独立计算。

### 5.8 发送时间对齐

普通七轴发送保持原始 10 ms 截止时间网格，跳过错过的槽位，不补发。新参考或唤醒延迟不重设网格；普通批次的主机起点至少相隔 5 ms。源参考可被合并，也可在多个发送时刻重复取样。

发送线程在本批次主机起点选取最新源参考，以其来源时间计算 `age`，要求 `0 ≤ age ≤ 50 ms`。先按 `dq + ddq*age` 取样速度，在关节速度上限或首次速度零点封顶；已零速的停止参考不会自行启动。首次伺服取样按该段速度积分位置，未触边界时为 `q + dq*age + 0.5*ddq*age²`。

有上一成功整批伺服目标时，位置按 `q_previous + 0.5*(dq_previous + dq_sample)*Δt_tx` 连续积分，使用两次主机批次起点的实际间隔。新控制参考只更新取样速度，不重新锚定发送位置，避免控制器离散积分与外推交接时的位置回跳。

新点动、固定步长和收纳回程从当前连接最近成功发送的位置建立起点；尚无发送记录时使用当前有限参考。此位置交接不额外等待发送记录的年龄、generation/epoch 或零速判定；动作取消仍由 epoch 和输入有效期执行。此交接只读内存快照，不访问日志或数据库。

发送端按当前有效关节速度对预测速度限幅，保留 TCP 和动作目标，不再次进行路径、实测跟随或固定关节增量检查。轨迹生成与发送积分的细小残差不会产生运动失败。

关节目标越界时限制在目标或本次积分起点，并将对应速度置零；末端目标越界时保持上一成功伺服位置（无历史则保持源位置），整组速度置零。取消后的减速不再追逐旧目标。

源参考携带 generation、epoch 与有效截止时间；运动期间截止时间取输入有效期与租约到期时间的较早者。取样前及写入前复查 epoch 和有效期，写入前同时检查优先命令；失效候选不提交。单报文写入保留 I/O 进度预算，完整报文之间的调度间隙不再触发整批失败；不在半帧中插入其它报文。这不是电机确认的时限保证。

参考过旧或时间暂时未对齐时，若有上一成功整批伺服目标，就保持位置并将速度置零；新参考到达后恢复取样，不结束当前校准或点动。非有限输入与动作有效期到期仍结束动作。迟到的旧 generation/epoch 事件不会影响新的动作。

使能确认中的匹配拒绝同样有效：结束 `enable_request`，记录 `failed/send_reference`，维持原有 `enabled=false`，同次新鲜反馈不能再将其置为 Ready。迟到拒绝不能取消新动作。首次无效且没有成功伺服历史时走 I/O 错误处理。已换代或到期而在写前跳过的批次不产生 `transmit` 样本。

### 5.9 伺服调试配置

`configure_servo` 应用完整 `config:ServoTuning`，不是部分更新。`friction_enabled` 为必填 boolean，`joints` 为 J1..J7 顺序的恰好七项；各项以下六个字段仍全部必填且必须有限，另可提供 `stribeck`。表中范围均含端点，基线值不表示请求可以省略这六个字段。

| 每轴字段         | 含义与单位                  | 有效数值             | 基线 |
| ---------------- | --------------------------- | -------------------- | ---- |
| `kd`             | 阻尼，Nm·s/rad              | 非负，实际协议可表示 | 10   |
| `velocity_ff`    | 速度前馈权重，无量纲        | 非负                 | 1    |
| `coulomb`        | 库仑摩擦幅值 Fc，Nm         | 非负                 | 0    |
| `viscous`        | 黏性系数 B，Nm·s/rad        | 非负                 | 0    |
| `velocity_scale` | 平滑过零尺度 epsilon，rad/s | 正数                 | 0.02 |
| `max_friction`   | 摩擦补偿幅值上限，Nm        | 非负                 | 0.5  |

`stribeck:StribeckTuning` 为可选对象，包含 `enabled:boolean`、`positive:FrictionCurve` 和 `negative:FrictionCurve`。省略整个对象时按关闭读取，兼容原有六字段配置。显式提供时三个字段都需完整；正、负方向分别提供以下四个有限参数：

| 曲线字段            | 含义与单位           | 有效数值        | 默认曲线 |
| ------------------- | -------------------- | --------------- | -------- |
| `coulomb`           | 库仑幅值 Fc，Nm      | 非负            | 0        |
| `static_friction`   | 低速幅值 Fs，Nm      | 非负，可小于 Fc | 0        |
| `viscous`           | 黏性系数 B，Nm·s/rad | 非负            | 0        |
| `stribeck_velocity` | 衰减速度 Vs，rad/s   | 正数            | 0.02     |

全局基线为 `friction_enabled:false`，各轴 `stribeck.enabled:false`。以下原有完整请求继续有效：

```json
{
  "request_id": "req-configure-servo-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 4,
  "kind": "configure_servo",
  "config": {
    "friction_enabled": false,
    "joints": [
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      },
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      },
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      },
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      },
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      },
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      },
      {
        "kd": 10,
        "velocity_ff": 1,
        "coulomb": 0,
        "viscous": 0,
        "velocity_scale": 0.02,
        "max_friction": 0.5
      }
    ]
  }
}
```

例如启用 J2 的双向 Stribeck 曲线时，把完整请求的 `config.friction_enabled` 设为 `true`，并把 `joints[1]` 替换为以下对象；其余六轴仍须一并提交：

```json
{
  "kd": 10,
  "velocity_ff": 1,
  "coulomb": 0,
  "viscous": 0,
  "velocity_scale": 0.02,
  "max_friction": 0.6,
  "stribeck": {
    "enabled": true,
    "positive": {
      "coulomb": 0.18,
      "static_friction": 0.32,
      "viscous": 0.04,
      "stribeck_velocity": 0.025
    },
    "negative": {
      "coulomb": 0.22,
      "static_friction": 0.36,
      "viscous": 0.05,
      "stribeck_velocity": 0.03
    }
  }
}
```

认证、租约、去重和 `generation` 校验沿用普通命令。先连接并等待连接确认，再配置；没有控制权不得应用。未连接或 `connecting` 返回 `rejected/not_connected`，关闭期间按原有关闭检查拒绝；故障或待清错返回 `rejected/fault`。运动、减速停止或使能确认期间返回 `rejected/motion_busy`。有限数值超出业务范围时，HTTP 仍可先返回 **202 queued**，再出现 `rejected/invalid_servo_tuning`；缺字段、类型错误或数组长度错误属于 JSON 提取错误。JSON 不支持 NaN/Infinity。

已使能且空闲时，控制器读取当前连接最后成功发送的位置，尚无记录时使用当前有限参考，清零导数再应用配置。完整新鲜反馈用于当前连接状态；不另设发送年龄、零速或跟随误差门槛。接受后 `servo_tuning_revision` 与 `generation` 均提升，旧运动请求将因 `stale_context` 失效；使用最新快照继续操作。结果为 `accepted/configure_servo`，无需等待独立的 completed；接受只确认软件配置应用，不表示设备已确认参数或物理效果。

每次连接或断开都恢复基线并更新配置版本；配置只保存在当前连接内存中，不自动持久化。在控制页点击“点动 → 参数”打开弹窗，编辑、恢复基线只改草稿，须显式应用；复制 JSON 不发送命令。关闭弹窗保留当前连接的草稿，已提交请求继续等待确认；重新连接清除旧草稿。

发送端使用对齐后的未缩放轨迹速度 `v`。全局摩擦开启且该轴 `stribeck.enabled:true` 时，`v>0` 选 positive，`v<0` 选 negative，计算：

```text
raw = [Fc + (Fs - Fc) * exp(-(abs(v) / Vs)^2)] * tanh(v / velocity_scale) + B * v
requested = clamp(raw, -max_friction, max_friction)
C(x) = clamp(x, -effort_limit, effort_limit)
tau_gravity = C(G)
tau_ff = C(G + requested)
friction_applied = tau_ff - tau_gravity
clamped = (requested != raw) or (tau_ff != G + requested)
```

`G` 为该轴限幅前的模型重力扭矩，`effort_limit` 为当前臂 URDF 的关节 effort 上限（Nm）。发送样本的 `tuning.tau_gravity` 记录单独施加重力时的限幅值。例：上限 10 Nm、`G=12`、`raw=requested=-3` 时，`tau_gravity=10`、`tau_ff=9`、`friction_applied=-1` Nm，`clamped=false`。合计前馈触限时动作继续，沿用常规保持策略。

该轴 Stribeck 关闭或省略时，保留原公式 `raw=Fc*tanh(v/velocity_scale)+B*v`，Fc/B 使用原六字段中的 `coulomb/viscous`。全局摩擦关闭或 `v=0` 时取 `raw=requested=0`；两种模型共用该轴原有 `max_friction` 限幅。速度前馈独立为 `dq_feedforward=velocity_ff*v`，只作用于最终报文速度字段；`Reference.dq`、位置积分、TCP/目标复核、停止与收纳时序保持原值。纯 Rust 无状态计算只改变附加前馈，输出继续按实际协议编码。

候选方案和证据见[补偿方案调查](research/motion-compensation-repositories-2026-11-21.md)。

### 5.10 低速稳定性校准

桌面先生成轨迹预览，由使用者确认运动范围后启动。预览只计算几何，不申请控制权或下发动作；启动仍使用普通命令的控制权、generation 与停止处理。没有额外的静止观察窗口。

**预览**：`POST /v1/calibrations/preview`，请求可为 `{}`，或：

```json
{ "amplitude": 0.05, "speed": 0.01, "axes": [0, 1, 2] }
```

| 字段        | 默认        | 范围与单位                                                               |
| ----------- | ----------- | ------------------------------------------------------------------------ |
| `amplitude` | null        | 自动计算各方向连续可达行程；指定时为有限正数，单位 m，按实际可达范围截取 |
| `speed`     | 0.01        | 有限正数，单位 m/s；运动设置与模型决定实际可执行速度                     |
| `axes`      | [0,1,2]     | 基座 X/Y/Z；重复值合并                                                   |
| `joint_speeds` | [] | 关节试验速度组，rad/s，空数组采用默认三档 |
| `motion`    | 1 / 0.6 / 6 | joint_speed / acceleration / jerk，分别为 rad/s、rad/s²、rad/s³          |

预览使用当前指令保持位置和当前 TCP，沿各方向生成正向、反向、回原位片段。保持 TCP 姿态，连续求解关节构型；不足整段时保留可达行程。返回 `program` 与 `points`：

- `program`：side、origin（七轴 rad）、tcp（Link7→TCP）、amplitude、speed、joint_excursion（默认 16°，目标 ±8°）、motion、auto_range、segments、guide。
- `guide[]`：axis、offset（基座 m）、joints（七轴 rad），保存定时轨迹经过的关节节点，随 program 原样提交；旧报告可以缺省。
- `ranges[]`：axis、negative、positive，单位 m；表示从当前姿态出发、保持 TCP 方向时，各基座方向的连续可达范围。它不是所有可能关节构型的全局工作空间。
- `segments[]`：axis（0..2）、offset（相对原始 TCP 的绝对带符号偏移，m）。其他分量仍以原位为准。
- `points[]`：joints（七轴 rad）、tcp（基座坐标 XYZ，m）、segment、space、axis。space 为 cartesian、joint 或 joint_return；points 同时包含末端路线、冗余自由度回原位和全部关节试验。

生成时已确定每段末端轨迹与各档关节试验的关节位置和时标。笛卡尔点的 `t_s` 从各段起点计秒，`cartesian_duration_s` 是单轮所选 XYZ 方向往返的计划用时。关节试验、参数分析以及第二轮往返另计。`program.preview_id` 指向生成的计划，`joint_targets` 给出 J1–J7 各自的负向、正向目标角，单位 rad。确认后，控制线程和串口发送线程按各自的时钟从同一条曲线取样。若起始保持目标有轻微变化，先从最近发送位置平滑接入起点。

前端使用 points 按阶段显示彩色路线、方向、关键姿态与连杆扫掠，可切换视角和播放。范围盒可单独打开。预览在工作线程计算，不占用控制周期；生成预览不改变参数或使能状态。

**确认启动**：把收到的 `program` 原样放入 `settings.cartesian`，提交普通命令：

```json
{
  "request_id": "req-calibration-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 5,
  "kind": "calibrate_start",
  "settings": {
    "cartesian": {
      "side": "right",
      "origin": [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3],
      "tcp": { "translation": [0, 0, 0], "rotation": [0, 0, 0, 1] },
      "amplitude": 0.05,
      "speed": 0.01,
      "joint_excursion": 0.2792526803190927,
      "segments": [
        { "axis": 0, "offset": 0.05 },
        { "axis": 0, "offset": -0.05 },
        { "axis": 0, "offset": 0 }
      ]
    }
  }
}
```

启动时原样提交预览返回的 `program`。省略关节速度设置会沿用预览的速度组。如果另给 `settings.speeds`，或使用旧接口的非默认 `settings.speed`，对应的速度组应与预览一致。更换机械臂、修改 TCP、明显改变起始姿态或更改速度组后，需要重新生成预览，原计划返回 `stale_preview`。

程序顺序：

1. 按预览的末端三向路线记录七轴基线。
2. 以原位为中心逐轴 ±8° 往返，默认使用 1°/s、2°/s、4°/s 三档，采集正、负两个方向的实测速度、位置与反馈扭矩。
3. 进入 `identifying`，后台依据逐轴反馈扭矩减实际反馈位置处 URDF 模型重力的残差，分别拟合正负方向的 Fc、Fs、B、Vs，同时拟合加速度项 `inertia*a`。
4. 对每个所选轴产生的有效拟合候选执行实测比较，合并各轴优选结果；数据不足的轴保持起始配置并报告部分辨识结果。
5. 执行同一条末端路线验证合并参数；纯关节模式使用合并参数逐轴复测。正常结束回到原始关节姿态，恢复开始前的伺服与运动设置。

Kp、Kd 和速度前馈权重在整个校准中保持起始值，不进行 PID 自动整定。候选来自本次辨识，不再搜索固定 Fc=0.1/0.2 Nm、75% 速度前馈或只挑两轴。只有合并参数复测改善后才生成推荐，仍需显式应用；没有改善或某轴不可辨识也正常结束并保留数据。

这里拟合的是 `feedback_effort_residual`，即反馈扭矩与模型重力的经验残差。`inertia*a` 用于分离加速段影响，不作为补偿项下发；这些结果不等同于独立测得的摩擦力矩或真实等效刚度。重力、加速度和拟合均在后台计算，控制线程不等待拟合、日志或 SQLite。

`settings` 字段可单独省略。省略 cartesian 时为纯关节模式；完整省略 settings 的现有 API 仍可用：

| 字段           | 默认                        | 范围                                                                                                                                |
| -------------- | --------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `axis`         | null，七轴                  | 0..6 指定单轴；末端程序测试七轴                                                                                                     |
| `excursion`    | 0.2792526803190927（16°）   | 有限正数，rad，目标取一半；末端程序采用 program.joint_excursion                                                                     |
| `speed`        | 0.03490658503988659（2°/s） | 有限正数，rad/s；用于推导缺省速度档                                                                                                 |
| `speeds`       | []                          | 可选正有限数数组，rad/s；省略或空数组时为 speed/2、speed、2×speed，显式数组不另设档数或速度上限                                     |
| `max_friction` | null                        | 可省略或为 null，按各轴有效样本的 `abs(反馈扭矩−URDF重力)` 峰值自动确定补偿限幅；非负有限数为使用者指定的 Nm 上限，0 不生成补偿候选 |
| `cartesian`    | null                        | 已预览的 CartesianProgram                                                                                                           |

校准的自动限幅与 `JointTuning.max_friction` 的 0.5 Nm 基线用途不同，不以固定候选幅值替代实测残差。显式速度与幅度是试验设置，实际动作继续受现有运动设置和模型约束，不增加另一组固定上限。测试沿用已有运动学、停止和设备故障处理。片段完成看生成的参考轨迹，不用原始反馈速度或另设的位置静止窗决定下一步。HTTP `202 queued` 后按请求 ID 查看结果及 `Snapshot.calibration`；后端自行推进，客户端只需续期控制权，不发送逐段点动。

| 状态字段                  | 含义                                                                                                                                                         |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `id` / `request_id`       | 报告 / 启动请求 ID                                                                                                                                           |
| `state`                   | running、stopping、completed、cancelled、failed                                                                                                              |
| `stage`                   | cartesian_before、baseline、identifying（后台辨识）、candidate、cartesian_after、validation（纯关节复测）、restoring、done；保留 baseline_check 供旧报告读取 |
| `space` / `axis`          | 当前阶段为 joint 或 cartesian；axis 为 J1..J7 或基座 XYZ 的零起始索引                                                                                        |
| `trial_index/trial_count` | 组进度，候选由实测选择，总数会调整                                                                                                                           |
| `leg_index/leg_count`     | 当前组片段进度                                                                                                                                               |
| `progress`                | 0..1，终态以 state/report 判断                                                                                                                               |
| `live`                    | 运行中的最近一个曲线点，终态省略                                                                                                                             |
| `report`                  | 终态摘要，curves 为空，sample_count 与 captured_samples 保留                                                                                                 |

`trials[]` 含 space、phase、axis、config、metrics 与 metrics_by_joint。关节试验的 metrics 为该轴统计；末端试验 axis=null、metrics=null，metrics_by_joint 按 J1..J7 保存七项统计。窗口数 windows、有效时长 valid_ms、速度误差 RMS velocity_rms（rad/s）、位置 RMS/峰值 tracking_rms/tracking_peak（rad），other_peak 为其他轴相对原位偏移。历史试验仍可有 baseline_check/validation 阶段。

候选按相对基线的速度误差 0.6、位置误差 0.4 综合比较，没有固定改善百分比门槛。末端程序用末端复测的七轴综合指标给出结论。`choices[]` 保存所选候选索引及单项相对变化；`recommended` 为完整 ServoTuning 或 null，`improvement` 为综合指标变化，不能替代各轴位置和速度对比。

校准因电机故障帧终止时，`report.fault` 保留触发故障的单轴反馈，即使该帧落在两个曲线采样点之间。`axis` 为 J1..J7 的 0..6 索引，`code` 为原始非零故障位（u32），`message` 为中文说明，`t_ms` 为主机收到该帧时距校准开始的毫秒数。帧内的 `driver_temperature`、`winding_temperature` 分别为驱动和绕组温度（°C），`position` 为实测位置（rad），`effort` 为实际反馈扭矩（Nm），`voltage` 为电压（V）。`command` 为收帧时该轴最近成功发送的目标位置（rad），无对应记录时为 null。可选的 `feedforward_effort` 为同一成功批次中该轴的合计前馈扭矩（Nm），无对应批次或旧报告可缺省或为 null。无对应故障帧时省略 `fault`。读取历史报告时，缺省或 null 均表示未记录。

`report.identification` 为可选数组，旧报告省略时按 `[]` 读取。每项包含：

| 字段                    | 含义与单位                                                  |
| ----------------------- | ----------------------------------------------------------- |
| `axis` / `source`       | J1..J7 的 0..6 索引；来源固定为 `feedback_effort_residual`  |
| `samples`               | 参与该轴辨识的样本数                                        |
| `positive` / `negative` | `DirectionFit` 或 null；该方向不可辨识时为 null，不补造曲线 |
| `inertia`               | 加速度干扰项的拟合系数，kg·m²；不作为刚度或已验证的物理惯量 |
| `max_friction`          | 本次实际使用的逐轴补偿预算值，Nm                            |

`DirectionFit` 包含 `curve:FrictionCurve`、`samples`、`velocity_min/velocity_max`（该方向有符号的实测速度范围，rad/s）、`rmse`（加权模型残差 RMSE，Nm）及 `points:FitPoint[]`。每个绘图点包含 `velocity`（rad/s）、`effort/predicted`（残差与模型预测的加权均值，Nm）和聚合样本数 `samples`；预测为未截断摩擦模型加 `inertia*a`；补偿幅值只用于实际输出，不参与辨识模型的截断，velocity、effort、predicted 均保留方向符号。模型 RMSE 与位置/速度跟随 RMS 分别报告。曲线参数含义与 5.9 相同；所有参数、速度及力矩采用 SI 单位。仅 Stribeck 衰减不可辨识时可退化为 Fc=Fs 的平滑库仑/黏性曲线；整个方向不可辨识时为 null，缺失方向沿用起始补偿模型，已有辨识结果的方向仍参与比较；两方向均不可辨识时该轴保持起始配置。

**曲线**每 20 ms 采集七轴数据，在 90000 点的内存空间内按试验组保留真实样本。较长试验使用保留每组端点的抽样，覆盖全部试验和最终复测，不截断后半段。`captured_samples` 是采集次数（旧报告缺失时为 0），`sample_count` 是保留点数，详情抽取后仍保持这两个计数。后台按时间排序，曲线采用实际时间戳。所有数组为 J1..J7：

| 字段                       | 含义                                                  |
| -------------------------- | ----------------------------------------------------- |
| `t_ms`                     | 距测试开始的主机时间，ms                              |
| `trial` / `space` / `axis` | 试验及运动空间/轴；其他六轴仍完整采样                 |
| `reference`                | 控制参考位置，rad                                     |
| `command`                  | 最近成功发送整批目标，rad；无记录时 null              |
| `feedback`                 | 缓存实测位置，rad                                     |
| `velocity`                 | 成功发送的未缩放轨迹速度，rad/s；无记录时使用控制参考 |
| `feedback_velocity`        | 原始回传速度，rad/s                                   |
| `effort`                   | 可选七轴实际反馈扭矩，Nm                              |
| `feedforward_effort`       | 与 `command` 同一成功发送批次的七轴合计前馈扭矩，Nm，可缺省或为 null |
| `driver_temperature`       | 可选七轴驱动温度，°C                                  |
| `winding_temperature`      | 可选七轴绕组温度，°C                                  |
| `feedback_age_ms`          | 各轴反馈年龄，ms                                      |
| `command_at_ms`            | 该批发送完成的相对时间，ms；无记录时 null             |

曲线点与故障记录的 `feedforward_effort` 都取发送端限幅后、编码前的扭矩指令（Nm）。驱动器另行叠加位置与速度 PD 得到总输出，电流另以 A 计。每轴的 `effort`、两类温度与实测位置取自该轴同一反馈帧，各帧距采样点的年龄见 `feedback_age_ms`。旧报告可独立省略任一可选数组，缺省或 null 均表示未记录。只有统计时展示指标，不生成时间曲线。辨识所用重力在后台按实际反馈关节位置和 URDF 计算。抽样前保留每条反馈的原始邻帧时间和速度，后台据此计算加速度与时间权重，避免把抽样间隔误作反馈间隔。

**应用**：用最新 generation 提交 `kind:"calibrate_apply", calibration_id:"..."`。只应用当前连接最近完成且含推荐的报告；手工改过伺服配置或已应用的结果不重复覆盖。沿用普通参数应用的连接、控制权和空闲要求。结果界面列出逐轴正负方向拟合、参数原值/推荐值及同轨迹前后表现，单轴 Stribeck 开关可在伺服参数中显式切换。

正常完成回原位并恢复原参数；停止取消后续动作、按原停止流程减速并恢复参数，不继续回原位。关闭弹窗可重新查看进度，Esc 请求停止。故障不自动重新使能。

**保存与读取**：完整结果在发布 completed 前即保留于本机内存，详情、下载和历史可立即读到；SQLite 编码、写入及重试仍在写入线程，不阻塞控制。元数据与曲线同事务保存，保留最近 100 份，重启后读取数据库。内部完整结果不会重复进入 HTTP/WS 状态 JSON。

- `GET /v1/calibrations?limit=20`：摘要，limit 1..100，合并刚完成结果。
- `GET /v1/calibrations/{id}?points=3000`：默认约 3000 个真实点，保留段端点和七轴位置/速度误差极值；points=0 返回全部保留点。
- `GET /v1/calibrations/{id}?download=true`：完整 JSON 附件。

读取、绘图点抽取和编码使用工作线程。未知历史 ID 返回 `404 calibration_not_found`；写盘尚未完成不会让当前结果返回 404。历史可查看、导入和下载，重启后不自动取得应用资格。操作与试用见[校准流程](low-speed-calibration-test-plan.md)。发送时间对齐、轨迹、采样及本轮辨识补偿尚未真机验证。

### 5.11 拖动示教

示教沿用本机 Bearer/Origin 认证。历史、详情、下载与预览只读，无需运动租约；四种示教命令仍使用公共请求字段与当前控制权；模式和重放检查当前 generation，独立录制开始/结束不比较该值。拖动模式、位置录制和轨迹重放分别报告状态；录制可以与保持、拖动、点动、步长、校准、收纳返回或重放并行。运动入口继续使用原有使能、反馈及 TCP 条件。操作与离线验收见[示教操作与测试](teaching-test-plan.md)。

**拖动与保持**：控制页“示教”弹窗始终显示“位置保持 / 拖动示教”。点击拖动后持续生效，可用手移动机械臂；点击位置保持后，以切换时的实测位置建立保持目标。位置刚度在 `transition_s` 内平滑切换，保留伺服模式、模型重力补偿和当前阻尼，不跳回起始姿态。重复开启不重启过渡。模式本身不开始录制，也不需要按住按钮、空格或定时续发拖动输入。

**独立录制**：`teach_start` 只记录七轴实测位置，不改变运动模式、控制参考、使能或 generation。有有效反馈时，已连接未使能也可录制；软件运动期间开始录制不会取消原动作。`teach_finish` 在拖动模式中先平滑恢复当前位置保持再保存；其他情况下只结束录制，不取消点动、校准、收纳返回或重放。切回保持继续录制；切换页签、失焦或页面隐藏不暂停录制、不改变模式。

普通机械臂停止、点动输入到期、动作完成、故障以及使能/失能都不结束独立录制；反馈有效时继续采集，缺失时不编造样本。停止按钮和 Esc 停止机械臂，录制仍可继续，适合连续记录多次点动与保持。显式 `teach_finish`、控制权失效/释放、断开、关闭示教弹窗和应用退出才结束并保存。“收起”仅隐藏弹窗，控制页入口继续显示录制状态；再打开沿用当前记录。关闭按钮或实际卸载弹窗会显式发送结束录制，而不只调用机械臂停止。

| 命令字段                     | 默认 | 要求                                                                               |
| ---------------------------- | ---- | ---------------------------------------------------------------------------------- |
| `teach_mode.enabled`         | 必填 | boolean；true 持续进入拖动，false 平滑恢复当前位置保持                             |
| `teach_mode.transition_s`    | 0.4  | 有限正数，s，拖动/保持切换时间                                                     |
| `teach_start.name`           | `""` | string；空白名称由后端命名为“示教轨迹”                                             |
| `teach_finish.recording_request_id` | 省略 | 开始录制的请求 ID；绑定指定录制，开始请求尚未执行时也能取消。省略时结束当前录制。 |
| `teach_replay.trajectory_id` | 必填 | 已保存轨迹 ID，可读取尚未提交 SQLite 的最新完整录制                                |
| `teach_replay.speed_scale`   | 1    | 有限正数；请求倍速，实际结果见预览                                                 |
| `teach_replay.smoothing_s`   | 0.12 | 有限非负数，s；0 有效                                                              |

示教模式没有独立输入有效期；控制权仍按现有租约续期。停止、控制权失效或退出会结束拖动并恢复保持。录制的开始不推进 generation；`teach_start/teach_finish` 不因编号变化而拒绝，也不会绕过控制权检查。这两种请求在当前控制权期间保留 `request_id` 并跨 generation 去重；其他命令仍从最新快照读取 generation。结束不增加静止等待判定或固定请求时限，当前反馈缺失也可保存已经采集的数据。未知 JSON 字段仍会被忽略；例如在 `teach_start` 中发送额外的 `transition_s` 不会切换模式，切换时间只由 `teach_mode` 使用。

关闭弹窗时，`teach_finish.recording_request_id` 使用原 `teach_start.request_id`。若开始请求仍未执行，会取消该请求而不生成空记录，结束请求返回 `completed/recording_ended`，开始请求返回 `cancelled/recording_cancelled`；迟到的开始请求不会重新录制。指定旧录制编号不会结束另一条新记录。

以下为一次拖动录制的完整请求，标识和 generation 均为示例，实际使用每一步确认后的上下文。先进入持续拖动，再开始录制：

```json
{
  "request_id": "req-teach-mode-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 5,
  "kind": "teach_mode",
  "enabled": true,
  "transition_s": 0.4
}
```

```json
{
  "request_id": "req-teach-start-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 6,
  "kind": "teach_start",
  "name": "装配路径"
}
```

需要暂时固定姿态时切回保持，录制继续：

```json
{
  "request_id": "req-teach-hold-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 6,
  "kind": "teach_mode",
  "enabled": false,
  "transition_s": 0.4
}
```

完成录制提交；处于拖动模式时也可直接结束，无需先发送切回保持：

```json
{
  "request_id": "req-teach-finish-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 6,
  "kind": "teach_finish"
}
```

**保存与读取**：每个点为 `{t_ms:u64,q:[number;7]}`，分别表示相对录制开始的单调主机毫秒和 J1..J7 实测位置（rad），包含保持段及其他运动期间的反馈。控制周期采用分块追加，不截断样本、不全量重采样或序列化。结束形成完整内存录制并发布 `latest_teaching`；`teaching_saved` 不表示 SQLite 事务已经提交。后台写入线程展开、编码并同事务写入摘要和全部点，队列受阻时控制线程保留补投。用户轨迹不受校准报告保留数量影响。

`GET /v1/teachings` 返回按 `created_at_ms`、`id` 降序排列的摘要，并按 ID 合并最新内存录制；详情与下载也优先读取相同 ID 的内存录制，因此无需等数据库提交。`GET /v1/teachings/{id}?download=true` 只增加 `Content-Disposition: attachment; filename="teaching-<id>.json"`，JSON 结构相同；文件名中除 ASCII 字母、数字、`-`、`_` 外的字符替换为下划线。读取和预览均返回 `Cache-Control:no-store`。

| `TeachingSummary` 字段         | 类型 / 含义                                     |
| ------------------------------ | ----------------------------------------------- |
| `id` / `name`                  | string，轨迹 ID / 名称                          |
| `created_at_ms`                | i64，创建时的 Unix 毫秒                         |
| `side` / `tcp`                 | left 或 right / `Pose`；录制时的 Link7→TCP      |
| `duration_ms` / `sample_count` | u64 / 非负整数；录制时长 / 全部点数             |
| `outcome`                      | string，结束原因，如 completed、stopped、failed |

`TeachingTrajectory` 在摘要的同一层增加 `points:TeachingPoint[]`，没有 `summary` 或 `data` 包装；下载包含全部录制点，详细示例见 OpenAPI。

**预览与回放**：`POST /v1/teachings/{id}/preview` 请求可为 `{}`，或 `{"speed_scale":1.5,"smoothing_s":0.12}`。读取、平滑、重定时及逐点 FK 在工作线程执行，不改变控制权、generation、配置或动作。当前侧别必须与录制相同；预览包含从当前指令保持位置接近首点及完整录制路径，使用真实机械臂模型和录制 TCP。

| `TeachingPreview` 字段                            | 类型 / 含义                                                                                                                   |
| ------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `trajectory_id` / `side` / `tcp`                  | 轨迹 ID / 左右臂 / 录制 TCP                                                                                                   |
| `origin`                                          | 七轴起始指令位置，rad                                                                                                         |
| `duration_ms` / `approach_duration_ms`            | 有限非负 number，ms；包含接近的总时长 / 接近时长                                                                              |
| `requested_speed_scale` / `effective_speed_scale` | 有限正 number，请求 / 规划实际倍速                                                                                            |
| `points[]`                                        | `{t_ms,joints:[number;7],tcp:[number;3],phase}`；时间相对程序开始，关节为 rad，TCP 为基座 XYZ（m），phase 为 approach 或 path |

3D 区分接近/录制路线并显示整臂连杆扫掠，支持按阶段播放与拖动；曲线分别显示录制实测、平滑重定时后的规划指令和本次回放反馈。反馈按各关节的接收时间排列，回放首点以执行时实际使用的接近段时长对齐。预览与状态反馈曲线不代替实际发送记录，后者仍由既有短时采样读取。

回放仅提交轨迹 ID 和选项，不上传预览点或内部执行程序：

```json
{
  "request_id": "req-teach-replay-001",
  "client_id": "example-service",
  "lease_id": "11111111-1111-4111-8111-111111111111",
  "generation": 7,
  "kind": "teach_replay",
  "trajectory_id": "44444444-4444-4444-8444-444444444444",
  "speed_scale": 1.5,
  "smoothing_s": 0.12
}
```

服务端在入队前通过 worker 读取和准备程序，保留调用方原 `generation` 与租约。准备失败可直接返回 JSON 400，轨迹不存在为 404，存储不可读为 503；成功入队后仍可能异步 `rejected/not_owner`、`stale_context` 或 `motion_busy`。准备期间上下文变化不会被服务端自动修正，外部也不能提交内部 `teaching_prepared`。

平滑开启时，规划用平滑窗口上的等时节点拟合并保留首末点，原始录制仍完整保存。回放按当前运动设置与模型速度约束重定时，不另加倍速、幅度、时长或样本数量上限；实际倍速和总时长以规划为准。执行从当前有效运动起点重新接近首点，再输出整条路径的 `q/dq/ddq`，复用现有发送取样、伺服补偿、TCP、停止和收纳流程，不重放原始扭矩。正常完成保持末点；停止取消余下轨迹，不自动返回首点或收纳。

**三个独立状态**：`Snapshot.teaching_mode` 只表示拖动模式，`teaching_recording` 只表示录制，`teaching` 只表示重放。没有对应活动时各自为 null。录制可与拖动或重放同时存在，不能用其中一个字段推断另外两个状态。`latest_teaching` 为最近完整录制摘要，尚无记录时为 null。状态不带完整点，完整点通过详情读取。

| `TeachingModeStatus` 字段 | 含义                                                                  |
| ------------------------- | --------------------------------------------------------------------- |
| `request_id`              | 设置当前拖动模式的请求 ID                                              |
| `state`                   | entering、dragging、leaving 或 holding；正常恢复位置保持后整个字段为 null |

| `TeachingStatus` 字段 | `teaching_recording` / `teaching` 的含义                                |
| --------------------- | ----------------------------------------------------------------------- |
| `id` / `request_id`   | 当前轨迹 ID / 启动录制或回放的请求 ID                                   |
| `state`               | 录制为 recording 或 finishing；重放为 approaching 或 replaying           |
| `recording`           | `teaching_recording` 中恒为 true；`teaching` 中恒为 false                |
| `sample_count`        | 当前录制点数；回放为 0                                                  |
| `duration_ms`         | 录制已采集时长或回放计划总时长，u64 ms                                  |
| `progress`            | 0..1；录制为 0，回放为包含接近的时间进度，终态以请求结果判断            |
| `elapsed_ms` / `approach_duration_ms` | 可省略。回放实际经过时间 / 从执行起点计算的接近段时长，毫秒 |
| `trajectory_id`       | string 或 null；录制时等于 `id`，重放时关联正在执行的已保存轨迹           |

仅手动拖动及其过渡使用 `phase:"teaching"`。录制不创建 `action`，也不改变原有 phase；点动、步长、校准、收纳返回和重放继续报告各自动作。重放使用 `phase:"moving"`、`action.mode:"replay"`、`action.space:"joint"`、`axis:null`。录制重放反馈时，`teaching_recording` 与 `teaching` 同时存在。

保存完成后 `teaching_recording:null`，启动与结束录制请求产生 `completed/teaching_saved`；其他软件动作不因保存被取消。普通停止或故障本身不结束录制；若已请求结束、正在恢复保持时被停止或故障打断，则保存已有点，并向等待结束的请求报告实际 stopped/failed 结果。重放末点发送成功并收到新反馈后返回 `completed/replayed`。如果过程中任何一轴的实测位置与同一时刻最近下发的位置相差超过 0.02 rad，则返回 `completed/replayed_with_deviation`，并显示最大偏差对应的关节和角度。取消或故障沿用现有结果语义。

## 6. 命令输出与执行结果

请求成功入队后返回；`teach_replay` 先等待后台准备完成：

```json
{ "request_id": "req-step-001", "status": "queued" }
```

**HTTP 202 只表示请求入队。** 控制权、`generation`、状态与运动参数在控制线程中继续校验，校验失败会出现在状态的 `results` 中。

`CommandResult` 格式：

```json
{
  "request_id": "req-step-001",
  "status": "completed",
  "code": "completed",
  "message": "动作已完成"
}
```

| `status`    | 含义                                                     |
| ----------- | -------------------------------------------------------- |
| `accepted`  | 控制线程已接受请求；连接、配置等操作可在此阶段按状态确认 |
| `rejected`  | 请求未执行，查看 `code`                                  |
| `completed` | 需要完成确认的操作已完成                                 |
| `cancelled` | 已开始的操作被停止、释放控制权或输入过期等原因取消       |
| `failed`    | 执行过程中失败                                           |

`queued` 只出现在 HTTP 入队响应中，不属于 `CommandResult.status`。连续点动通常以取消或失败结束，不自动产生 `completed`。

调用方在 `results` 中按 `request_id` 查找**第一条匹配记录**。数组按新到旧排列，同一请求可先有 `accepted`，后有 `completed`。`last_result` 是所有客户端共享的最新结果，不能单独用它跟踪自己的请求。

`results` 最多保留 32 条结果记录。去重历史按 `(generation,request_id)` 保存，最多 256 项；相同上下文内的重复请求可能被忽略，不产生新结果。它不是持久化的去重记录。网络超时后先查状态与结果，不自动补发运动；旧结果已经移出缓存也不能据此判定请求未执行。

## 7. 状态输出

`GET /v1/state` 返回 `Snapshot` 本体。WebSocket 使用相同结构作为 `data`。所有字段均存在；暂无数据的可选字段返回 `null`。

| 字段                      | 类型                           | 含义                                                  |
| ------------------------- | ------------------------------ | ----------------------------------------------------- |
| `seq`                     | u64                            | 当前应用运行期间的快照序号                            |
| `generation`              | u64                            | 当前控制上下文编号，提交命令时原样带回                |
| `phase`                   | string                         | 运行阶段，见下表                                      |
| `side` / `port`           | string / string 或 null        | `left` 或 `right`；当前串口                           |
| `enabled`                 | boolean                        | 已确认的使能状态                                      |
| `owner`                   | object 或 null                 | `{client_id,expires_in_ms}`；不包含控制权凭据         |
| `joints`                  | 7 个 `JointSnapshot`           | J1..J7 的实测反馈                                     |
| `tcp`                     | `Pose` 或 null                 | 根据实测关节计算的基座 → 当前 TCP 位姿                |
| `tcp_transform`           | `Pose` 或 null                 | 当前 Link7 → TCP 变换；连接后默认为单位变换           |
| `tcp_configured`          | boolean                        | 是否有可用的 TCP；连接后默认 Link7 法兰为 true        |
| `servo_tuning`            | `ServoTuning`                  | 当前完整伺服配置，见 5.9；连接/断开恢复基线           |
| `servo_tuning_revision`   | u64                            | 当前配置版本；应用及连接/断开重置时更新，不跨进程保存 |
| `motion_tuning`           | `MotionTuning`                 | 当前关节速度、加速度与 jerk 设置，见 5.7              |
| `feedback_age_ms`         | u64 或 null                    | 已有反馈中最大的接收年龄                              |
| `action`                  | object 或 null                 | 当前动作：`request_id,space,mode,axis,progress`       |
| `calibration`             | `CalibrationStatus` 或 null    | 当前校准或最近一次结果，见 5.10                       |
| `teaching_mode`           | `TeachingModeStatus` 或 null   | 拖动模式与进入/恢复保持过渡，见 5.11                  |
| `teaching_recording`      | `TeachingStatus` 或 null       | 独立录制，state 为 recording/finishing，recording 为 true |
| `teaching`                | `TeachingStatus` 或 null       | 重放，state 为 approaching/replaying，recording 为 false |
| `latest_teaching`         | `TeachingSummary` 或 null      | 最近完整示教摘要；SQLite 提交前即可读取完整轨迹       |
| `last_result` / `results` | `CommandResult` 或 null / 数组 | 最新一条结果 / 最近结果                               |
| `error`                   | `{code,message}` 或 null       | 当前故障                                              |
| `journal`                 | object                         | `pending,written,dropped,errors,healthy`              |
| `communication`           | object                         | `received,crc_errors,timeouts`，本次运行的通信计数    |

| `phase`        | 含义                          |
| -------------- | ----------------------------- |
| `disconnected` | 未连接                        |
| `connecting`   | 串口已打开，等待反馈          |
| `disabled`     | 已连接，未使能                |
| `enabling`     | 已发送使能，等待反馈确认      |
| `teaching`     | 手动拖动及进入/恢复保持过渡   |
| `ready`        | 已使能，可以接受动作          |
| `moving`       | 动作执行中                    |
| `stopping`     | 减速停止中                    |
| `fault`        | 存在需要清除的故障            |
| `closing`      | 应用正在退出                  |

`JointSnapshot` 字段如下：

| 字段                      | 类型 / 单位            | 含义                     |
| ------------------------- | ---------------------- | ------------------------ |
| `id` / `name`             | 整数 / string          | 0..6 / J1..J7            |
| `q`                       | number 或 null，rad    | 位置                     |
| `dq`                      | number 或 null，rad/s  | 速度                     |
| `tau`                     | number 或 null，Nm     | 估算扭矩                 |
| `age_ms`                  | u64 或 null，ms        | 这条反馈距快照生成的时间 |
| `temperature`             | number 或 null，°C     | 驱动温度                 |
| `winding_temperature`     | number 或 null，°C     | 绕组温度                 |
| `voltage`                 | number 或 null，V      | 电压                     |
| `error`                   | u32 或 null            | 原始设备故障位           |
| `error_reason`            | string 或 null         | 当前非零故障位的中文说明，无反馈或故障位为 0 时为 null |

从未收到的反馈为 `null`。反馈过期后，关节数值保留最后一次测量，`age_ms` 继续增长。控制使用的七轴反馈需全部存在、单轴年龄不超过 200 ms，不再附加七轴接收跨度门槛。仅检查 `feedback_age_ms` 不能证明七轴已经齐全。

控制线程先取串口已收到的帧，再按各轴的接收时间判断反馈年龄。执行校准轨迹时，发送端还会检查最旧反馈的年龄。短时中断期间发送端保持最近成功写入的位置，暂停轨迹计时。七轴反馈恢复并确认保持指令已写入后，从保持点平滑继续。等待期间不累计测量时间，摩擦拟合不跨反馈空档计算加速度。持续缺少有效反馈时，校准报告保留已采集的数据并指出过期关节。

TCP 未设置或控制反馈无效时，`tcp:null`。`action.mode` 为 `continuous`、`step`、`return`、`calibration` 或 `replay`，收纳与重放的 `axis:null`，`progress` 范围 0..1。录制不创建虚拟动作，不改变正在执行的动作状态。`seq` 和 `generation` 不跨应用重启保存；重新连接应用后重新读取状态。快照通常约 30 Hz 更新，状态变化时也可提前发布。HTTP 返回缓存中的最近快照，调用方还需考虑接收后经过的时间。

完整的未连接状态示例和嵌套类型见 [OpenAPI](openapi.json) 的 `Snapshot` 与 `GET /v1/state` 示例。

## 8. WebSocket 输入与输出

连接 `ws://127.0.0.1:8177/v1/events` 后，**2 秒内**发送第一条文本消息：

```json
{ "token": "example-application-token" }
```

认证通过后直接收到完整状态，没有单独的认证成功消息。HTTP 的 Authorization 头不能替代认证首帧，令牌不放入 URL。

输出消息格式为 `{"type":"state","data":Snapshot}`，其中 `Snapshot` 为第 7 节定义的对象。连接后立即推送当前状态，随后推送更新；允许跳过中间序号，不提供历史重放。WebSocket 只用于订阅，设备命令仍走 HTTP。

认证之后，客户端只发送 WebSocket 协议层 Ping/Pong。额外的应用文本或二进制消息会结束连接，不使用 JSON 心跳。当前最多 16 个订阅连接；客户端输入帧和消息上限为 8 KiB；服务端发送超时为 500 ms，慢速连接会被断开。应用退出时发送 `closing` 状态后关闭连接。

## 9. 串口与日志输出

`GET /v1/ports` 返回数组。macOS 上同一设备的 `/dev/cu.*` 与 `/dev/tty.*` 会合并为一项并优先返回 `cu`；结果按 `name` 排序：

```json
[{ "name": "/dev/cu.usbserial-001", "label": "USB Serial", "kind": "usb" }]
```

三个字段都是字符串。`kind` 为 `usb`、`bluetooth`、`pci` 或 `unknown`。没有枚举到串口时返回 `[]`；系统枚举错误目前也会折叠为空数组。

`GET /v1/logs?limit=50` 返回按 `id` 降序的日志：

```json
[
  {
    "id": 101,
    "ts_ms": 1794268800000,
    "level": "info",
    "source": "control",
    "code": "acquired",
    "message": "控制权已取得",
    "request_id": null
  }
]
```

`id` 为 SQLite 记录编号，跨应用运行保留；`ts_ms` 为 Unix UTC 毫秒时间戳。`level/source/code/message` 均为字符串，`request_id` 为字符串或 null。

`limit` 为可省略的 u32，默认 50；传入 0 时按 1 查询，超过 500 时按 500 查询。日志保留最近 100000 条。日志异步写入 SQLite，提交动作不会等待数据库；刚产生的事件可能尚未出现在查询中。

`journal.pending` 表示已接收但尚未写入的记录，`written` 表示本次运行成功提交的数量；`dropped` 统计队列竞争、锁损坏、满队列、关闭后提交或退出时放弃的记录；`errors` 统计数据库操作失败。事件提交只尝试一次取得预分配队列的锁，失败立即返回；后台轮询取出数据后执行 SQL。计数均针对本次应用运行，不等同于数据库总行数。查询使用独立阻塞任务，不在控制线程执行。

### 历史日志分页

`GET /v1/logs/page` 返回完整数据库中的历史记录，无需取得控制权。日志存放在应用日志目录的 `events.sqlite3`，退出后保留，仍按最近 100000 条清理。

| 查询参数     | 类型与默认值     | 规则                                                                                     |
| ------------ | ---------------- | ---------------------------------------------------------------------------------------- |
| `page`       | u32，默认 1      | 0 按 1，超过末页时返回末页                                                               |
| `page_size`  | u32，默认 50     | 限制到 1..500，0 按 1                                                                    |
| `search`     | string，默认空串 | UTF-8 最多 512 字节；在级别、来源、编号、内容和请求 ID 中搜索字面子串；ASCII 不区分大小写 |
| `through_id` | 可选 i64         | 非负；省略时取当前最大 ID，传入时只查询不大于该 ID 的记录；0 表示空范围                  |

可组合 `level`、`source`、`code` 精确匹配，以及 `from_ms`、`to_ms` 两个含端点的 Unix 毫秒时间。省略任一端即可查询开放区间。所有条件与 `search` 同时生效，计数和翻页使用同一组条件。修改条件后从第一页重新查询。`GET /v1/logs/filters` 返回数据库已有的级别、来源、编号，例如 `{"levels":["info"],"sources":["control"],"codes":["acquired"]}`。

```http
GET /v1/logs/page?page=2&page_size=50&through_id=123
Authorization: Bearer <token>
```

```json
{
  "items": [
    {
      "id": 73,
      "ts_ms": 1794268800000,
      "level": "info",
      "source": "app",
      "code": "started",
      "message": "控制工作台已启动",
      "request_id": null
    }
  ],
  "page": 2,
  "page_size": 50,
  "total": 123,
  "total_pages": 3,
  "through_id": 123
}
```

示例 `items` 只列一条；实际返回该页全部记录，顺序为 `id` 降序。`total` 是当前搜索与 ID 范围内的总条数；空结果仍返回 `page:1,total_pages:1,items:[]`。最大 ID、总数与本页记录在同一只读事务中读取。

首屏省略 `through_id`。后续翻页沿用响应中的值，新增日志不会推移历史页；现有保留上限仍会清理最早记录。刷新最新记录或更换搜索条件时省略此值。超过当前最大值的 `through_id` 会缩到当前最大值。

参数格式、负数、整数溢出、过长搜索返回 `400 {"code":"invalid_log_query","message":"日志查询参数无效"}`。数据库不可读返回 `503 journal_unavailable`。旧的 `/v1/logs?limit=50` 继续返回数组。

### 9.1 短时采样接口

全部采样路由沿用 Bearer 与 Origin 校验，无需 `client_id`、`lease_id` 或 `generation`。开始只观察现有连接，不申请控制权、打开串口、使能或启动动作。开始要求状态中已有 `port`，且 `phase` 不为 `disconnected/connecting/closing`；否则返回 `400 not_connected`。停止采样不取消机械臂动作，也不改变控制权。

| 请求                      | 行为                                                                                                                                                                                                              |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `POST /v1/traces`         | JSON 对象，`duration_ms` 为整数，默认 10000，可表示的正整数毫秒；`{}` 或 `{"duration_ms":null}` 使用默认值，未知字段被拒绝。返回 `200 TraceSummary`。                                                             |
| `GET /v1/traces?limit=20` | `limit` 为可省略的 u32，默认 20；实际限制到 1..10，0 按 1，超过 10 按 10。返回最近的 `TraceSummary[]`。                                                                                                           |
| `POST /v1/traces/stop`    | 无需请求体。结束当前记录并返回 `200 TraceSummary`；没有正在记录的采样时返回 JSON `null`。                                                                                                                         |
| `GET /v1/traces/{id}`     | 返回 `{trace:TraceSummary,samples:TraceSample[]}`。`download=true` 或别名 `attachment=true` 增加 `Content-Disposition: attachment; filename="trace-{id}.json"`，内容仍为 JSON。响应带 `Cache-Control: no-store`。 |

开始示例：

```json
{ "duration_ms": 10000 }
```

```json
{
  "id": "22222222-2222-4222-8222-222222222222",
  "started_at_ms": 1795248000000,
  "duration_ms": 10000,
  "side": "right",
  "port": "/dev/cu.example",
  "status": "recording",
  "accepted": 0,
  "written": 0,
  "dropped": 0,
  "pending": 0,
  "errors": 0
}
```

`id` 为 UUID，`started_at_ms` 为 Unix UTC 毫秒，`duration_ms` 为请求的采样窗口；提前停止不改写该值。`side` 为 `left/right`，`port` 为开始时的串口名或 null。每次状态与计数分开解释：

| 字段       | 含义                                                                                                                             |
| ---------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `status`   | `recording` 正在接收；`writing` 已结束接收、后台仍在写入；`complete` 已完成提交；`failed` 写入数据被放弃或上次运行的采样被中断。 |
| `accepted` | 成功接纳入队的样本数。                                                                                                           |
| `written`  | 已成功提交的样本数；读取完整采样时对应该数据库快照内的样本数量。                                                                 |
| `pending`  | 已接收但尚未提交的样本数。                                                                                                       |
| `dropped`  | 队列竞争、满队列、达到提交尝试上限或已接收数据被放弃的数量。                                                                     |
| `errors`   | 后台存储、序列化等操作的失败次数；恢复后仍保留计数。                                                                             |

到时或停止后可先返回 `writing`，通过列表继续查看提交结果。`complete` 仍可能有 `dropped` 或此前的 `errors`；完整性需同时检查 `pending=0`、`written=accepted`、丢弃与错误数。读取记录中或写入中的采样只返回当时已提交的样本。

采样单独写入事件库路径追加 `.traces.sqlite3` 的数据库；默认文件名为 `events.sqlite3.traces.sqlite3`。保留最近 10 次，每次前 100000 次样本提交尝试可入队，之后计 dropped。预分配队列容量 4096，内联完整伺服配置后样本存储约 5 MiB（不含 writer/SQLite 开销）；生产者用 `Mutex.try_lock` 提交，竞争或满队列立即丢弃，无等待、SQL、JSON 编码或分配。独立 writer 轮询并取出数据后进行序列化和 SQLite 写入，查询和下载在阻塞任务中执行。这些计数按次采样统计，`Snapshot.journal` 仍只统计事件日志。

### 9.2 样本字段与时间

每条样本把 `TraceData` 直接展开在顶层，以 `source` 区分来源，没有额外 `data` 包装。`at_us` 是原始来源的单调时间相对采样开始的微秒数，使用控制周期开始、串口批次写入开始或主机接收反馈时的时间，不使用出队/提交时间。下载按 `at_us` 排序，同一时间保留提交顺序；七轴反馈仍是分别收到的观测。

所有七元素数组按 J1..J7 排列。关节位置单位 rad、速度 rad/s、加速度 rad/s²、jerk rad/s³、扭矩 Nm、电压 V、温度 °C；`at_us/age_us/write_us` 单位 μs。

| `source`   | 其余字段                                                                                                                                                                                                                                                                                                                                                           |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `control`  | `seq`、`generation`（u64），`enabled/stopping`（boolean），`motion`（0..5），`axis`（整数或 null）；`servo_tuning`（完整配置或 null，可缺省）、`servo_tuning_revision`（u64，可缺省）；`q_ref/dq_ref/ddq_ref/jerk_ref/tau_ff` 为七个数值；`q/dq/tau/voltage` 为七个数值或 null；`age_us` 为七个 u64 或 null。缺失反馈的对应元素为 null，已有旧反馈保留数值与年龄。 |
| `transmit` | `seq`（u64），`timing`（对象或 null，可缺省）、`tuning`（ServoSample 或 null，可缺省），`mode` 为七个实际编码的命令模式（0 disabled、1 servo、6 clear）；对齐后位置/未缩放轨迹速度 `q_ref/dq_ref`、最终总附加前馈 `tau_ff`、编码后反标定为关节 SI 的 `q_wire/dq_wire/tau_wire`、`kp/kd` 均为七个数值，`write_us`（u64）为主机成功写入整个七轴批次的耗时。          |
| `feedback` | `joint`（0..6），`q/dq/tau/voltage`（number），`temperature` 为驱动温度，`winding_temperature` 为同帧绕组温度（°C），旧样本缺省或为 null 表示未记录，`error`（u32）、`mode`（u8）、`timeout`（boolean）。 |

`control.motion`：0 无当前动作、1 关节连续、2 关节步长、3 笛卡尔连续、4 笛卡尔步长、5 收纳返回。`axis` 在无动作或收纳返回时为 null，关节为 0..6，笛卡尔为 0..5；减速阶段可能为 `motion:0,stopping:true`。

`transmit.seq` 关联被串口线程选中的源 `control.seq`，仍是源索引。重复序号表示同一源被再次取样，不表示相同的 `q_ref` 或 `q_wire`；序号跳过可能是参考被合并或采样丢弃。`seq:0` 用于初始或直接停止/清错写入，不要求存在对应的控制样本；普通 disabled 心跳也可以使用非零序号。

`control.q_ref/dq_ref` 是控制器原始参考；`transmit.q_ref/dq_ref` 是发送时对齐或回退保持的位置与未缩放轨迹速度，不要求与同序号控制样本相同。`q_wire/dq_wire/tau_wire` 来自实际编码值并转回关节 SI，用于观察量化、截断与理想目标的差异。每轴只有 `mode:1` 时这些字段才是伺服目标；`mode:0/6` 保留停机/清错报文字段的反标定结果，诊断编码误差时应排除。`write_us` 描述主机批次写入，不是 USB 线上时间或电机确认。

控制样本的 `servo_tuning` 保存该周期的完整配置，`servo_tuning_revision` 保存配置版本；旧文件缺省时按 null/0 读取。发送样本的 `tuning` 为 `ServoSample` 或 null，旧文件缺省按 null 读取；直接写入和非伺服批次可为 null。先通过 `transmit.seq → control.seq` 找源周期，再用 `tuning.revision` 关联 `control.servo_tuning_revision` 与完整配置；源样本丢失时保留未知，不从当前配置反推历史。

| `ServoSample` 字段   | 含义                                                                        |
| -------------------- | --------------------------------------------------------------------------- |
| `revision`           | u64，当前发送使用的配置版本                                                 |
| `dq_feedforward`     | 七轴加权后的编码前速度前馈，rad/s；速度编码误差的参照                       |
| `tau_gravity`        | 七轴单独施加重力时的限幅值 `C(G)`，Nm                                                            |
| `friction_requested` | 七轴经自身 `max_friction` 限幅后的摩擦请求，Nm                              |
| `friction_applied`   | 总前馈限幅后的摩擦增量 `tau_ff-tau_gravity`，Nm，仍为编码前值               |
| `clamped`            | 七个 boolean；摩擦项或实际合计 `G + requested` 被截断时为 true，恰在边界时该项未被截断 |

`transmit.dq_ref` 仍记录未缩放的轨迹速度，`tau_ff` 是按 5.9 计算的 `C(G + requested)`，`dq_wire` 是实际编码值。计算速度编码误差必须优先比较 `tuning.dq_feedforward` 与 `dq_wire`，仅在 `tuning` 缺省或 null 时回退到 `dq_ref`；权重为零导致的速度差不属于编码误差。位置与扭矩仍分别比较 `q_ref/q_wire`、`tau_ff/tau_wire`。限幅标记描述软件命令，不代表测得的电机总扭矩或物理补偿效果。

发送样本的 `timing` 为 `{"status":"aligned","age_us":100}` 这样的对象或 null。对象包含 `status` 和 `age_us`；后者为 u64 微秒或 null，无法计算非负源年龄时为 null。未执行发送端取样的直接写入、disabled/clear 批次为 `timing:null`。该字段以 `#[serde(default)]` 读取，旧 JSON 缺省时按 null 处理，接口不要求旧文件补字段。

| `timing.status` | 含义                                                 |
| --------------- | ---------------------------------------------------- |
| `aligned`       | 完成发送时刻取样；速度上限或首次零点封顶也归此类。   |
| `bounded`       | 关节目标使候选限幅，或末端目标越界后保持位置。       |
| `stale`         | 源年龄超过 50 ms。                                   |
| `future`        | 取样时刻早于源时刻，或不晚于上一成功伺服批次起点。   |
| `invalid`       | 数值非有限或取样参数无效。                           |
| `constraint`    | 目标 FK 无法计算；旧采样也使用此值记录路径检查失败。 |
| `expired`       | 已到输入或租约的有效截止时间。                       |

完整三类样本示例见 [OpenAPI](openapi.json) 的 `TraceSample` 和读取响应。事件日志与短时采样仍使用单次 `try_lock`、竞争即丢弃和独立 SQLite writer，不等待队列或磁盘。

当前协议速度量化步距为 `2π/(64r)`，位置步距为 `2π/(32768r)`，分组如下。极低的速度前馈（新样本为 `tuning.dq_feedforward`，旧样本为 `dq_ref`）可编码为 `dq_wire:0`，而 `q_wire` 随位置字段按阶梯前进；采样同时保留理想参考和编码值，可分别查看参考变化与量化。这是已确认的编码行为，是否造成实机抖动仍需测量。

| 关节  | 减速比 r | 速度步距（rad/s） | 位置步距（rad） |
| ----- | -------- | ----------------- | --------------- |
| J1–J2 | 32       | 0.003067962       | 5.992×10⁻⁶      |
| J3–J5 | 38/3     | 0.007750640       | 1.514×10⁻⁵      |
| J6–J7 | 16       | 0.006135923       | 1.198×10⁻⁵      |

### 9.3 采样与验收流程

离线检查从仓库根目录运行：

```bash
cargo test --manifest-path src-tauri/Cargo.toml --all-targets --locked
cargo clippy --manifest-path src-tauri/Cargo.toml --all-targets --locked -- -D warnings
cargo fmt --manifest-path src-tauri/Cargo.toml -- --check
npm run format:check
npm run build
```

下载的采样文件可单独分析：

```bash
cargo run --manifest-path src-tauri/Cargo.toml --example analyze_trace -- /path/to/capture.json
```

此示例只读取 JSON，输出 JSON 统计：控制、发送和各关节反馈的间隔 p50/p95/max，七轴批次写入耗时，参考速度/加速度/jerk 峰值，逐关节位置/速度跟随 RMS，编码值与理想参考的最大差异，发送序号重复/跳过/倒退，以及采样汇总与完整性计数。RMS 使用已使能且该轴 `q/dq` 均存在、反馈年龄不超过 200 ms 的控制样本；编码差异只比较 `seq!=0` 且该轴 `mode:1` 的数据，发送序号统计排除 `seq:0`，时间统计保留所有模式。无有效样本时对应统计为 null。间隔是主机来源时间的统计；这些数值用于比较采样，单项异常不能直接确定抖动原因。

CLI 原有输出字段保留，新增 `tuning`：`servo_batches_with_tuning/servo_batches_without_tuning` 统计有/无调试元数据的有效伺服批次，`clamped_samples_by_joint` 统计逐轴限幅样本数，`by_revision` 以版本号为键，包含 `config`（缺少匹配控制样本时为 null）、`control_samples`、`transmit_samples_by_joint` 和 `clamped_samples_by_joint`。发送及限幅统计沿用 `seq!=0` 且逐轴 `mode:1` 的筛选；时间统计继续保留所有模式。新旧文件可混合，缺少元数据不推定为基线，非有限数值或溢出的派生误差拒绝分析。

发送一致性回放另外比较相邻批次的 `Δq_wire/Δt` 与 `dq_wire`，只度量指令位置与速度的一致性，不代表真机减抖或物理 jerk。私有 capture 的 Rust 回放结果见[测试记录](implementation-status.md)；公开的[匿名相对时序 fixture](../src-tauri/tests/fixtures/send-clock.json)仅保留控制/发送相对时间与源索引，不发布私有 capture。

真机采集每次记录一种场景，保留速度、轴、方向、姿态、TCP 和负载说明，便于重复对照：

| 场景           | 采集与确认                                                                                       |
| -------------- | ------------------------------------------------------------------------------------------------ |
| 已连接未使能   | 开始采样，等待窗口结束并下载；确认连接状态与控制权保持原值。                                     |
| 静止保持       | 使用原有使能流程，采集当前保持状态；比较参考、编码值与反馈的变化。                               |
| 关节连续与步长 | 在同一轴和姿态分别采集低速、常用速度及两个方向；每次记录起步、运动和停止，等待 `stopping` 结束。 |
| 末端平移与旋转 | 在当前有效 TCP 下分别检查基座/工具坐标与六个轴；对照所选路径、实际跟随和停止过程。               |
| 收纳返回       | 使用已有机械支撑处记录的收纳位；检查返回到位和稳定确认，取消后保持连接，不继续计划中的断开。     |

每次先在控制页设置 10 秒并点击“录制”，再手动执行场景，保留一段停止后的反馈。动作由工作台停止按钮取消；“停止录制”只结束记录。等待状态离开 `writing`，检查 `written=accepted`、`pending=0` 和丢弃/错误计数后下载；有数据损失时保留该记录，比较时标明缺口。对每个文件运行分析命令，再结合实际观察和已有事件日志判断下一次要测的条件。新轨迹、低速抖动改善、负载下跟随与实际 jerk 的验收结果分别记录。

伺服参数按以下顺序做对照，保持相同初始姿态、TCP、轴、速度与负载，每次只修改一项并保存对应采样：

1. 摩擦补偿关闭、速度前馈权重为 1，从基线开始逐轴调整 `kd`，比较运动跟随和停止后的保持状态。
2. 固定 `kd`，保持摩擦关闭，逐轴改变 `velocity_ff`，比较相同低速正反向运动。`dq_ref` 用于轨迹跟随，`tuning.dq_feedforward` 用于核对速度编码。
3. 固定阻尼和速度前馈，开启摩擦补偿，从零系数开始逐轴调节 `coulomb`、`viscous`，再比较过零尺度与限幅。检查 `friction_requested/friction_applied/clamped`，并与该轴的关闭补偿记录对照。

参数弹窗中点击“基线”填入草稿后仍需点击“应用”。采样中的配置版本用于对应每组参数；复制 JSON 可保留完整配置，重新连接后恢复默认值。

## 10. 错误输入与输出

应用错误返回 JSON：

```json
{ "code": "owned", "message": "控制权已被占用" }
```

| HTTP 状态 | 典型 `code`                                                                                                                                                                             | 处理方式                                 |
| --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------- |
| 400       | `invalid_request`、`invalid_client`、`invalid_action`、`ports_failed`、`invalid_trace_request`、`invalid_trace_duration`、`invalid_trace_context`、`invalid_log_query`、`not_connected` | 修正输入或检查当前连接                   |
| 401       | `unauthorized`                                                                                                                                                                          | 使用本次应用启动后的令牌                 |
| 403       | `forbidden_origin`                                                                                                                                                                      | 检查浏览器来源                           |
| 404       | `trace_not_found`                                                                                                                                                                       | 采样 ID 无效、不存在或已被保留清理       |
| 409       | `owned`、`not_owner`、`stale_context`、`motion_busy`、`trace_active`                                                                                                                    | 重新读取控制或采样状态                   |
| 429       | `queue_full`、`too_many_connections`、`trace_capacity`                                                                                                                                  | 降低提交速率、关闭多余订阅或等待采样写入 |
| 503       | `closing`、`journal_unavailable`、`trace_unavailable`                                                                                                                                   | 应用退出或存储当前不可读写               |
| 504       | `timeout`                                                                                                                                                                               | 控制权请求等待超时；先读取状态           |

示教还可能返回 `400 invalid_teaching_request/teaching_side_mismatch/teaching_preparation_failed/preview_failed`、`404 teaching_not_found`、`503 teaching_unavailable`。预览 JSON 提取错误（含格式、类型、Content-Type 与体积错误）统一转为 `400 invalid_teaching_request` JSON；详情查询参数错误同样返回该 code。`teach_replay` 使用普通命令 JSON 提取行为，但准备失败可在 202 前返回上述 JSON 错误。

控制权和命令接口的 JSON 解析与类型错误由 HTTP 框架直接返回纯文本：格式错误 400、超大请求体 413、缺少或错误的 JSON Content-Type 415、字段缺失或类型/枚举错误 422。旧日志接口的查询参数格式错误返回 400 纯文本；`/v1/logs/page` 统一返回 `400` JSON `invalid_log_query`。采样开始的 JSON 提取错误和采样查询参数错误统一返回 `400` JSON `invalid_trace_request`；有效整数但时长超出范围返回 `invalid_trace_duration`。调用方先判断 HTTP 状态和 Content-Type，再解析错误内容。

`POST /v1/commands` 的业务校验在入队后完成。例如控制权不符，通常是 **202 入队后出现 `rejected/not_owner`**，而非该请求直接返回 409。

示教录制的参数、阶段或会话校验可异步返回 `invalid_teaching`、`no_teaching`，回放程序的控制线程复核可返回 `invalid_trajectory`；回放入队前的准备错误见上文。程序仍按字符串 code 分支处理。

| 异步结果或故障中的 `code`                                                                                                        | 含义与处理                                                        |
| -------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| `not_owner`、`stale_context`                                                                                                     | 控制权或上下文变化；读取状态，重新确认操作                        |
| `not_connected`、`fault`、`enable_busy`、`motion_busy`                                                                           | 当前运行阶段不接受该操作                                          |
| `invalid_motion`、`invalid_step`、`speed_limit`、`invalid_tcp`、`invalid_ttl`、`invalid_servo_tuning`                            | 修改输入                                                          |
| `feedback_missing`、`feedback_stale`、`invalid_feedback`                                                                         | 反馈不满足控制条件                                                |
| `tcp_missing`                                                                                                                    | 设置 TCP 后再使用末端点动                                         |
| `support_required`、`disable_first`                                                                                              | 确认支撑后释放保持；清错前先失能                                  |
| `invalid_pose`                                                                                                                   | 检查收纳位的七轴数值和相对行程                                    |
| `no_action`、`stale_input`、`input_expired`                                                                                      | 连续点动已经结束，或更新序号/有效期不符                           |
| `joint_dynamics`、`kinematics`                                                                                                   | 本次运动不能继续，结合状态和日志检查原因                          |
| `send_reference`                                                                                                                 | 参考无法安全取样；动作或使能请求 failed，保留原有使能状态与控制权 |
| `communication_error`、`feedback_timeout`、`device_error`、`device_timeout`、`device_disabled`、`enable_timeout`、`clear_failed` | 通信、设备或确认失败                                              |
| `stopped`、`host_stop`、`released`、`lease_expired`、`disabled`、`disconnected`、`clearing`、`closing`                           | 动作取消原因                                                      |

程序逻辑依据 `code`，`message` 用于显示。上表列出常用处理分组；接口中的 `code` 为字符串，不限制为这张表中的枚举。

## 11. 桌面专用调用

Tauri 仅向主窗口提供三个命令：

| 命令           | 输入                    | 输出          | 用途                                                      |
| -------------- | ----------------------- | ------------- | --------------------------------------------------------- |
| `session_info` | 无                      | `SessionInfo` | 取得本次会话地址、令牌、版本与日志位置                    |
| `host_stop`    | `reason:string`         | `null`        | 请求取消任意客户端的当前动作，原因写入日志                |
| `close_app`    | `allowRelease?:boolean` | `null`        | 请求退出；保持中且未允许释放时发送 `close-requested` 事件 |

`SessionInfo` 示例：

```json
{
  "base_url": "http://127.0.0.1:8177",
  "token": "example-application-token",
  "client_id": "desktop",
  "version": "0.1.0",
  "journal_path": "/path/to/events.sqlite3"
}
```

桌面的停止权限不取得控制权、不自动失能。HTTP `stop` 则需要当前控制权及 `generation`；把 HTTP 的 `client_id` 写成 `desktop` 不会取得桌面停止权限。

普通窗口关闭与菜单退出也在控制线程检查是否仍在保持。存在已确认或待确认使能时，先向主窗口发送 `close-requested`，保留连接并显示收纳/释放操作。用户明确确认支撑后，可调用 `close_app({allowRelease:true})`；收纳后退出则等待本次 `return_to_pose` 的 `completed/parked`，再调用 `close_app`。退出检查通过后拒绝已排队的使能和运动请求，避免关闭期间重新开始保持。

## 12. 模块对应关系

控制状态集中在 `arm.rs`；`protocol.rs` 处理通信与标定；`kinematics.rs` 处理当前模型与局部运动学，`trajectory.rs` 提供共享导数窗口和有界求解，`tuning.rs` 提供完整伺服配置校验与无状态前馈计算，`friction.rs` 负责基于真实样本的双向模型拟合，`teaching.rs` 负责示教录制、刚度过渡与后台回放规划；`server.rs` 提供本机接口；`lib.rs` 管理 Tauri 生命周期；`journal.rs` 处理事件与示教持久化并提供采样入口，`trace.rs` 管理短时采样。前端位于 `src`，模型与标定位于 `resources`。

### protocol.rs

导出 `PortInfo {name,label,kind}`、`enumerate_ports() -> Vec<PortInfo>`。

导出 `DriveMode::{Disabled,Servo,ClearFault}`、`Setpoint {mode,q,dq,tau,kp,kd}`，数值均为标定后的 URDF 关节量。`Feedback {id,q,dq,tau,temperature,winding_temperature,voltage,error,warning,mode,timeout}`。`kp` 按当前保持或跟踪状态生成，`kd` 按当前配置逐轴设置。`tau` 为按模型上限约束的重力与可选摩擦前馈之和。`dq` 为加权后的速度前馈。

`BusHandle::open_recorded(port,side,baud,timeout,journal,reference_epoch)` 创建串口会话；`publish(values,seq,reference)` 发布最新七轴参考及其来源时间、导数和动作约束。`drain()->(Vec<(Feedback,Instant)>,Option<BusError>)` 读取带时间戳反馈，以及 I/O 错误或带 generation/epoch 的参考拒绝。`disable()` 和 `write_blocking()` 提交优先停机/清错写入；`stats()->BusStats` 读取通信与批次时间统计。原始 `send` / `send_with_sequence` 只用于协议测试。

发送线程沿原始 10 ms 网格取样，跳过错过槽位；每包保留 200 μs 间隔，控制线程不等待七帧。仅成功写完整批后更新历史目标并记录 transmit，接收线程独立记录 feedback。测试保留单周期 `exchange_all` 作为协议对照。标定从 resources/calibration 的官方文件编译内嵌。

`latest_reference()->Option<SentReference>` 返回最近成功整批伺服目标的单值快照，包含批次起点、generation/epoch、位置和速度；用于静止后的动作交接。停机、清错或发送失败时清空，不保存额外历史。

### reference.rs

`Reference::sample(at,previous)` 是无 I/O 的取样函数。`Reference` 保存来源时刻、七轴位置/速度/加速度、generation、epoch、有效截止时间和当前目标/路径，以及不可变的 `servo_tuning/tuning_revision`；`Previous {at,q,dq}` 只来自成功整批发送。取样先计算当前速度，再连续积分位置，检查新鲜度、数值、源偏离、实际发送增量、方向、目标与 TCP 路径。`Sample {q,dq,timing}` 返回理想发送量；拒绝原因用 `SampleStatus` 表示，保持与动作状态转换由 protocol.rs / arm.rs 处理。该模块不写日志或数据库。

### tuning.rs

`ServoTuning {friction_enabled,joints:[JointTuning;7]}` 和 `JointTuning {kd,velocity_ff,coulomb,viscous,velocity_scale,max_friction,stribeck}` 具备序列化；六个原字段仍必填；新增 stribeck 缺失时默认关闭。`validate()` 检查有限数与范围，`apply_with_effort_limits(dq, gravity, effort_limits)` 返回七轴 `dq_feedforward/friction_requested/friction_applied/tau/clamped` 和单独施加重力时的限幅值，无 I/O 或内部辨识状态。基线、公式和输入限制见 5.9。协议层在 Reference 取样和路径检查后应用前馈；用于后续位置积分和静止交接的成功发送历史保存未缩放轨迹速度。

### kinematics.rs

导出 `Pose {translation:[f64;3],rotation:[f64;4]}`、`Frame::{Base,Tool}`（serde 小写）、`TcpTransform`（可与 Pose 为同一类型）及 identity 方法。

`Kinematics::load(side:&str)->Result<Self,String>` 从 resources/models 内嵌当前模型；`joint_names()->[String;7]`；`position_limits()->([f64;7],[f64;7])`；`fk(q:&[f64;7],tcp:&TcpTransform)->Result<Pose,String>`；`jacobian(q,tcp)` 返回 6×7 nalgebra 矩阵，便于数值检查。

`MotionLimits {joint_velocity:[f64;7],joint_acceleration:[f64;7],max_reference_error:f64,max_joint_step:f64,linear_speed:f64,angular_speed:f64}`。

`LocalTarget::Velocity {twist:[f64;6],frame:Frame}` 或 `LocalTarget::Pose(Pose)`；`StepInput {reference:[f64;7],measured:[f64;7],previous_velocity:[f64;7],target:LocalTarget,dt:f64,tcp:TcpTransform,limits:MotionLimits,smoothing:Option<SmoothingState>}`；`SmoothingState {acceleration:[f64;7],jerk:[f64;7],path:Option<CartesianPath>,stopping:bool}`。`step(input)->Result<StepCandidate,String>`，候选包含 `q:[f64;7],dq:[f64;7],ddq:[f64;7],speed_scale:f64`。控制器传入前一周期导数和所选路径，把速度/加速度/jerk 窗口纳入有界 Jacobian 求解，再检查 FK、单步增量和相对实测的参考偏差；绝对行程由物理限位约束。拒绝 NaN、无效 dt 和不能推进的候选，不随机跳构型。

提供 `offset_pose(pose:&Pose,delta:[f64;6],frame:Frame)->Result<Pose,String>` 和 `pose_error(current:&Pose,target:&Pose)->[f64;6]`。动作和目标由 arm.rs 管理，运动学模块只计算每一步的候选输出。

资源：resources/models/{right,left}/robot.urdf 和 meshes/\*.STL；URDF mesh 引用统一为 meshes/文件名；Vite publicDir 指向 resources，前后端使用同一资源。manifest.json 记录 sides、joint_names、lower、upper、source，不改变原始轴与 origin。

### calibration.rs

集中定义默认逐轴程序、往返目标、固定窗口测量、候选评分、报告和绘图采样，不执行串口或数据库 I/O。`arm.rs` 保存一项校准任务，复用现有收纳轨迹与参数应用，在控制周期推进并处理停止、恢复和应用资格。

### trajectory.rs

提供 `MotionTuning`、`velocity_window`、`integrate` 和七变量盒约束的 `bounded_quadratic`。动态窗口由前一速度、加速度、目标、限制与实际 `dt` 计算；关节与末端求解使用相同导数约束。默认值为 1 rad/s、0.60 rad/s²、6 rad/s³；模型各轴额定速度单独读取，数值输入显式传入，可独立测试。

### teaching.rs

定义 `Point`、`Summary`、摘要展开的 `Trajectory`、低成本共享的 `Capture`、`PlaybackOptions`、`Preview` 与 `PreviewPoint`。`Recorder::push(Point)` 按小块常数时间追加，`finish(outcome)` 生成共享录制；`Capture::trajectory()` 的全量展开在工作线程调用。`Guidance` 按当前实测位置和相对秒数完成拖动/保持的连续过渡。`plan(trajectory,origin,model,motion,options)` 在后台生成可共享 `Program`，其 `sample(t)` 返回七轴 `q/dq/ddq`，`preview(model)` 计算接近与完整路径的真实 TCP/关节序列；`rebase_approach(origin)` 在接受回放时更新起点。该模块只做纯 Rust 数据与数值计算，不访问串口或 SQLite。

### journal.rs

使用 rusqlite 0.41.1（bundled），只在专用 writer 线程使用写连接。`Journal` 可 Clone，`Journal::start(path:PathBuf)->Result<Self,String>`、`log(&self,level:&str,source:&str,code:&str,message:&str,request_id:Option<&str>)` 构造字段长度有界的字符串记录，再以 `Mutex.try_lock` 单次尝试提交到容量 1024 的预分配队列；竞争、锁损坏或满队列立即丢弃，不等待锁、队列或数据库。`stats()->JournalStats {pending,written,dropped,errors,healthy}` 仅原子读取。

`recent(limit:u32)->Result<Vec<LogRecord>,String>` 是阻塞读取接口，调用方必须放到 spawn_blocking。`LogRecord {id:i64,ts_ms:i64,level,source,code,message,request_id:Option<String>}`，带 serde。`shutdown(timeout:Duration)->bool` 仅在应用关闭协调线程调用，有界等待；Drop 不阻塞。

事件库使用 WAL、批量事务、有限 busy_timeout；记录连接、控制权、动作、故障和应用事件，不逐帧存储串口或遥测。不记录 token/lease。队列满计 dropped，写失败计 errors，后台重试不反向阻塞控制。保留最近 100000 条，清理与 checkpoint 都在 writer 内。测试覆盖持久化、队列满和数据库不可写时的行为。

`trace_start(duration_ms,side,port)`、`trace_stop()`、`trace_list(limit)`、`trace_read(id)` 提供采样管理与读取，server 放到 `spawn_blocking`。`sample(at:Instant,data:TraceData)` 提交固定大小数据；采样有独立队列、writer、数据库与按次统计。

报告使用独立容量 8 的有界队列，`record_calibration(&Report)` 单次尝试锁，仅接纳后克隆；JSON 与事务在 writer 内。控制器保留一份未接纳报告，下周期补投。关闭时先结束控制，再由关闭线程有界补投，随后刷新事件与报告。读取与历史清理不在控制线程执行。

`record_teaching(&Capture)->bool` 单次尝试锁并共享录制，不在控制线程展开或编码；失败由控制器保留补投。writer 同事务保存示教摘要与全部点，不对用户轨迹自动清理。`teachings()->Result<Vec<Summary>,String>` 与 `teaching(id)->Result<Option<Trajectory>,String>` 为阻塞只读接口，server 放到工作线程；尚未写入的最新完整录制通过快照内部共享对象提供读取，该对象不进入公开 JSON。

### trace.rs

定义 `TraceSummary`、带 `source` 标签的 `TraceData`、展开数据的 `TraceSample {at_us,data}` 和 `TraceRecording {trace,samples}`。`Recorder` 管理有界采样窗口和预分配队列；生产者只更新计数并尝试取得队列锁，失败立即丢弃。writer 取出数据后再执行序列化和持久化，不在生产者持锁期间执行 SQL。读取保留来源时间、重复发送序号和数据损失状态，字段见第 9 节及 OpenAPI。

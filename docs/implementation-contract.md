# API 接入说明

本文对应当前桌面软件的实现，说明请求、返回值、状态订阅和动作确认方式。更新日期：2026-11-10。机器可读定义见 [OpenAPI 3.1](openapi.json)。

## 1. 连接与认证

| 项目           | 当前行为                                            |
| -------------- | --------------------------------------------------- |
| HTTP 地址      | `http://127.0.0.1:8177`                             |
| WebSocket 地址 | `ws://127.0.0.1:8177/v1/events`                     |
| 服务生命周期   | 随桌面应用启动和退出                                |
| 访问范围       | 当前仅监听本机回环地址                              |
| HTTP 认证      | `Authorization: Bearer <token>`                     |
| POST 格式      | `Content-Type: application/json`，请求体最多 16 KiB |
| 状态与数值     | JSON；长度用米，角度用弧度，时间间隔字段用毫秒      |

应用每次启动生成新的 `token`。外部程序从桌面顶部“接口”页复制令牌；当前没有通过 HTTP 获取令牌的接口。令牌用于访问应用，`lease_id` 用于持有控制权，两者用途不同。

浏览器 Origin 当前只接受 `tauri://localhost`、`http://tauri.localhost`、`https://tauri.localhost`、`http://127.0.0.1:1420`、`http://localhost:1420`。原生程序可以不发送 Origin。

## 2. 接口总表

| 方法与路径              | 输入                         | 成功输出                           | 含义                     |
| ----------------------- | ---------------------------- | ---------------------------------- | ------------------------ |
| `GET /v1/ports`         | 无                           | `200 PortInfo[]`                   | 枚举本机串口             |
| `GET /v1/state`         | 无                           | `200 Snapshot`                     | 读取最近一次状态快照     |
| `POST /v1/control`      | `ControlRequest`             | `200 ControlReply`                 | 取得、续期或释放控制权   |
| `POST /v1/commands`     | `CommandRequest`             | `202 {request_id,status:"queued"}` | 提交设备或动作请求       |
| `GET /v1/events`        | WebSocket 升级后发送认证首帧 | `101`，随后推送 `StateEvent`       | 订阅完整状态快照         |
| `GET /v1/logs?limit=50` | 可选条数                     | `200 LogRecord[]`                  | 查询 SQLite 中最近的日志 |

普通 HTTP 返回值直接使用表中的结构，没有统一的 `data` 包装。只有 WebSocket 状态消息使用 `{type:"state",data:Snapshot}`。

## 3. 推荐调用顺序

1. 取得应用令牌，连接 WebSocket 并发送认证首帧。
2. 读取首个状态；只读程序从此持续接收状态即可。
3. 调用 `acquire` 取得控制权，保存返回的 `lease_id` 和 `generation`。
4. 枚举串口，发送 `connect`，等待连接确认。
5. 使用末端点动前，发送 `configure_tcp` 设置工具变换。
6. 发送 `enable`，等待使能确认，再发送 `step` 或 `jog_start`。
7. 操作期间独立续期控制权；连续点动还需发送 `jog_update`。
8. 结束动作时发送 `stop`；结束控制时按需失能、断开并释放控制权。

每个会改变状态的步骤都等待确认，再从最新状态读取 `generation`。调用方不自行递增该值。关节点动不要求设置 TCP。

## 4. 控制权输入与输出

同一时间只有一个客户端持有控制权；其他客户端可以读取状态和日志。控制权有效期为 **2500 ms**，建议每 **800 ms** 续期。

| 字段        | 类型           | 输入要求                                                          |
| ----------- | -------------- | ----------------------------------------------------------------- |
| `action`    | string         | `acquire`、`renew`、`release`                                     |
| `client_id` | string         | 非空，UTF-8 编码后不超过 64 字节；建议使用可辨认的 ASCII 名称     |
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

| 字段         | 类型          | 要求                                                       |
| ------------ | ------------- | ---------------------------------------------------------- |
| `request_id` | string        | 非空，UTF-8 不超过 100 字节；每个新请求使用新值，建议 UUID |
| `client_id`  | string        | 当前持有控制权的客户端标识                                 |
| `lease_id`   | string        | 当前控制权凭据                                             |
| `generation` | 非负整数，u64 | 最新状态或控制权响应中的值                                 |
| `kind`       | string        | 下表中的命令名                                             |

`generation` 会随控制权变化、连接、配置、动作结束或取消等状态变化而更新。收到 `stale_context` 后先读取状态，重新确认操作意图，不直接重发旧动作。

### 5.2 操作参数与确认条件

| `kind`          | 附加输入                                              | 确认方式                                                                |
| --------------- | ----------------------------------------------------- | ----------------------------------------------------------------------- |
| `connect`       | `port:string`，`side:"left"\|"right"`                 | 请求被接受后，等待 `phase:"disabled"`，七轴反馈齐全且新鲜               |
| `disconnect`    | 无                                                    | `phase:"disconnected"`、`port:null`                                     |
| `enable`        | 无                                                    | 该请求 `completed/enabled`，`enabled:true`、`phase:"ready"`             |
| `disable`       | 无                                                    | 请求被接受且 `enabled:false`；若已有故障，`phase` 仍可为 `fault`        |
| `clear_fault`   | 无                                                    | 该请求 `completed/cleared`，`error:null`、`enabled:false`               |
| `configure_tcp` | `tcp:Pose`                                            | 请求被接受，`tcp_configured:true`，`tcp_transform` 与归一化后的输入一致 |
| `jog_start`     | `space,axis,direction,speed,frame?,ttl_ms?,input_seq` | 请求被接受且 `action.request_id` 等于本次 `request_id`                  |
| `jog_update`    | `action_id,input_seq,ttl_ms?`                         | 成功更新不产生 `accepted` 记录；按当前动作与后续状态判断                |
| `step`          | `space,axis,direction,amount,speed,frame?`            | 该请求 `completed/completed`                                            |
| `stop`          | 无                                                    | 原动作收到 `cancelled`、`action:null`，等待 `stopping` 结束             |

`connect.port` 使用枚举返回的 `name`，非空且 UTF-8 不超过 300 字节。串口当前使用 6,000,000 波特率。更换串口或左右臂前先断开连接；切换左右臂会清除已设置的 TCP。

连接被接受表示串口已经打开，连接确认仍需反馈。使能进入 `enabling` 后，最多等待一秒，收齐七轴在使能请求之后的新鲜反馈，并确认设备模式、错误与超时状态。清错也需要反馈确认，完成后不会自动使能。

### 5.3 坐标、单位与范围

| 点动类型 | `space` / `axis`                | `speed` 单位与上限 | `amount` 单位与上限 |
| -------- | ------------------------------- | ------------------ | ------------------- |
| 关节     | `joint`，0..6 对应 J1..J7       | rad/s，≤ 0.20      | rad，≤ 0.35         |
| 末端平移 | `cartesian`，0..2 对应 X/Y/Z    | m/s，≤ 0.015       | m，≤ 0.05           |
| 末端旋转 | `cartesian`，3..5 对应 Rx/Ry/Rz | rad/s，≤ 0.15      | rad，≤ 0.35         |

`speed`、`amount` 必须为正数，方向由 `direction:-1|1` 指定。`frame` 默认 `base`：`base` 沿固定基座轴运动，`tool` 沿当前 TCP 轴运动。关节点动建议传 `base`，该字段不影响关节轴。

平移保持姿态；旋转围绕 TCP。一次请求只操作一个轴；当前没有绝对位姿移动、轨迹批量下发或原始电机控制接口。

`Pose` 的格式如下，`rotation` 是 **x、y、z、w 顺序的四元数**：

```json
{ "translation": [0.0, 0.0, 0.02], "rotation": [0.0, 0.0, 0.0, 1.0] }
```

`configure_tcp.tcp` 表示 **Link7 → TCP** 的工具变换，不是运动目标。每个平移分量范围为 [-2,2] m；四元数范数需至少为 1e-9，后端会归一化。取得控制权后可离线设置；运动、减速停止或待确认使能时不能设置。配置保存在本次应用内存中。

### 5.4 完整请求示例

下列标识与 `generation` 为示例值，实际调用使用本次会话返回的数据。全部十种命令的完整输入见 [OpenAPI](openapi.json) 的 `POST /v1/commands` 示例。

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

连续点动的 `request_id` 同时是 `action_id`。建议设置 `ttl_ms:350`，每 100 ms 发送一次 `jog_update`，并独立续期控制权。`ttl_ms` 范围为 50..500，省略时默认 150；有效期从后端首次收到请求开始计算，排队不会延长有效期。

每次更新使用新的 `request_id`，保持相同的 `action_id`、动作期间有效的 `generation`，并递增 `input_seq`。更新只延长当前动作，不改变轴、方向或速度。后端只保留最新一条待处理更新，中间更新可能被合并。

停止发送更新后，动作在输入过期时取消。主动松开按钮时同时发送 `stop`，缩短等待。过期或已结束的动作不能通过迟到的更新恢复。WebSocket 断开不会直接释放控制权；点动输入有效期与控制权有效期分别计时。

固定步长一次执行一个动作，不将后续点击排成运动队列。`progress` 按时间估计，达到 1 不表示已完成；以 `completed` 结果为准。完成前会同时检查实测值与参考值，并在减速结束后报告结果。

## 6. 命令输出与执行结果

提交成功立即返回：

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

| 字段                      | 类型                           | 含义                                               |
| ------------------------- | ------------------------------ | -------------------------------------------------- |
| `seq`                     | u64                            | 当前应用运行期间的快照序号                         |
| `generation`              | u64                            | 当前控制上下文编号，提交命令时原样带回             |
| `phase`                   | string                         | 运行阶段，见下表                                   |
| `side` / `port`           | string / string 或 null        | `left` 或 `right`；当前串口                        |
| `enabled`                 | boolean                        | 已确认的使能状态                                   |
| `owner`                   | object 或 null                 | `{client_id,expires_in_ms}`；不包含控制权凭据      |
| `joints`                  | 7 个 `JointSnapshot`           | J1..J7 的实测反馈                                  |
| `tcp`                     | `Pose` 或 null                 | 根据实测关节计算的基座 → 当前 TCP 位姿             |
| `tcp_transform`           | `Pose` 或 null                 | 已设置的 Link7 → TCP 变换                          |
| `tcp_configured`          | boolean                        | 是否设置了工具变换                                 |
| `feedback_age_ms`         | u64 或 null                    | 已有反馈中最大的接收年龄                           |
| `action`                  | object 或 null                 | 当前动作：`request_id,space,mode,axis,progress`    |
| `last_result` / `results` | `CommandResult` 或 null / 数组 | 最新一条结果 / 最近结果                            |
| `error`                   | `{code,message}` 或 null       | 当前故障                                           |
| `journal`                 | object                         | `pending,written,dropped,errors,healthy`           |
| `communication`           | object                         | `received,crc_errors,timeouts`，本次运行的通信计数 |

| `phase`        | 含义                     |
| -------------- | ------------------------ |
| `disconnected` | 未连接                   |
| `connecting`   | 串口已打开，等待反馈     |
| `disabled`     | 已连接，未使能           |
| `enabling`     | 已发送使能，等待反馈确认 |
| `ready`        | 已使能，可以接受动作     |
| `moving`       | 动作执行中               |
| `stopping`     | 减速停止中               |
| `fault`        | 存在需要清除的故障       |
| `closing`      | 应用正在退出             |

`JointSnapshot` 字段如下：

| 字段                      | 类型 / 单位            | 含义                     |
| ------------------------- | ---------------------- | ------------------------ |
| `id` / `name`             | 整数 / string          | 0..6 / J1..J7            |
| `q`                       | number 或 null，rad    | 位置                     |
| `dq`                      | number 或 null，rad/s  | 速度                     |
| `tau`                     | number 或 null，Nm     | 估算扭矩                 |
| `age_ms`                  | u64 或 null，ms        | 这条反馈距快照生成的时间 |
| `temperature` / `voltage` | number 或 null，°C / V | 温度 / 电压              |
| `error`                   | u32 或 null            | 设备错误值               |

从未收到的反馈为 `null`。反馈过期后，关节数值保留最后一次测量，`age_ms` 继续增长。控制使用的七轴反馈需全部存在、单轴年龄不超过 200 ms，七轴接收时间跨度不超过 100 ms。仅检查 `feedback_age_ms` 不能证明七轴已经齐全。

TCP 未设置或控制反馈无效时，`tcp:null`。`action.mode` 为 `continuous` 或 `step`，`progress` 范围 0..1。`seq` 和 `generation` 不跨应用重启保存；重新连接应用后重新读取状态。快照通常约 30 Hz 更新，状态变化时也可提前发布。HTTP 返回缓存中的最近快照，调用方还需考虑接收后经过的时间。

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

`GET /v1/ports` 返回数组，按 `name` 排序并去重：

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

`journal.pending` 表示已接收但尚未写入的记录，`written` 表示本次运行成功提交的数量；`dropped` 统计队列满、关闭后提交或退出时未写完的记录；`errors` 统计数据库操作失败。计数均针对本次应用运行，不等同于数据库总行数。查询使用独立阻塞任务，不在控制线程执行。

## 10. 错误输入与输出

应用错误返回 JSON：

```json
{ "code": "owned", "message": "控制权已被占用" }
```

| HTTP 状态 | 典型 `code`                                                           | 处理方式                                     |
| --------- | --------------------------------------------------------------------- | -------------------------------------------- |
| 400       | `invalid_request`、`invalid_client`、`invalid_action`、`ports_failed` | 修正输入或检查串口枚举                       |
| 401       | `unauthorized`                                                        | 使用本次应用启动后的令牌                     |
| 403       | `forbidden_origin`                                                    | 检查浏览器来源                               |
| 409       | `owned`、`not_owner`                                                  | 当前用于同步控制权请求的冲突；重新读取控制者 |
| 429       | `queue_full`、`too_many_connections`                                  | 降低提交速率或关闭多余订阅                   |
| 503       | `closing`、`journal_unavailable`                                      | 应用退出或日志当前不可读                     |
| 504       | `timeout`                                                             | 控制权请求等待超时；先读取状态               |

JSON 解析与类型错误由 HTTP 框架直接返回纯文本：格式错误 400、超大请求体 413、缺少或错误的 JSON Content-Type 415、字段缺失或类型/枚举错误 422。日志查询参数格式错误同样返回 400 纯文本。调用方先判断 HTTP 状态和 Content-Type，再解析错误内容。

`POST /v1/commands` 的业务校验在入队后完成。例如控制权不符，通常是 **202 入队后出现 `rejected/not_owner`**，而非该请求直接返回 409。

| 异步结果或故障中的 `code`                                                                                                        | 含义与处理                                 |
| -------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| `not_owner`、`stale_context`                                                                                                     | 控制权或上下文变化；读取状态，重新确认操作 |
| `not_connected`、`fault`、`enable_busy`、`motion_busy`                                                                           | 当前运行阶段不接受该操作                   |
| `invalid_motion`、`invalid_step`、`speed_limit`、`invalid_tcp`、`invalid_ttl`                                                    | 修改输入                                   |
| `feedback_missing`、`feedback_stale`、`invalid_feedback`、`feedback_span`                                                        | 反馈不满足控制条件                         |
| `tcp_missing`                                                                                                                    | 设置 TCP 后再使用末端点动                  |
| `no_action`、`stale_input`、`input_expired`                                                                                      | 连续点动已经结束，或更新序号/有效期不符    |
| `joint_limit`、`joint_dynamics`、`kinematics`、`following_error`、`target_timeout`                                               | 本次运动不能继续，结合状态和日志检查原因   |
| `communication_error`、`feedback_timeout`、`device_error`、`device_timeout`、`device_disabled`、`enable_timeout`、`clear_failed` | 通信、设备或确认失败                       |
| `stopped`、`host_stop`、`released`、`lease_expired`、`disabled`、`disconnected`、`clearing`、`closing`                           | 动作取消原因                               |

程序逻辑依据 `code`，`message` 用于显示。上表列出常用处理分组；接口中的 `code` 为字符串，不限制为这张表中的枚举。

## 11. 桌面专用调用

Tauri 仅向主窗口提供两个命令：

| 命令           | 输入 | 输出          | 用途                                   |
| -------------- | ---- | ------------- | -------------------------------------- |
| `session_info` | 无   | `SessionInfo` | 取得本次会话地址、令牌、版本与日志位置 |
| `host_stop`    | 无   | `null`        | 请求取消任意客户端的当前动作，立即返回 |

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

## 12. 模块对应关系

控制状态集中在 `arm.rs`；`protocol.rs` 处理通信与标定；`kinematics.rs` 处理当前模型与局部运动学；`server.rs` 提供本机接口；`lib.rs` 管理 Tauri 生命周期；`journal.rs` 处理事件持久化。前端位于 `src`，模型与标定位于 `resources`。

### protocol.rs

导出 `PortInfo {name,label,kind}`、`enumerate_ports() -> Vec<PortInfo>`。

导出 `DriveMode::{Disabled,Servo,ClearFault}`、`Setpoint {mode,q,dq,tau,kp,kd}`，数值均为标定后的 URDF 关节量。`Feedback {id,q,dq,tau,temperature,voltage,error,warning,mode,timeout}`。

`Bus::open(port:&str,side:&str,baud:u32,timeout:Duration)->Result<Bus,String>`；`exchange(&mut self,id:u8,setpoint:Setpoint)->Result<Option<Feedback>,String>` 为单关节有界事务；`stats()->BusStats {received,crc_errors,timeouts}`。控制线程在每个事务之间检查取消信号，控制状态和动作取消由 arm.rs 管理。标定从 resources/calibration 的官方文件编译内嵌。

### kinematics.rs

导出 `Pose {translation:[f64;3],rotation:[f64;4]}`、`Frame::{Base,Tool}`（serde 小写）、`TcpTransform`（可与 Pose 为同一类型）及 identity 方法。

`Kinematics::load(side:&str)->Result<Self,String>` 从 resources/models 内嵌当前模型；`joint_names()->[String;7]`；`position_limits()->([f64;7],[f64;7])`；`fk(q:&[f64;7],tcp:&TcpTransform)->Result<Pose,String>`；`jacobian(q,tcp)` 返回 6×7 nalgebra 矩阵，便于数值检查。

`MotionLimits {joint_velocity:[f64;7],joint_acceleration:[f64;7],max_reference_error:f64,max_joint_step:f64,linear_speed:f64,angular_speed:f64}`。

`LocalTarget::Velocity {twist:[f64;6],frame:Frame}` 或 `LocalTarget::Pose(Pose)`；`StepInput {reference:[f64;7],measured:[f64;7],previous_velocity:[f64;7],target:LocalTarget,dt:f64,tcp:TcpTransform,limits:MotionLimits}`；`step(input)->Result<StepCandidate,String>`，候选包含 `q:[f64;7],dq:[f64;7],speed_scale:f64`。统一局部加权 DLS，限制关节位置/速度/加速度/参考偏差，拒绝 NaN、无效 dt 和不能推进的候选，不随机跳构型。

提供 `offset_pose(pose:&Pose,delta:[f64;6],frame:Frame)->Result<Pose,String>` 和 `pose_error(current:&Pose,target:&Pose)->[f64;6]`。动作和目标由 arm.rs 管理，运动学模块只计算每一步的候选输出。

资源：resources/models/{right,left}/robot.urdf 和 meshes/\*.STL；URDF mesh 引用统一为 meshes/文件名；Vite publicDir 指向 resources，前后端使用同一资源。manifest.json 记录 sides、joint_names、lower、upper、source，不改变原始轴与 origin。

### journal.rs

使用 rusqlite 0.41.1（bundled），只在专用 writer 线程使用写连接。`Journal` 可 Clone，`Journal::start(path:PathBuf)->Result<Self,String>`、`log(&self,level:&str,source:&str,code:&str,message:&str,request_id:Option<&str>)` 使用有界 try_send，不能等待数据库或队列。`stats()->JournalStats {pending,written,dropped,errors,healthy}` 仅原子读取。

`recent(limit:u32)->Result<Vec<LogRecord>,String>` 是阻塞读取接口，调用方必须放到 spawn_blocking。`LogRecord {id:i64,ts_ms:i64,level,source,code,message,request_id:Option<String>}`，带 serde。`shutdown(timeout:Duration)->bool` 仅在应用关闭协调线程调用，有界等待；Drop 不阻塞。

WAL、批量事务、有限 busy_timeout；记录连接、控制权、动作、故障和应用事件，不逐帧存储串口或遥测。不记录 token/lease。队列满计 dropped，写失败计 errors，后台重试不反向阻塞控制。保留最近 100000 条，清理与 checkpoint 都在 writer 内。测试覆盖持久化、队列满和数据库不可写时的行为。

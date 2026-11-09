# MVP 实施接口（2026-11-09）

本接口记录 macOS 软件 MVP 的模块边界与数据约定。SQLite 保存结构化事件日志；生产 UI 只显示当前状态、数据来源、操作与必要错误原因。界面在 1280×720 和 1024×768 中同时展示连接、七轴、3D、点动与日志，不出现页面级滚动。真机验收状态写在文档，缺失反馈不填充为伪数据。

控制状态集中在 arm.rs；protocol.rs 处理通信与标定；kinematics.rs 处理当前模型与局部运动学；server.rs 提供本机接口；lib.rs 协调 Tauri 生命周期；journal.rs 处理事件持久化。前端位于 src，模型与标定位于 resources。

## 一般接口

Tauri `session_info` 返回 `{base_url,token,client_id:"desktop",version,journal_path}`；`host_stop` 无参数，取消任意动作。其它设备操作走 HTTP，Authorization 为 Bearer 应用 token。WS 首帧 `{token}` 认证，随后发送 `{type:"state",data:Snapshot}`。本机默认端口 8177。

- GET /v1/ports → `[{name,label,kind}]`
- GET /v1/state → Snapshot
- POST /v1/control，body `{action:"acquire"|"renew"|"release",client_id,lease_id?}` → `{lease_id?,expires_in_ms,generation}`
- POST /v1/commands → `{request_id,status:"queued"}`；结果通过 snapshot.last_result 和 results 获取。
- GET /v1/logs?limit=50 → LogRecord[]，按 id 降序；查询不运行在控制线程上。

命令统一 `{request_id,client_id,lease_id,generation,kind,...}`；kind 包括 connect(port,side)、disconnect、enable、disable、clear_fault、configure_tcp(tcp)、jog_start(space,axis,direction,speed,frame,ttl_ms,input_seq)、jog_update(action_id,input_seq,ttl_ms)、step(space,axis,direction,amount,speed,frame)、stop。space 为 joint/cartesian；axis 为 0..6 / 0..5；direction 为 ±1；frame 为 base/tool。数值用米、弧度、秒，UI 显示毫米和度。

`jog_start` 的 request_id 同时是 action_id。update 只续期现有动作，不创建动作。有效期从服务端首次接收计算。step 不排队、不自动重试；同一 generation 的 request_id 至多执行一次。宿主停止不需要 lease，不改变所有者和 enabled。

Snapshot 字段：

```
seq: number; generation: number;
phase: "disconnected"|"connecting"|"disabled"|"enabling"|"ready"|"moving"|"stopping"|"fault"|"closing";
side: "right"|"left"; port: string|null; enabled: boolean;
owner: {client_id:string,expires_in_ms:number}|null;
joints: {id:number,name:string,q:number|null,dq:number|null,tau:number|null,age_ms:number|null,temperature:number|null,voltage:number|null,error:number|null}[];
tcp: {translation:[number,number,number],rotation:[number,number,number,number]}|null;
tcp_transform: {translation:[number,number,number],rotation:[number,number,number,number]}|null;
tcp_configured: boolean;
feedback_age_ms: number|null;
action: {request_id:string,space:"joint"|"cartesian",mode:"continuous"|"step",axis:number,progress:number}|null;
last_result: {request_id:string,status:string,code:string,message:string}|null;
results: CommandResult[]; error: {code:string,message:string}|null;
journal: {pending:number,written:number,dropped:number,errors:number,healthy:boolean};
communication: {received:number,crc_errors:number,timeouts:number};
```

rotation 四元数按 x,y,z,w。tcp 未配置时为 null；UI 使用 Link7 参考帧的名称。无设备时 q/dq/tau 都是 null；3D 只显示明确的参考姿态。请求 body 不含设备验收或免责声明字段。

enable 接受后进入 enabling，enabled 保持 false；仅在一秒内收齐七轴请求之后的新鲜 mode=1、error=0、timeout=false 反馈后确认 ready。停止、释放控制权、退出和故障均取消待确认使能。运行中设备报告 timeout 或退出 mode=1 时锁定故障并取消动作；后续正常帧不会恢复旧动作，须明确清错再使能。清错也需新鲜、零故障且无超时反馈确认。

最终发送的关节 dq 按实际 dt 约束速度、加速度、位置、参考偏差与离散刹车距离，然后一次积分 q。step 的实测和参考均达到容差后继续受限减速，dq 归零才报告 completed，不直接把非零候选速度清零。

## protocol.rs

导出 `PortInfo {name,label,kind}`、`enumerate_ports() -> Vec<PortInfo>`。

导出 `DriveMode::{Disabled,Servo,ClearFault}`、`Setpoint {mode,q,dq,tau,kp,kd}`，数值均为标定后的 URDF 关节量。`Feedback {id,q,dq,tau,temperature,voltage,error,warning,mode,timeout}`。

`Bus::open(port:&str,side:&str,baud:u32,timeout:Duration)->Result<Bus,String>`；`exchange(&mut self,id:u8,setpoint:Setpoint)->Result<Option<Feedback>,String>` 为单关节有界事务；`stats()->BusStats {received,crc_errors,timeouts}`。主控制线程在每个事务之间检查优先取消。不要在协议模块增加控制状态机或重试整条动作。标定从 resources/calibration 的官方文件编译内嵌。

## kinematics.rs

导出 `Pose {translation:[f64;3],rotation:[f64;4]}`、`Frame::{Base,Tool}`（serde 小写）、`TcpTransform`（可与 Pose 为同一类型）及 identity 方法。

`Kinematics::load(side:&str)->Result<Self,String>` 从 resources/models 内嵌当前模型；`joint_names()->[String;7]`；`position_limits()->([f64;7],[f64;7])`；`fk(q:&[f64;7],tcp:&TcpTransform)->Result<Pose,String>`；`jacobian(q,tcp)` 返回 6×7 nalgebra 矩阵，便于数值检查。

`MotionLimits {joint_velocity:[f64;7],joint_acceleration:[f64;7],max_reference_error:f64,max_joint_step:f64,linear_speed:f64,angular_speed:f64}`。

`LocalTarget::Velocity {twist:[f64;6],frame:Frame}` 或 `LocalTarget::Pose(Pose)`；`StepInput {reference:[f64;7],measured:[f64;7],previous_velocity:[f64;7],target:LocalTarget,dt:f64,tcp:TcpTransform,limits:MotionLimits}`；`step(input)->Result<StepCandidate,String>`，候选包含 `q:[f64;7],dq:[f64;7],speed_scale:f64`。统一局部加权 DLS，限制关节位置/速度/加速度/参考偏差，拒绝 NaN、无效 dt 和不能推进的候选，不随机跳构型。

提供 `offset_pose(pose:&Pose,delta:[f64;6],frame:Frame)->Result<Pose,String>` 和 `pose_error(current:&Pose,target:&Pose)->[f64;6]`。主 Agent 拥有动作和目标，数值模块不拥有持续控制状态。

资源：resources/models/{right,left}/robot.urdf 和 meshes/*.STL；URDF mesh 引用统一为 meshes/文件名；Vite publicDir 指向 resources，前后端使用同一资源。manifest.json 记录 sides、joint_names、lower、upper、source，不改变原始轴与 origin。

## journal.rs

使用 rusqlite 0.41.1（bundled），只在专用 writer 线程使用写连接。`Journal` 可 Clone，`Journal::start(path:PathBuf)->Result<Self,String>`、`log(&self,level:&str,source:&str,code:&str,message:&str,request_id:Option<&str>)` 使用有界 try_send，不能等待数据库或队列。`stats()->JournalStats {pending,written,dropped,errors,healthy}` 仅原子读取。

`recent(limit:u32)->Result<Vec<LogRecord>,String>` 是阻塞读取接口，调用方必须放到 spawn_blocking。`LogRecord {id:i64,ts_ms:i64,level,source,code,message,request_id:Option<String>}`，带 serde。`shutdown(timeout:Duration)->bool` 仅在应用关闭协调线程调用，有界等待；Drop 不阻塞。

WAL、批量事务、有限 busy_timeout；记录连接、控制权、动作、故障和应用事件，不逐帧存储串口或遥测。不记录 token/lease。队列满计 dropped，写失败计 errors，后台重试不反向阻塞控制。保留最近 100000 条，清理与 checkpoint 都在 writer 内。需要持久化/队列满/数据库不可写仍可返回的有意义检查，不写镜像实现的测试。

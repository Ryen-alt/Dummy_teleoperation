# Linux Host ↔ STM32 Binary Protocol v5（文件名历史保留）

本文件名为兼容既有链接保留；本文描述当前固件和上位机共同使用的协议 v5。
所有多字节值均为小端。线帧为
`COBS(header || payload || CRC32C) || 0x00`，CRC32C Castagnoli 覆盖解码后的
header 与 payload，最大解码帧为 576 bytes。

24-byte packed header：

```c
uint16_t magic;          // 0x4459
uint8_t  version;        // 5
uint8_t  message_type;
uint16_t payload_length;
uint16_t flags;
uint32_t session_id;
uint32_t sequence;
uint64_t sender_time_us;
```

消息编号为 `HELLO=0x01`、`ACQUIRE_CONTROL=0x02`、
`RELEASE_CONTROL=0x03`、`SET_MODE=0x04`、`HEARTBEAT=0x05`、
`SET_JOINT_TARGET=0x06`、`HOLD=0x07`、`ESTOP=0x08`、
`CLEAR_FAULT=0x09`、`TARGET_KEEPALIVE=0x0a`、`TIME_SYNC=0x0b`、
`GET_CAN_DIAGNOSTICS=0x0c`、`GET_CAN_TIMING_PROFILE=0x0d`、
`SEED_JOINT_POSITION=0x0e`，响应为 `HELLO_ACK=0x81`、`STATE=0x82`、
`ACK=0x83`、`NACK=0x84`、`FAULT=0x85`、`EVENT=0x86`、
`TIME_SYNC_ACK=0x87`、`CAN_DIAGNOSTICS=0x88`、
`CAN_TIMING_PROFILE=0x89`。

`SET_JOINT_TARGET` 为 60 bytes：七个 float32 绝对位置（J1..J6 为 URDF
弧度，夹爪为归一化 `[0,1]`）、六个关节速度上限、uint16 TTL 和 uint16
flags。夹爪速度及加速度上限由固件配置独立控制。

`STATE` 为 508 bytes，并包含逐节点反馈时间、coherent sweep 和动作生命周期
证据。其安全诊断字段包括：

```c
float    following_error[7];
uint32_t following_error_duration_ms[7];
uint32_t feedback_age_ms[7];
uint32_t feedback_loss_count[7];
uint16_t consecutive_feedback_loss[7];
uint16_t node_fault_bits[7];
uint8_t  node_validity[7];
uint8_t  reserved;
uint16_t hold_reason_bits;
uint16_t telemetry_validity;
```

CAN 节点 1..7 分别统计反馈年龄、累计丢失和连续丢失；从未收到位置时年龄为
`0xffffffff`。跟随误差以 200 Hz 加减速限幅器输出的实际指令为基准，而不是
Linux 端尚未执行到的远端目标。持续跟随误差或短时反馈陈旧进入 HOLD，反馈丢失
超过严重阈值或实测温度持续超限进入锁存 FAULT。

`hold_reason_bits`：目标 TTL `0x0001`、租约 `0x0002`、跟随误差
`0x0004`、CAN 反馈陈旧 `0x0008`、操作员 HOLD `0x0010`、运行时限幅
`0x0020`。`fault_bits`：ESTOP `0x0001`、持续反馈丢失 `0x0002`、过温
`0x0004`；编码器/堵转/过流预留 `0x0008/0x0010/0x0020`。

`node_validity` 明确反馈来源是否存在：bit 0 位置、bit 1 温度，bit 2/3/4
分别预留编码器故障、堵转和电流来源。当前 CtrlStep CAN 响应只提供位置/完成与
温度，因此固件必须让后三种来源的有效位和故障位保持清零，不能由跟随误差伪造。

控制仍要求匹配的 32-byte 配置哈希、非零独占会话、有效租约、
TELEOP/POLICY 模式、递增序号、有限且在软限位内的目标和本地 TTL。目标采用
latest-wins。目标超时和租约超时由 MCU 本地进入 HOLD，ESTOP 与会话无关并锁存
FAULT。协议 v5 必须与 RobotConfig v9 配套部署。

二进制边界使用 URDF 关节坐标，固件历史角度仅保留在内部：

```text
q_urdf     = joint_sign * (q_firmware - joint_zero_offset_rad)
q_firmware = joint_zero_offset_rad + joint_sign * q_urdf
joint_zero_offset_rad = [0, -73°, 180°, 0, 0, 0]
joint_sign            = [+1,+1,+1,-1,+1,-1]
```

ASCII 维护指令仍使用固件历史角度，不得与二进制/URDF 目标混用。配置哈希用于阻止
旧日志或旧固件被静默解释成新坐标及安全参数。

## 上电后的绝对关节分支

六个关节电机使用单圈编码器。电机板只在本次通电期间累计多圈位置；重新上电会让
输出关节读数以 `2π / reduction` 为周期折叠（当前 50:1 减速比对应 7.2°）。因此：

- 主控每次启动后必须令 `STATE.position_valid=false`，且拒绝
  `ACQUIRE_CONTROL`；不能把折叠读数拿去做硬限位判断；
- `SEED_JOINT_POSITION` payload 为六个 URDF float32 和固定确认字
  `0x53454544`；只允许在哈希匹配的 HELLO、无租约、HOLD、无故障时执行；
- 固件检查参考姿态有限、位于硬限位内，并与当前单圈读数模
  `2π / reduction` 一致（容差 0.02 rad）；只恢复整数电机圈，不修改零偏；
- 校准请求仅写主控 RAM 中的分支偏移，不使能电机、不触发运动，主控重启后自动失效；
- 上位机命令 `dummy-host-seed-joints` 还要求明确确认机械臂断电期间没有移动，或参考
  姿态来自可靠机械定位夹具。没有外部参考时，软件无法从单圈编码器唯一恢复整数圈。

HELLO capability bit 8 表示上述 fail-closed 校准能力；当前主机拒绝缺少该能力位的
实机固件。

纯 C++ 回归测试：

```bash
cmake -S tests/host_protocol -B build-host-protocol
cmake --build build-host-protocol
ctest --test-dir build-host-protocol --output-on-failure
```

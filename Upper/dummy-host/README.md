# Dummy Linux 上位机

这是当前工作区的第一版安全闭环。范围是配置与固件代码生成、二进制协议、
USB CDC 串口、`DummyRobot`、动作安全过滤、20 Hz 调度、单关节 bring-up，
键盘/手柄遥操作采集、按逻辑角色管理的多相机同步、Raw Session v5 和离线数据集导出契约。
训练与推理运行时没有接入；LeRobot v3 依赖被隔离在相邻的 `lerobot-robot-dummy` 包。

## 安全状态

`configs/robot_config.yaml` 版本 7 以 URDF 关节角为上位机和二进制协议的统一坐标，
并在 MCU 协议边界映射到旧固件坐标。方向、零点、减速比、限位和夹爪映射已经由
`dummy_v2_001-arm-gripper-20260821-v2` 标定基线冻结。真实控制仍要求上位机与固件
配置哈希完全一致，并同时满足控制租约、dead-man、TTL 和状态有效性门禁。

`allow_unverified_hardware=True` 只保留给无电机协议台架和单元测试，生产配置不使用。

## 安装和测试

真机上电前应核对 `logs/calibration/` 中与当前 `robot_calibration_id` 对应的记录，
并执行单关节到全关节的分级低速检查。相机标定是独立门禁；当前
`calibration_version: uncalibrated-v0` 不能用于正式 Policy 运行。

```bash
cd Upper/dummy-host
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dev,d435,bringup,teleop]'
pytest
```

机器人安全配置与相机 rig 可以独立部署。`--camera-rig` 可覆盖内嵌相机段，同时保持
RobotConfig/固件哈希不变；双相机模板见 `configs/camera_rig_dual.example.yaml`：

```bash
dummy-host-camera --config configs/robot_config.yaml \
  --camera-rig configs/camera_rig_dual.example.yaml --seconds 30
```

正式带相机采集还要求每个必需角色引用版本化标定 YAML。标定文件绑定设备序列号、
型号、分辨率、内参、畸变、相对 `base_link`/`tool0` 的外参和 SHA-256；模板位于
`configs/calibrations/`。`--require-camera` 会拒绝 `uncalibrated-v0`、示例标定、
缺少固定曝光或标定身份不匹配的 rig。每次 Raw Session 会复制标定原文件并校验哈希。

由唯一 YAML 生成并检查固件配置头：

```bash
dummy-host-generate-firmware-config \
  --config configs/robot_config.yaml \
  --output ../../Firmware/dummy-ref-core-fw/UserApp/configurations/robot_config_generated.hpp
dummy-host-generate-firmware-config \
  --config configs/robot_config.yaml \
  --output ../../Firmware/dummy-ref-core-fw/UserApp/configurations/robot_config_generated.hpp \
  --check
```

单关节离线规划和假 MCU 闭环（不会访问真机）：

```bash
dummy-host-joint-bringup --config configs/robot_config.yaml \
  --joint 2 --delta-deg 0.2 --max-velocity 0.02 --duration 1 --dry-run
dummy-host-joint-bringup --config configs/robot_config.yaml \
  --joint 2 --delta-deg 0.2 --max-velocity 0.02 --duration 1 --simulate
```

## 键盘和手柄离线采集

先列出 Linux evdev 设备，并用 `evtest` 确认实际按键/轴代码：

```bash
dummy-host-list-inputs --json
ls -l /dev/input/by-id/*-event-joystick
export DUMMY_GAMEPAD="$(find /dev/input/by-id -maxdepth 1 -type l \
  -name '*event-joystick' -print -quit)"
test -n "$DUMMY_GAMEPAD" && test -e "$DUMMY_GAMEPAD"
sudo evtest "$DUMMY_GAMEPAD"
```

默认映射在 `configs/teleop_inputs.yaml`，并按本机 Flydigi Vader 5 Pro 实测配置为
物理 X=`BTN_NORTH`、Y=`BTN_WEST`。键盘必须持续按住 `KEY_SPACE`，手柄必须持续
按住 LB（逻辑控制 `lb`），否则上位机发送 HOLD 并释放租约。物理 evdev 协议与
机械臂操作映射相互独立，详见 `docs/Xbox手柄映射与虚拟测试.md`。

先运行完全虚拟的映射测试；该程序不会打开串口或创建机器人实例：

```bash
dummy-host-gamepad-test \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --simulate --demo
```

也可以连接真实手柄但仍只更新上位机内的虚拟关节：

```bash
dummy-host-gamepad-test \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --device auto --duration 120
```

下面使用假 MCU，
不连接机械臂，但会经过与真实串口相同的积分、安全过滤、序号和记录路径：

```bash
mkdir -p sessions/offline
dummy-host-teleop-collect \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --source keyboard --device /dev/input/eventX \
  --simulate --allow-joint 2 --duration 60 \
  --session-root sessions/offline

dummy-host-teleop-collect \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --source gamepad --device auto \
  --simulate --allow-joint 2 --duration 60 \
  --session-root sessions/offline
```

末端位姿逆解遥操作使用显式并行模式；Xbox/兼容手柄输出 `base_link` 下的末端速度，
上位机积分为标定 TCP 目标并通过 URDF IK 生成关节候选，随后仍走相同的安全网关和
`SET_JOINT_TARGET`。普通手柄摇杆、扳机和十字键即可，不要求陀螺仪：

```bash
dummy-host-teleop-collect \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --source gamepad --mode cartesian --device auto \
  --simulate --urdf ../../Dummy_URDF/dummy.urdf --duration 60 \
  --session-root sessions/offline
```

`--simulate` 不提供标定时使用 engineering-only 的 `tool0` identity。真实 Cartesian
入口必须额外提供 `--cartesian-calibration`，并显式放行 1～6 轴；校准文件必须通过
robot ID、URDF hash、TCP 和 Cartesian-ready pose 门禁。仓库示例
`configs/cartesian_calibration.example.yaml` 为 `validated: false`，因此不会解锁真机：

```bash
dummy-host-teleop-collect \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --source gamepad --mode cartesian --device auto \
  --execute --port /dev/ttyACM0 \
  --urdf ../../Dummy_URDF/dummy.urdf \
  --cartesian-calibration /path/to/site-validated-cartesian.yaml \
  --allow-joint 1 --allow-joint 2 --allow-joint 3 \
  --allow-joint 4 --allow-joint 5 --allow-joint 6 \
  --session-root sessions/real
```

连接后必须连续三个 coherent sweep 落入标定的 ready tolerance，才会进入首次控制权
获取。校准原件、hash、ready pose、TCP 变换和实际 base/tip frame 会归档到 Raw
Session。设计、轴映射、已知初始位形奇异风险和验收门禁见根目录
`06_末端位姿逆解遥操作与现有关节遥操作并行对齐设计.md`。

IK 延迟基准（保存 P50/P95/P99、硬超时阶段和失败尾部）：

```bash
dummy-host-cartesian-ik-benchmark \
  --config configs/robot_config.yaml \
  --input-config configs/teleop_inputs.yaml \
  --urdf ../../Dummy_URDF/dummy.urdf \
  --samples 300 --stress-fraction 0.2 \
  --output logs/cartesian_ik_benchmark_phase1.json
```

每次运行生成独立目录，包含：

- `manifest.json`：机器人/输入配置哈希、固件版本、来源、放行轴和 Cartesian 标定身份；
- `samples.sqlite`：WAL 模式下的原始输入、requested/applied action、机器人状态、
  session epoch/control tick、完整动作生命周期、2 Hz 仿射时钟模型和 1 Hz CAN 诊断；
- `events.jsonl`：dead-man、HOLD、ESTOP、Episode 和错误事件；
- `frames/<camera-role>/`：每个已启用逻辑相机角色的分段原始彩色/深度 NPZ；
- `checksums.json`：所有已完成文件的 SHA-256。

记录队列是有界的；写盘跟不上时程序报错并进入 HOLD，不会静默丢弃动作样本。
启用 `--with-cameras --require-camera` 后，必需相机缺帧或同步超限同样会停止采集并进入 HOLD。
完成后使用 `dummy-host-session-check --session /path/to/session_dir` 实际复算校验和、
运行 SQLite 完整性检查并汇总 received/ACK/CAN_QUEUED_EXACT/
CAN_TX_COMPLETE_EXACT/POST_COMMAND_FEEDBACK 序号。
进一步的自动 QA 会统计采样频率、调度间隔、Episode 结果、故障/裁剪样本、每个相机
角色的帧号缺口、时间戳来源、捕获延迟和同步偏差，以及时钟 RTT/残差、严格合格动作、
CAN timeout/error/fan-out，并生成不依赖 GUI 的 HTML 轨迹报告：

```bash
dummy-host-session-qa --session /path/to/session_dir \
  --json-output /tmp/session_qa.json \
  --html-output /tmp/session_qa.html
```

v2.2 的 10 分钟基础验收和 60 分钟 soak 使用同一个严格检查器；短测显式覆盖默认
3600 秒时长，其余阈值完全相同：

```bash
dummy-host-soak-check --session /path/to/session_dir \
  --minimum-duration-s 600 \
  --json-output /tmp/soak_10m.json

dummy-host-soak-check --session /path/to/session_dir \
  --json-output /tmp/soak_60m.json
```

检查器只接受 clean Raw Session v5，并同时核对 20 Hz 控制率、invalid/fault 为零、
coherent sweep、动作 ACK/TX-complete/post-feedback 闭合、TTL/BAD_MODE/action-credit/
串口可靠队列、CAN 诊断、七节点发送率和安全抢占延迟。CAN 固件提供累计最大 fan-out，
因此这里执行比“p99 < 10 ms”更严格的 `max < 10 ms` 门禁；命令返回非零即不得导出
该 session。

Raw Session v2～v5 也可通过 `ReplayCamera` 走相同的 Camera/CameraManager 接口。回放 rig
将 `driver` 设为 `replay`，`device_serial` 填 clean session 目录，并保持角色、分辨率和
`calibration_version` 与源记录一致；回放时间戳会重基到当前单调时钟，因此过期帧和
同步门禁仍然生效。
LeRobot 严格导出只接受 schema v5，并固定按 20 Hz 真实控制时间重采样；observation 使用
coherent reference 的仿射主机时间，camera 使用硬件曝光或显式 arrival 时间，action 只取
实际通过 SafetyFilter 且具备 ACK、TX-complete exact、post-feedback 证据的目标。schema v4
必须在 recipe 中显式设置 `legacy_mode: true`，导出侧车会标为 legacy 证据，不能与 v5 合并。
真实 `--execute` 还必须显式指定 `--allow-joint` 或 `--allow-gripper`，并继续受
硬件参数和固件执行门禁约束。详细验收步骤见真机指南第 17 节。

真实 `--execute` 需要匹配的已验证配置、固件执行门、串口和 evdev dead-man。

检查配置中的全部已启用相机（不会启动机器人）：

```bash
dummy-host-camera --config configs/robot_config.yaml --seconds 10 \
  --json-output logs/d435_smoke.json
```

完成配置标定并刷入匹配的固件后，查看串口诊断：

```bash
dummy-host-diagnose --config configs/robot_config.yaml --port /dev/ttyACM0
```

Linux 用户需确保当前账号有串口访问权限。D435 和机械臂 USB CDC 应避免共享带宽
不足的 Hub。相机线程与串口线程都使用有界缓冲，不会在控制线程内等待相机或磁盘。

## 已冻结的接口

协议定义见 `protocol_spec.md`。上线前必须用 Python 与纯 C++ 测试向量互验，并确认：

- 配置 SHA-256 完全一致；
- 关节顺序固定为 `joint1..joint6, gripper`；
- 关节角和目标在线上使用 URDF 坐标弧度，URDF 零位应上报六轴全零，夹爪使用 `[0, 1]`；
- 状态过期、相机过期、CRC/版本/哈希错误均显式失败；
- 退出路径发送 HOLD 并释放租约。

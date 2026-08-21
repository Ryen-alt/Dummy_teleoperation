# Dummy Linux 上位机

这是当前工作区的第一版安全闭环。范围是配置与固件代码生成、二进制协议、
USB CDC 串口、`DummyRobot`、动作安全过滤、20 Hz 调度、单关节 bring-up，
键盘/手柄遥操作采集、按逻辑角色管理的多相机同步、Raw Session v2 和离线数据集导出契约。
训练与推理运行时没有接入；LeRobot v3 依赖被隔离在相邻的 `lerobot-robot-dummy` 包。

## 安全状态

`configs/robot_config.yaml` 版本 3 以 URDF 关节角为上位机和二进制协议的统一坐标，
并在 MCU 协议边界映射到旧固件坐标。方向、零点、减速比、限位和夹爪映射已经由
`dummy_v2_001-arm-gripper-20260811-v1` 标定基线冻结。真实控制仍要求上位机与固件
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

每次运行生成独立目录，包含：

- `manifest.json`：机器人/输入配置哈希、固件版本、来源和放行轴；
- `samples.sqlite`：WAL 模式下的原始输入、requested/applied action、机器人状态和序号；
- `events.jsonl`：dead-man、HOLD、ESTOP、Episode 和错误事件；
- `frames/<camera-role>/`：每个已启用逻辑相机角色的分段原始彩色/深度 NPZ；
- `checksums.json`：所有已完成文件的 SHA-256。

记录队列是有界的；写盘跟不上时程序报错并进入 HOLD，不会静默丢弃动作样本。
启用 `--with-cameras --require-camera` 后，必需相机缺帧或同步超限同样会停止采集并进入 HOLD。
完成后使用 `dummy-host-session-check --session /path/to/session_dir` 实际复算校验和、
运行 SQLite 完整性检查并汇总 sent/received/applied 序号。
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

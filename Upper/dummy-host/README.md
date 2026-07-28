# Dummy Linux 上位机

这是 `upper_computer` 分支上的第一版安全闭环。当前范围是配置、二进制协议、
USB CDC 串口、`DummyRobot`、动作安全过滤、20 Hz 调度，以及单台 Intel
RealSense D435 腕部彩色/深度相机。训练与推理层没有接入。

## 安全状态

`configs/robot_config.yaml` 中的关节参数来自现有固件，只用于建立接口和假 MCU
测试，尚未经过真机逐关节标定，因此 `hardware_parameters_verified: false`。默认的
`DummyRobot` 会拒绝获取真实控制权；完成方向、零点、减速比、限位和夹爪标定后，
更新配置版本并将该字段改为 `true`。

不要用 `allow_unverified_hardware=True` 连接真机。这个开关仅供无电机的协议台架和
单元测试使用。

## 安装和测试

真机上电前请先完整执行 [真机标定、D435 测试与 200 Hz 联调指南](docs/真机标定与200Hz联调指南.md)。该指南明确了逐级放行条件，以及当前禁止直接使用的整机/夹爪自动标定命令。

```bash
cd Upper/dummy-host
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dev,d435]'
pytest
```

仅检查一台 D435（不会启动机器人）：

```bash
dummy-host-camera --config configs/robot_config.yaml --seconds 10
```

完成配置标定并刷入匹配的固件后，查看串口诊断：

```bash
dummy-host-diagnose --config configs/robot_config.yaml --port /dev/ttyACM0
```

Linux 用户需确保当前账号有串口访问权限。D435 和机械臂 USB CDC 应避免共享带宽
不足的 Hub。相机线程与串口线程都使用有界缓冲，不会在控制线程内等待相机或磁盘。

## 已冻结的接口

协议定义见 `protocol_spec.md`。上线前必须用两个分支中的测试向量互验，并确认：

- 配置 SHA-256 完全一致；
- 关节顺序固定为 `joint1..joint6, gripper`；
- 关节角和目标在线上使用弧度，夹爪使用 `[0, 1]`；
- 状态过期、相机过期、CRC/版本/哈希错误均显式失败；
- 退出路径发送 HOLD 并释放租约。


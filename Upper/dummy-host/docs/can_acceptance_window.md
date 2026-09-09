# CAN 软件修复与稳定验收窗口

更新：2026-09-09。适用于本次源码；须重新构建并刷写主控后再做真机验收。

## 本次修复

- USB 动作账本的 Failed / Preempted / Superseded 为终态，迟到的 queued、TX-complete、coherent feedback 不得恢复该动作。动作回调显式携带原始 session epoch；会话切换后即使序号复用，也不能修改新会话账本。
- 控制任务发布的目标邮箱携带 session epoch。CAN 调度器不接受旧会话邮箱中的同序号目标；失败 tracker 在取消前仍保留原始身份。
- 反馈发布进度增加独立的 published 标记，避免把 uint32 时钟回绕到 0 时的成功发布当作“从未发布”。消费者使用自己的时钟计算样本年龄和 dispatcher 活跃度。
- G0 组合测试直接编译实际 feedback runtime、USB 解析器/动作账本和 binary state bridge，连接真实安全监督器、ControlSession 和完成跟踪器。覆盖持续发布失败、dispatcher 暂停、时钟回绕、HOLD/FAULT、恢复不自动运动、切换会话及迟到/重复完成。HAL/RTOS、机器人硬件及旧 ASCII 通道使用主机桩；这不替代板上任务调度和机械停止测试。

## 窗口契约 v1

在已有的、经授权的遥操作采集命令中增加 `--acceptance-window --duration 660`（长测使用 3660）。`--duration` 仍是从 collection_started 起算的总采集预算，实际稳定窗口可能更短；必须由检查器确认至少 600/3600 秒。这个选项不赋予真机执行权限，既有 acceptance-session、关节许可和 dead-man 要求继续有效。

1. 采集前在 manifest 写入 `acceptance_window_contract=1`。连接、初始 HOLD、等待反馈和获取租约等样本完整保留。
2. 获取控制及相干反馈后，等待当前 epoch 的有效 CAN 诊断基线，再记录唯一的 `acceptance_window_started`。等待期间不生成目标，也不积累积分时间。
3. 样本统计范围为 `[start, stop)`。窗口开始后不能重新选取更有利的起点；HOLD、故障、无效样本、控制周期丢失、信用不足、动作失败继续使验收失败。
4. 达到总时长时停止新增动作，控制/心跳/dead-man 监督继续运行。最后动作必须在 stop 后 250 ms 内完成；最终诊断与 A9 快照必须在 stop 后 2 s 内收齐。随后记录 `acceptance_window_stopped` 与 `collection_stopped`，再执行常规安全退出。
5. 严格检查器按窗口样本及发送时间核对动作归属，允许所属动作在有界收尾内完成，但拒绝窗口外的新动作、缺失动作账本、错误 epoch 和重复/不匹配的 fanout 序号。
6. CAN 诊断基线与最终快照包围稳定窗口；检查所有中间快照的身份、有效位和计数回退。不能只对比首尾，从而漏掉中途重置。
7. A9 直方图不可相减，因此保守使用同一 MCU 窗口的累计直方图（包含该窗口内的启动数据）。验证其 epoch、起始 MCU 时间、reset count 连续性及最终 active 快照；不会用旧 active 快照覆盖无效的最终快照。A9 和诊断至少须覆盖要求的时长。

检查器只读原始数据库，通过连接内的临时视图选取统计范围；不删除样本、不重写 collection 时间、不改文件校验和。缺少完整窗口标记的 v1 session 直接失败，不回退到旧的整段统计。未声明该契约的历史 session 继续使用旧整段检查。

## 离线检查命令

以下命令不操作机械臂。SESSION 指向自然结束并完成 recorder.close 的完整 session；CONFIG 必须是该批冻结配置。

```bash
dummy-host-session-check --session "$SESSION"
dummy-host-can-a9-profile --events "$SESSION/events.jsonl"
dummy-host-can-r5-check --session "$SESSION" --config "$CONFIG" \
  --minimum-duration-s 600 --json-output r5.json
dummy-host-soak-check --session "$SESSION" --config "$CONFIG" \
  --minimum-duration-s 600 --json-output soak.json
```

G5 对 R5 和 soak 都改为 `--minimum-duration-s 3600`。soak 新增 `--config` 校验配置 hash 并读取真实发送频率，适用于固定 40/30/25 Hz 分支。R5 仍要求正常窗口零重试；不要只凭允许少量重试的通用 soak 结果放行。

`can-a9-profile --events` 会识别同目录的 v1 manifest，使用其明确窗口与最终 A9 快照。A9 通过只表示测量指标满足要求；完整 G4/G5 还须通过 R5、soak 和现场检查。`--allow-incomplete` 仍仅用于诊断，不能作为正式验收结果。

## 软件验证与剩余验收

本次结果：Python 全量 **252/252**；Release 主机 CTest **3/3**；主控 Release 与 Debug 均构建通过；配置头 `--check` 通过。Release FLASH 308968 B、Debug FLASH 312468 B，两者普通 RAM 均 70616 B（CCMRAM 为既有 64 KB 静态区）。本地镜像、SHA-256 和测试日志归档于仓库 `build/can_software_20260909/`，不纳入 Git。

全量测试曾暴露旧笛卡尔正常路径用例的时序波动：真实 IK 求解跨反馈轮次时，安全检查正确进入 HOLD。该网关/录制契约测试现使用已有的轻量测试运动学模型，并增加明确的过期结果注入用例；真实 URDF 求解器测试及生产新鲜度检查保持有效。最终全量复跑通过，首次失败日志一并保留。

本次验证命令：

```bash
Upper/dummy-host/.venv/bin/python -m pytest Upper/dummy-host/tests -q
cmake -S Firmware/dummy-ref-core-fw/tests/host_protocol -B /tmp/dummy-can-host -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/dummy-can-host -j 4
ctest --test-dir /tmp/dummy-can-host --output-on-failure
cmake -S Firmware/dummy-ref-core-fw -B /tmp/dummy-can-release -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/dummy-can-release -j 6
cmake -S Firmware/dummy-ref-core-fw -B /tmp/dummy-can-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/dummy-can-debug -j 6
Upper/dummy-host/.venv/bin/python -m dummy_host.config_codegen \
  --config Upper/dummy-host/configs/robot_config.yaml \
  --output Firmware/dummy-ref-core-fw/UserApp/configurations/robot_config_generated.hpp --check
```

主机 CTest 应包含协议、实际 CAN 驱动和实际 runtime/USB 组合三个目标。Python 覆盖完整原始日志保留、正常启动、结束动作收尾，以及缺失/重复/反转窗口、跨会话、计数回退、窗口内故障和错误 A9/fanout 证据的拒绝。Fake MCU 不生成真实 CAN 时序证据，不能以其窗口契约测试通过代替 CAN 速率或 A9 通过。

本次没有刷写真机，也没有关闭 G1 启动 100 次、G2 CAN 故障注入、G3 A9 参数冻结、G4 600 s、G5 3600 s 等硬件验收。`5000/4000 us` 仍待实测，正式执行开关仍为 false。节点采用确认、本地 TTL、版本化新 CAN 协议与同步执行属于后续独立开发/验收范围。

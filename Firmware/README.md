--原理图基于并且拥有者稚晖君，原理图修改版和PCB设计木子晓文。--

Based on ZhiHuiJun schematic, the oringal design owned by ZhiHuiJun. Schematic modified and PCB designed by Muzi Xiaowen. 
The schematic and PCB design are free and open, and comes with ABSOLUTELY NO WARRANTY, to the extent permitted by applicable law.

References
https://github.com/peng-zhihui/Dummy-Robot

## 线轨夹爪（dummy-ref-core-fw）

线轨夹爪使用 CtrlStep 兼容驱动器，CAN 节点 ID 为 7，减速比为 8，
默认行程角为 -115°（全开）到 115°（全闭）。六轴机械臂仍使用节点 1~6。

USB 和 UART4 ASCII 命令：

- `!HAND_EN`：使能夹爪。
- `!HAND_DIS`：电流清零并失能夹爪。
- `!HAND_POS <0-100>`：位置控制，0 为全开，100 为全闭。
- `!HAND_I <0-2.0>`：设置电流控制幅值，单位 A，默认 0.7 A。
- `!HAND_O`：以设定电流向打开方向运动。
- `!HAND_C`：以设定电流向闭合方向运动。
- `!HAND_ZERO`：自动执行全开/全闭行程标定；执行前必须清空夹爪周围区域。

通用电机配置命令 `#REBOOT`、`#OFFSET_J`、`#ACC_J`、`#SPEED_J`、
`#I_LIMIT_J` 均支持节点 7。二进制协议中夹爪仍位于 `hand` 对象下。

首次通电建议先设置较小电流并点动验证开合方向。若实物方向与上述定义相反，
应在确认电机接线和驱动器方向配置后再调整 `HAND_O/HAND_C` 的符号。

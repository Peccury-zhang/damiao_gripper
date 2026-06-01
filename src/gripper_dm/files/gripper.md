# OmniGripper（DM-J4310-2EC）ROS 2 控制方案

> 目标：基于 `u2canfdpy` SDK，编写 ROS 2 (Python) 功能包 `gripper_dm` + 自定义消息包 `gripper_dm_msgs`，
> 通过 **ROS 2 service** 控制夹爪 **打开 / 闭合（夹取）**；能稳定夹住较硬较重的物体。
>
> Python 环境：conda `grip`（Python 3.10.20 + pyusb 1.3.1）。

---

## 1. 硬件与通信参数核对

| 项 | 取值 | 来源 |
|---|---|---|
| 电机 | DM-J4310-2EC（型号枚举 `DM_Motor_Type.DM4310`） | 电机说明书 / SDK |
| 控制接口 | CAN FD（标准帧，1 Mbps 仲裁 + 5 Mbps 数据） | SDK README、电机说明书 |
| USB-CAN FD 设备 VID/PID | `0x34B7 / 0x6877` | `dev_sn.py` 输出 |
| USB-CAN FD 设备 SN | `14AA044B241402B10DDBDAFE448040BB` | 终端截图 |
| CAN ID（电机接收） | `0x01` | SDK 默认，与 `damiao.py` 一致 |
| Master ID（电机反馈） | `0x11` | SDK 默认 |
| 终端电阻 | 总线末端 120 Ω | 5 Mbps 数据率必需 |
| 开/闭位置 | open = `0.1 rad`，close = `1.1 rad` | 用户实测 |
| 最大速度 | `1.0 rad/s` | 用户要求 |

> **关于“串口 960000”**：电机自身有一个 921600 bps 的 UART 调参口（用于达妙调试助手 + GH1.25 线），
> 与本项目无关。我们通过 **USB-CANFD 模块** 用 `pyusb` 直接与电机走 CAN FD（仲裁 1M、数据 5M），
> 不会用到电机端 UART。如果你确实是指设备端的 UART 调参口波特率，需要先用达妙调试助手把它调到 960000，
> 但 ROS 包的运行不依赖它。下文按 CAN FD 路径实现。

---

## 2. SDK 关键梳理（来自 `USB_Gripper/u2canfdpy/damiao.py`）

- `Motor_Control(nom_baud, dat_baud, sn, init_data_list)`：构造时会
  1. 打开 USB-CANFD 设备；
  2. 把每颗电机切到 MIT 模式（`switchControlMode → RID=10`）；
  3. 读取并打印当前控制模式；
  4. 连发 5 次 `0xFC` 帧使能；
  退出时自动 `disable_all()` + 关闭 USB（`__exit__` / `close`）。
- 控制接口：
  - `control_mit(motor, kp, kd, q, dq, tau)` —— MIT 模式。
  - `control_pos_vel(motor, pos, vel)` —— 位置-速度模式（位置环 + 限速）。
  - `control_vel(motor, vel)` —— 速度模式。
- 反馈：`motor.Get_Position() / Get_Velocity() / Get_tau()`（USB 回调线程异步刷新）。
- 模式枚举：`Control_Mode.MIT_MODE=0x000`、`POS_VEL_MODE=0x100`、`VEL_MODE=0x200`，
  发送帧 ID = `can_id + mode`，因此切换 SDK 函数就够，不必另调参数。

---

## 3. 控制策略（如何夹住硬物）

要求：
- 打开：到 `0.1 rad`；
- 闭合：到 `1.1 rad`，若途中被硬物挡住，需要 **持续输出一定夹持力**，但不能损坏电机。

**选用 MIT 模式**（比 POS_VEL 更易做“受阻则保持夹紧力”的行为）：

`τ = Kp·(q_des − q) + Kd·(dq_des − dq) + τ_ff`

- 闭合时设 `q_des = 1.1`，若实际停在 `q ≈ 0.7`（被物体挡住），稳态力矩 ≈ `Kp · 0.4`。
- DM4310-2EC 额定 3 N·m / 峰值 7 N·m。我们取稳态夹持力矩 ~2 N·m（安全且“稍微用力”）。

推荐参数（写在配置里，可在 launch 时覆盖）：

| 参数 | 闭合 | 打开 | 说明 |
|---|---|---|---|
| `kp` | `5.0` | `5.0` | 与示例一致；硬物典型停留误差 0.3~0.4 rad ⇒ ~1.5–2.0 N·m |
| `kd` | `0.3` | `0.3` | 阻尼，避免抖动（注意：位控时 `kd` 不能为 0） |
| `dq_des` | `1.0` | `-1.0` | 限制目标速度方向（MIT 是前馈，不是硬限速） |
| `tau_ff` | `0.5` | `0.0` | 闭合时给一点前馈力矩，帮助“咬住” |
| `q_open` | — | `0.10` | rad |
| `q_close` | `1.10` | — | rad |
| 软速度限幅 | `1.0 rad/s` | `1.0 rad/s` | 由节点内部做“梯形/线性”插补，每个控制周期推进 `Δq = v_max · dt` |
| 保护力矩上限 | `3.0 N·m` | — | 反馈 `|τ|` 超过则停止递增 q_des |

> **为什么不用 POS_VEL 模式？**
> 它可以直接指定 `(p_des, v_des)`，速度上限自然就是 1 rad/s，写起来更简单；
> 但卡住时电流环会持续推到 `T_MAX`，**力矩没有显式上限**，对“硬物 + 稍微用力”不易精细控制。
> 因此默认 **MIT 模式 + 软插补**；同时在节点内提供 `use_pos_vel` 参数（True 时改用 POS_VEL），便于对照测试。

**到位判定**（service 返回 success）：
- 在限定时间窗口内（如 3 s），满足下面任一即可结束：
  1. `|q − q_target| < pos_tol`（默认 0.02 rad），认为空载到位（打开方向常见）；
  2. `|dq| < vel_stall`（默认 0.05 rad/s）且 `|τ| > tau_stall`（默认 0.8 N·m）持续 `stall_hold`（默认 0.2 s），认为夹住物体（闭合方向常见）；
  3. 超时 → `success=false`，但保持当前控制目标（不断电），等待下一次 service。

---

## 4. ROS 2 包结构

```
ros2_ws/src/
├── gripper_dm_msgs/                # ament_cmake，仅放 srv
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/
│       └── SetGripper.srv
└── gripper_dm/                     # ament_python，节点 + 业务逻辑
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/gripper_dm
    ├── gripper_dm/
    │   ├── __init__.py
    │   ├── damiao_sdk/             # 直接放 u2canfdpy 的 damiao.py + src/（.so）
    │   │   ├── __init__.py
    │   │   ├── damiao.py
    │   │   └── src/                # 含 usb_class*.so
    │   ├── gripper_node.py         # 主节点：service server + 控制循环
    │   └── gripper_controller.py   # 封装 Motor_Control 的高层 API
    ├── config/
    │   └── gripper.yaml            # 全部参数（CAN ID、SN、kp/kd、位置等）
    └── launch/
        └── gripper.launch.py
```

**`gripper_dm_msgs/srv/SetGripper.srv`**：
```text
# request
bool close              # true = 闭合, false = 打开
float64 hold_torque     # 可选；<=0 则使用默认值
---
# response
bool success
string message
float64 position        # 结束时电机实际位置 (rad)
float64 effort          # 结束时电机力矩 (N·m)
```

也提供一个便捷服务 `~/open`、`~/close`（`std_srvs/Trigger`），方便命令行调用：
```bash
ros2 service call /gripper/close std_srvs/srv/Trigger
ros2 service call /gripper/open  std_srvs/srv/Trigger
```

---

## 5. 节点行为

`gripper_node.py` 启动后：
1. 读取参数 → 构造 `Motor_Control(1_000_000, 5_000_000, sn, [DmActData(DM4310, MIT_MODE, 0x01, 0x11)])`；
2. 启动 **控制定时器**（100 Hz / 周期 10 ms）：
   - 维护内部 `q_setpoint`，每周期向 `q_target` 推进 `Δq = v_max · dt`（线性插补，实现 1 rad/s 限速）；
   - 调 `control_mit(motor, kp, kd, q_setpoint, dq_des, tau_ff)`；
   - 读取 `Get_Position/Velocity/tau`，发布到 `/gripper/state`（自定义或 `sensor_msgs/JointState`，joint 名 `gripper_finger`）。
3. **service 回调**：设置新的 `q_target`（open=0.1 / close=1.1）、`tau_ff`、`dq_des`，启动一次到位等待协程；按第 3 节判定到位/夹紧/超时后返回。
4. 关闭：节点析构调用 `Motor_Control.close()`（disable + USB close）。

---

## 6. 安装与运行步骤

1. udev 规则（一次性，与 SDK README 一致）：
   ```bash
   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"' \
     | sudo tee /etc/udev/rules.d/99-u2canfd.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
2. 依赖：
   ```bash
   pip3 install pyusb
   sudo apt install ros-humble-std-srvs   # 已自带
   ```
3. 把 `USB_Gripper/u2canfdpy/damiao.py` 与 `src/` 目录（含与你 Python 版本匹配的 `usb_class*.so`，本机 Python 3.10 用 `cpython-310-x86_64-linux-gnu.so`）拷到 `gripper_dm/gripper_dm/damiao_sdk/`，并删除其 `if __name__ == "__main__"` 段中的 demo（或保留，只 import 不执行）。
4. 构建：
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gripper_dm_msgs gripper_dm
   source install/setup.bash
   ```
5. 启动：
   ```bash
   ros2 launch gripper_dm gripper.launch.py
   # 另开终端
   ros2 service call /gripper/close std_srvs/srv/Trigger
   ros2 service call /gripper/open  std_srvs/srv/Trigger
   # 或自定义服务
   ros2 service call /gripper/set_gripper gripper_dm_msgs/srv/SetGripper "{close: true, hold_torque: 0.5}"
   ```

---

## 7. 调试 / 风险与“拿不准就写测试脚本”的部分

- **`usb_class.so` 与 Python 版本匹配**：仓库带了 3.8/3.10、x86_64/aarch64 三个 `.so`，本机查 `python3 --version` 后选对的一个；若都不匹配则必须用对应 conda env。
- **SN 必须精确匹配**：第一次启动前跑一次 `python3 dev_sn.py` 验证 SN（截图里是 `14AA044B241402B10DDBDAFE448040BB`，已写进默认配置）。
- **方向 / 零点**：用户给出的 0.1（开）→ 1.1（关）假设零点未被改动；若拆装过夹爪可能需要在调试助手重新设零位。会写一个 `scripts/sanity_check.py`，启动后手动给 `q=0.1` 和 `q=1.1`，确认运动方向。
- **dq_des 方向**：MIT 模式下 `dq_des` 不是硬限速，只是给阻尼项的目标。真正限速通过节点端线性插补。
- **力矩与温度保护**：节点内部对 `|τ| > τ_max(=3 N·m)` 触发回退；同时反馈 `T_MOS/T_Rotor`（如果需要可订阅）。
- **掉电/异常**：节点 `destroy_node` 必须调用 `Motor_Control.close()`，否则电机保持使能；用 `try/finally` 包住。

如运行时发现实际参数（kp、tau_ff、stall 阈值）不合适，先用一个 `scripts/tune_grip.py` 脚本以命令行参数试不同组合，再写回 yaml。

---

## 8. 任务清单

1. 新建 `gripper_dm_msgs` 包，定义 `SetGripper.srv`。
2. 新建 `gripper_dm` 包：
   - 集成 SDK（拷 `damiao.py` + `src/`）。
   - `gripper_controller.py`：包一层 `open()/close()/get_state()`，做插补与到位判定。
   - `gripper_node.py`：起 service（`SetGripper` + `Trigger open/close`）+ 100 Hz 控制定时器 + 状态发布。
   - `config/gripper.yaml`、`launch/gripper.launch.py`。
3. 写 `scripts/sanity_check.py` 验证方向与端到端通信。
4. `colcon build` → 服务调用测试夹紧硬物，必要时微调 `kp / tau_ff / stall` 阈值。

---

## 9. 通信链路测试结果（2026-05-30）

### 9.1 环境检测

| 检测项 | 结果 | 状态 |
|---|---|---|
| 当前 conda 环境 | `py312`（Python 3.12.13） | ⚠️ 不兼容 |
| **`grip` conda 环境（选定）** | Python 3.10.20 + pyusb 1.3.1 | ✅ 可用 |
| `usb_class.cpython-310-x86_64-linux-gnu.so` | 在 `grip` 环境下 import 成功 | ✅ 正常 |
| udev 规则（`99-u2canfd.rules`） | **未设置** | ❌ 缺失 |

**选定环境**：SDK 的 `usb_class.so` 仅编译了 Python 3.8 和 3.10 版本。
当前默认的 `py312` 环境（Python 3.12）**无法加载** `.so` 文件（`ModuleNotFoundError: No module named 'src.usb_class'`）。
**必须使用 conda `grip` 环境**（Python 3.10.20，已安装 pyusb 1.3.1）。

所有后续操作（构建、运行、测试）均需在 `grip` 环境下执行：
```bash
conda activate grip
```

### 9.2 USB 设备扫描（2026-05-30 深度诊断）

#### pyusb 扫描（grip 环境）

```
Total USB devices found: 8
  VID=0x05e3 PID=0x0620  (GL3523 Hub)
  VID=0x1d6b PID=0x0003  (USB 3.0 root hub)
  VID=0x2e88 PID=0x4603  (HDSC CDC Device)     ← 电机 UART 调参口
  VID=0x04f2 PID=0xb85c  (HD Webcam)
  VID=0x8087 PID=0x0026  (Bluetooth)
  VID=0x046d PID=0xc539  (Logitech Receiver)
  VID=0x05e3 PID=0x0610  (USB2.1 Hub)
  VID=0x1d6b PID=0x0002  (USB 2.0 root hub)
```

#### dmesg 内核日志

```
usb 1-9: new full-speed USB device number 7 using xhci_hcd
usb 1-9: New USB device found, idVendor=2e88, idProduct=4603, bcdDevice= 2.00
usb 1-9: Product: CDC Device
usb 1-9: Manufacturer: HDSC
usb 1-9: SerialNumber: 00000000050C
cdc_acm 1-9:1.0: ttyACM0: USB ACM device
```

**关键发现**：dmesg 中没有出现过 VID=0x34B7 / PID=0x6877 的任何记录。系统启动至今，U2CANFD 适配器从未被插过。唯一出现的外部 USB 设备是 HDSC CDC Device（电机 UART）和一部华为手机（短暂插入后拔出）。

#### `dev_sn.py` 结果

**输出为空**（exit code 0，无任何设备）。确认 U2CANFD 适配器不存在。

### 9.3 结论：当前连接的是错误接口

**你目前插入计算机的是电机 UART 调参口线缆（GH1.25），不是 U2CANFD 适配器。**

```
┌─────────────────────────────────────────────────┐
│             OmniGripper 夹爪                     │
│                                                  │
│  ┌─ DM-J4310-2EC 电机 ───────────────────────┐  │
│  │                                             │  │
│  │  ① UART 调参口 (GH1.25)                    │  │
│  │     → 你现在连的是这个！                      │  │
│  │     → 仅用于达妙调试助手配参数                 │  │
│  │     → 不能做实时控制                         │  │
│  │     → VID=0x2E88, 驱动 cdc_acm             │  │
│  │                                             │  │
│  │  ② CAN-FD 总线 (CAN_H + CAN_L)              │  │
│  │     → 需要接 U2CANFD 适配器                   │  │
│  │     → CAN_H → U2CANFD CAN_H                │  │
│  │     → CAN_L → U2CANFD CAN_L                │  │
│  │     → U2CANFD → USB → 计算机                │  │
│  │     → VID=0x34B7, pyusb 直接操作            │  │
│  └─────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
```

| 接口 | 你连的 | 需要连的 |
|---|---|---|
| 物理线缆 | GH1.25 调试线 | CAN_H/CAN_L 双绞线 |
| 中间设备 | 无（直连电机） | U2CANFD 适配器 |
| 电脑端 USB VID | `0x2e88`（HDSC） | `0x34B7`（达妙） |
| 用途 | 调参 | 实时控制 |
| 驱动 | `cdc_acm`（内核自带） | `pyusb`（Python libusb） |

**需要额外连接 U2CANFD 适配器**——它是一个独立的小盒子/模块，夹爪的 CAN_H/CAN_L 线需要接到它上面，然后它再通过 USB 连到电脑。

### 9.4 接线检查清单

完成以下检查后，重新运行 `dev_sn.py`：

- [ ] U2CANFD 适配器通过 USB 线连接到计算机
- [ ] 电机 CAN_H 接 U2CANFD CAN_H（通常黄/白线）
- [ ] 电机 CAN_L 接 U2CANFD CAN_L（通常绿/蓝线）
- [ ] CAN 总线末端有 120Ω 终端电阻
- [ ] 电机供电正常（24V 或 48V DC）
- [ ] `lsusb` 出现 `ID 34b7:6877` 设备
- [ ] udev 规则已设置（`/etc/udev/rules.d/99-u2canfd.rules`）

---

## 10. 方案补充与更新

### 10.1 Python 环境要求

**必须使用 conda `grip` 环境**，原因：
- SDK 的 `usb_class.so` 仅编译了 CPython 3.8 和 3.10 版本。
- `grip` 环境已安装 Python 3.10.20 + pyusb 1.3.1，与 `.so` 匹配。
- 默认的 `py312`（Python 3.12）和 `base`（Python 3.13）均不兼容。

使用方式：
```bash
conda activate grip
```

ROS 2 Humble 默认使用系统 Python 3.10，与 `grip` 环境版本一致，因此 `gripper_dm` 包可以在 `grip` 环境下正常 `colcon build` 和 `ros2 run`。

### 10.2 ROS 2 Service 控制方案（最终方案）

**最终控制方式**：通过 ROS 2 service 控制夹爪打开和关闭（夹取），不另做 GUI。

**Service 接口**（已在 `gripper_dm_msgs/srv/SetGripper.srv` 中定义）：

| Service | 类型 | 用途 |
|---|---|---|
| `/gripper/set_gripper` | `gripper_dm_msgs/srv/SetGripper` | 主接口：`close=true` 闭合，`close=false` 打开 |
| `/gripper/open` | `std_srvs/srv/Trigger` | 快捷打开 |
| `/gripper/close` | `std_srvs/srv/Trigger` | 快捷关闭（夹取） |
| `/gripper/reconnect` | `std_srvs/srv/Trigger` | **新增**：硬件恢复后重新连接控制器 |

**调用示例**：
```bash
conda activate grip
# 打开夹爪
ros2 service call /gripper/open std_srvs/srv/Trigger
# 关闭夹爪（夹取物体）
ros2 service call /gripper/close std_srvs/srv/Trigger
# 自定义夹持力矩
ros2 service call /gripper/set_gripper gripper_dm_msgs/srv/SetGripper "{close: true, hold_torque: 0.5}"
```

**架构**：
- `gripper_node.py` → ROS 2 节点，提供上述 service，内部启动 100 Hz 控制定时器。
- `gripper_controller.py` → `DMGripperController` 封装 MIT 控制 + 软插补 + 到位/夹紧判定。
- `damiao_sdk/damiao.py` → SDK 封装，`Motor_Control` 类。
- `config/gripper.yaml` → 所有参数（SN、CAN ID、kp/kd、位置、阈值等）。
- `launch/gripper.launch.py` → 一键启动。

**启动方式**：
```bash
conda activate grip
source ~/gripper/install/setup.bash
ros2 launch gripper_dm gripper.launch.py
```

### 10.3 已有代码资产

项目中已存在以下可用代码（位于 `gripper_dm/` 包内）：

| 文件 | 用途 | 可复用性 |
|---|---|---|
| `gripper_dm/damiao_sdk/damiao.py` | SDK 封装（Motor_Control 等） | ✅ 直接复用 |
| `gripper_dm/damiao_sdk/src/usb_class.cpython-310-*.so` | USB 底层通信 | ✅ 直接复用 |
| `gripper_dm/gripper_controller.py` | 高层控制 API（open/close/get_state） | ✅ 直接复用 |
| `gripper_dm/comm_test.py` | 通信测试脚本（USB 诊断 + 电机通信） | ✅ 可用于验证链路 |
| `gripper_dm/gripper_node.py` | ROS 2 节点（service server + 控制循环） | ✅ 直接复用 |

### 10.4 代码改进记录（2026-05-30）

| 文件 | 改进内容 |
|---|---|
| `damiao_sdk/damiao.py` | 恢复被误删的 `import time` 和 `import threading`（Motor 类依赖）；清理底部 demo 代码 |
| `comm_test.py` | USB 扫描改用 sysfs（`/sys/bus/usb/devices/`）代替 pyusb（更可靠）；增加 OmniGripper 双接口接线图提示；增加 Python 环境检测 |
| `gripper_controller.py` | `_wait_for_goal()` 增加 `self._closed` 检查（关闭时提前退出）；新增 `is_connected` 属性；新增 `emergency_stop()` 方法；`Motor_Control` 初始化失败转为 `RuntimeError` |
| `gripper_node.py` | **新增 `/gripper/reconnect` service**（硬件恢复后重新连接）；控制器初始化失败时节点不崩溃，记录诊断信息；所有 service handler 增加 `_controller_ok()` 检查 |

### 10.5 关于"串口 960000"的澄清

用户提到的"串口波特率 960000"指的是电机 UART 调参口。实际情况：
- 电机 UART 口的默认波特率是 **921600**（达妙出厂设置），对应 `/dev/ttyACM0`（HDSC CDC Device）。
- 此 UART 口**仅用于达妙调试助手配置电机参数**，不用于实时控制。
- 本项目的控制通路是 **USB → U2CANFD 适配器 → CAN FD 总线 → 电机**，
  使用 `pyusb` 直接操作 USB 设备（VID=0x34B7），**不依赖任何串口**。
- CAN FD 通信参数：仲裁域 1 Mbps，数据域 5 Mbps（已在 SDK 和出厂固件中设定）。

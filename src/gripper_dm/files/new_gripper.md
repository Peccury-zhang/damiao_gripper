# OmniGripper（DM-J4310-2EC）ROS 2 控制方案

> 基于 CANFD 分析仪（`libcontrolcanfd.so`）+ ctypes，通过 ROS 2 service 控制夹爪打开/闭合（夹取）。
>
> Python 环境：conda `py312`（Python 3.12.13），ROS 2 Jazzy rclpy 兼容。
>
> **状态：MIT 控制已调通，夹爪打开/关闭功能正常（2026-06-01）。**

---

## 1. 硬件与通信参数

| 项 | 取值 | 来源 |
|---|---|---|
| 电机 | DM-J4310-2EC（达妙 OmniGripper 内置） | 电机说明书 |
| CAN-FD 分析仪 | USB CANFD DEBUG（VID=0x04D8, PID=0x0053） | lsusb / dmesg |
| 分析仪驱动 | `libcontrolcanfd.so`（x86_64, ctypes 调用） | Linux资料包V1.02 |
| 分析仪 SN | `USBCANFD212606182279` | ZCAN_GetDeviceInf |
| 分析仪通道数 | 2（CH0, CH1） | ZCAN_GetDeviceInf |
| CAN-FD 仲裁域波特率 | 1 Mbps | SDK / 电机规格 |
| CAN-FD 数据域波特率 | 5 Mbps | SDK / 电机规格 |
| CAN-FD 标准 | ISO 11898-1 | SDK SetCANFDStandard(0) |
| 电机 CAN ID（接收） | `0x02`（已确认，通道 0） | confirm_id.py 实测 |
| 电机 Master ID（反馈） | `0x12`（已确认） | confirm_id.py 实测 |
| 终端电阻 | 120Ω（分析仪内置可启用） | ZCAN_SetResistanceEnable |
| 打开位置 | 0.10 rad | 用户指定 |
| 闭合位置 | 1.05 rad | 用户指定 |
| 最大速度 | 2.0 rad/s | 用户指定 |

### 1.1 与旧方案的区别

| 项目 | 旧方案 | 新方案 |
|---|---|---|
| CAN-FD 适配器 | 达妙 U2CANFD（VID=0x34B7） | CANFD 分析仪（VID=0x04D8） |
| 通信方式 | pyusb + `usb_class.so`（私有 Python 扩展） | ctypes + `libcontrolcanfd.so`（标准 C 共享库） |
| API 风格 | `Motor_Control` 类封装 | ZCAN API（OpenDevice/InitCAN/TransmitFD/ReceiveFD） |
| Python 限制 | 必须 CPython 3.8 或 3.10 | 任何支持 ctypes 的 Python 版本 |

---

## 2. CANFD 分析仪 SDK（libcontrolcanfd.so）

### 2.1 核心 API

| 函数 | 用途 |
|---|---|
| `ZCAN_OpenDevice(USBCAN2, dev_idx, reserved)` | 打开 USB 设备，返回 dev_handle |
| `ZCAN_CloseDevice(dev_handle)` | 关闭设备 |
| `ZCAN_GetDeviceInf(dev_handle, info)` | 获取设备信息（FW版本/SN/通道数） |
| `ZCAN_SetAbitBaud(dev, ch, baudrate)` | 设置通道仲裁域波特率 |
| `ZCAN_SetDbitBaud(dev, ch, baudrate)` | 设置通道数据域波特率 |
| `ZCAN_SetCANFDStandard(dev, ch, std)` | 设置 ISO(0) / BOSCH(1) |
| `ZCAN_InitCAN(dev, ch, config)` | 初始化通道，返回 ch_handle |
| `ZCAN_StartCAN(ch_handle)` | 启动通道 |
| `ZCAN_ResetCAN(ch_handle)` | 复位通道 |
| `ZCAN_TransmitFD(ch, msgs, count)` | 发送 CAN-FD 帧 |
| `ZCAN_ReceiveFD(ch, msgs, count, timeout)` | 接收 CAN-FD 帧 |
| `ZCAN_Transmit(ch, msgs, count)` | 发送经典 CAN 帧 |
| `ZCAN_Receive(ch, msgs, count, timeout)` | 接收经典 CAN 帧 |
| `ZCAN_SetResistanceEnable(dev, ch, enable)` | 启用/禁用 120Ω 终端电阻 |
| `ZCAN_ClearFilter / AckFilter / SetFilterMode / SetFilterStartID / SetFilterEndID` | 配置接收过滤器 |

### 2.2 CAN-FD 帧结构（ctypes）

> **重要**：发送帧结构体（`ZCAN_TRANSMITFD_FRAME`）必须与 `ZCAN_CANFD_FRAME` 的头部布局完全一致（`can_id` 后紧跟 `len` 和 `flags`），不能使用包含 `transmit_type`/`remote_flag`/`ext_flag` 的扩展布局，否则 ZCAN 库会将 `len` 字段读为 0，导致发送空帧。

```python
class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),       # 32位: bits[28:0]=ID, bit[29]=ERR, bit[30]=RTR, bit[31]=EFF
        ("len", c_ubyte),          # 数据长度 (0~64)
        ("flags", c_ubyte),        # bit[0]=BRS
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 64),
    ]

# 发送帧必须使用与 ZCAN_CANFD_FRAME 相同的布局
class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME)]

class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]
```

### 2.3 Python ctypes 调用流程

```python
lib = cdll.LoadLibrary("libcontrolcanfd.so")
dev = lib.ZCAN_OpenDevice(41, 0, 0)        # USBCAN2=41
lib.ZCAN_SetAbitBaud(dev, 0, 1000000)       # 1Mbps 仲裁域
lib.ZCAN_SetDbitBaud(dev, 0, 5000000)       # 5Mbps 数据域
lib.ZCAN_SetCANFDStandard(dev, 0, 0)         # ISO
lib.ZCAN_SetResistanceEnable(dev, 0, 1)      # 120Ω 终端电阻
cfg = ZCAN_CHANNEL_INIT_CONFIG()
cfg.can_type = 1  # TYPE_CANFD
ch = lib.ZCAN_InitCAN(dev, 0, byref(cfg))
lib.ZCAN_StartCAN(ch)
# 发送帧（发送到电机 CAN_ID=0x02）
msg = ZCAN_TransmitFD_Data()
msg.frame.can_id = 0x02
msg.frame.len = 8
msg.frame.flags = 1  # BRS 启用
msg.frame.data[0:8] = ...
lib.ZCAN_TransmitFD(ch, byref(msg), 1)
# 接收帧（从 MST_ID=0x12）
num = lib.ZCAN_GetReceiveNum(ch, 1)  # TYPE_CANFD=1
msgs = (ZCAN_ReceiveFD_Data * num)()
cnt = lib.ZCAN_ReceiveFD(ch, byref(msgs), num, timeout_ms)
```

> **重要**：此 `libcontrolcanfd.so` 的所有 API 函数返回值为 **1 = 成功**（非标准的 0 = 成功）。

---

## 3. 电机控制协议（来自 damiao.py）

### 3.1 MIT 控制模式

MIT 模式控制力矩公式：`τ = Kp·(q_des − q) + Kd·(dq_des − dq) + τ_ff`

**控制帧格式**（8 字节，发送到 CAN_ID + 0x000）：

| 字节 | 内容 |
|---|---|
| [0] | q_des 高 8 位（16 bit，范围 −12.566~12.566 rad） |
| [1] | q_des 低 8 位 |
| [2] | dq_des 高 8 位（12 bit，范围 −30~30 rad/s） |
| [3] | dq_des 低 4 位 \| kp 高 4 位（12 bit，范围 0~500） |
| [4] | kp 低 8 位 |
| [5] | kd 高 8 位（12 bit，范围 0~5） |
| [6] | kd 低 4 位 \| tau_ff 高 4 位（12 bit，范围 −10~10 N·m） |
| [7] | tau_ff 低 8 位 |

### 3.2 使能/去使能

- **使能**：发送 `[FF FF FF FF FF FF FF FC]` 到 CAN_ID + mode_offset（5 次）
- **去使能**：发送 `[FF FF FF FF FF FF FF FD]` 到 CAN_ID + mode_offset（5 次）
- **清零**：发送 `[FF FF FF FF FF FF FF FE]`（3 次）
- **刹车**：发送 `[FF FF FF FF FF FF FF FB]`（3 次）

### 3.3 反馈帧格式（从 Master ID = 0x12 接收，8 字节）

| 字节 | 内容 |
|---|---|
| [0] | 电机 ID |
| [1] | q 高 8 位（16 bit） |
| [2] | q 低 8 位 |
| [3] | dq 高 8 位（12 bit） |
| [4] | dq 低 4 位 \| tau 高 4 位（12 bit） |
| [5] | tau 低 8 位 |
| [6] | MOS 温度 |
| [7] | 转子温度 |

### 3.4 模式切换（通过 0x7FF 参数帧）

- **写入模式**：发送到 `0x7FF`，data = `[motor_id_low, motor_id_high, 0x55, 10, mode, 0, 0, 0]`
  - mode: 1=MIT, 2=POS_VEL, 3=VEL, 4=POS_FORCE
- **读取模式**：发送到 `0x7FF`，data = `[motor_id_low, motor_id_high, 0x33, 10, 0, 0, 0, 0]`
- **响应**：在 Master ID 上返回，data[2]=命令类型, data[3]=RID

### 3.5 控制策略（夹取硬物）

#### 运动阶段参数

| 参数 | 闭合 | 打开 | 说明 |
|---|---|---|---|
| kp_move | 10.0 | 10.0 | 运动时位置刚度 |
| kd_move | 0.5 | 0.5 | 运动时阻尼 |
| kp_hold | 20.0 | — | 保持时位置刚度（更高） |
| kd_hold | 1.0 | — | 保持时阻尼 |
| q_open | — | 0.10 rad | 打开目标位置 |
| q_close | 1.05 rad | — | 闭合目标位置 |
| max_speed | 2.0 rad/s | 2.0 rad/s | 最大速度 |
| decel_distance | 0.15 rad | 0.15 rad | 减速距离（梯形速度曲线） |
| position_tolerance | 0.05 rad | 0.05 rad | 位置容差 |
| hold_torque | 1.0 N·m | — | 闭合保持力矩 |
| motion_timeout | 5.0 s | 5.0 s | 运动超时 |
| stall_delay | 0.5 s | — | 堵转检测延迟 |

#### 梯形速度曲线

为避免超调和突然停止，采用梯形速度曲线：
- 当剩余距离 > `decel_distance`（0.15 rad）时，使用 `max_speed`（2.0 rad/s）
- 当剩余距离 < `decel_distance` 时，线性减速到 0.2 rad/s
- 公式：`speed = max(0.2, max_speed * (remaining / decel_distance))`

#### 到位判定（必须同时满足）

1. **插补完成**：`|interpolated_pos - target| < 1e-6`
2. **位置到位**：`|position - target| < position_tolerance`（0.05 rad）
3. **速度稳定**：`|velocity| < stall_speed_threshold`（0.1 rad/s）

#### 状态转换

| 当前状态 | 转换条件 | 目标状态 |
|---|---|---|
| MOVING_TO_CLOSE | 到位判定通过 | HOLDING_TORQUE |
| MOVING_TO_CLOSE | 堵转检测（0.5s 后速度<0.1） | HOLDING_TORQUE |
| MOVING_TO_CLOSE | 超时（5s） | HOLDING_TORQUE |
| MOVING_TO_OPEN | 到位判定通过 | IDLE |
| MOVING_TO_OPEN | 超时（5s） | IDLE |
| HOLDING_TORQUE | 夹紧检测（力矩>阈值） | CLAMPED |

#### 力矩保持模式（HOLDING_TORQUE）

切换到力矩保持模式时，**不再使用纯力矩控制**（会导致失控），而是：
- 发送 `send_mit(q=close_pos, dq=0, kp=kp_hold, kd=kd_hold, tau=hold_torque)`
- 使用高刚度（kp_hold=20.0）保持位置
- 使用前馈力矩（hold_torque=1.0 N·m）增加夹持力
- 阻尼（kd_hold=1.0）抑制振动

---

## 4. ROS 2 包结构

```
ros2_ws/src/
├── gripper_dm_msgs/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   └── GripperStatus.msg    # 电机状态消息
│   └── srv/
│       └── SetGripper.srv       # bool close, float64 hold_torque → bool success, string message, float64 position, float64 effort
└── gripper_dm/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/gripper_dm
    ├── gripper_dm/
    │   ├── __init__.py
    │   ├── canfd_driver.py       # libcontrolcanfd.so 的 ctypes 封装
    │   ├── motor_protocol.py     # MIT 帧编码/解码、使能/去使能
    │   ├── gripper_controller.py # 高层 API：open/close/get_state
    │   └── gripper_node.py       # ROS 2 节点
    ├── config/
    │   └── gripper.yaml
    ├── launch/
    │   └── gripper.launch.py
    └── scripts/
        ├── comm_test.py          # 通信测试
        ├── diag_can.py           # CAN 诊断
        ├── fast_scan.py          # ID 快速扫描
        └── libcontrolcanfd.so    # SDK 库文件
```

### 4.1 Service 接口

| Service | 类型 | 用途 |
|---|---|---|
| `/set_gripper` | `gripper_dm_msgs/srv/SetGripper` | 主接口（close=true 闭合，close=false 打开） |
| `/open_gripper` | `std_srvs/srv/Trigger` | 快捷打开 |
| `/close_gripper` | `std_srvs/srv/Trigger` | 快捷关闭 |
| `/reconnect_gripper` | `std_srvs/srv/Trigger` | 重连硬件 |

### 4.2 Topic 接口

| Topic | 类型 | 频率 | 用途 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | 关节状态（position/velocity/effort） |
| `/gripper_status` | `gripper_dm_msgs/GripperStatus` | 20 Hz | 电机完整状态（位置/速度/力矩/电流/温度/模式/状态） |

#### GripperStatus.msg 字段说明

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | 时间戳 + frame_id |
| `position` | `float64` | rad | 当前位置（来自反馈帧） |
| `velocity` | `float64` | rad/s | 当前速度（来自反馈帧） |
| `torque` | `float64` | N·m | 反馈力矩（来自反馈帧） |
| `current` | `float64` | A | 估算电流 = torque / Kt（Kt=0.335 Nm/A，可配置） |
| `temperature_mos` | `uint8` | °C | MOS 管温度（来自反馈帧） |
| `temperature_rotor` | `uint8` | °C | 转子温度（来自反馈帧） |
| `state` | `string` | — | 控制器状态：idle / moving_to_open / moving_to_close / holding_torque / clamped |
| `enabled` | `bool` | — | 电机是否使能 |
| `mode` | `string` | — | 电机控制模式：MIT / POS_VEL / VEL / POS_FORCE |
| `target_position` | `float64` | rad | 当前目标位置 |

> **关于电流估算**：电机反馈帧不直接提供电流值，通过 `I = τ / Kt` 估算。默认 `Kt = 0.335 Nm/A`（DM-J4310-2EC 典型值），可通过 `torque_constant` 参数调整。

### 4.3 调用示例

```bash
conda activate py312  # Python 3.12
source ~/gripper/install/setup.bash
ros2 launch gripper_dm gripper.launch.py

# 另一个终端
ros2 service call /close_gripper std_srvs/srv/Trigger
ros2 service call /open_gripper std_srvs/srv/Trigger
ros2 service call /set_gripper gripper_dm_msgs/srv/SetGripper "{close: true, hold_torque: 0.5}"
ros2 topic echo /joint_states  # 查看实时位置/速度/力矩
ros2 topic echo /gripper_status  # 查看电机完整状态（20Hz）
```

---

## 5. 代码模块设计

### 5.1 canfd_driver.py（CANFD 分析仪驱动）

```python
class CANFDDriver:
    def __init__(self, lib_path: str, channel: int = 0,
                 abit: int = 1_000_000, dbit: int = 5_000_000):
        # 加载 libcontrolcanfd.so，配置 ctypes 函数签名
        # OpenDevice → SetBaud → SetResistance → InitCAN → StartCAN
        # 配置接收过滤器（全 ID 范围）

    def send_fd(self, can_id: int, data: bytes, brs: int = 0) -> int:
        # ZCAN_TransmitFD

    def recv_fd(self, timeout_ms: int = 100) -> list[dict]:
        # ZCAN_ReceiveFD → 返回 [{"id": int, "data": bytes, "ts": int}, ...]

    def close(self):
        # ResetCAN + CloseDevice

    @property
    def is_open(self) -> bool: ...
```

### 5.2 motor_protocol.py（电机协议编解码）

```python
class MotorProtocol:
    def __init__(self, can_id: int = 0x02, mst_id: int = 0x12): ...

    def build_enable_frame(self) -> bytes: ...
    def build_disable_frame(self) -> bytes: ...
    def build_mit_frame(self, kp, kd, q, dq, tau) -> bytes: ...
    def build_mode_switch_frame(self, mode: int) -> bytes: ...
    def decode_feedback(self, data: bytes) -> tuple[float, float, float, int, int]: ...
        # 返回 (position, velocity, torque, t_mos, t_rotor)

    @property
    def control_can_id(self) -> int:
        return self._can_id  # MIT mode offset = 0x000
```

### 5.3 gripper_controller.py（高层控制器）

```python
class DMGripperController:
    def __init__(self, driver: CANFDDriver, protocol: MotorProtocol,
                 open_pos=0.1, close_pos=1.1, max_vel=1.0, ...): ...

    def enable(self) -> bool: ...
    def disable(self) -> None: ...
    def open(self, timeout=3.0) -> GripperResult: ...
    def close(self, hold_torque=0.0, timeout=3.0) -> GripperResult: ...
    def get_state(self) -> GripperState: ...

    def _control_loop(self, target_pos, kp, kd, tau_ff, timeout):
        # 100Hz 循环：
        #   - 线性插补 q_setpoint 向 target 推进 Δq = max_vel * dt
        #   - 发送 MIT 控制帧
        #   - 接收并解码反馈
        #   - 判定到位 / 夹紧 / 超时
```

### 5.4 gripper_node.py（ROS 2 节点）

- 100Hz 定时器：运行控制循环 + 发布 JointState
- 20Hz 定时器：发布 GripperStatus（电机完整状态）
- 4 个 service server（ReentrantCallbackGroup + MultiThreadedExecutor）
- `_try_connect()` 失败不崩溃，记录诊断信息
- `/reconnect_gripper` 用于硬件恢复后重连

---

## 6. 安装与运行

### 6.1 环境准备

```bash
conda activate py312  # Python 3.12.13（ROS 2 Jazzy rclpy 兼容）
source /opt/ros/jazzy/setup.bash
```

> **注意**：不能使用 `grip` 环境（Python 3.10），rclpy 的 C 扩展仅兼容 Python 3.12。

### 6.2 udev 规则

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-canfd.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 6.3 构建

```bash
cd ~/gripper
colcon build --packages-select gripper_dm_msgs gripper_dm
source install/setup.bash
```

### 6.4 运行

```bash
ros2 launch gripper_dm gripper.launch.py
```

---

## 7. 通信测试结果（2026-06-01）

### 7.1 CANFD 分析仪检测

| 检测项 | 结果 |
|---|---|
| USB 设备 | ✅ VID=0x04D8 PID=0x0053, Product="USB CANFD DEBUG" |
| dmesg 记录 | ✅ Manufacturer=NXP SEMICONDUCTORS, Device=Bus 001 Device 007 |
| lsusb | ✅ 已识别 |
| udev 规则 | ✅ 已设置 (`/etc/udev/rules.d/99-canfd.rules`) |

### 7.2 SDK 加载与设备初始化

| 检测项 | 结果 |
|---|---|
| libcontrolcanfd.so | ✅ ELF 64-bit, x86-64, 动态链接 |
| ctypes 加载 | ✅ 成功 |
| ZCAN_OpenDevice | ✅ handle 有效 |
| ZCAN_GetDeviceInf | ✅ SN=USBCANFD212606182279, HW=USBCANFD, 2通道 |
| ZCAN_SetAbitBaud(1M) | ✅ OK |
| ZCAN_SetDbitBaud(5M) | ✅ OK |
| ZCAN_SetCANFDStandard(ISO) | ✅ OK |
| ZCAN_SetResistanceEnable(120Ω) | ✅ OK |
| ZCAN_InitCAN + StartCAN | ✅ OK |

### 7.3 电机通信（confirm_id.py 逐 ID 扫描确认）

| 检测项 | 结果 |
|---|---|
| 通道 0 使能 + MIT (ID=0x02) | ✅ 收到反馈 14 帧，MST_ID=0x12 |
| 通道 1 扫描 (ID 0x01~0x20) | ❌ 0 帧（电机接在 CH0） |
| 反馈解码 | ✅ pos=1.1303 rad, vel≈0, tau≈0 |
| CAN-FD BRS 模式 | ✅ 数据域 5Mbps 工作正常 |

### 7.4 MIT 控制测试（test_mit_full.py）

| 目标位置 | 实际位置 | 误差 | 结果 |
|---|---|---|---|
| 0.5 rad | 0.4857 rad | 0.0143 | ✅ 到位 |
| 1.0 rad | 0.9789 rad | 0.0211 | ✅ 到位 |
| 0.1 rad | 0.1229 rad | 0.0229 | ✅ 到位 |
| 1.1 rad | 1.0836 rad | 0.0164 | ✅ 到位 |
| 0.1 rad | 0.1229 rad | 0.0229 | ✅ 到位 |

> 所有位置在 2 秒内到位，误差 < 0.025 rad。

### 7.5 ROS 2 节点验证

| 检测项 | 结果 |
|---|---|
| `ros2 launch gripper_dm gripper.launch.py` | ✅ 启动成功，电机使能 |
| `/open_gripper` service | ✅ 电机从 1.09→0.12 rad，状态→idle，速度 -1.9 rad/s |
| `/close_gripper` service | ✅ 电机从 0.13→1.09 rad，状态→holding_torque，平滑减速 |
| `/joint_states` topic | ✅ 100Hz 发布，name=gripper_joint |

**关键改进**：
- 闭合位置：1.0916 rad（目标 1.05 rad，误差 0.0416 rad，在 ±0.05 容差内）
- 无超调：位置未超过 1.1 rad
- 无突然加速：接近目标时平滑减速到 0.2 rad/s
- 力矩保持稳定：位置稳定在 1.0916 rad，速度 < 0.04 rad/s

### 7.6 确认的设备配置

| 参数 | 确认值 | 来源 |
|---|---|---|
| CANFD 分析仪通道 | **CH0** | confirm_id.py（CH1 无响应） |
| 电机命令 CAN ID | **0x02** | 逐 ID 扫描 0x01~0x20，仅 0x02 有响应 |
| 电机反馈 MST ID | **0x12** | 所有反馈帧均为 ID=0x012 |
| 电机物理 ID | 2（data[0]=0x02） | 反馈帧首字节 |
| 仲裁域波特率 | 1 Mbps | 确认 OK |
| 数据域波特率 | 5 Mbps | 确认 OK |
| CAN-FD 标准 | ISO 11898-1 | 确认 OK |
| BRS（位速率切换） | 启用（CAN-FD 帧 flags=1） | 确认 OK |
| ZCAN API 返回值 | **1 = 成功**（非标准 0） | 实测所有函数均返回 1 |

---

## 8. 测试脚本

所有测试脚本位于 `scripts/` 目录：

| 脚本 | 用途 |
|---|---|
| `comm_test.py` | 完整通信测试（USB 扫描 + udev 检查 + 设备初始化 + 电机通信 + 多阶段诊断） |
| `confirm_id.py` | **精确 ID 确认**（逐 ID 扫描 0x01~0x20，确认 CAN_ID 和 MST_ID） |
| `test_mit_full.py` | **MIT 完整行程测试**（使能 + 连续控制到 5 个目标位置 + 实时反馈读取） |
| `test_mit_mode.py` | MIT 模式测试（使能 + 单点控制 + 去使能） |
| `diag_can.py` | CAN 总线深度诊断（被动监听 + 模式切换 + 全 ID 扫描 + 120Ω 电阻控制） |
| `fast_scan.py` | 快速 ID 扫描（多通道 + 多波特率 + ID 0x01~0x20） |
| `diag_mode.py` | 电机模式诊断（读取/设置电机控制模式） |

运行方式：
```bash
conda activate py312
cd ~/gripper/src/gripper_dm/scripts
python3 comm_test.py
python3 confirm_id.py
python3 diag_can.py
python3 fast_scan.py
```

---

## 9. 后续步骤

### 9.1 已完成 ✅

1. ✅ 确认电机 CAN ID = 0x02，MST_ID = 0x12，通道 CH0
2. ✅ 通信测试通过（`confirm_id.py` 逐 ID 扫描确认）
3. ✅ `canfd_driver.py`：ctypes 封装 libcontrolcanfd.so
4. ✅ `motor_protocol.py`：MIT 帧编解码 + 使能/去使能
5. ✅ `gripper_controller.py`：控制循环 + 线性插补 + 到位/夹紧判定
6. ✅ `gripper_node.py`：ROS 2 节点（4 service + JointState）
7. ✅ `config/gripper.yaml` + `launch/gripper.launch.py`
8. ✅ colcon build 编译通过
9. ✅ MIT 控制测试通过（test_mit_full.py 5 个位置全部到位）
10. ✅ ROS 2 节点打开/关闭功能验证通过
11. ✅ `/gripper_status` Topic：20Hz 发布电机完整状态（GripperStatus.msg）

### 9.2 待优化

1. 微调 MIT 控制参数（kp_move, kd_move, hold_torque）
2. 验证实际夹取物体的力矩保持效果
3. 添加通信断线自动恢复机制

---

## 10. 调试过程中发现的关键问题

### 10.1 CANFD 发送帧结构体布局错误（致命）

**问题**：`canfd_driver.py` 中 `ZCAN_TRANSMITFD_FRAME` 使用了错误的结构体布局，在 `can_id`（4字节）和 `len` 字段之间插入了 4 个多余字节（`transmit_type`, `remote_flag`, `ext_flag`, `reserved`）。

**影响**：ZCAN 库读取到的 `len` 字段为 0（实际是 `transmit_type` 的值），导致 **所有发送帧的 data 长度被识别为 0**。电机收到空数据帧，MIT 控制帧完全被忽略。

**修复**：
```python
# 错误布局（12 字节头部）
class ZCAN_TRANSMITFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),        # 4 bytes
        ("transmit_type", c_ubyte),# 1 byte ← 多余
        ("remote_flag", c_ubyte),  # 1 byte ← 多余
        ("ext_flag", c_ubyte),     # 1 byte ← 多余
        ("reserved", c_ubyte),     # 1 byte ← 多余
        ("len", c_ubyte),          # 1 byte ← 被推到偏移 8
        ("flags", c_ubyte),        # 1 byte
        ...
    ]

# 正确布局（8 字节头部，与 ZCAN_CANFD_FRAME 一致）
class ZCAN_TRANSMITFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),        # 4 bytes
        ("len", c_ubyte),          # 1 byte ← 偏移 4
        ("flags", c_ubyte),        # 1 byte
        ("reserved1", c_ubyte),    # 1 byte
        ("reserved2", c_ubyte),    # 1 byte
        ("data", c_ubyte * 64),
    ]
```

**教训**：必须确保 ctypes 结构体的内存布局与 C 头文件完全一致。参考 `comm_test.py` 中已验证可工作的简化结构体。

### 10.2 电机模式必须为 MIT 模式

**问题**：电机被设置为 POS_VEL 模式（速度位置模式），导致所有 MIT 控制帧被电机忽略。

**表现**：使能命令有效（电机有反馈帧），但 MIT 控制帧完全无响应（电机位置不变）。

**修复**：通过达妙官方调试工具将电机模式改回 MIT 模式（mode=1）。

**注意**：代码中 `initialize()` 会发送模式切换到 0x7FF 的 MIT 模式命令，但电机可能不通过 CAN 响应模式切换（需要通过达妙调试工具设置）。如果电机已通过调试工具固定为 MIT 模式，则模式切换命令可忽略。

### 10.3 反馈帧与参数响应帧混淆

**问题**：模式切换命令（发送到 0x7FF）的响应帧与电机反馈帧都来自 MST_ID=0x12，`receive_feedback()` 可能将参数响应帧误解码为反馈帧。

**修复**：在 `decode_feedback()` 中添加过滤：
```python
if data[2] in (0x33, 0x55, 0xAA) and data[3] in (9, 10, 11):
    return None  # 参数响应帧，非反馈帧
```

### 10.4 堵转检测误触发

**问题**：`_is_motor_stalled()` 在电机开始移动前就触发（速度≈0 且位置误差大）。

**修复**：添加 0.5 秒最小运动时间要求：
```python
def _is_motor_stalled(self):
    elapsed = time.monotonic() - self._motion_start_time
    if elapsed < 0.5:
        return False
    ...
```

### 10.5 位置容差过小

**问题**：MIT 控制器稳态误差约 0.023 rad，大于原容差 0.02 rad，导致状态永远无法转换。

**修复**：将 `position_tolerance` 从 0.02 增加到 0.05，并添加速度辅助判定（当插补到达目标且速度接近 0 时，即使位置误差略大于容差也视为到位）。

### 10.6 运动超时保护

**新增**：添加 `motion_timeout`（默认 5 秒）参数，防止电机在无法到达目标时状态永久卡住。

### 10.7 关闭时超调问题（超过 1.1 rad）

**问题**：关闭夹爪时，电机位置超过目标值 1.05 rad，达到 1.1~1.2 rad。

**原因分析**：
1. **线性速度曲线**：原代码使用恒定速度（max_speed）直线运动，到达目标位置时突然停止，但电机有惯性继续运动
2. **到位判定条件宽松**：使用 `(pos_close or speed_low)`，任一条件满足就切换状态，导致高速通过目标时也认为到位

**修复**：
1. **梯形速度曲线**：在距离目标 < `decel_distance`（0.15 rad）时线性减速到 0.2 rad/s
2. **严格到位判定**：改为 `(interp_done and pos_close and speed_low)`，必须同时满足三个条件

```python
remaining = abs(target - self._interpolated_pos)
speed = self._max_speed
if remaining < self._decel_distance:
    ratio = remaining / self._decel_distance
    speed = max(0.2, self._max_speed * ratio)

if interp_done and pos_close and speed_low:
    # 切换状态
```

### 10.8 接近闭合时突然加速问题

**问题**：电机在接近目标位置（~1.0 rad）时突然加速，速度从 ~1 rad/s 飙升到 2~3 rad/s。

**原因分析**：
原到位判定使用 `(pos_close or speed_low)`，当位置进入容差带（±0.05 rad）时，即使速度还很高（1~2 rad/s），也会立即切换到 HOLDING_TORQUE 状态。状态切换瞬间控制指令突变，导致电机加速。

**修复**：
到位判定改为必须同时满足三个条件：
```python
if interp_done and pos_close and speed_low:
    # interp_done: 插补完成
    # pos_close: 位置在容差带内
    # speed_low: 速度接近 0
```

配合梯形速度曲线，电机在到达目标时速度已经降到 0.2 rad/s，不会发生突然加速。

### 10.9 力矩保持模式失控问题

**问题**：切换到 HOLDING_TORQUE 状态后，电机位置从 ~1.09 rad 快速增加到 1.24 rad，速度高达 7 rad/s。

**原因分析**：
原代码在 HOLDING_TORQUE 状态发送纯力矩控制：
```python
self._motor.send_mit(0.0, 0.0, 0.0, 0.0, self._hold_torque_cmd)
# q=0, dq=0, kp=0, kd=0, tau=1.0
```

**kp=0, kd=0 意味着完全没有位置反馈和阻尼**，电机失去控制，纯力矩 1.0 N·m 推动电机飞转。

**修复**：
改为使用位置反馈 + 力矩前馈：
```python
self._motor.send_mit(
    self._close_pos, 0.0,
    self._kp_hold, self._kd_hold,
    self._hold_torque_cmd
)
# q=1.05, dq=0, kp=20.0, kd=1.0, tau=1.0
```

- `kp_hold=20.0`：高刚度保持位置
- `kd_hold=1.0`：阻尼抑制振动
- `hold_torque=1.0`：前馈力矩增加夹持力

**测试结果**：
- 闭合位置：1.0916 rad（目标 1.05 rad，误差 0.0416 rad，在容差内）
- 保持稳定：位置稳定在 1.0916 rad，速度 < 0.04 rad/s
- 打开位置：0.1233 rad（目标 0.1 rad，误差 0.0233 rad，在容差内）

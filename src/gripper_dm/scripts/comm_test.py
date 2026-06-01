#!/usr/bin/env python3
import os
import sys
import time
import struct
import ctypes
from ctypes import (
    cdll, c_uint, c_int, c_ubyte, c_ushort, c_ulong, c_ulonglong,
    c_void_p, c_char, c_char_p, byref, Structure, Union
)
import signal

CANFD_ANALYZER_VID = "04d8"
CANFD_ANALYZER_PID = "0053"

USBCAN2 = 41
STATUS_OK = 1
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0
TYPE_CAN = 0
TYPE_CANFD = 1

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000


class ZCAN_CAN_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),
        ("can_dlc", c_ubyte),
        ("__pad", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 8),
    ]


class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),
        ("len", c_ubyte),
        ("flags", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 64),
    ]


class _CAN_INIT_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("reserved", c_uint),
        ("filter", c_ubyte),
        ("timing0", c_ubyte),
        ("timing1", c_ubyte),
        ("mode", c_ubyte),
    ]


class _CANFD_INIT_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("abit_timing", c_uint),
        ("dbit_timing", c_uint),
        ("brp", c_uint),
        ("filter", c_ubyte),
        ("mode", c_ubyte),
        ("pad", c_ushort),
        ("reserved", c_uint),
    ]


class _CHANNEL_CONFIG_UNION(Union):
    _fields_ = [
        ("can", _CAN_INIT_CONFIG),
        ("canfd", _CANFD_INIT_CONFIG),
    ]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [
        ("can_type", c_uint),
        ("config", _CHANNEL_CONFIG_UNION),
    ]


class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_ubyte * 21),
        ("str_hw_Type", c_ubyte * 40),
        ("reserved", c_ushort * 4),
    ]


class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]


class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


def make_can_id(id_val, eff=0, rtr=0):
    return id_val | ((1 if eff else 0) << 31) | ((1 if rtr else 0) << 30)


def get_id(raw_id):
    return raw_id & 0x1FFFFFFF


def is_eff(raw_id):
    return bool(raw_id & CAN_EFF_FLAG)


DM4310_PMAX = 12.566
DM4310_VMAX = 30.0
DM4310_TMAX = 10.0

CAN_ID = 0x02
MST_ID = 0x12
MIT_MODE_OFFSET = 0x000


def float_to_uint(x, xmin, xmax, bits):
    x = max(xmin, min(x, xmax))
    return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))


def uint_to_float(x, xmin, xmax, bits):
    return (float(x) / ((1 << bits) - 1)) * (xmax - xmin) + xmin


def build_mit_frame(kp, kd, q, dq, tau):
    kp_uint = float_to_uint(kp, 0, 500, 12)
    kd_uint = float_to_uint(kd, 0, 5, 12)
    q_uint = float_to_uint(q, -DM4310_PMAX, DM4310_PMAX, 16)
    dq_uint = float_to_uint(dq, -DM4310_VMAX, DM4310_VMAX, 12)
    tau_uint = float_to_uint(tau, -DM4310_TMAX, DM4310_TMAX, 12)
    data = [0] * 8
    data[0] = (q_uint >> 8) & 0xFF
    data[1] = q_uint & 0xFF
    data[2] = dq_uint >> 4
    data[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
    data[4] = kp_uint & 0xFF
    data[5] = kd_uint >> 4
    data[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
    data[7] = tau_uint & 0xFF
    return data


def decode_feedback(data_bytes):
    q_uint = (data_bytes[1] << 8) | data_bytes[2]
    dq_uint = (data_bytes[3] << 4) | (data_bytes[4] >> 4)
    tau_uint = ((data_bytes[4] & 0xF) << 8) | data_bytes[5]
    q = uint_to_float(q_uint, -DM4310_PMAX, DM4310_PMAX, 16)
    dq = uint_to_float(dq_uint, -DM4310_VMAX, DM4310_VMAX, 12)
    tau = uint_to_float(tau_uint, -DM4310_TMAX, DM4310_TMAX, 12)
    return q, dq, tau


def scan_usb_devices():
    print("=" * 60)
    print("[1] USB 设备扫描")
    print("=" * 60)
    found = False
    base = "/sys/bus/usb/devices"
    if not os.path.isdir(base):
        print("  ERROR: /sys/bus/usb/devices 不存在")
        return False
    for entry in sorted(os.listdir(base)):
        dev_path = os.path.join(base, entry)
        vid_path = os.path.join(dev_path, "idVendor")
        pid_path = os.path.join(dev_path, "idProduct")
        if not os.path.isfile(vid_path):
            continue
        with open(vid_path) as f:
            vid = f.read().strip()
        with open(pid_path) as f:
            pid = f.read().strip()
        product = ""
        product_path = os.path.join(dev_path, "product")
        if os.path.isfile(product_path):
            with open(product_path) as f:
                product = f.read().strip()
        serial = ""
        serial_path = os.path.join(dev_path, "serial")
        if os.path.isfile(serial_path):
            with open(serial_path) as f:
                serial = f.read().strip()
        is_target = (vid == CANFD_ANALYZER_VID and pid == CANFD_ANALYZER_PID)
        marker = " <<<< CANFD ANALYZER!" if is_target else ""
        if product or is_target:
            print(f"  VID=0x{vid} PID=0x{pid}  {product}  SN={serial}{marker}")
        if is_target:
            found = True
    if not found:
        print(f"  CANFD 分析仪 (VID=0x{CANFD_ANALYZER_VID}, PID=0x{CANFD_ANALYZER_PID}) 未找到!")
        return False
    print("  CANFD 分析仪已检测到!")
    return True


def check_udev():
    print("\n" + "=" * 60)
    print("[2] udev 规则检查")
    print("=" * 60)
    rules_dir = "/etc/udev/rules.d"
    found = False
    for fname in sorted(os.listdir(rules_dir)):
        if not fname.endswith(".rules"):
            continue
        fpath = os.path.join(rules_dir, fname)
        if not os.path.isfile(fpath):
            continue
        with open(fpath, errors="ignore") as f:
            for line in f:
                if CANFD_ANALYZER_VID in line.lower() and CANFD_ANALYZER_PID in line.lower() and "idVendor" in line:
                    print(f"  已找到规则: {fpath}")
                    print(f"    {line.strip()}")
                    found = True
                    break
    if not found:
        print(f"  未找到 CANFD 分析仪的 udev 规则!")
        print("  请执行:")
        print(f'    echo \'SUBSYSTEM=="usb", ATTR{{idVendor}}=="{CANFD_ANALYZER_VID}", ATTR{{idProduct}}=="{CANFD_ANALYZER_PID}", MODE="0666"\' | sudo tee /etc/udev/rules.d/99-canfd.rules')
        print("    sudo udevadm control --reload-rules && sudo udevadm trigger")
        return False
    return True


def find_lib_path():
    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "libcontrolcanfd.so"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
        if os.path.isdir(path):
            so = os.path.join(path, "libcontrolcanfd.so")
            if os.path.isfile(so):
                return so
    return None


def setup_lib():
    lib_path = find_lib_path()
    if lib_path is None:
        print("  ERROR: 找不到 libcontrolcanfd.so!")
        sys.exit(1)
    print(f"  库路径: {lib_path}")
    try:
        lib = cdll.LoadLibrary(lib_path)
    except OSError as e:
        print(f"  ERROR: 加载 .so 失败: {e}")
        sys.exit(1)
    print("  加载成功!")

    lib.ZCAN_OpenDevice.restype = c_void_p
    lib.ZCAN_OpenDevice.argtypes = (c_uint, c_uint, c_uint)
    lib.ZCAN_CloseDevice.restype = c_uint
    lib.ZCAN_CloseDevice.argtypes = (c_void_p,)
    lib.ZCAN_GetDeviceInf.restype = c_uint
    lib.ZCAN_GetDeviceInf.argtypes = (c_void_p, c_void_p)
    lib.ZCAN_SetAbitBaud.restype = c_uint
    lib.ZCAN_SetAbitBaud.argtypes = (c_void_p, c_uint, c_uint)
    lib.ZCAN_SetDbitBaud.restype = c_uint
    lib.ZCAN_SetDbitBaud.argtypes = (c_void_p, c_uint, c_uint)
    lib.ZCAN_SetCANFDStandard.restype = c_uint
    lib.ZCAN_SetCANFDStandard.argtypes = (c_void_p, c_uint, c_uint)
    lib.ZCAN_InitCAN.restype = c_void_p
    lib.ZCAN_InitCAN.argtypes = (c_void_p, c_uint, c_void_p)
    lib.ZCAN_StartCAN.restype = c_uint
    lib.ZCAN_StartCAN.argtypes = (c_void_p,)
    lib.ZCAN_ResetCAN.restype = c_uint
    lib.ZCAN_ResetCAN.argtypes = (c_void_p,)
    lib.ZCAN_ClearBuffer.restype = c_uint
    lib.ZCAN_ClearBuffer.argtypes = (c_void_p,)
    lib.ZCAN_Transmit.restype = c_uint
    lib.ZCAN_Transmit.argtypes = (c_void_p, c_void_p, c_uint)
    lib.ZCAN_TransmitFD.restype = c_uint
    lib.ZCAN_TransmitFD.argtypes = (c_void_p, c_void_p, c_uint)
    lib.ZCAN_GetReceiveNum.restype = c_int
    lib.ZCAN_GetReceiveNum.argtypes = (c_void_p, c_ubyte)
    lib.ZCAN_Receive.restype = c_uint
    lib.ZCAN_Receive.argtypes = (c_void_p, c_void_p, c_uint, c_int)
    lib.ZCAN_ReceiveFD.restype = c_uint
    lib.ZCAN_ReceiveFD.argtypes = (c_void_p, c_void_p, c_uint, c_int)
    lib.ZCAN_ClearFilter.restype = c_uint
    lib.ZCAN_ClearFilter.argtypes = (c_void_p,)
    lib.ZCAN_AckFilter.restype = c_uint
    lib.ZCAN_AckFilter.argtypes = (c_void_p,)
    lib.ZCAN_SetFilterMode.restype = c_uint
    lib.ZCAN_SetFilterMode.argtypes = (c_void_p, c_uint)
    lib.ZCAN_SetFilterStartID.restype = c_uint
    lib.ZCAN_SetFilterStartID.argtypes = (c_void_p, c_uint)
    lib.ZCAN_SetFilterEndID.restype = c_uint
    lib.ZCAN_SetFilterEndID.argtypes = (c_void_p, c_uint)
    lib.ZCAN_SetResistanceEnable.restype = c_uint
    lib.ZCAN_SetResistanceEnable.argtypes = (c_void_p, c_uint, c_uint)
    return lib


def open_device_and_channel(lib, channel_idx=0):
    print(f"\n{'=' * 60}")
    print(f"[4] 初始化 CAN-FD 通道 {channel_idx}")
    print(f"{'=' * 60}")

    dev_handle = lib.ZCAN_OpenDevice(USBCAN2, 0, 0)
    if dev_handle == INVALID_DEVICE_HANDLE:
        print("  ERROR: ZCAN_OpenDevice 失败!")
        return None, None
    print(f"  设备打开成功, handle=0x{dev_handle:x}")

    dev_info = ZCAN_DEVICE_INFO()
    ret = lib.ZCAN_GetDeviceInf(dev_handle, byref(dev_info))
    if ret == STATUS_OK:
        sn_bytes = bytes(dev_info.str_Serial_Num).rstrip(b'\x00')
        hw_bytes = bytes(dev_info.str_hw_Type).rstrip(b'\x00')
        print(f"  设备信息:")
        print(f"    HW版本: {dev_info.hw_Version}")
        print(f"    FW版本: {dev_info.fw_Version}")
        print(f"    DR版本: {dev_info.dr_Version}")
        print(f"    CAN通道数: {dev_info.can_Num}")
        print(f"    序列号: {sn_bytes.decode('ascii', errors='replace')}")
        print(f"    硬件类型: {hw_bytes.decode('ascii', errors='replace')}")

    ret = lib.ZCAN_SetAbitBaud(dev_handle, channel_idx, 1000000)
    if ret != STATUS_OK:
        print(f"  ERROR: 设置仲裁域 1M 失败!")
        lib.ZCAN_CloseDevice(dev_handle)
        return None, None
    print(f"  仲裁域波特率: 1 Mbps OK")

    ret = lib.ZCAN_SetDbitBaud(dev_handle, channel_idx, 5000000)
    if ret != STATUS_OK:
        print(f"  ERROR: 设置数据域 5M 失败!")
        lib.ZCAN_CloseDevice(dev_handle)
        return None, None
    print(f"  数据域波特率: 5 Mbps OK")

    ret = lib.ZCAN_SetCANFDStandard(dev_handle, channel_idx, 0)
    if ret != STATUS_OK:
        print(f"  ERROR: 设置 ISO CAN-FD 失败!")
        lib.ZCAN_CloseDevice(dev_handle)
        return None, None
    print(f"  CAN-FD 标准: ISO OK")

    init_config = ZCAN_CHANNEL_INIT_CONFIG()
    init_config.can_type = TYPE_CANFD
    init_config.config.canfd.mode = 0

    ch_handle = lib.ZCAN_InitCAN(dev_handle, channel_idx, byref(init_config))
    if ch_handle == INVALID_CHANNEL_HANDLE:
        print(f"  ERROR: ZCAN_InitCAN 失败!")
        lib.ZCAN_CloseDevice(dev_handle)
        return None, None
    print(f"  通道初始化成功, handle=0x{ch_handle:x}")

    ret = lib.ZCAN_StartCAN(ch_handle)
    if ret != STATUS_OK:
        print(f"  ERROR: ZCAN_StartCAN 失败!")
        lib.ZCAN_CloseDevice(dev_handle)
        return None, None
    print(f"  通道启动成功!")

    return dev_handle, ch_handle


def send_canfd_frame(lib, ch_handle, can_id, data_bytes, brs=0):
    msg = ZCAN_TransmitFD_Data()
    msg.transmit_type = 0
    msg.frame.can_id = make_can_id(can_id, eff=0, rtr=0)
    msg.frame.len = len(data_bytes)
    msg.frame.flags = (1 if brs else 0)
    for i in range(len(data_bytes)):
        msg.frame.data[i] = data_bytes[i]
    return lib.ZCAN_TransmitFD(ch_handle, byref(msg), 1)


def send_can_frame(lib, ch_handle, can_id, data_bytes):
    msg = ZCAN_Transmit_Data()
    msg.transmit_type = 0
    msg.frame.can_id = make_can_id(can_id, eff=0, rtr=0)
    msg.frame.can_dlc = len(data_bytes)
    for i in range(len(data_bytes)):
        msg.frame.data[i] = data_bytes[i]
    return lib.ZCAN_Transmit(ch_handle, byref(msg), 1)


def recv_all_frames(lib, ch_handle, timeout_ms=200):
    frames = []

    num_can = lib.ZCAN_GetReceiveNum(ch_handle, TYPE_CAN)
    if num_can > 0:
        msgs = (ZCAN_Receive_Data * num_can)()
        count = lib.ZCAN_Receive(ch_handle, byref(msgs), num_can, timeout_ms)
        for i in range(count):
            frame = msgs[i].frame
            raw_id = frame.can_id
            data = bytes(frame.data[:frame.can_dlc])
            frames.append({
                "type": "CAN",
                "id": get_id(raw_id),
                "eff": is_eff(raw_id),
                "len": frame.can_dlc,
                "data": data,
                "ts": msgs[i].timestamp,
            })

    num_fd = lib.ZCAN_GetReceiveNum(ch_handle, TYPE_CANFD)
    if num_fd > 0:
        msgs = (ZCAN_ReceiveFD_Data * num_fd)()
        count = lib.ZCAN_ReceiveFD(ch_handle, byref(msgs), num_fd, timeout_ms)
        for i in range(count):
            frame = msgs[i].frame
            raw_id = frame.can_id
            data = bytes(frame.data[:frame.len])
            frames.append({
                "type": "CANFD",
                "id": get_id(raw_id),
                "eff": is_eff(raw_id),
                "len": frame.len,
                "brs": bool(frame.flags & 0x01),
                "data": data,
                "ts": msgs[i].timestamp,
            })

    return frames


def print_frame(f):
    data_hex = " ".join(f"{b:02X}" for b in f["data"])
    eff_str = "EFF" if f.get("eff") else "SFF"
    print(f"    [{f['type']}] ID=0x{f['id']:03X} {eff_str} DLC={f['len']} data=[{data_hex}]")


def motor_enable(lib, ch_handle):
    print("\n  [使能] 发送 0xFC x5 (CAN-FD 帧)...")
    enable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    for i in range(5):
        send_canfd_frame(lib, ch_handle, CAN_ID, enable_data)
        time.sleep(0.005)
    print("  [使能] 发送完成")


def motor_disable(lib, ch_handle):
    print("\n  [去使能] 发送 0xFD x5...")
    disable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    for i in range(5):
        send_canfd_frame(lib, ch_handle, CAN_ID, disable_data)
        time.sleep(0.005)


def test_motor_communication(lib, ch_handle):
    print(f"\n{'=' * 60}")
    print("[5] 电机通信测试")
    print(f"{'=' * 60}")
    print(f"  电机参数: CAN_ID=0x{CAN_ID:02X}, MST_ID=0x{MST_ID:02X}")
    print(f"  电机型号: DM-J4310-2EC, 控制模式: MIT")

    print("\n  === 阶段 A: 监听总线 (无发送, 3秒) ===")
    print("  (检查电机是否自发发送任何帧)")
    time.sleep(0.5)
    frames = recv_all_frames(lib, ch_handle, timeout_ms=3000)
    if frames:
        print(f"  收到 {len(frames)} 帧:")
        for f in frames[:20]:
            print_frame(f)
    else:
        print("  总线上无任何流量 (电机未发送或 CAN 总线未连通)")

    print("\n  === 阶段 B: 使能电机并监听反馈 ===")
    motor_enable(lib, ch_handle)
    time.sleep(0.3)
    frames = recv_all_frames(lib, ch_handle, timeout_ms=1000)
    feedback_found = False
    if frames:
        print(f"  收到 {len(frames)} 帧:")
        for f in frames[:20]:
            print_frame(f)
            if f["id"] == MST_ID and f["len"] >= 6:
                q, dq, tau = decode_feedback(f["data"])
                print(f"      -> 解码: pos={q:.4f} rad, vel={dq:.4f} rad/s, tau={tau:.4f} N·m")
                feedback_found = True
    else:
        print("  使能后无反馈帧")

    if not feedback_found:
        print("\n  === 阶段 C: 发送 MIT 控制帧并监听 ===")
        kp, kd, q_des, dq_des, tau_ff = 5.0, 0.3, 0.5, 0.0, 0.0
        mit_data = build_mit_frame(kp, kd, q_des, dq_des, tau_ff)
        for _ in range(30):
            send_canfd_frame(lib, ch_handle, CAN_ID, bytes(mit_data))
            time.sleep(0.01)
        time.sleep(0.2)
        frames = recv_all_frames(lib, ch_handle, timeout_ms=1000)
        if frames:
            print(f"  收到 {len(frames)} 帧:")
            for f in frames[:20]:
                print_frame(f)
                if f["id"] == MST_ID and f["len"] >= 6:
                    q, dq, tau = decode_feedback(f["data"])
                    print(f"      -> 解码: pos={q:.4f} rad, vel={dq:.4f} rad/s, tau={tau:.4f} N·m")
                    feedback_found = True
        else:
            print("  发送控制帧后仍无反馈")

    if not feedback_found:
        print("\n  === 阶段 D: 尝试经典 CAN 帧发送 ===")
        enable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        for i in range(5):
            send_can_frame(lib, ch_handle, CAN_ID, enable_data)
            time.sleep(0.005)
        time.sleep(0.3)

        kp, kd, q_des = 5.0, 0.3, 0.5
        mit_data = build_mit_frame(kp, kd, q_des, 0.0, 0.0)
        for _ in range(30):
            send_can_frame(lib, ch_handle, CAN_ID, bytes(mit_data))
            time.sleep(0.01)
        time.sleep(0.2)
        frames = recv_all_frames(lib, ch_handle, timeout_ms=1000)
        if frames:
            print(f"  收到 {len(frames)} 帧:")
            for f in frames[:20]:
                print_frame(f)
                if f["id"] == MST_ID and f["len"] >= 6:
                    q, dq, tau = decode_feedback(f["data"])
                    print(f"      -> 解码: pos={q:.4f} rad, vel={dq:.4f} rad/s, tau={tau:.4f} N·m")
                    feedback_found = True
        else:
            print("  经典 CAN 帧也无反馈")

    if not feedback_found:
        print("\n  === 阶段 E: 尝试扫描多个 CAN ID ===")
        for test_id in range(1, 16):
            enable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
            send_canfd_frame(lib, ch_handle, test_id, enable_data)
            time.sleep(0.005)
        time.sleep(0.5)
        frames = recv_all_frames(lib, ch_handle, timeout_ms=500)
        if frames:
            print(f"  收到 {len(frames)} 帧:")
            for f in frames[:20]:
                print_frame(f)
                if f["len"] >= 6:
                    feedback_found = True
        else:
            print("  扫描多个 ID 仍无反馈")

    motor_disable(lib, ch_handle)

    if feedback_found:
        print("\n  *** 通信测试通过! ***")
        return True
    else:
        print("\n  *** 通信测试失败: 未收到电机任何反馈 ***")
        print("  请检查:")
        print("    1. 电机 CAN_H 接 CANFD 分析仪 CAN_H (通常红线/白线)")
        print("    2. 电机 CAN_L 接 CANFD 分析仪 CAN_L (通常黑线/蓝线)")
        print("    3. CAN 总线末端有 120Ω 终端电阻")
        print("    4. 电机供电正常 (24V DC)")
        print("    5. 分析仪上选择的是正确的通道 (通道 0)")
        return False


def main():
    print("=" * 60)
    print(" OmniGripper CAN-FD 通信测试 v2.0")
    print(" CANFD 分析仪 + DM-J4310-2EC 电机")
    print("=" * 60)
    print(f"Python: {sys.version}")
    print(f"平台: {sys.platform}")

    print(f"\n[0] 环境检查")
    print(f"  conda env: {os.environ.get('CONDA_DEFAULT_ENV', 'N/A')}")
    print(f"  Python: {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")

    usb_ok = scan_usb_devices()
    udev_ok = check_udev()

    print(f"\n{'=' * 60}")
    print("[3] 加载 libcontrolcanfd.so")
    print(f"{'=' * 60}")
    lib = setup_lib()

    if not usb_ok:
        print("\nCANFD 分析仪未检测到，无法继续。")
        sys.exit(1)

    dev_handle, ch_handle = open_device_and_channel(lib, 0)
    if dev_handle is None:
        print("\n设备初始化失败，退出。")
        sys.exit(1)

    comm_ok = False
    try:
        comm_ok = test_motor_communication(lib, ch_handle)
    except KeyboardInterrupt:
        print("\n\n  用户中断")
    finally:
        print(f"\n{'=' * 60}")
        print("[6] 清理")
        print(f"{'=' * 60}")
        motor_disable(lib, ch_handle)
        lib.ZCAN_ResetCAN(ch_handle)
        lib.ZCAN_CloseDevice(dev_handle)
        print("  设备已关闭")

    print(f"\n{'=' * 60}")
    print("测试结果")
    print(f"{'=' * 60}")
    print(f"  USB 设备:     {'PASS' if usb_ok else 'FAIL'}")
    print(f"  udev 规则:    {'PASS' if udev_ok else 'MISSING'}")
    print(f"  CAN-FD 通信:  {'PASS' if comm_ok else 'FAIL'}")
    if comm_ok:
        print("\n  通信链路已验证通过! 可以开始开发 ROS 2 控制节点。")
    else:
        print("\n  通信链路未通过，请检查硬件连接后重试。")


if __name__ == "__main__":
    main()

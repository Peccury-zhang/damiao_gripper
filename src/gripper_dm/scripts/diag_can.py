#!/usr/bin/env python3
import os
import sys
import time
import ctypes
from ctypes import (
    cdll, c_uint, c_int, c_ubyte, c_ushort, c_ulonglong,
    c_void_p, byref, Structure, Union
)

LIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libcontrolcanfd.so")
USBCAN2 = 41
STATUS_OK = 1
TYPE_CANFD = 1

CAN_EFF_FLAG = 0x80000000
DM4310_PMAX = 12.566
DM4310_VMAX = 30.0
DM4310_TMAX = 10.0
CAN_ID = 0x01
MST_ID = 0x11


class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint), ("len", c_ubyte), ("flags", c_ubyte),
        ("__res0", c_ubyte), ("__res1", c_ubyte), ("data", c_ubyte * 64),
    ]


class _CAN_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint), ("acc_mask", c_uint), ("reserved", c_uint),
                 ("filter", c_ubyte), ("timing0", c_ubyte), ("timing1", c_ubyte), ("mode", c_ubyte)]


class _CANFD_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint), ("acc_mask", c_uint), ("abit_timing", c_uint),
                 ("dbit_timing", c_uint), ("brp", c_uint), ("filter", c_ubyte),
                 ("mode", c_ubyte), ("pad", c_ushort), ("reserved", c_uint)]


class _CONFIG_UNION(Union):
    _fields_ = [("can", _CAN_INIT_CONFIG), ("canfd", _CANFD_INIT_CONFIG)]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint), ("config", _CONFIG_UNION)]


class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [("hw_Version", c_ushort), ("fw_Version", c_ushort), ("dr_Version", c_ushort),
                 ("in_Version", c_ushort), ("irq_Num", c_ushort), ("can_Num", c_ubyte),
                 ("str_Serial_Num", c_ubyte * 21), ("str_hw_Type", c_ubyte * 40),
                 ("reserved", c_ushort * 4)]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


def uint_to_float(x, xmin, xmax, bits):
    return (float(x) / ((1 << bits) - 1)) * (xmax - xmin) + xmin


def float_to_uint(x, xmin, xmax, bits):
    x = max(xmin, min(x, xmax))
    return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))


def make_can_id(id_val, eff=0):
    return id_val | ((1 if eff else 0) << 31)


def get_id(raw_id):
    return raw_id & 0x1FFFFFFF


def decode_feedback(data_bytes):
    q_uint = (data_bytes[1] << 8) | data_bytes[2]
    dq_uint = (data_bytes[3] << 4) | (data_bytes[4] >> 4)
    tau_uint = ((data_bytes[4] & 0xF) << 8) | data_bytes[5]
    return (uint_to_float(q_uint, -DM4310_PMAX, DM4310_PMAX, 16),
            uint_to_float(dq_uint, -DM4310_VMAX, DM4310_VMAX, 12),
            uint_to_float(tau_uint, -DM4310_TMAX, DM4310_TMAX, 12))


def build_mit_frame(kp, kd, q, dq, tau):
    kp_u = float_to_uint(kp, 0, 500, 12)
    kd_u = float_to_uint(kd, 0, 5, 12)
    q_u = float_to_uint(q, -DM4310_PMAX, DM4310_PMAX, 16)
    dq_u = float_to_uint(dq, -DM4310_VMAX, DM4310_VMAX, 12)
    tau_u = float_to_uint(tau, -DM4310_TMAX, DM4310_TMAX, 12)
    d = [0]*8
    d[0] = (q_u >> 8) & 0xFF
    d[1] = q_u & 0xFF
    d[2] = dq_u >> 4
    d[3] = ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF)
    d[4] = kp_u & 0xFF
    d[5] = kd_u >> 4
    d[6] = ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF)
    d[7] = tau_u & 0xFF
    return bytes(d)


def fmt_frame(f):
    data_hex = " ".join(f"{b:02X}" for b in f["data"])
    eff = "EFF" if f.get("eff") else "SFF"
    return f"  ID=0x{f['id']:03X} {eff} DLC={f['len']} [{data_hex}]"


def send_fd(lib, ch, can_id, data_bytes, brs=0):
    msg = ZCAN_TransmitFD_Data()
    msg.transmit_type = 0
    msg.frame.can_id = make_can_id(can_id)
    msg.frame.len = len(data_bytes)
    msg.frame.flags = brs
    for i, b in enumerate(data_bytes):
        msg.frame.data[i] = b
    return lib.ZCAN_TransmitFD(ch, byref(msg), 1)


def recv_fd(lib, ch, timeout_ms=200):
    num = lib.ZCAN_GetReceiveNum(ch, TYPE_CANFD)
    if num <= 0:
        return []
    msgs = (ZCAN_ReceiveFD_Data * num)()
    count = lib.ZCAN_ReceiveFD(ch, byref(msgs), num, timeout_ms)
    frames = []
    for i in range(count):
        f = msgs[i].frame
        frames.append({
            "id": get_id(f.can_id),
            "eff": bool(f.can_id & CAN_EFF_FLAG),
            "len": f.len,
            "data": bytes(f.data[:f.len]),
            "ts": msgs[i].timestamp,
        })
    return frames


def drain(lib, ch):
    while True:
        frames = recv_fd(lib, ch, 10)
        if not frames:
            break


def main():
    print("CAN-FD 深度诊断工具 v1.0")
    print(f"CAN_ID=0x{CAN_ID:02X}, MST_ID=0x{MST_ID:02X}\n")

    lib = cdll.LoadLibrary(LIB_PATH)
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
    lib.ZCAN_TransmitFD.restype = c_uint
    lib.ZCAN_TransmitFD.argtypes = (c_void_p, c_void_p, c_uint)
    lib.ZCAN_GetReceiveNum.restype = c_int
    lib.ZCAN_GetReceiveNum.argtypes = (c_void_p, c_ubyte)
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

    dev = lib.ZCAN_OpenDevice(USBCAN2, 0, 0)
    if dev == 0:
        print("ZCAN_OpenDevice 失败"); sys.exit(1)

    info = ZCAN_DEVICE_INFO()
    lib.ZCAN_GetDeviceInf(dev, byref(info))
    null_byte = b'\x00'
    sn_str = bytes(info.str_Serial_Num).rstrip(null_byte).decode('ascii', errors='replace')
    hw_str = bytes(info.str_hw_Type).rstrip(null_byte).decode('ascii', errors='replace')
    print(f"SN={sn_str}, HW={hw_str}, 通道数={info.can_Num}")

    for ch_idx in range(min(info.can_Num, 2)):
        print(f"\n--- 设置通道 {ch_idx} ---")
        lib.ZCAN_SetAbitBaud(dev, ch_idx, 1000000)
        lib.ZCAN_SetDbitBaud(dev, ch_idx, 5000000)
        lib.ZCAN_SetCANFDStandard(dev, ch_idx, 0)

        r = lib.ZCAN_SetResistanceEnable(dev, ch_idx, 1)
        print(f"  120Ω 终端电阻: {'启用' if r == STATUS_OK else f'不支持(ret={r})'}")

    ch_idx = 0
    cfg = ZCAN_CHANNEL_INIT_CONFIG()
    cfg.can_type = TYPE_CANFD
    cfg.config.canfd.mode = 0
    ch = lib.ZCAN_InitCAN(dev, ch_idx, byref(cfg))
    if ch == 0:
        print("ZCAN_InitCAN 失败"); lib.ZCAN_CloseDevice(dev); sys.exit(1)

    lib.ZCAN_StartCAN(ch)
    print(f"通道 {ch_idx} 已启动")

    lib.ZCAN_ClearFilter(ch)
    lib.ZCAN_AckFilter(ch)
    lib.ZCAN_SetFilterMode(ch, 0)
    lib.ZCAN_SetFilterStartID(ch, 0x000)
    lib.ZCAN_SetFilterEndID(ch, 0x7FF)
    print("接收过滤器: 全 ID 范围 0x000~0x7FF")

    lib.ZCAN_ClearBuffer(ch)
    drain(lib, ch)

    print("\n=== 1. 被动监听总线 (5秒) ===")
    time.sleep(0.2)
    frames = recv_fd(lib, ch, 5000)
    if frames:
        print(f"  收到 {len(frames)} 帧:")
        for f in frames[:30]:
            print(fmt_frame(f))
    else:
        print("  总线无流量")

    print("\n=== 2. 切换电机到 MIT 模式 (写 RID=10, data=1) ===")
    id_low = CAN_ID & 0xFF
    id_high = (CAN_ID >> 8) & 0xFF
    switch_data = bytes([id_low, id_high, 0x55, 10, 1, 0, 0, 0])
    send_fd(lib, ch, 0x7FF, switch_data)
    time.sleep(0.05)
    frames = recv_fd(lib, ch, 500)
    print(f"  模式切换后收到 {len(frames)} 帧:")
    for f in frames[:10]:
        print(fmt_frame(f))
        if f["id"] == MST_ID and len(f["data"]) >= 8:
            d = f["data"]
            if d[2] == 0x55:
                rid = d[3]
                val = d[4] | (d[5] << 8) | (d[6] << 16) | (d[7] << 24)
                modes = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "POS_FORCE"}
                print(f"    -> RID={rid} 值={val} ({modes.get(val, '?')})")

    print("\n=== 3. 读电机当前模式 (读 RID=10) ===")
    read_data = bytes([id_low, id_high, 0x33, 10, 0, 0, 0, 0])
    send_fd(lib, ch, 0x7FF, read_data)
    time.sleep(0.05)
    frames = recv_fd(lib, ch, 500)
    print(f"  收到 {len(frames)} 帧:")
    for f in frames[:10]:
        print(fmt_frame(f))

    print("\n=== 4. 使能电机 (发 0xFC x5 到 ID=0x01) ===")
    lib.ZCAN_ClearBuffer(ch)
    drain(lib, ch)
    enable = bytes([0xFF]*7 + [0xFC])
    for _ in range(5):
        send_fd(lib, ch, CAN_ID, enable)
        time.sleep(0.01)
    time.sleep(0.5)
    frames = recv_fd(lib, ch, 1000)
    print(f"  使能后收到 {len(frames)} 帧:")
    got_fb = False
    for f in frames[:30]:
        marker = ""
        if f["id"] == MST_ID and f["len"] >= 6:
            q, dq, tau = decode_feedback(f["data"])
            marker = f"  >>> pos={q:.4f} vel={dq:.4f} tau={tau:.4f}"
            got_fb = True
        print(fmt_frame(f) + marker)

    if not got_fb:
        print("\n=== 5. 发送 MIT 控制 (q=0.5, kp=5, kd=0.3, 100帧) ===")
        lib.ZCAN_ClearBuffer(ch)
        drain(lib, ch)
        mit = build_mit_frame(5.0, 0.3, 0.5, 0.0, 0.0)
        for _ in range(100):
            send_fd(lib, ch, CAN_ID, mit)
            time.sleep(0.01)
        time.sleep(0.5)
        frames = recv_fd(lib, ch, 1000)
        print(f"  控制后收到 {len(frames)} 帧:")
        for f in frames[:50]:
            marker = ""
            if f["id"] == MST_ID and f["len"] >= 6:
                q, dq, tau = decode_feedback(f["data"])
                marker = f"  >>> pos={q:.4f} vel={dq:.4f} tau={tau:.4f}"
                got_fb = True
            print(fmt_frame(f) + marker)

    if not got_fb:
        print("\n=== 6. 尝试不同 MST_ID (0x10~0x20) 使能+监听 ===")
        lib.ZCAN_ClearBuffer(ch)
        drain(lib, ch)
        for _ in range(5):
            send_fd(lib, ch, CAN_ID, enable)
            time.sleep(0.01)
        mit = build_mit_frame(5.0, 0.3, 0.5, 0.0, 0.0)
        for _ in range(50):
            send_fd(lib, ch, CAN_ID, mit)
            time.sleep(0.01)
        time.sleep(1.0)
        frames = recv_fd(lib, ch, 2000)
        print(f"  收到 {len(frames)} 帧:")
        id_counts = {}
        for f in frames[:50]:
            print(fmt_frame(f))
            id_counts[f["id"]] = id_counts.get(f["id"], 0) + 1
        if len(frames) > 50:
            print(f"  ... 共 {len(frames)} 帧")
        print(f"\n  ID 统计: {dict(sorted(id_counts.items()))}")

    print("\n=== 7. 去使能 ===")
    disable = bytes([0xFF]*7 + [0xFD])
    for _ in range(5):
        send_fd(lib, ch, CAN_ID, disable)
        time.sleep(0.01)

    lib.ZCAN_ResetCAN(ch)
    lib.ZCAN_CloseDevice(dev)
    print("设备已关闭")


if __name__ == "__main__":
    main()

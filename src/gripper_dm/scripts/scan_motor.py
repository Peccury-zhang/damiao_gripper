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


def float_to_uint(x, xmin, xmax, bits):
    x = max(xmin, min(x, xmax))
    return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))


def uint_to_float(x, xmin, xmax, bits):
    return (float(x) / ((1 << bits) - 1)) * (xmax - xmin) + xmin


def make_can_id(id_val):
    return id_val & 0x7FF


def get_id(raw_id):
    return raw_id & 0x1FFFFFFF


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


def decode_feedback(data_bytes):
    q_uint = (data_bytes[1] << 8) | data_bytes[2]
    dq_uint = (data_bytes[3] << 4) | (data_bytes[4] >> 4)
    tau_uint = ((data_bytes[4] & 0xF) << 8) | data_bytes[5]
    return (uint_to_float(q_uint, -DM4310_PMAX, DM4310_PMAX, 16),
            uint_to_float(dq_uint, -DM4310_VMAX, DM4310_VMAX, 12),
            uint_to_float(tau_uint, -DM4310_TMAX, DM4310_TMAX, 12))


def fmt_frame(f):
    data_hex = " ".join(f"{b:02X}" for b in f["data"])
    eff = "EFF" if f.get("eff") else "SFF"
    return f"  ID=0x{f['id']:03X} {eff} DLC={f['len']} [{data_hex}]"


def setup_lib():
    lib = cdll.LoadLibrary(LIB_PATH)
    for name, rtype, atypes in [
        ("ZCAN_OpenDevice", c_void_p, (c_uint, c_uint, c_uint)),
        ("ZCAN_CloseDevice", c_uint, (c_void_p,)),
        ("ZCAN_GetDeviceInf", c_uint, (c_void_p, c_void_p)),
        ("ZCAN_SetAbitBaud", c_uint, (c_void_p, c_uint, c_uint)),
        ("ZCAN_SetDbitBaud", c_uint, (c_void_p, c_uint, c_uint)),
        ("ZCAN_SetCANFDStandard", c_uint, (c_void_p, c_uint, c_uint)),
        ("ZCAN_InitCAN", c_void_p, (c_void_p, c_uint, c_void_p)),
        ("ZCAN_StartCAN", c_uint, (c_void_p,)),
        ("ZCAN_ResetCAN", c_uint, (c_void_p,)),
        ("ZCAN_ClearBuffer", c_uint, (c_void_p,)),
        ("ZCAN_TransmitFD", c_uint, (c_void_p, c_void_p, c_uint)),
        ("ZCAN_GetReceiveNum", c_int, (c_void_p, c_ubyte)),
        ("ZCAN_ReceiveFD", c_uint, (c_void_p, c_void_p, c_uint, c_int)),
        ("ZCAN_ClearFilter", c_uint, (c_void_p,)),
        ("ZCAN_AckFilter", c_uint, (c_void_p,)),
        ("ZCAN_SetFilterMode", c_uint, (c_void_p, c_uint)),
        ("ZCAN_SetFilterStartID", c_uint, (c_void_p, c_uint)),
        ("ZCAN_SetFilterEndID", c_uint, (c_void_p, c_uint)),
        ("ZCAN_SetResistanceEnable", c_uint, (c_void_p, c_uint, c_uint)),
    ]:
        fn = getattr(lib, name)
        fn.restype = rtype
        fn.argtypes = atypes
    return lib


def send_fd(lib, ch, can_id, data_bytes, brs=0):
    msg = ZCAN_TransmitFD_Data()
    msg.transmit_type = 0
    msg.frame.can_id = make_can_id(can_id)
    msg.frame.len = len(data_bytes)
    msg.frame.flags = brs
    for i, b in enumerate(data_bytes):
        msg.frame.data[i] = b
    return lib.ZCAN_TransmitFD(ch, byref(msg), 1)


def recv_fd(lib, ch, timeout_ms=100):
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
        if not recv_fd(lib, ch, 5):
            break


def init_channel(lib, dev, ch_idx, abit, dbit):
    lib.ZCAN_SetAbitBaud(dev, ch_idx, abit)
    lib.ZCAN_SetDbitBaud(dev, ch_idx, dbit)
    lib.ZCAN_SetCANFDStandard(dev, ch_idx, 0)
    lib.ZCAN_SetResistanceEnable(dev, ch_idx, 1)

    cfg = ZCAN_CHANNEL_INIT_CONFIG()
    cfg.can_type = TYPE_CANFD
    cfg.config.canfd.mode = 0
    ch = lib.ZCAN_InitCAN(dev, ch_idx, byref(cfg))
    if ch == 0:
        return None
    lib.ZCAN_StartCAN(ch)
    lib.ZCAN_ClearFilter(ch)
    lib.ZCAN_AckFilter(ch)
    lib.ZCAN_SetFilterMode(ch, 0)
    lib.ZCAN_SetFilterStartID(ch, 0x000)
    lib.ZCAN_SetFilterEndID(ch, 0x7FF)
    lib.ZCAN_ClearBuffer(ch)
    return ch


def try_motor(lib, ch, can_id, verbose=False):
    drain(lib, ch)
    lib.ZCAN_ClearBuffer(ch)

    enable = bytes([0xFF]*7 + [0xFC])
    for _ in range(5):
        send_fd(lib, ch, can_id, enable)
        time.sleep(0.005)

    mit = build_mit_frame(5.0, 0.3, 0.5, 0.0, 0.0)
    for _ in range(20):
        send_fd(lib, ch, can_id, mit)
        time.sleep(0.005)

    time.sleep(0.2)
    frames = recv_fd(lib, ch, 300)

    real_frames = [f for f in frames if f["id"] != can_id]

    if verbose or real_frames:
        print(f"  CAN_ID=0x{can_id:03X}: 收到 {len(frames)} 帧 (过滤自身后 {len(real_frames)} 帧)")
        for f in frames[:10]:
            self_tag = " (SELF)" if f["id"] == can_id else ""
            print(f"    {fmt_frame(f)}{self_tag}")
            if f["id"] != can_id and f["len"] >= 6:
                q, dq, tau = decode_feedback(f["data"])
                print(f"      -> 解码: pos={q:.4f} vel={dq:.4f} tau={tau:.4f}")

    disable = bytes([0xFF]*7 + [0xFD])
    for _ in range(3):
        send_fd(lib, ch, can_id, disable)
        time.sleep(0.003)

    return len(real_frames) > 0


def scan_baud_config(lib, dev, ch_idx, abit, dbit, label):
    print(f"\n{'='*60}")
    print(f"配置: {label} (abit={abit}, dbit={dbit}, 通道={ch_idx})")
    print(f"{'='*60}")

    ch = init_channel(lib, dev, ch_idx, abit, dbit)
    if ch is None:
        print("  通道初始化失败!")
        return False

    found_any = False
    test_ids = list(range(0x01, 0x10)) + [0x11, 0x14, 0x15, 0x20, 0x28, 0x30, 0x40, 0x50, 0x64, 0x80, 0x100, 0x200]
    test_ids = sorted(set(test_ids))

    for can_id in test_ids:
        if try_motor(lib, ch, can_id, verbose=True):
            found_any = True
            print(f"  *** 在 CAN_ID=0x{can_id:03X} 发现电机响应! ***")

    if not found_any:
        print(f"\n  扫描 {len(test_ids)} 个 ID，未发现电机响应")

    lib.ZCAN_ResetCAN(ch)
    return found_any


def main():
    print("="*60)
    print(" OmniGripper 电机 CAN ID + 波特率 全面扫描")
    print("="*60)

    lib = setup_lib()

    dev = lib.ZCAN_OpenDevice(USBCAN2, 0, 0)
    if dev == 0:
        print("ZCAN_OpenDevice 失败!")
        sys.exit(1)

    info = ZCAN_DEVICE_INFO()
    lib.ZCAN_GetDeviceInf(dev, byref(info))
    null_byte = b'\x00'
    sn = bytes(info.str_Serial_Num).rstrip(null_byte).decode('ascii', errors='replace')
    print(f"设备: SN={sn}, 通道数={info.can_Num}")

    configs = [
        (1000000, 5000000, "CAN-FD: 1M仲裁 + 5M数据"),
        (5000000, 5000000, "CAN-FD: 5M仲裁 + 5M数据"),
        (1000000, 1000000, "CAN-FD: 1M仲裁 + 1M数据"),
        (500000, 500000, "CAN-FD: 500K仲裁 + 500K数据"),
    ]

    found = False
    for ch_idx in range(min(info.can_Num, 2)):
        for abit, dbit, label in configs:
            if scan_baud_config(lib, dev, ch_idx, abit, dbit, f"CH{ch_idx} {label}"):
                found = True

    lib.ZCAN_CloseDevice(dev)

    print(f"\n{'='*60}")
    print("扫描结果")
    print(f"{'='*60}")
    if found:
        print("  已发现电机响应! 请查看上方输出获取 CAN_ID 和波特率信息。")
    else:
        print("  所有配置和 ID 均未发现电机响应。")
        print("  可能原因:")
        print("    1. 电机未供电 (检查电源)")
        print("    2. CAN_H/CAN_L 未连接到分析仪")
        print("    3. 电机 CAN ID 不在扫描范围内")
        print("    4. 电机波特率不在测试列表中")
        print("    5. 电机需要先通过 UART 配置 CAN 参数")


if __name__ == "__main__":
    main()

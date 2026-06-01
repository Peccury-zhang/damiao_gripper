#!/usr/bin/env python3
import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ctypes import cdll, c_uint, c_void_p, c_ubyte, c_int, c_ulong, c_char, c_ushort, Structure, Union, byref

USBCAN2 = 41
STATUS_OK = 0
TYPE_CAN = 0
TYPE_CANFD = 1

PMAX = 12.566
VMAX = 30.0
TMAX = 10.0


def find_lib():
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


class ZCAN_CAN_FRAME(Structure):
    _fields_ = [("can_id", c_uint), ("can_dlc", c_ubyte), ("data", c_ubyte * 8)]

class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [("can_id", c_uint), ("len", c_ubyte), ("flags", c_ubyte),
                 ("reserved1", c_ubyte), ("reserved2", c_ubyte), ("data", c_ubyte * 64)]

class CAN_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint), ("acc_mask", c_uint), ("reserved", c_uint * 5)]

class CANFD_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint), ("acc_mask", c_uint), ("abit_timing", c_uint),
                 ("dbit_timing", c_uint), ("brp", c_uint), ("canfd_standard", c_ubyte),
                 ("mode", c_ubyte), ("reserved", c_ubyte * 18)]

class CAN_CONFIG_UNION(Union):
    _fields_ = [("can", CAN_CONFIG), ("canfd", CANFD_CONFIG)]

class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint), ("config", CAN_CONFIG_UNION)]

class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [("hw_Version", c_ushort), ("fw_Version", c_ushort), ("dr_Version", c_ushort),
                 ("in_Version", c_ushort), ("irq_Num", c_ushort), ("can_Num", c_ubyte),
                 ("str_Serial_Num", c_char * 20), ("str_hw_Type", c_char * 40),
                 ("reserved", c_ushort * 4)]

class ZCAN_TRANSMITFD_FRAME(Structure):
    _fields_ = [("can_id", c_uint), ("transmit_type", c_ubyte), ("remote_flag", c_ubyte),
                 ("ext_flag", c_ubyte), ("reserved", c_ubyte), ("len", c_ubyte),
                 ("flags", c_ubyte), ("reserved2", c_ubyte * 2), ("data", c_ubyte * 64)]

class ZCAN_TRANSMIT_FRAME(Structure):
    _fields_ = [("can_id", c_uint), ("transmit_type", c_ubyte), ("remote_flag", c_ubyte),
                 ("ext_flag", c_ubyte), ("reserved", c_ubyte), ("can_dlc", c_ubyte),
                 ("data", c_ubyte * 8)]

class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_TRANSMITFD_FRAME), ("transmit_type", c_uint)]

class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_TRANSMIT_FRAME), ("transmit_type", c_uint)]

class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulong)]

class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulong)]


def make_can_id(can_id, eff=0, rtr=0):
    raw = can_id & 0x1FFFFFFF
    if rtr: raw |= 1 << 30
    if eff: raw |= 1 << 31
    return raw


def get_id(raw_id):
    return raw_id & 0x1FFFFFFF


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    if span == 0: return 0
    return int((x - x_min) / span * ((1 << bits) - 1))


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return x_int * span / ((1 << bits) - 1) + x_min


def build_mit_frame(q_des, dq_des, kp, kd, tau_ff):
    q_uint = float_to_uint(q_des, -PMAX, PMAX, 16)
    dq_uint = float_to_uint(dq_des, -VMAX, VMAX, 12)
    kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = float_to_uint(kd, 0.0, 5.0, 12)
    tau_uint = float_to_uint(tau_ff, -TMAX, TMAX, 12)
    data = bytearray(8)
    data[0] = (q_uint >> 8) & 0xFF
    data[1] = q_uint & 0xFF
    data[2] = (dq_uint >> 4) & 0xFF
    data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
    data[4] = kp_uint & 0xFF
    data[5] = (kd_uint >> 4) & 0xFF
    data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
    data[7] = tau_uint & 0xFF
    return bytes(data)


def decode_feedback(data):
    if len(data) < 8:
        return None
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    q = uint_to_float(q_uint, -PMAX, PMAX, 16)
    dq = uint_to_float(dq_uint, -VMAX, VMAX, 12)
    tau = uint_to_float(tau_uint, -TMAX, TMAX, 12)
    return q, dq, tau, data[6], data[7]


def main():
    lib_path = find_lib()
    if not lib_path:
        print("ERROR: libcontrolcanfd.so not found")
        return
    lib = cdll.LoadLibrary(lib_path)

    lib.ZCAN_OpenDevice.restype = c_void_p
    lib.ZCAN_OpenDevice.argtypes = (c_uint, c_uint, c_uint)
    lib.ZCAN_CloseDevice.restype = c_uint
    lib.ZCAN_CloseDevice.argtypes = (c_void_p,)
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
    lib.ZCAN_Transmit.restype = c_uint
    lib.ZCAN_Transmit.argtypes = (c_void_p, c_void_p, c_uint)
    lib.ZCAN_GetReceiveNum.restype = c_int
    lib.ZCAN_GetReceiveNum.argtypes = (c_void_p, c_ubyte)
    lib.ZCAN_Receive.restype = c_uint
    lib.ZCAN_Receive.argtypes = (c_void_p, c_void_p, c_uint, c_int)
    lib.ZCAN_ReceiveFD.restype = c_uint
    lib.ZCAN_ReceiveFD.argtypes = (c_void_p, c_void_p, c_uint, c_int)
    lib.ZCAN_SetResistanceEnable.restype = c_uint
    lib.ZCAN_SetResistanceEnable.argtypes = (c_void_p, c_uint, c_uint)

    for channel in [0, 1]:
        print(f"\n{'='*60}")
        print(f"Testing channel {channel}")
        print(f"{'='*60}")

        dev = lib.ZCAN_OpenDevice(USBCAN2, 0, 0)
        if dev == 0:
            print("  OpenDevice failed")
            continue

        lib.ZCAN_SetAbitBaud(dev, channel, 1000000)
        lib.ZCAN_SetDbitBaud(dev, channel, 5000000)
        lib.ZCAN_SetCANFDStandard(dev, channel, 0)

        cfg = ZCAN_CHANNEL_INIT_CONFIG()
        cfg.can_type = TYPE_CANFD
        cfg.config.canfd.mode = 0

        ch = lib.ZCAN_InitCAN(dev, channel, byref(cfg))
        if ch == 0:
            print("  InitCAN failed")
            lib.ZCAN_CloseDevice(dev)
            continue

        lib.ZCAN_StartCAN(ch)
        lib.ZCAN_ClearBuffer(ch)
        lib.ZCAN_SetResistanceEnable(dev, channel, 1)
        print(f"  Channel {channel} ready")

        print(f"\n  [Test 1] Listen on channel {channel} for 2 seconds (passive)")
        time.sleep(0.5)
        num_fd = lib.ZCAN_GetReceiveNum(ch, TYPE_CANFD)
        num_can = lib.ZCAN_GetReceiveNum(ch, TYPE_CAN)
        if num_fd > 0:
            msgs = (ZCAN_ReceiveFD_Data * num_fd)()
            cnt = lib.ZCAN_ReceiveFD(ch, byref(msgs), num_fd, 2000)
            print(f"  CANFD frames: {cnt}")
            for i in range(cnt):
                f = msgs[i].frame
                data_hex = " ".join(f"{f.data[j]:02X}" for j in range(f.len))
                print(f"    ID=0x{get_id(f.can_id):03X} DLC={f.len} data=[{data_hex}]")
        if num_can > 0:
            msgs = (ZCAN_Receive_Data * num_can)()
            cnt = lib.ZCAN_Receive(ch, byref(msgs), num_can, 2000)
            print(f"  CAN frames: {cnt}")
            for i in range(cnt):
                f = msgs[i].frame
                data_hex = " ".join(f"{f.data[j]:02X}" for j in range(f.can_dlc))
                print(f"    ID=0x{get_id(f.can_id):03X} DLC={f.can_dlc} data=[{data_hex}]")
        if num_fd == 0 and num_can == 0:
            print("  No frames on bus")

        print(f"\n  [Test 2] Enable ID=0x01 + send MIT on channel {channel}")
        enable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        msg_fd = ZCAN_TransmitFD_Data()
        msg_fd.transmit_type = 0
        msg_fd.frame.can_id = make_can_id(0x01)
        msg_fd.frame.len = 8
        msg_fd.frame.flags = 1
        for i in range(8):
            msg_fd.frame.data[i] = enable_data[i]
        for _ in range(5):
            lib.ZCAN_TransmitFD(ch, byref(msg_fd), 1)
            time.sleep(0.005)

        mit_data = build_mit_frame(0.5, 0.0, 5.0, 0.3, 0.0)
        msg_mit = ZCAN_TransmitFD_Data()
        msg_mit.transmit_type = 0
        msg_mit.frame.can_id = make_can_id(0x01)
        msg_mit.frame.len = 8
        msg_mit.frame.flags = 1
        for i in range(8):
            msg_mit.frame.data[i] = mit_data[i]
        for _ in range(30):
            lib.ZCAN_TransmitFD(ch, byref(msg_mit), 1)
            time.sleep(0.01)

        time.sleep(0.3)
        num_fd = lib.ZCAN_GetReceiveNum(ch, TYPE_CANFD)
        num_can = lib.ZCAN_GetReceiveNum(ch, TYPE_CAN)
        print(f"  After enable+MIT on ID=0x01: CANFD={num_fd}, CAN={num_can}")
        for _type, _num in [(TYPE_CANFD, num_fd), (TYPE_CAN, num_can)]:
            if _num > 0:
                if _type == TYPE_CANFD:
                    msgs = (ZCAN_ReceiveFD_Data * _num)()
                    cnt = lib.ZCAN_ReceiveFD(ch, byref(msgs), _num, 500)
                else:
                    msgs = (ZCAN_Receive_Data * _num)()
                    cnt = lib.ZCAN_Receive(ch, byref(msgs), _num, 500)
                for i in range(cnt):
                    if _type == TYPE_CANFD:
                        f = msgs[i].frame
                        data_hex = " ".join(f"{f.data[j]:02X}" for j in range(f.len))
                        fb = decode_feedback(bytes(f.data[:f.len]))
                    else:
                        f = msgs[i].frame
                        data_hex = " ".join(f"{f.data[j]:02X}" for j in range(f.can_dlc))
                        fb = decode_feedback(bytes(f.data[:f.can_dlc]))
                    fb_str = ""
                    if fb:
                        fb_str = f" -> pos={fb[0]:.4f} vel={fb[1]:.4f} tau={fb[2]:.4f}"
                    print(f"    ID=0x{get_id(f.can_id):03X} data=[{data_hex}]{fb_str}")

        lib.ZCAN_ClearBuffer(ch)

        print(f"\n  [Test 3] Scan CAN IDs 0x01-0x20 one-by-one on channel {channel}")
        for test_id in range(1, 33):
            lib.ZCAN_ClearBuffer(ch)

            msg_e = ZCAN_TransmitFD_Data()
            msg_e.transmit_type = 0
            msg_e.frame.can_id = make_can_id(test_id)
            msg_e.frame.len = 8
            msg_e.frame.flags = 1
            for i in range(8):
                msg_e.frame.data[i] = enable_data[i]
            for _ in range(5):
                lib.ZCAN_TransmitFD(ch, byref(msg_e), 1)
                time.sleep(0.003)

            mit = build_mit_frame(0.5, 0.0, 5.0, 0.3, 0.0)
            msg_m = ZCAN_TransmitFD_Data()
            msg_m.transmit_type = 0
            msg_m.frame.can_id = make_can_id(test_id)
            msg_m.frame.len = 8
            msg_m.frame.flags = 1
            for i in range(8):
                msg_m.frame.data[i] = mit[i]
            for _ in range(10):
                lib.ZCAN_TransmitFD(ch, byref(msg_m), 1)
                time.sleep(0.005)

            time.sleep(0.15)

            nfd = lib.ZCAN_GetReceiveNum(ch, TYPE_CANFD)
            ncan = lib.ZCAN_GetReceiveNum(ch, TYPE_CAN)
            if nfd > 0 or ncan > 0:
                print(f"  >>> ID=0x{test_id:02X}: CANFD={nfd}, CAN={ncan} <<< RESPONSE!")
                if nfd > 0:
                    msgs = (ZCAN_ReceiveFD_Data * nfd)()
                    cnt = lib.ZCAN_ReceiveFD(ch, byref(msgs), nfd, 200)
                    for i in range(min(cnt, 3)):
                        f = msgs[i].frame
                        data_hex = " ".join(f"{f.data[j]:02X}" for j in range(f.len))
                        fb = decode_feedback(bytes(f.data[:f.len]))
                        fb_str = f" -> pos={fb[0]:.4f} vel={fb[1]:.4f} tau={fb[2]:.4f}" if fb else ""
                        print(f"      CANFD: ID=0x{get_id(f.can_id):03X} [{data_hex}]{fb_str}")
                if ncan > 0:
                    msgs = (ZCAN_Receive_Data * ncan)()
                    cnt = lib.ZCAN_Receive(ch, byref(msgs), ncan, 200)
                    for i in range(min(cnt, 3)):
                        f = msgs[i].frame
                        data_hex = " ".join(f"{f.data[j]:02X}" for j in range(f.can_dlc))
                        fb = decode_feedback(bytes(f.data[:f.can_dlc]))
                        fb_str = f" -> pos={fb[0]:.4f} vel={fb[1]:.4f} tau={fb[2]:.4f}" if fb else ""
                        print(f"      CAN: ID=0x{get_id(f.can_id):03X} [{data_hex}]{fb_str}")
            else:
                if test_id % 8 == 0:
                    print(f"  ID 0x01-0x{test_id:02X}: no response")

        disable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        msg_d = ZCAN_TransmitFD_Data()
        msg_d.transmit_type = 0
        msg_d.frame.can_id = make_can_id(0x01)
        msg_d.frame.len = 8
        msg_d.frame.flags = 1
        for i in range(8):
            msg_d.frame.data[i] = disable_data[i]
        for _ in range(5):
            lib.ZCAN_TransmitFD(ch, byref(msg_d), 1)
            time.sleep(0.003)

        lib.ZCAN_ResetCAN(ch)
        lib.ZCAN_CloseDevice(dev)
        print(f"\n  Channel {channel} done")

    print("\n" + "="*60)
    print("Scan complete")


if __name__ == "__main__":
    main()

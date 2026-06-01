import os
import logging
from ctypes import (
    cdll, c_uint, c_int, c_void_p, c_ubyte, c_ulong,
    c_char, c_ushort, Structure, Union, byref, POINTER, sizeof
)

logger = logging.getLogger(__name__)

USBCAN2 = 41
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0
STATUS_OK = 1
TYPE_CAN = 0
TYPE_CANFD = 1
TYPE_CANFD_EXT = 0x02
TYPE_DATA = 0x03
TYPE_AUTO = 0x05

CANFD_MAX_DLC = 15

DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def _len_to_dlc(length):
    for dlc, l in enumerate(DLC_TO_LEN):
        if l >= length:
            return dlc
    return 15


def make_can_id(eff, rtr, can_id):
    raw = can_id & 0x1FFFFFFF
    if rtr:
        raw |= 1 << 30
    if eff:
        raw |= 1 << 31
    return raw


def get_id(raw_id):
    return raw_id & 0x1FFFFFFF


def is_eff(raw_id):
    return bool(raw_id & (1 << 31))


def is_rtr(raw_id):
    return bool(raw_id & (1 << 30))


class ZCAN_CAN_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),
        ("can_dlc", c_ubyte),
        ("data", c_ubyte * 8),
    ]


class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),
        ("len", c_ubyte),
        ("flags", c_ubyte),
        ("reserved1", c_ubyte),
        ("reserved2", c_ubyte),
        ("data", c_ubyte * 64),
    ]


class ZCAN_CANFD_FRAME_UNION(Union):
    _fields_ = [
        ("can", ZCAN_CAN_FRAME),
        ("canfd", ZCAN_CANFD_FRAME),
    ]


class CAN_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("reserved", c_uint * 5),
    ]


class CANFD_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("abit_timing", c_uint),
        ("dbit_timing", c_uint),
        ("brp", c_uint),
        ("canfd_standard", c_ubyte),
        ("mode", c_ubyte),
        ("reserved", c_ubyte * 18),
    ]


class CAN_CONFIG_UNION(Union):
    _fields_ = [
        ("can", CAN_CONFIG),
        ("canfd", CANFD_CONFIG),
    ]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [
        ("can_type", c_uint),
        ("config", CAN_CONFIG_UNION),
    ]


class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_char * 20),
        ("str_hw_Type", c_char * 40),
        ("reserved", c_ushort * 4),
    ]


class ZCAN_TRANSMIT_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),
        ("can_dlc", c_ubyte),
        ("data", c_ubyte * 8),
    ]


class ZCAN_TRANSMITFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint),
        ("len", c_ubyte),
        ("flags", c_ubyte),
        ("reserved1", c_ubyte),
        ("reserved2", c_ubyte),
        ("data", c_ubyte * 64),
    ]


class ZCAN_Transmit_Data(Structure):
    _fields_ = [
        ("frame", ZCAN_TRANSMIT_FRAME),
    ]


class ZCAN_Receive_Data(Structure):
    _fields_ = [
        ("frame", ZCAN_CAN_FRAME),
        ("timestamp", c_ulong),
    ]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [
        ("frame", ZCAN_TRANSMITFD_FRAME),
    ]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [
        ("frame", ZCAN_CANFD_FRAME),
        ("timestamp", c_ulong),
    ]


class CANFDDriver:
    def __init__(self, lib_path, device_type=USBCAN2, device_index=0):
        self._lib_path = lib_path
        self._device_type = device_type
        self._device_index = device_index
        self._lib = None
        self._dev_handle = INVALID_DEVICE_HANDLE
        self._ch_handle = INVALID_CHANNEL_HANDLE
        self._load_library()

    def _load_library(self):
        if not os.path.isfile(self._lib_path):
            raise FileNotFoundError(f"libcontrolcanfd.so not found: {self._lib_path}")
        self._lib = cdll.LoadLibrary(self._lib_path)
        self._setup_api()

    def _setup_api(self):
        lib = self._lib
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

    def open_device(self):
        if self._dev_handle != INVALID_DEVICE_HANDLE:
            raise RuntimeError("Device already open")
        handle = self._lib.ZCAN_OpenDevice(
            self._device_type, self._device_index, 0
        )
        if handle == INVALID_DEVICE_HANDLE:
            raise RuntimeError("ZCAN_OpenDevice failed")
        self._dev_handle = handle
        info = ZCAN_DEVICE_INFO()
        if self._lib.ZCAN_GetDeviceInf(self._dev_handle, byref(info)) == STATUS_OK:
            sn = bytes(info.str_Serial_Num).rstrip(b'\x00').decode('ascii', errors='replace')
            hw = bytes(info.str_hw_Type).rstrip(b'\x00').decode('ascii', errors='replace')
            logger.info(f"Device opened: SN={sn}, HW={hw}, channels={info.can_Num}")
        return self._dev_handle

    def init_channel(self, channel_index=0, abit_baud=1000000, dbit_baud=5000000,
                     canfd_standard=0, mode=0):
        if self._dev_handle == INVALID_DEVICE_HANDLE:
            raise RuntimeError("Device not open")

        steps = [
            ("SetAbitBaud", self._lib.ZCAN_SetAbitBaud, (self._dev_handle, channel_index, abit_baud)),
            ("SetDbitBaud", self._lib.ZCAN_SetDbitBaud, (self._dev_handle, channel_index, dbit_baud)),
            ("SetCANFDStandard", self._lib.ZCAN_SetCANFDStandard, (self._dev_handle, channel_index, canfd_standard)),
        ]
        for name, func, args in steps:
            ret = func(*args)
            if ret != STATUS_OK:
                raise RuntimeError(f"ZCAN_{name} failed (ret={ret})")

        init_config = ZCAN_CHANNEL_INIT_CONFIG()
        init_config.can_type = TYPE_CANFD
        init_config.config.canfd.mode = mode

        ch_handle = self._lib.ZCAN_InitCAN(
            self._dev_handle, channel_index, byref(init_config)
        )
        if ch_handle == INVALID_CHANNEL_HANDLE:
            raise RuntimeError("ZCAN_InitCAN failed")
        self._ch_handle = ch_handle

        ret = self._lib.ZCAN_StartCAN(ch_handle)
        if ret != STATUS_OK:
            raise RuntimeError("ZCAN_StartCAN failed")
        self._lib.ZCAN_ClearBuffer(ch_handle)
        logger.info(f"Channel {channel_index} initialized: handle=0x{ch_handle:x}")
        return ch_handle

    def close(self):
        if self._ch_handle != INVALID_CHANNEL_HANDLE:
            logger.info("Closing channel and device")
            self._ch_handle = INVALID_CHANNEL_HANDLE
        if self._dev_handle != INVALID_DEVICE_HANDLE:
            self._lib.ZCAN_CloseDevice(self._dev_handle)
            self._dev_handle = INVALID_DEVICE_HANDLE

    def transmit_fd(self, can_id, data_bytes, brs=1, eff=0, rtr=0):
        if self._ch_handle == INVALID_CHANNEL_HANDLE:
            raise RuntimeError("Channel not initialized")
        msg = ZCAN_TransmitFD_Data()
        msg.frame.can_id = make_can_id(eff, rtr, can_id)
        msg.frame.len = len(data_bytes)
        msg.frame.flags = (1 if brs else 0)
        for i in range(min(len(data_bytes), 64)):
            msg.frame.data[i] = data_bytes[i]
        return self._lib.ZCAN_TransmitFD(self._ch_handle, byref(msg), 1)

    def transmit(self, can_id, data_bytes, eff=0, rtr=0):
        if self._ch_handle == INVALID_CHANNEL_HANDLE:
            raise RuntimeError("Channel not initialized")
        msg = ZCAN_Transmit_Data()
        msg.frame.can_id = make_can_id(eff, rtr, can_id)
        msg.frame.can_dlc = len(data_bytes)
        for i in range(min(len(data_bytes), 8)):
            msg.frame.data[i] = data_bytes[i]
        return self._lib.ZCAN_Transmit(self._ch_handle, byref(msg), 1)

    def receive_fd(self, count=100, timeout_ms=0):
        if self._ch_handle == INVALID_CHANNEL_HANDLE:
            return []
        num = self._lib.ZCAN_GetReceiveNum(self._ch_handle, TYPE_CANFD)
        if num <= 0:
            return []
        to_read = min(num, count)
        msgs = (ZCAN_ReceiveFD_Data * to_read)()
        actual = self._lib.ZCAN_ReceiveFD(
            self._ch_handle, byref(msgs), to_read, timeout_ms
        )
        frames = []
        for i in range(actual):
            frame = msgs[i].frame
            raw_id = frame.can_id
            data = bytes(frame.data[:frame.len])
            frames.append({
                'id': get_id(raw_id),
                'eff': is_eff(raw_id),
                'rtr': is_rtr(raw_id),
                'brs': bool(frame.flags & 0x01),
                'esi': bool(frame.flags & 0x02),
                'len': frame.len,
                'data': data,
                'timestamp': msgs[i].timestamp,
            })
        return frames

    def receive(self, count=100, timeout_ms=0):
        if self._ch_handle == INVALID_CHANNEL_HANDLE:
            return []
        num = self._lib.ZCAN_GetReceiveNum(self._ch_handle, TYPE_CAN)
        if num <= 0:
            return []
        to_read = min(num, count)
        msgs = (ZCAN_Receive_Data * to_read)()
        actual = self._lib.ZCAN_Receive(
            self._ch_handle, byref(msgs), to_read, timeout_ms
        )
        frames = []
        for i in range(actual):
            frame = msgs[i].frame
            raw_id = frame.can_id
            data = bytes(frame.data[:frame.can_dlc])
            frames.append({
                'id': get_id(raw_id),
                'eff': is_eff(raw_id),
                'rtr': is_rtr(raw_id),
                'len': frame.can_dlc,
                'data': data,
                'timestamp': msgs[i].timestamp,
            })
        return frames

    def receive_all(self, count=100, timeout_ms=0):
        return self.receive(count, timeout_ms) + self.receive_fd(count, timeout_ms)

    @property
    def is_open(self):
        return self._dev_handle != INVALID_DEVICE_HANDLE

    @property
    def channel_ready(self):
        return self._ch_handle != INVALID_CHANNEL_HANDLE

    def __repr__(self):
        return (
            f"CANFDDriver(device_type={self._device_type}, "
            f"device_index={self._device_index}, "
            f"open={self.is_open}, channel_ready={self.channel_ready})"
        )

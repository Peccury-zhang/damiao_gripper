import logging
import time

logger = logging.getLogger(__name__)

PMAX = 12.566
VMAX = 30.0
TMAX = 10.0

CAN_ID = 0x02
MST_ID = 0x12

MIT_MODE_OFFSET = 0x000

FRAME_LEN = 8


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    if span == 0:
        return 0
    return int((x - x_min) / span * ((1 << bits) - 1))


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return x_int * span / ((1 << bits) - 1) + x_min


def encode_mit_frame(q_des, dq_des, kp, kd, tau_ff):
    q_uint = float_to_uint(q_des, -PMAX, PMAX, 16)
    dq_uint = float_to_uint(dq_des, -VMAX, VMAX, 12)
    kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = float_to_uint(kd, 0.0, 5.0, 12)
    tau_uint = float_to_uint(tau_ff, -TMAX, TMAX, 12)

    data = bytearray(FRAME_LEN)
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
    if len(data) < FRAME_LEN:
        return None
    if data[2] in (0x33, 0x55, 0xAA) and data[3] in (9, 10, 11):
        return None
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    t_mos = data[6]
    t_rotor = data[7]
    q = uint_to_float(q_uint, -PMAX, PMAX, 16)
    dq = uint_to_float(dq_uint, -VMAX, VMAX, 12)
    tau = uint_to_float(tau_uint, -TMAX, TMAX, 12)
    return {
        'position': q,
        'velocity': dq,
        'torque': tau,
        't_mos': t_mos,
        't_rotor': t_rotor,
    }


def enable_motor(can_id=CAN_ID):
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])


def disable_motor(can_id=CAN_ID):
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])


def set_mode_frame(mode_code=1, can_id=CAN_ID):
    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0xFF
    return bytes([id_low, id_high, 0x55, 10, mode_code, 0, 0, 0])


MODE_SWITCH_CAN_ID = 0x7FF
MODE_MIT = 1
MODE_POS_VEL = 2
MODE_VEL = 3
MODE_POS_FORCE = 4


class MotorProtocol:
    def __init__(self, driver, can_id=CAN_ID, mst_id=MST_ID):
        self._driver = driver
        self._can_id = can_id
        self._mst_id = mst_id

    @property
    def can_id(self):
        return self._can_id

    @property
    def mst_id(self):
        return self._mst_id

    def send_mit(self, q_des, dq_des, kp, kd, tau_ff, brs=1):
        data = encode_mit_frame(q_des, dq_des, kp, kd, tau_ff)
        return self._driver.transmit_fd(self._can_id, data, brs=brs)

    def send_mit_can20(self, q_des, dq_des, kp, kd, tau_ff):
        data = encode_mit_frame(q_des, dq_des, kp, kd, tau_ff)
        return self._driver.transmit(self._can_id, data)

    def send_enable(self, count=5, interval=0.005):
        data = enable_motor(self._can_id)
        for _ in range(count):
            self._driver.transmit_fd(self._can_id, data)
            time.sleep(interval)

    def send_disable(self, count=5, interval=0.005):
        data = disable_motor(self._can_id)
        for _ in range(count):
            self._driver.transmit_fd(self._can_id, data)
            time.sleep(interval)

    def send_set_mode(self, mode_code=MODE_MIT, count=3, interval=0.005):
        data = set_mode_frame(mode_code, self._can_id)
        for _ in range(count):
            self._driver.transmit_fd(MODE_SWITCH_CAN_ID, data, brs=1)
            time.sleep(interval)

    def receive_feedback(self, timeout_ms=0):
        frames = self._driver.receive_fd(count=10, timeout_ms=timeout_ms)
        for frame in frames:
            if frame['id'] == self._mst_id and frame['len'] >= FRAME_LEN:
                return decode_feedback(frame['data'])
        frames_can = self._driver.receive(count=10, timeout_ms=0)
        for frame in frames_can:
            if frame['id'] == self._mst_id and frame['len'] >= FRAME_LEN:
                return decode_feedback(frame['data'])
        return None

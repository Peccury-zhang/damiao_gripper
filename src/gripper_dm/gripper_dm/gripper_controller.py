import logging
import math
import time
from enum import Enum

logger = logging.getLogger(__name__)


class GripperState(Enum):
    IDLE = "idle"
    MOVING_TO_OPEN = "moving_to_open"
    MOVING_TO_CLOSE = "moving_to_close"
    HOLDING_TORQUE = "holding_torque"
    CLAMPED = "clamped"


class DMGripperController:
    def __init__(self, motor_protocol, params):
        self._motor = motor_protocol
        self._open_pos = params.get('open_position', 0.1)
        self._close_pos = params.get('close_position', 1.05)
        self._max_speed = params.get('max_speed', 2.0)
        self._kp_move = params.get('kp_move', 10.0)
        self._kd_move = params.get('kd_move', 0.5)
        self._kp_hold = params.get('kp_hold', 20.0)
        self._kd_hold = params.get('kd_hold', 1.0)
        self._hold_torque = params.get('hold_torque', 1.0)
        self._position_tolerance = params.get('position_tolerance', 0.05)
        self._stall_speed_threshold = params.get('stall_speed_threshold', 0.1)
        self._stall_torque_threshold = params.get('stall_torque_threshold', 0.3)
        self._control_rate = params.get('control_rate', 100.0)
        self._motion_timeout = params.get('motion_timeout', 5.0)
        self._decel_distance = params.get('decel_distance', 0.15)

        self._state = GripperState.IDLE
        self._target_position = self._open_pos
        self._current_position = 0.0
        self._current_velocity = 0.0
        self._current_torque = 0.0
        self._t_mos = 0
        self._t_rotor = 0
        self._mode = "MIT"
        self._hold_torque_cmd = 0.0
        self._enabled = False
        self._interpolated_pos = self._open_pos
        self._motion_start_time = 0.0

    @property
    def state(self):
        return self._state

    @property
    def position(self):
        return self._current_position

    @property
    def velocity(self):
        return self._current_velocity

    @property
    def torque(self):
        return self._current_torque

    @property
    def target_position(self):
        return self._target_position

    @property
    def is_enabled(self):
        return self._enabled

    @property
    def temperature_mos(self):
        return self._t_mos

    @property
    def temperature_rotor(self):
        return self._t_rotor

    @property
    def mode(self):
        return self._mode

    def initialize(self):
        self._motor._driver.receive_fd(count=100, timeout_ms=0)
        time.sleep(0.05)
        logger.info("Switching motor to MIT mode...")
        self._motor.send_set_mode()
        time.sleep(0.1)
        self._motor._driver.receive_fd(count=100, timeout_ms=0)
        logger.info("Enabling motor...")
        self._motor.send_enable()
        time.sleep(0.3)
        fb = None
        for i in range(10):
            fb = self._motor.receive_feedback(timeout_ms=100)
            if fb is not None:
                break
            time.sleep(0.05)
        if fb is None:
            logger.warning("No feedback after enable, motor may not be connected")
            self._enabled = False
            return False
        self._current_position = fb['position']
        self._current_velocity = fb['velocity']
        self._current_torque = fb['torque']
        self._interpolated_pos = self._current_position
        self._enabled = True
        logger.info(
            f"Motor enabled, position={self._current_position:.4f} rad"
        )
        return True

    def update(self):
        if not self._enabled:
            return
        fb = self._motor.receive_feedback(timeout_ms=10)
        if fb:
            self._current_position = fb['position']
            self._current_velocity = fb['velocity']
            self._current_torque = fb['torque']
            self._t_mos = fb.get('t_mos', 0)
            self._t_rotor = fb.get('t_rotor', 0)
        if self._state == GripperState.MOVING_TO_OPEN:
            self._run_position_interpolation(self._open_pos)
        elif self._state == GripperState.MOVING_TO_CLOSE:
            self._run_position_interpolation(self._close_pos)
        elif self._state == GripperState.HOLDING_TORQUE:
            self._run_torque_hold()
        elif self._state == GripperState.IDLE:
            self._motor.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)

    def close_gripper(self, hold_torque=None):
        if hold_torque is not None:
            self._hold_torque_cmd = hold_torque
        else:
            self._hold_torque_cmd = self._hold_torque
        self._interpolated_pos = self._current_position
        self._state = GripperState.MOVING_TO_CLOSE
        self._target_position = self._close_pos
        self._motion_start_time = time.monotonic()
        logger.info(
            f"Closing gripper: target={self._close_pos:.4f}, "
            f"hold_torque={self._hold_torque_cmd:.2f}"
        )

    def open_gripper(self):
        self._interpolated_pos = self._current_position
        self._state = GripperState.MOVING_TO_OPEN
        self._target_position = self._open_pos
        self._motion_start_time = time.monotonic()
        logger.info(f"Opening gripper: target={self._open_pos:.4f}")

    def reconnect(self):
        self._enabled = False
        self._state = GripperState.IDLE
        return self.initialize()

    def get_state_dict(self):
        return {
            'state': self._state.value,
            'position': self._current_position,
            'velocity': self._current_velocity,
            'torque': self._current_torque,
            'target_position': self._target_position,
            'enabled': self._enabled,
            't_mos': self._t_mos,
            't_rotor': self._t_rotor,
            'mode': self._mode,
        }

    def _run_position_interpolation(self, target):
        elapsed = time.monotonic() - self._motion_start_time
        if elapsed > self._motion_timeout:
            logger.warning(
                f"Motion timeout after {elapsed:.1f}s, "
                f"pos={self._current_position:.4f}, target={target:.4f}"
            )
            if self._state == GripperState.MOVING_TO_CLOSE:
                self._state = GripperState.HOLDING_TORQUE
                self._interpolated_pos = self._current_position
            else:
                self._state = GripperState.IDLE
            return

        remaining = abs(target - self._interpolated_pos)
        speed = self._max_speed
        if remaining < self._decel_distance:
            ratio = remaining / self._decel_distance
            speed = max(0.2, self._max_speed * ratio)

        step = speed / self._control_rate
        diff = target - self._interpolated_pos
        if abs(diff) <= step:
            self._interpolated_pos = target
        else:
            self._interpolated_pos += step * (1.0 if diff > 0 else -1.0)
        self._motor.send_mit(
            self._interpolated_pos, 0.0,
            self._kp_move, self._kd_move, 0.0
        )

        interp_done = abs(target - self._interpolated_pos) < 1e-6
        pos_close = abs(self._current_position - target) < self._position_tolerance
        speed_low = abs(self._current_velocity) < self._stall_speed_threshold

        if interp_done and pos_close and speed_low:
            if self._state == GripperState.MOVING_TO_CLOSE:
                self._state = GripperState.HOLDING_TORQUE
                logger.info(
                    f"Reached close position, switching to torque hold "
                    f"(tau={self._hold_torque_cmd:.2f})"
                )
            elif self._state == GripperState.MOVING_TO_OPEN:
                self._state = GripperState.IDLE
                logger.info("Reached open position")

        if self._state == GripperState.MOVING_TO_CLOSE:
            if self._is_motor_stalled():
                self._state = GripperState.HOLDING_TORQUE
                self._interpolated_pos = self._current_position
                logger.info(
                    f"Motor stalled at pos={self._current_position:.4f}, "
                    f"switching to torque hold"
                )

    def _run_torque_hold(self):
        self._motor.send_mit(
            self._close_pos, 0.0,
            self._kp_hold, self._kd_hold,
            self._hold_torque_cmd
        )
        if self._is_clamped():
            if self._state != GripperState.CLAMPED:
                self._state = GripperState.CLAMPED
                logger.info("Object clamped successfully")

    def _is_motor_stalled(self):
        elapsed = time.monotonic() - self._motion_start_time
        if elapsed < 0.5:
            return False
        pos_error = abs(self._target_position - self._current_position)
        return (
            abs(self._current_velocity) < self._stall_speed_threshold
            and pos_error > self._position_tolerance
        )

    def _is_clamped(self):
        pos_error = abs(self._target_position - self._current_position)
        return (
            self._state == GripperState.HOLDING_TORQUE
            and pos_error > self._position_tolerance
            and abs(self._current_torque) > self._stall_torque_threshold
        )

    def _check_communication(self):
        for _ in range(3):
            self._motor.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.02)
        fb = self._motor.receive_feedback(timeout_ms=100)
        return fb is not None

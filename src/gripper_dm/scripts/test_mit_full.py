#!/usr/bin/env python3
"""
MIT 模式完整行程测试 - 连续发送控制帧 + 实时读取反馈
"""

import sys
import os
import time
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0
PMAX = 12.566
VMAX = 30.0
TMAX = 10.0


def float_to_uint(x, xmin, xmax, bits):
    x = max(xmin, min(xmax, x))
    return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))


def uint_to_float(x_int, xmin, xmax, bits):
    return x_int * (xmax - xmin) / ((1 << bits) - 1) + xmin


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


def parse_feedback(data):
    if len(data) < 8:
        return None
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    return {
        'position': uint_to_float(q_uint, -PMAX, PMAX, 16),
        'velocity': uint_to_float(dq_uint, -VMAX, VMAX, 12),
        'torque': uint_to_float(tau_uint, -TMAX, TMAX, 12),
    }


def hold_position(driver, target_pos, duration, kp=10.0, kd=0.5, rate_hz=100):
    """持续发送 MIT 控制帧并读取反馈"""
    frame = build_mit_frame(target_pos, 0.0, kp, kd, 0.0)
    dt = 1.0 / rate_hz
    end_time = time.time() + duration
    last_fb = None

    while time.time() < end_time:
        driver.transmit_fd(CAN_ID, frame, brs=1)
        frames = driver.receive_fd(count=5, timeout_ms=0)
        for f in frames:
            if f['id'] == MST_ID:
                fb = parse_feedback(f['data'])
                if fb:
                    last_fb = fb
        time.sleep(dt)

    return last_fb


def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 70)
        print("MIT 模式完整行程测试")
        print("=" * 70)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        print("[1] 使能电机...")
        enable_data = bytes([0xFF] * 7 + [0xFC])
        for _ in range(10):
            driver.transmit_fd(CAN_ID, enable_data, brs=1)
            time.sleep(0.005)

        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        initial_pos = None
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                initial_pos = fb['position']
                print(f"  初始位置: {fb['position']:.4f} rad")
                print(f"  初始速度: {fb['velocity']:.4f} rad/s")
                print(f"  初始力矩: {fb['torque']:.4f} N·m")

        targets = [
            (0.5, "中间位置 0.5 rad"),
            (1.0, "闭合位置 1.0 rad"),
            (0.1, "打开位置 0.1 rad"),
            (1.1, "完全闭合 1.1 rad"),
            (0.1, "回到打开 0.1 rad"),
        ]

        for target_pos, label in targets:
            print(f"\n[{label}] 持续控制 2 秒...")
            fb = hold_position(driver, target_pos, duration=2.0)
            if fb:
                error = abs(fb['position'] - target_pos)
                print(f"  最终位置: {fb['position']:.4f} rad (误差: {error:.4f})")
                print(f"  最终速度: {fb['velocity']:.4f} rad/s")
                print(f"  最终力矩: {fb['torque']:.4f} N·m")
                if error < 0.1:
                    print(f"  ✓ 到位")
                else:
                    print(f"  ⚠ 未到位（误差 {error:.4f} rad）")

        print("\n[6] 去使能...")
        disable_data = bytes([0xFF] * 7 + [0xFD])
        for _ in range(5):
            driver.transmit_fd(CAN_ID, disable_data, brs=1)
            time.sleep(0.005)

        print("\n" + "=" * 70)
        print("测试完成")

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()


if __name__ == "__main__":
    main()

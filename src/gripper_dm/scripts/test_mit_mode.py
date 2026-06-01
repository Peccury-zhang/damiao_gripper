#!/usr/bin/env python3
"""
测试 MIT 模式控制
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0

def parse_feedback(data):
    """解析反馈帧"""
    if len(data) < 8:
        return None
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    
    PMAX = 12.566
    VMAX = 30.0
    TMAX = 10.0
    
    q = q_uint / 65535.0 * 2 * PMAX - PMAX
    dq = dq_uint / 4095.0 * 2 * VMAX - VMAX
    tau = tau_uint / 4095.0 * 2 * TMAX - TMAX
    
    return {'position': q, 'velocity': dq, 'torque': tau}

def build_mit_frame(q_des, dq_des, kp, kd, tau_ff):
    """构建 MIT 控制帧"""
    PMAX = 12.566
    VMAX = 30.0
    TMAX = 10.0
    
    q_uint = int((q_des + PMAX) / (2 * PMAX) * 65535)
    dq_uint = int((dq_des + VMAX) / (2 * VMAX) * 4095)
    kp_uint = int(kp / 500.0 * 4095)
    kd_uint = int(kd / 5.0 * 4095)
    tau_uint = int((tau_ff + TMAX) / (2 * TMAX) * 4095)
    
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

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 70)
        print("测试 MIT 模式控制")
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
        print(f"  收到 {len(frames)} 帧")
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  初始位置: {fb['position']:.4f} rad")
                print(f"  初始速度: {fb['velocity']:.4f} rad/s")
                print(f"  初始力矩: {fb['torque']:.4f} N·m")
                initial_pos = fb['position']

        print("\n[2] MIT 控制 (目标: 0.5 rad, kp=10, kd=0.5)...")
        mit_frame = build_mit_frame(0.5, 0.0, 10.0, 0.5, 0.0)
        print(f"  发送帧: {mit_frame.hex()}")
        
        for _ in range(100):
            driver.transmit_fd(CAN_ID, mit_frame, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  反馈位置: {fb['position']:.4f} rad")
                print(f"  位置变化: {abs(fb['position'] - initial_pos):.4f} rad")
                moved = abs(fb['position'] - initial_pos) > 0.05
                print(f"  ✓ 电机移动: {moved}" if moved else f"  ✗ 电机未移动")

        print("\n[3] MIT 控制 (目标: 1.0 rad, kp=10, kd=0.5)...")
        mit_frame = build_mit_frame(1.0, 0.0, 10.0, 0.5, 0.0)
        
        for _ in range(100):
            driver.transmit_fd(CAN_ID, mit_frame, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  反馈位置: {fb['position']:.4f} rad")

        print("\n[4] MIT 控制 (目标: 0.1 rad, kp=10, kd=0.5)...")
        mit_frame = build_mit_frame(0.1, 0.0, 10.0, 0.5, 0.0)
        
        for _ in range(100):
            driver.transmit_fd(CAN_ID, mit_frame, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  反馈位置: {fb['position']:.4f} rad")

        print("\n[5] 去使能...")
        disable_data = bytes([0xFF] * 7 + [0xFD])
        for _ in range(5):
            driver.transmit_fd(CAN_ID, disable_data, brs=1)
            time.sleep(0.005)

        print("\n" + "=" * 70)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()

if __name__ == "__main__":
    main()

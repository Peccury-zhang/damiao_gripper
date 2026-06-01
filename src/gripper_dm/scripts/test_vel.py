#!/usr/bin/env python3
"""
测试 VEL 模式（纯速度控制）
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0
VEL_OFFSET = 0x200  # VEL 模式的 CAN ID 偏移

def encode_vel_frame(vel_des, kp, kd):
    """编码 VEL 帧"""
    VMAX = 30.0
    
    vel_uint = int((vel_des + VMAX) / (2 * VMAX) * 65535)
    kp_uint = int(kp / 500.0 * 4095)
    kd_uint = int(kd / 5.0 * 4095)
    
    data = bytearray(8)
    data[0] = 0x00
    data[1] = 0x00
    data[2] = (vel_uint >> 8) & 0xFF
    data[3] = vel_uint & 0xFF
    data[4] = (kp_uint >> 8) & 0xFF
    data[5] = kp_uint & 0xFF
    data[6] = (kd_uint >> 8) & 0xFF
    data[7] = kd_uint & 0xFF
    return bytes(data)

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

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 60)
        print("VEL 模式测试（纯速度控制）")
        print("=" * 60)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        print(f"[1] 使能电机 (CAN ID 0x{CAN_ID:02X})...")
        enable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        for _ in range(10):
            driver.transmit_fd(CAN_ID, enable_data, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  初始位置: {fb['position']:.4f} rad\n")

        vel_can_id = CAN_ID + VEL_OFFSET
        print(f"[2] VEL 控制 (CAN ID 0x{vel_can_id:03X})...")
        
        for vel_des in [1.0, -1.0, 0.0]:
            print(f"\n  目标速度: {vel_des:.2f} rad/s")
            vel_frame = encode_vel_frame(vel_des, 10.0, 0.5)
            print(f"  帧: {vel_frame.hex()}")
            
            initial_pos = None
            for i in range(50):
                driver.transmit_fd(vel_can_id, vel_frame, brs=1)
                time.sleep(0.02)
                
                frames = driver.receive_fd(count=5, timeout_ms=50)
                if frames and initial_pos is None:
                    fb = parse_feedback(frames[-1]['data'])
                    if fb:
                        initial_pos = fb['position']

            time.sleep(0.5)
            frames = driver.receive_fd(count=20, timeout_ms=200)
            if frames:
                fb = parse_feedback(frames[-1]['data'])
                if fb:
                    print(f"  反馈位置: {fb['position']:.4f} rad")
                    if initial_pos is not None:
                        print(f"  移动: {abs(fb['position'] - initial_pos) > 0.05}")

        print("\n[3] 去使能...")
        disable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        for _ in range(5):
            driver.transmit_fd(CAN_ID, disable_data, brs=1)
            time.sleep(0.01)

        print("\n" + "=" * 60)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
测试 POS_VEL 模式（位置速度模式）
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0
POS_VEL_OFFSET = 0x100  # POS_VEL 模式的 CAN ID 偏移

def encode_pos_vel_frame(q_des, dq_des, kp, kd):
    """编码 POS_VEL 帧"""
    PMAX = 12.566
    VMAX = 30.0
    
    q_uint = int((q_des + PMAX) / (2 * PMAX) * 65535)
    dq_uint = int((dq_des + VMAX) / (2 * VMAX) * 65535)
    kp_uint = int(kp / 500.0 * 4095)
    kd_uint = int(kd / 5.0 * 4095)
    
    data = bytearray(8)
    data[0] = (q_uint >> 8) & 0xFF
    data[1] = q_uint & 0xFF
    data[2] = (dq_uint >> 8) & 0xFF
    data[3] = dq_uint & 0xFF
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
        print("POS_VEL 模式测试")
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

        pos_vel_can_id = CAN_ID + POS_VEL_OFFSET
        print(f"[2] POS_VEL 控制 (CAN ID 0x{pos_vel_can_id:03X}, 目标: 0.5 rad)...")
        
        for target_pos in [0.5, 1.0, 0.0]:
            print(f"\n  目标位置: {target_pos:.4f} rad")
            pos_vel_frame = encode_pos_vel_frame(target_pos, 0.0, 10.0, 0.5)
            print(f"  帧: {pos_vel_frame.hex()}")
            
            for _ in range(100):
                driver.transmit_fd(pos_vel_can_id, pos_vel_frame, brs=1)
                time.sleep(0.01)

            time.sleep(0.5)
            frames = driver.receive_fd(count=20, timeout_ms=200)
            if frames:
                fb = parse_feedback(frames[-1]['data'])
                if fb:
                    print(f"  反馈位置: {fb['position']:.4f} rad")
                    print(f"  移动: {abs(fb['position'] - 1.1303) > 0.05}")

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

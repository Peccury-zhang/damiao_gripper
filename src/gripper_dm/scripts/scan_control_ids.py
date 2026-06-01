#!/usr/bin/env python3
"""
扫描控制 CAN ID，找到电机响应的 ID
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0

def encode_mit_frame(q_des, kp, kd):
    """编码简化的 MIT 帧"""
    PMAX = 12.566
    
    q_uint = int((q_des + PMAX) / (2 * PMAX) * 65535)
    kp_uint = int(kp / 500.0 * 4095)
    kd_uint = int(kd / 5.0 * 4095)
    dq_uint = 2047
    tau_uint = 2047
    
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
    """解析反馈帧"""
    if len(data) < 8:
        return None
    q_uint = (data[1] << 8) | data[2]
    PMAX = 12.566
    q = q_uint / 65535.0 * 2 * PMAX - PMAX
    return q

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 60)
        print("控制 CAN ID 扫描")
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
        initial_pos = None
        if frames:
            initial_pos = parse_feedback(frames[-1]['data'])
            if initial_pos is not None:
                print(f"  初始位置: {initial_pos:.4f} rad\n")

        print("[2] 扫描控制 CAN ID (0x00 - 0x1F)...")
        mit_frame = encode_mit_frame(0.5, 10.0, 0.5)
        
        for test_id in range(0x00, 0x20):
            driver.clear_buffer()
            
            for _ in range(20):
                driver.transmit_fd(test_id, mit_frame, brs=1)
                time.sleep(0.01)

            time.sleep(0.2)
            frames = driver.receive_fd(count=20, timeout_ms=200)
            
            if frames:
                final_pos = parse_feedback(frames[-1]['data'])
                if final_pos is not None and initial_pos is not None:
                    moved = abs(final_pos - initial_pos) > 0.05
                    if moved:
                        print(f"  ✓ CAN ID 0x{test_id:02X}: 电机移动! {initial_pos:.4f} -> {final_pos:.4f}")
                        initial_pos = final_pos
                    elif test_id in [0x00, 0x01, 0x02, 0x10, 0x11, 0x12]:
                        print(f"  - CAN ID 0x{test_id:02X}: 位置 {final_pos:.4f} (未移动)")

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

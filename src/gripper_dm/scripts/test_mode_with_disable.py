#!/usr/bin/env python3
"""
测试：先去使能，再切换模式，再使能
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0
MODE_SWITCH_CAN_ID = 0x7FF

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
        print("=" * 70)
        print("测试：去使能 → 切换模式 → 使能 → MIT 控制")
        print("=" * 70)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        print("[1] 先去使能...")
        disable_data = bytes([0xFF] * 7 + [0xFD])
        for _ in range(20):
            driver.transmit_fd(CAN_ID, disable_data, brs=1)
            time.sleep(0.005)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")

        print("\n[2] 切换到 MIT 模式 (mode=1)...")
        id_low = CAN_ID & 0xFF
        id_high = (CAN_ID >> 8) & 0xFF
        mode_data = bytes([id_low, id_high, 0x55, 10, 1, 0x00, 0x00, 0x00])
        
        for _ in range(20):
            driver.transmit_fd(MODE_SWITCH_CAN_ID, mode_data, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")
        for f in frames:
            print(f"    ID=0x{f['id']:03X}, data={f['data'].hex()}")
            if f['id'] == MST_ID and len(f['data']) >= 8:
                rid = f['data'][3]
                if rid == 10:
                    mode_value = (f['data'][7] << 24) | (f['data'][6] << 16) | (f['data'][5] << 8) | f['data'][4]
                    mode_names = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "POS_FORCE"}
                    print(f"      → 模式响应: {mode_value} ({mode_names.get(mode_value, '未知')})")

        print("\n[3] 使能电机...")
        enable_data = bytes([0xFF] * 7 + [0xFC])
        for _ in range(10):
            driver.transmit_fd(CAN_ID, enable_data, brs=1)
            time.sleep(0.005)
        
        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")
        if frames:
            pos = parse_feedback(frames[-1]['data'])
            if pos is not None:
                print(f"  初始位置: {pos:.4f} rad")
                initial_pos = pos

        print("\n[4] 发送 MIT 控制 (目标: 0.5 rad)...")
        PMAX = 12.566
        VMAX = 30.0
        TMAX = 10.0
        
        q_des = 0.5
        q_uint = int((q_des + PMAX) / (2 * PMAX) * 65535)
        dq_uint = 2047  # 中间值
        kp_uint = int(10.0 / 500.0 * 4095)
        kd_uint = int(0.5 / 5.0 * 4095)
        tau_uint = 2047  # 中间值
        
        mit_data = bytearray(8)
        mit_data[0] = (q_uint >> 8) & 0xFF
        mit_data[1] = q_uint & 0xFF
        mit_data[2] = (dq_uint >> 4) & 0xFF
        mit_data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
        mit_data[4] = kp_uint & 0xFF
        mit_data[5] = (kd_uint >> 4) & 0xFF
        mit_data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
        mit_data[7] = tau_uint & 0xFF
        
        for _ in range(100):
            driver.transmit_fd(CAN_ID, bytes(mit_data), brs=1)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            pos = parse_feedback(frames[-1]['data'])
            if pos is not None:
                print(f"  反馈位置: {pos:.4f} rad")
                print(f"  移动: {abs(pos - initial_pos) > 0.05}")

        print("\n[5] 去使能...")
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

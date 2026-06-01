#!/usr/bin/env python3
"""
调试脚本：打印 MIT 帧字节并测试不同位置
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver
from gripper_dm.motor_protocol import MotorProtocol, encode_mit_frame

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

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 60)
        print("MIT 帧调试")
        print("=" * 60)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        motor = MotorProtocol(driver, can_id=CAN_ID, mst_id=MST_ID)

        print("[1] 使能电机...")
        motor.send_enable(count=10)
        time.sleep(0.3)
        
        fb = motor.receive_feedback(timeout_ms=500)
        if fb:
            print(f"  初始位置: {fb['position']:.4f} rad\n")
        else:
            print("  ✗ 未收到反馈\n")
            return

        test_positions = [0.5, 1.0, 0.0, 1.1303]
        kp = 10.0
        kd = 0.5

        for target_pos in test_positions:
            print(f"[测试] 目标位置: {target_pos:.4f} rad")
            
            mit_frame = encode_mit_frame(target_pos, 0.0, kp, kd, 0.0)
            print(f"  MIT 帧: {mit_frame.hex()}")
            print(f"  MIT 帧字节: {' '.join(f'{b:02X}' for b in mit_frame)}")
            
            for _ in range(50):
                motor.send_mit(target_pos, 0.0, kp, kd, 0.0, brs=1)
                time.sleep(0.01)

            time.sleep(0.5)
            
            fb = motor.receive_feedback(timeout_ms=200)
            if fb:
                print(f"  反馈位置: {fb['position']:.4f} rad")
                print(f"  位置误差: {abs(fb['position'] - target_pos):.4f} rad\n")
            else:
                print("  ✗ 未收到反馈\n")

        print("[去使能]")
        motor.send_disable()
        
        print("\n" + "=" * 60)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()

if __name__ == "__main__":
    main()

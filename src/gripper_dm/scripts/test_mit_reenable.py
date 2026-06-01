#!/usr/bin/env python3
"""
测试：每次 MIT 控制前重新使能
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver
from gripper_dm.motor_protocol import MotorProtocol

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 60)
        print("MIT 控制测试（重新使能）")
        print("=" * 60)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        motor = MotorProtocol(driver, can_id=CAN_ID, mst_id=MST_ID)

        print("[1] 初始使能...")
        motor.send_enable(count=10)
        time.sleep(0.3)
        
        fb = motor.receive_feedback(timeout_ms=500)
        if fb:
            print(f"  初始位置: {fb['position']:.4f} rad\n")

        print("[2] 发送 MIT 控制 (目标: 0.5 rad) + 重新使能...")
        for i in range(10):
            motor.send_enable(count=2)
            motor.send_mit(0.5, 0.0, 10.0, 0.5, 0.0, brs=1)
            time.sleep(0.05)

        time.sleep(0.5)
        fb = motor.receive_feedback(timeout_ms=200)
        if fb:
            print(f"  反馈位置: {fb['position']:.4f} rad")
            print(f"  移动: {abs(fb['position'] - 1.1303) > 0.05}\n")

        print("[3] 去使能...")
        motor.send_disable(count=5)
        time.sleep(0.2)

        print("[4] 重新使能 + MIT 控制 (目标: 1.0 rad)...")
        for i in range(10):
            motor.send_enable(count=2)
            motor.send_mit(1.0, 0.0, 10.0, 0.5, 0.0, brs=1)
            time.sleep(0.05)

        time.sleep(0.5)
        fb = motor.receive_feedback(timeout_ms=200)
        if fb:
            print(f"  反馈位置: {fb['position']:.4f} rad")
            print(f"  移动: {abs(fb['position'] - 1.1303) > 0.05}\n")

        print("[5] 最终去使能...")
        motor.send_disable(count=5)

        print("\n" + "=" * 60)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()

if __name__ == "__main__":
    main()

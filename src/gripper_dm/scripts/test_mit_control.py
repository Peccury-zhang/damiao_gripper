#!/usr/bin/env python3
"""
测试脚本：验证电机能否响应 MIT 控制命令
流程：
1. 切换到 MIT 模式（发送到 0x7FF）
2. 使能电机
3. 发送 MIT 控制命令移动到不同位置
4. 读取反馈确认电机是否移动
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver
from gripper_dm.motor_protocol import (
    MotorProtocol, MODE_MIT, MODE_SWITCH_CAN_ID,
    encode_mit_frame, decode_feedback
)

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 60)
        print("MIT 控制测试")
        print("=" * 60)

        driver.open_device()
        print("✓ 设备打开成功")

        driver.init_channel(CHANNEL)
        print(f"✓ 通道 {CHANNEL} 初始化成功")

        motor = MotorProtocol(driver, can_id=CAN_ID, mst_id=MST_ID)

        print("\n[1/5] 切换到 MIT 模式...")
        print(f"  发送模式切换帧到 CAN ID 0x{MODE_SWITCH_CAN_ID:03X}")
        motor.send_set_mode(mode_code=MODE_MIT, count=5)
        time.sleep(0.1)
        print("  ✓ 模式切换命令已发送")

        print("\n[2/5] 使能电机...")
        motor.send_enable(count=10)
        time.sleep(0.2)

        fb = motor.receive_feedback(timeout_ms=500)
        if fb is None:
            print("  ✗ 未收到电机反馈！")
            return
        print(f"  ✓ 收到反馈: pos={fb['position']:.4f} rad, vel={fb['velocity']:.4f}")

        initial_pos = fb['position']
        print(f"\n  初始位置: {initial_pos:.4f} rad")

        print("\n[3/5] 发送 MIT 控制命令 (目标: 0.5 rad)...")
        target_pos = 0.5
        kp = 10.0
        kd = 0.5

        for i in range(50):
            motor.send_mit(target_pos, 0.0, kp, kd, 0.0, brs=1)
            time.sleep(0.01)

        time.sleep(0.5)

        fb = motor.receive_feedback(timeout_ms=200)
        if fb is None:
            print("  ✗ 未收到反馈！")
            return

        current_pos = fb['position']
        print(f"  当前位置: {current_pos:.4f} rad")
        print(f"  位置变化: {abs(current_pos - initial_pos):.4f} rad")

        if abs(current_pos - initial_pos) > 0.1:
            print("  ✓ 电机已移动！MIT 控制生效")
        else:
            print("  ✗ 电机未移动，MIT 控制可能未生效")

        print("\n[4/5] 发送 MIT 控制命令 (目标: 1.0 rad)...")
        target_pos = 1.0

        for i in range(50):
            motor.send_mit(target_pos, 0.0, kp, kd, 0.0, brs=1)
            time.sleep(0.01)

        time.sleep(0.5)

        fb = motor.receive_feedback(timeout_ms=200)
        if fb:
            print(f"  当前位置: {fb['position']:.4f} rad")

        print("\n[5/5] 去使能电机...")
        motor.send_disable(count=5)
        print("  ✓ 电机已去使能")

        print("\n" + "=" * 60)
        print("测试完成")
        print("=" * 60)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()
        print("设备已关闭")

if __name__ == "__main__":
    main()

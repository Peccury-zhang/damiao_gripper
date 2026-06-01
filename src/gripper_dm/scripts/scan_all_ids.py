#!/usr/bin/env python3
"""
扫描所有可能的 CAN ID，找到真正响应的电机
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CHANNEL = 0

def send_enable(driver, can_id):
    """使能命令 (0xFC)"""
    data = bytes([0xFF] * 7 + [0xFC])
    driver.transmit_fd(can_id, data, brs=1)

def send_disable(driver, can_id):
    """去使能命令 (0xFD)"""
    data = bytes([0xFF] * 7 + [0xFD])
    driver.transmit_fd(can_id, data, brs=1)

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
    
    return {'position': q, 'velocity': dq, 'torque': tau, 'motor_id': data[0]}

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 70)
        print("扫描所有 CAN ID (0x00 - 0x1F)")
        print("=" * 70)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        responding_ids = []

        for test_id in range(0x00, 0x20):
            # 清空缓冲区
            driver.receive_fd(count=100, timeout_ms=10)
            
            # 发送使能命令
            for _ in range(5):
                send_enable(driver, test_id)
                time.sleep(0.005)
            
            # 等待响应
            time.sleep(0.1)
            frames = driver.receive_fd(count=20, timeout_ms=300)
            
            if frames:
                fb = parse_feedback(frames[-1]['data'])
                if fb:
                    responding_ids.append({
                        'cmd_id': test_id,
                        'feedback_id': frames[-1]['id'],
                        'motor_id': fb['motor_id'],
                        'position': fb['position']
                    })
                    print(f"✓ CAN ID 0x{test_id:02X}: 响应!")
                    print(f"  反馈来自: 0x{frames[-1]['id']:03X}")
                    print(f"  电机 ID: 0x{fb['motor_id']:02X}")
                    print(f"  位置: {fb['position']:.4f} rad")

        # 去使能所有发现的电机
        if responding_ids:
            print(f"\n[去使能所有发现的电机]")
            for r in responding_ids:
                for _ in range(5):
                    send_disable(driver, r['cmd_id'])
                    time.sleep(0.005)

        print("\n" + "=" * 70)
        print("扫描结果:")
        if not responding_ids:
            print("  ✗ 没有找到响应的电机")
        else:
            for r in responding_ids:
                print(f"  命令 ID: 0x{r['cmd_id']:02X} → 反馈 ID: 0x{r['feedback_id']:03X} → 电机 ID: 0x{r['motor_id']:02X}")
        print("=" * 70)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()

if __name__ == "__main__":
    main()

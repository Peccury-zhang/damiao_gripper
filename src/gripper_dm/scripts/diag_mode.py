#!/usr/bin/env python3
"""
诊断脚本：检查电机模式寄存器（先使能）
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

def send_read_mode(driver, can_id):
    """发送读模式寄存器命令 (RID=10)"""
    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0xFF
    data = bytes([id_low, id_high, 0x33, 10, 0x00, 0x00, 0x00, 0x00])
    driver.transmit_fd(MODE_SWITCH_CAN_ID, data, brs=1)

def send_set_mode(driver, can_id, mode_code):
    """发送设置模式命令"""
    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0xFF
    data = bytes([id_low, id_high, 0x55, 10, mode_code, 0x00, 0x00, 0x00])
    driver.transmit_fd(MODE_SWITCH_CAN_ID, data, brs=1)

def send_enable(driver, can_id):
    """发送使能命令"""
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    driver.transmit_fd(can_id, data, brs=1)

def send_disable(driver, can_id):
    """发送去使能命令"""
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    driver.transmit_fd(can_id, data, brs=1)

def parse_mode_response(data):
    """解析模式响应"""
    if len(data) < 8:
        return None
    can_id = (data[1] << 8) | data[0]
    rid = data[3]
    if rid == 10:
        mode_value = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]
        return mode_value
    return None

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
        print("电机模式诊断（带使能）")
        print("=" * 60)

        driver.open_device()
        print("✓ 设备打开")

        driver.init_channel(CHANNEL)
        print(f"✓ 通道 {CHANNEL} 初始化")

        print("\n[1] 使能电机...")
        for _ in range(10):
            send_enable(driver, CAN_ID)
            time.sleep(0.01)

        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧:")
        for f in frames[-5:]:
            print(f"    ID=0x{f['id']:03X} data={f['data'].hex()}")
            if f['id'] == MST_ID:
                fb = parse_feedback(f['data'])
                if fb:
                    print(f"    → 反馈: pos={fb['position']:.4f} rad")

        print("\n[2] 读取当前模式...")
        for _ in range(10):
            send_read_mode(driver, CAN_ID)
            time.sleep(0.01)

        time.sleep(0.2)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧:")
        for f in frames:
            print(f"    ID=0x{f['id']:03X} data={f['data'].hex()}")
            if f['id'] == MST_ID:
                mode = parse_mode_response(f['data'])
                if mode is not None:
                    mode_names = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "POS_FORCE"}
                    print(f"    → 模式寄存器值: {mode} ({mode_names.get(mode, '未知')})")
                else:
                    fb = parse_feedback(f['data'])
                    if fb:
                        print(f"    → 反馈: pos={fb['position']:.4f} rad")

        print("\n[3] 切换到 MIT 模式 (mode=1)...")
        for _ in range(20):
            send_set_mode(driver, CAN_ID, 1)
            time.sleep(0.01)

        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  切换后收到 {len(frames)} 帧")

        print("\n[4] 再次读取模式...")
        for _ in range(10):
            send_read_mode(driver, CAN_ID)
            time.sleep(0.01)

        time.sleep(0.2)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧:")
        for f in frames:
            print(f"    ID=0x{f['id']:03X} data={f['data'].hex()}")
            if f['id'] == MST_ID:
                mode = parse_mode_response(f['data'])
                if mode is not None:
                    mode_names = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "POS_FORCE"}
                    print(f"    → 模式寄存器值: {mode} ({mode_names.get(mode, '未知')})")
                else:
                    fb = parse_feedback(f['data'])
                    if fb:
                        print(f"    → 反馈: pos={fb['position']:.4f} rad")

        print("\n[5] 发送 MIT 控制命令 (目标: 0.5 rad)...")
        PMAX = 12.566
        VMAX = 30.0
        TMAX = 10.0
        
        q_des = 0.5
        dq_des = 0.0
        kp = 10.0
        kd = 0.5
        tau_ff = 0.0
        
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
        
        for _ in range(100):
            driver.transmit_fd(CAN_ID, bytes(data), brs=1)
            time.sleep(0.01)

        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  控制后收到 {len(frames)} 帧:")
        for f in frames[-5:]:
            if f['id'] == MST_ID:
                fb = parse_feedback(f['data'])
                if fb:
                    print(f"    → 反馈: pos={fb['position']:.4f} rad")

        print("\n[6] 去使能电机...")
        for _ in range(5):
            send_disable(driver, CAN_ID)
            time.sleep(0.01)

        print("\n" + "=" * 60)

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()
        print("设备已关闭")

if __name__ == "__main__":
    main()

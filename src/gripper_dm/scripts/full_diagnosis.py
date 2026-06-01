#!/usr/bin/env python3
"""
完整诊断脚本：模拟 damiao.py 的 enable_all 流程
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_dm.canfd_driver import CANFDDriver

CAN_ID = 0x02
MST_ID = 0x12
CHANNEL = 0

def send_enable(driver, can_id):
    """使能命令 (0xFC)"""
    data = bytes([0xFF] * 7 + [0xFC])
    driver.transmit_fd(can_id, data, brs=1)

def send_disable(driver, can_id):
    """去使能命令 (0xFD)"""
    data = bytes([0xFF] * 7 + [0xFD])
    driver.transmit_fd(can_id, data, brs=1)

def switch_control_mode(driver, can_id, mode_code):
    """切换到指定控制模式"""
    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0xFF
    write_data = bytes([mode_code, 0x00, 0x00, 0x00])
    mydata = bytes([id_low, id_high, 0x55, 10]) + write_data
    driver.transmit_fd(0x7FF, mydata, brs=1)

def read_motor_param(driver, can_id, rid):
    """读取电机参数"""
    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0xFF
    mydata = bytes([id_low, id_high, 0x33, rid, 0x00, 0x00, 0x00, 0x00])
    driver.transmit_fd(0x7FF, mydata, brs=1)

def control_mit(driver, can_id, q, dq, kp, kd, tau):
    """MIT 控制"""
    PMAX = 12.566
    VMAX = 30.0
    TMAX = 10.0
    
    q_uint = int((q + PMAX) / (2 * PMAX) * 65535)
    dq_uint = int((dq + VMAX) / (2 * VMAX) * 4095)
    kp_uint = int(kp / 500.0 * 4095)
    kd_uint = int(kd / 5.0 * 4095)
    tau_uint = int((tau + TMAX) / (2 * TMAX) * 4095)
    
    data = bytearray(8)
    data[0] = (q_uint >> 8) & 0xFF
    data[1] = q_uint & 0xFF
    data[2] = (dq_uint >> 4) & 0xFF
    data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
    data[4] = kp_uint & 0xFF
    data[5] = (kd_uint >> 4) & 0xFF
    data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
    data[7] = tau_uint & 0xFF
    
    driver.transmit_fd(can_id, bytes(data), brs=1)

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

def parse_param_response(data):
    """解析参数响应"""
    if len(data) < 8:
        return None
    can_id = (data[1] << 8) | data[0]
    rid = data[3]
    if rid == 10:
        mode_value = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]
        return {'can_id': can_id, 'rid': rid, 'mode': mode_value}
    return None

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 70)
        print("完整诊断：模拟 damiao.py enable_all() 流程")
        print("=" * 70)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        print("[1] 切换控制模式到 MIT (mode=1)...")
        for _ in range(5):
            switch_control_mode(driver, CAN_ID, 1)
            time.sleep(0.01)
        
        time.sleep(0.2)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧:")
        for f in frames:
            print(f"    ID=0x{f['id']:03X} data={f['data'].hex()}")
            if f['id'] == MST_ID:
                param = parse_param_response(f['data'])
                if param:
                    mode_names = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "POS_FORCE"}
                    print(f"    → 模式响应: {param['mode']} ({mode_names.get(param['mode'], '未知')})")

        print("\n[2] 读取模式寄存器 (RID=10)...")
        for _ in range(5):
            read_motor_param(driver, CAN_ID, 10)
            time.sleep(0.01)
        
        time.sleep(0.2)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧:")
        for f in frames:
            if f['id'] == MST_ID:
                param = parse_param_response(f['data'])
                if param:
                    mode_names = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "POS_FORCE"}
                    print(f"    → 当前模式: {param['mode']} ({mode_names.get(param['mode'], '未知')})")
                else:
                    fb = parse_feedback(f['data'])
                    if fb:
                        print(f"    → 反馈: pos={fb['position']:.4f} rad")

        print("\n[3] 使能电机...")
        for _ in range(10):
            send_enable(driver, CAN_ID)
            time.sleep(0.005)
        
        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧:")
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  初始位置: {fb['position']:.4f} rad\n")
                initial_pos = fb['position']

        print("[4] 发送 MIT 控制命令 (目标: 0.5 rad)...")
        for i in range(100):
            control_mit(driver, CAN_ID, 0.5, 0.0, 10.0, 0.5, 0.0)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  反馈位置: {fb['position']:.4f} rad")
                print(f"  移动: {abs(fb['position'] - initial_pos) > 0.05}\n")

        print("[5] 发送 MIT 控制命令 (目标: 1.0 rad)...")
        for i in range(100):
            control_mit(driver, CAN_ID, 1.0, 0.0, 10.0, 0.5, 0.0)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            fb = parse_feedback(frames[-1]['data'])
            if fb:
                print(f"  反馈位置: {fb['position']:.4f} rad\n")

        print("[6] 去使能...")
        for _ in range(5):
            send_disable(driver, CAN_ID)
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

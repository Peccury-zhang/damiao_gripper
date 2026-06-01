#!/usr/bin/env python3
"""
详细诊断：逐字节打印所有发送的命令
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

def main():
    lib_path = os.path.join(os.path.dirname(__file__), "libcontrolcanfd.so")
    driver = CANFDDriver(lib_path)

    try:
        print("=" * 70)
        print("详细诊断：逐字节打印所有命令")
        print("=" * 70)

        driver.open_device()
        driver.init_channel(CHANNEL)
        print("✓ 设备就绪\n")

        print("[1] 使能电机 (CAN ID 0x02)...")
        enable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        print(f"  发送: CAN ID=0x02, data={enable_data.hex()}")
        print(f"  字节: {' '.join(f'{b:02X}' for b in enable_data)}")
        for _ in range(10):
            driver.transmit_fd(CAN_ID, enable_data, brs=1)
            time.sleep(0.005)
        
        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")
        for f in frames[:3]:
            print(f"    反馈 ID=0x{f['id']:03X}, data={f['data'].hex()}")
        
        print("\n[2] 切换到 MIT 模式 (mode=1)...")
        id_low = CAN_ID & 0xFF
        id_high = (CAN_ID >> 8) & 0xFF
        mode_code = 1
        rid = 10
        mode_data = bytes([id_low, id_high, 0x55, rid, mode_code, 0x00, 0x00, 0x00])
        print(f"  发送: CAN ID=0x{MODE_SWITCH_CAN_ID:03X}, data={mode_data.hex()}")
        print(f"  字节: {' '.join(f'{b:02X}' for b in mode_data)}")
        print(f"  解析: id_low=0x{id_low:02X}, id_high=0x{id_high:02X}, cmd=0x55, rid={rid}, mode={mode_code}")
        
        for _ in range(10):
            driver.transmit_fd(MODE_SWITCH_CAN_ID, mode_data, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")
        for f in frames:
            print(f"    ID=0x{f['id']:03X}, data={f['data'].hex()}")
            if f['id'] == MST_ID:
                # 检查是否是参数响应
                if len(f['data']) >= 8:
                    cmd_type = f['data'][2]
                    rid_resp = f['data'][3]
                    print(f"      cmd_type=0x{cmd_type:02X}, rid={rid_resp}")
                    if rid_resp == 10:
                        mode_value = (f['data'][7] << 24) | (f['data'][6] << 16) | (f['data'][5] << 8) | f['data'][4]
                        print(f"      → 模式值: {mode_value}")

        print("\n[3] 读取模式寄存器 (RID=10)...")
        read_data = bytes([id_low, id_high, 0x33, 10, 0x00, 0x00, 0x00, 0x00])
        print(f"  发送: CAN ID=0x{MODE_SWITCH_CAN_ID:03X}, data={read_data.hex()}")
        print(f"  字节: {' '.join(f'{b:02X}' for b in read_data)}")
        
        for _ in range(10):
            driver.transmit_fd(MODE_SWITCH_CAN_ID, read_data, brs=1)
            time.sleep(0.01)
        
        time.sleep(0.3)
        frames = driver.receive_fd(count=20, timeout_ms=500)
        print(f"  收到 {len(frames)} 帧")
        for f in frames:
            print(f"    ID=0x{f['id']:03X}, data={f['data'].hex()}")

        print("\n[4] 发送 MIT 控制命令 (目标: 0.5 rad)...")
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
        
        mit_data = bytearray(8)
        mit_data[0] = (q_uint >> 8) & 0xFF
        mit_data[1] = q_uint & 0xFF
        mit_data[2] = (dq_uint >> 4) & 0xFF
        mit_data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
        mit_data[4] = kp_uint & 0xFF
        mit_data[5] = (kd_uint >> 4) & 0xFF
        mit_data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
        mit_data[7] = tau_uint & 0xFF
        
        print(f"  发送: CAN ID=0x{CAN_ID:02X}, data={mit_data.hex()}")
        print(f"  字节: {' '.join(f'{b:02X}' for b in mit_data)}")
        print(f"  解析: q_uint=0x{q_uint:04X} ({q_des:.2f}), kp_uint=0x{kp_uint:03X} ({kp:.1f}), kd_uint=0x{kd_uint:03X} ({kd:.1f})")
        
        for _ in range(50):
            driver.transmit_fd(CAN_ID, bytes(mit_data), brs=1)
            time.sleep(0.01)
        
        time.sleep(0.5)
        frames = driver.receive_fd(count=20, timeout_ms=200)
        if frames:
            data = frames[-1]['data']
            q_uint = (data[1] << 8) | data[2]
            q = q_uint / 65535.0 * 2 * PMAX - PMAX
            print(f"  反馈位置: {q:.4f} rad")

        print("\n[5] 去使能...")
        disable_data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
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

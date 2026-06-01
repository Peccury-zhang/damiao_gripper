#!/usr/bin/env python3
"""
测试：直接使用 damiao.py 的原始流程
"""

import sys
import os
import time

sys.path.insert(0, '/home/yclab/gripper/src/gripper_dm/files/USB_Gripper/u2canfdpy')

from damiao import Motor_Control, Motor, DmActData, DM_Motor_Type, Control_Mode

def main():
    print("=" * 70)
    print("使用原始 damiao.py 测试电机控制")
    print("=" * 70)

    # 创建电机配置
    motor_data = [
        DmActData(
            motorType=DM_Motor_Type.DM4310,
            mode=Control_Mode.MIT_MODE,
            can_id=0x02,
            mst_id=0x12
        )
    ]

    try:
        # 初始化（会自动 enable_all）
        print("\n[1] 初始化 Motor_Control...")
        control = Motor_Control(
            nom_baud=1000000,
            dat_baud=5000000,
            sn="USBCANFD212606182279",
            data_ptr=motor_data
        )
        
        time.sleep(1.0)
        
        print("\n[2] 获取电机...")
        motor = control.getMotor(0x02)
        if motor is None:
            print("  ✗ 无法获取电机")
            return
        
        print(f"  电机 CAN ID: 0x{motor.GetCanId():02X}")
        print(f"  电机模式: {motor.GetMotorMode()}")
        print(f"  当前位置: {motor.Get_Position():.4f} rad")
        
        print("\n[3] 发送 MIT 控制 (目标: 0.5 rad)...")
        for i in range(100):
            control.control_mit(motor, kp=10.0, kd=0.5, q=0.5, dq=0.0, tau=0.0)
            time.sleep(0.01)
        
        time.sleep(0.5)
        print(f"  当前位置: {motor.Get_Position():.4f} rad")
        
        print("\n[4] 发送 MIT 控制 (目标: 1.0 rad)...")
        for i in range(100):
            control.control_mit(motor, kp=10.0, kd=0.5, q=1.0, dq=0.0, tau=0.0)
            time.sleep(0.01)
        
        time.sleep(0.5)
        print(f"  当前位置: {motor.Get_Position():.4f} rad")
        
        print("\n[5] 去使能...")
        control.disable_all()
        
        print("\n" + "=" * 70)
        
    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'control' in locals():
            control.close()

if __name__ == "__main__":
    main()

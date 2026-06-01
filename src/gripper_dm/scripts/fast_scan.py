#!/usr/bin/env python3
import os, sys, time, ctypes
from ctypes import cdll, c_uint, c_int, c_ubyte, c_ushort, c_ulonglong, c_void_p, byref, Structure, Union

LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libcontrolcanfd.so")

class F(Structure):
    _fields_=[("can_id",c_uint),("len",c_ubyte),("flags",c_ubyte),("_r0",c_ubyte),("_r1",c_ubyte),("data",c_ubyte*64)]
class CI(Structure):
    _fields_=[("acc_code",c_uint),("acc_mask",c_uint),("reserved",c_uint),("filter",c_ubyte),("t0",c_ubyte),("t1",c_ubyte),("mode",c_ubyte)]
class FI(Structure):
    _fields_=[("acc_code",c_uint),("acc_mask",c_uint),("abit",c_uint),("dbit",c_uint),("brp",c_uint),("filter",c_ubyte),("mode",c_ubyte),("pad",c_ushort),("reserved",c_uint)]
class CU(Union):
    _fields_=[("can",CI),("canfd",FI)]
class CC(Structure):
    _fields_=[("can_type",c_uint),("config",CU)]
class DI(Structure):
    _fields_=[("hw_Version",c_ushort),("fw_Version",c_ushort),("dr_Version",c_ushort),("in_Version",c_ushort),("irq_Num",c_ushort),("can_Num",c_ubyte),("str_Serial_Num",c_ubyte*21),("str_hw_Type",c_ubyte*40),("reserved",c_ushort*4)]
class TFD(Structure):
    _fields_=[("frame",F),("transmit_type",c_uint)]
class RFD(Structure):
    _fields_=[("frame",F),("timestamp",c_ulonglong)]

def f2u(x,xmin,xmax,bits):
    return int((max(xmin,min(x,xmax))-xmin)/(xmax-xmin)*((1<<bits)-1))

def u2f(x,xmin,xmax,bits):
    return float(x)/((1<<bits)-1)*(xmax-xmin)+xmin

def mit_frame(kp,kd,q,dq,tau):
    ku,du,qu,du2,tu=f2u(kp,0,500,12),f2u(kd,0,5,12),f2u(q,-12.566,12.566,16),f2u(dq,-30,30,12),f2u(tau,-10,10,12)
    d=[0]*8; d[0]=(qu>>8)&0xFF; d[1]=qu&0xFF; d[2]=du2>>4
    d[3]=((du2&0xF)<<4)|((ku>>8)&0xF); d[4]=ku&0xFF; d[5]=du>>4
    d[6]=((du&0xF)<<4)|((tu>>8)&0xF); d[7]=tu&0xFF
    return bytes(d)

def dec(d):
    qu=(d[1]<<8)|d[2]; du=(d[3]<<4)|(d[4]>>4); tu=((d[4]&0xF)<<8)|d[5]
    return u2f(qu,-12.566,12.566,16),u2f(du,-30,30,12),u2f(tu,-10,10,12)

def main():
    L=cdll.LoadLibrary(LIB)
    for n,r,a in [("ZCAN_OpenDevice",c_void_p,(c_uint,c_uint,c_uint)),("ZCAN_CloseDevice",c_uint,(c_void_p,)),
        ("ZCAN_GetDeviceInf",c_uint,(c_void_p,c_void_p)),("ZCAN_SetAbitBaud",c_uint,(c_void_p,c_uint,c_uint)),
        ("ZCAN_SetDbitBaud",c_uint,(c_void_p,c_uint,c_uint)),("ZCAN_SetCANFDStandard",c_uint,(c_void_p,c_uint,c_uint)),
        ("ZCAN_InitCAN",c_void_p,(c_void_p,c_uint,c_void_p)),("ZCAN_StartCAN",c_uint,(c_void_p,)),
        ("ZCAN_ResetCAN",c_uint,(c_void_p,)),("ZCAN_ClearBuffer",c_uint,(c_void_p,)),
        ("ZCAN_TransmitFD",c_uint,(c_void_p,c_void_p,c_uint)),("ZCAN_GetReceiveNum",c_int,(c_void_p,c_ubyte)),
        ("ZCAN_ReceiveFD",c_uint,(c_void_p,c_void_p,c_uint,c_int)),("ZCAN_ClearFilter",c_uint,(c_void_p,)),
        ("ZCAN_AckFilter",c_uint,(c_void_p,)),("ZCAN_SetFilterMode",c_uint,(c_void_p,c_uint)),
        ("ZCAN_SetFilterStartID",c_uint,(c_void_p,c_uint)),("ZCAN_SetFilterEndID",c_uint,(c_void_p,c_uint)),
        ("ZCAN_SetResistanceEnable",c_uint,(c_void_p,c_uint,c_uint))]:
        f=getattr(L,n);f.restype=r;f.argtypes=a

    dev=L.ZCAN_OpenDevice(41,0,0)
    if not dev: print("设备打开失败");sys.exit(1)
    info=DI();L.ZCAN_GetDeviceInf(dev,byref(info))
    nb=b'\x00'
    print(f"SN={bytes(info.str_Serial_Num).rstrip(nb).decode('ascii',errors='replace')} 通道={info.can_Num}")

    found=False
    for ch_idx in range(min(info.can_Num,2)):
        for abit,dbit,label in [(1000000,5000000,"1M+5M"),(5000000,5000000,"5M+5M")]:
            print(f"\n--- CH{ch_idx} {label} ---")
            L.ZCAN_SetAbitBaud(dev,ch_idx,abit)
            L.ZCAN_SetDbitBaud(dev,ch_idx,dbit)
            L.ZCAN_SetCANFDStandard(dev,ch_idx,0)
            L.ZCAN_SetResistanceEnable(dev,ch_idx,1)
            cfg=CC();cfg.can_type=1;cfg.config.canfd.mode=0
            ch=L.ZCAN_InitCAN(dev,ch_idx,byref(cfg))
            if not ch: print("  Init失败");continue
            L.ZCAN_StartCAN(ch)
            L.ZCAN_ClearFilter(ch);L.ZCAN_AckFilter(ch)
            L.ZCAN_SetFilterMode(ch,0);L.ZCAN_SetFilterStartID(ch,0);L.ZCAN_SetFilterEndID(ch,0x7FF)
            L.ZCAN_ClearBuffer(ch)

            en=bytes([0xFF]*7+[0xFC]); dis=bytes([0xFF]*7+[0xFD])
            mit=mit_frame(5.0,0.3,0.5,0.0,0.0)

            for can_id in range(1,33):
                while True:
                    n=L.ZCAN_GetReceiveNum(ch,1)
                    if n<=0:break
                    m=(RFD*n)();L.ZCAN_ReceiveFD(ch,byref(m),n,5)

                for _ in range(5):
                    msg=TFD();msg.transmit_type=0;msg.frame.can_id=can_id;msg.frame.len=8;msg.frame.flags=0
                    for i,b in enumerate(en):msg.frame.data[i]=b
                    L.ZCAN_TransmitFD(ch,byref(msg),1)
                    time.sleep(0.003)
                for _ in range(10):
                    msg=TFD();msg.transmit_type=0;msg.frame.can_id=can_id;msg.frame.len=8;msg.frame.flags=0
                    for i,b in enumerate(mit):msg.frame.data[i]=b
                    L.ZCAN_TransmitFD(ch,byref(msg),1)
                    time.sleep(0.003)

                time.sleep(0.1)
                n=L.ZCAN_GetReceiveNum(ch,1)
                if n>0:
                    m=(RFD*n)();cnt=L.ZCAN_ReceiveFD(ch,byref(m),n,100)
                    real=[m[i] for i in range(cnt) if (m[i].frame.can_id&0x7FF)!=can_id]
                    if real:
                        print(f"  *** CAN_ID=0x{can_id:03X} 响应! ***")
                        for r in real:
                            d=bytes(r.frame.data[:r.frame.len])
                            dh=" ".join(f"{b:02X}" for b in d)
                            print(f"    ID=0x{(r.frame.can_id&0x7FF):03X} [{dh}]")
                            if len(d)>=6:
                                q,dq,tau=dec(d)
                                print(f"    pos={q:.4f} vel={dq:.4f} tau={tau:.4f}")
                        found=True
                    elif cnt>0:
                        pass

                for _ in range(3):
                    msg=TFD();msg.transmit_type=0;msg.frame.can_id=can_id;msg.frame.len=8;msg.frame.flags=0
                    for i,b in enumerate(dis):msg.frame.data[i]=b
                    L.ZCAN_TransmitFD(ch,byref(msg),1)
                    time.sleep(0.002)

            L.ZCAN_ResetCAN(ch)

    L.ZCAN_CloseDevice(dev)
    if found:
        print("\n已发现电机!")
    else:
        print("\n未发现电机响应。请确认:")
        print("  1. 电机已供电 (24V)")
        print("  2. CAN_H/CAN_L 已连接到分析仪对应通道")
        print("  3. 电机 CAN ID 在 0x01~0x20 范围内")

if __name__=="__main__":
    main()

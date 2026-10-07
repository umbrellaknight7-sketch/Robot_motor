#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L30 灵巧手 CANFD 传输层（可插拔后端）

L30 使用 29 位扩展帧。本模块把"底层收发"从协议逻辑中解耦, 提供两种后端, 供
linker_hand_l30_v6_canfd.py 与 linker_hand_l30_v6_2_canfd.py 两个控制类共用:

  - LibCanBusTransport : 厂商私有库 libcanbus.so（默认, 原有通讯方式）。
  - SocketCANTransport : 内核原生 SocketCAN + python-can（用于"透明塑封 USB 转
                         CANFD 设备"）。不依赖 libcanbus.so, 通过 can0 接口收发,
                         帧格式与厂商设备一致（29 位扩展帧）。

两种后端接口一致（轮询式收发, 与 L30 协议原有的同步收发模型匹配）:
    initialize() -> bool                              初始化并连接
    send(frame_id: int, payload: bytes) -> bool       发送一个扩展帧 CANFD
    receive(timeout_ms: int) -> List[(frame_id, bytes)]  轮询取回已收帧
    close() -> None                                   关闭
    is_connected: bool                                连接状态

约定: payload 为"数据段有效字节"(BYTE0 数据长度 + BYTE1 事务控制 + 数据), 各后端
负责补零到合法 CANFD 帧长度。上层协议只管组帧/解析, 不关心具体传输方式。
"""

import os
import time
import logging
import subprocess
from typing import List, Optional, Tuple
from ctypes import (
    Structure, CDLL, cdll, cast, byref, RTLD_GLOBAL,
    c_uint, c_ushort, c_char, c_ubyte, c_uint16, POINTER,
)

logger = logging.getLogger(__name__)

STATUS_OK = 0


# =============================================================================
# DLC 编码 <-> 线路字节数（CAN FD 标准表）
# =============================================================================

DLC_TO_LENGTH = {
    0x00: 0, 0x01: 1, 0x02: 2, 0x03: 3, 0x04: 4, 0x05: 5, 0x06: 6, 0x07: 7,
    0x08: 8, 0x09: 12, 0x0A: 16, 0x0B: 20, 0x0C: 24, 0x0D: 32, 0x0E: 48, 0x0F: 64,
}
NON_STANDARD_DLC_MAP = {0x10: 16, 0x40: 64}  # 少数设备可能回填非标准 DLC 值


def get_dlc_from_length(length: int) -> int:
    """根据线路字节数向上取最接近的合法 DLC 编码（不足由设备/本层补 0x00）。"""
    if length <= 8:
        return length
    if length <= 12:
        return 0x09
    if length <= 16:
        return 0x0A
    if length <= 20:
        return 0x0B
    if length <= 24:
        return 0x0C
    if length <= 32:
        return 0x0D
    if length <= 48:
        return 0x0E
    return 0x0F


def get_length_from_dlc(dlc: int) -> int:
    """根据 DLC 编码得到线路字节数。"""
    if dlc in DLC_TO_LENGTH:
        return DLC_TO_LENGTH[dlc]
    if dlc in NON_STANDARD_DLC_MAP:
        return NON_STANDARD_DLC_MAP[dlc]
    return min(dlc, 64)


# =============================================================================
# libcanbus 结构体
# =============================================================================

class CanFD_Config(Structure):
    _fields_ = [
        ("NomBaud", c_uint),
        ("DatBaud", c_uint),
        ("NomPres", c_ushort),
        ("NomTseg1", c_char),
        ("NomTseg2", c_char),
        ("NomSJW", c_char),
        ("DatPres", c_char),
        ("DatTseg1", c_char),
        ("DatTseg2", c_char),
        ("DatSJW", c_char),
        ("Config", c_char),
        ("Model", c_char),
        ("Cantype", c_char),
    ]


class CanFD_Msg(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("FrameType", c_ubyte),
        ("DLC", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("BusSatus", c_ubyte),
        ("ErrSatus", c_ubyte),
        ("TECounter", c_ubyte),
        ("RECounter", c_ubyte),
        ("Data", c_ubyte * 64),
    ]


# =============================================================================
# 传输层基类
# =============================================================================

class L30Transport:
    """CANFD 传输层接口（29 位扩展帧, 轮询式收发）。子类实现具体后端。"""

    def initialize(self) -> bool:
        """初始化并连接底层设备。成功返回 True。"""
        raise NotImplementedError

    def send(self, frame_id: int, payload: bytes) -> bool:
        """发送一个扩展帧 CANFD。payload 为数据段有效字节, 由本层补零到合法帧长。"""
        raise NotImplementedError

    def receive(self, timeout_ms: int = 3) -> List[Tuple[int, bytes]]:
        """轮询取回已收帧, 返回 [(frame_id, data_bytes), ...]; 无数据返回空列表。"""
        raise NotImplementedError

    def close(self) -> None:
        """关闭底层设备。"""
        raise NotImplementedError


# =============================================================================
# 后端一：厂商私有库 libcanbus.so（默认）
# =============================================================================

class LibCanBusTransport(L30Transport):
    """基于厂商私有库 libcanbus.so 的 CANFD 传输层（29 位扩展帧, 轮询收发）。

    仲裁段 1Mbps、数据段 5Mbps；与 L30 原有实现一致。
    """

    ARBITRATION_BAUD = 1000000
    DATA_BAUD = 5000000

    def __init__(self, canfd_device: int = 0, channel: int = 0):
        """
        Args:
            canfd_device: CANFD 设备(盒子)索引
            channel: 通道号
        """
        self.canfd_device = canfd_device
        self.channel = channel
        self.canDLL = None
        self.is_connected = False

    def initialize(self) -> bool:
        """加载库 -> 扫描 -> 打开通道 -> CANFD 配置(1M/5M) -> 设置过滤器(全通)。"""
        try:
            CDLL("/usr/local/lib/libusb-1.0.so", RTLD_GLOBAL)
            time.sleep(0.1)
            self.canDLL = cdll.LoadLibrary("/usr/local/lib/libcanbus.so")

            ret = self.canDLL.CAN_ScanDevice()
            if ret <= 0:
                logger.error(f"未找到CANFD设备, 错误码: {ret}")
                return False
            print(f"找到 {ret} 个设备", flush=True)

            ret = self.canDLL.CAN_OpenDevice(self.canfd_device, self.channel)
            if ret != STATUS_OK:
                logger.error(f"打开设备失败, 错误码: {ret}")
                return False
            print(f"设备通道 {self.channel} 打开成功", flush=True)

            can_config = CanFD_Config(
                self.ARBITRATION_BAUD, self.DATA_BAUD,
                0x0, 0x0, 0x0, 0x0,
                0x0, 0x0, 0x0, 0x0,
                0x0, 0x0, 0x1,
            )
            ret = self.canDLL.CANFD_Init(self.canfd_device, self.channel, byref(can_config))
            if ret != STATUS_OK:
                logger.error(f"CANFD初始化失败, 错误码: {ret}")
                self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                return False

            ret = self.canDLL.CAN_SetFilter(self.canfd_device, self.channel, 0, 0, 0, 0, 1)
            if ret != STATUS_OK:
                logger.error(f"设置过滤器失败, 错误码: {ret}")
                self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                return False

            self.is_connected = True
            return True
        except OSError as e:
            logger.error(f"加载CAN库失败: {e}")
            return False
        except Exception as e:
            logger.error(f"CANFD初始化异常: {e}")
            return False

    def send(self, frame_id: int, payload: bytes) -> bool:
        """把 payload 拷入 64 字节缓冲、按长度取 DLC, 以扩展帧 CANFD 发送。"""
        if not self.is_connected:
            return False
        try:
            dlc = get_dlc_from_length(len(payload))
            buf = (c_ubyte * 64)()
            for i, b in enumerate(payload[:64]):
                buf[i] = b
            # FrameType=4(CANFD), ExternFlag=1(29 位扩展帧)
            msg = CanFD_Msg(
                ID=frame_id, TimeStamp=0, FrameType=4, DLC=dlc,
                ExternFlag=1, RemoteFlag=0, BusSatus=0, ErrSatus=0,
                TECounter=0, RECounter=0, Data=buf,
            )
            time.sleep(0.001)
            ret = self.canDLL.CANFD_Transmit(self.canfd_device, self.channel, byref(msg), 1, 100)
            return ret == 1
        except Exception as e:
            logger.error(f"发送消息异常(libcanbus): {e}")
            return False

    def receive(self, timeout_ms: int = 3) -> List[Tuple[int, bytes]]:
        """从驱动接收缓冲一次性取回若干帧, DLC 反查长度后返回 (id, data)。"""
        if not self.is_connected:
            return []
        try:
            class MsgArray(Structure):
                _fields_ = [('SIZE', c_uint16), ('ARRAY', CanFD_Msg * 100)]

                @property
                def ptr(self):
                    return cast(byref(self.ARRAY), POINTER(CanFD_Msg))

            receive_buffer = MsgArray()
            receive_buffer.SIZE = 100
            ret = self.canDLL.CANFD_Receive(self.canfd_device, self.channel,
                                            receive_buffer.ptr, 100, timeout_ms)
            if ret <= 0:
                return []
            out = []
            for i in range(ret):
                msg = receive_buffer.ARRAY[i]
                data_len = get_length_from_dlc(msg.DLC)
                out.append((msg.ID, bytes(msg.Data[:data_len])))
            return out
        except Exception as e:
            logger.error(f"接收消息异常(libcanbus): {e}")
            return []

    def close(self) -> None:
        """关闭 CANFD 设备。"""
        if self.canDLL and self.is_connected:
            try:
                self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
            except Exception as e:
                logger.error(f"关闭CANFD连接失败: {e}")
        self.is_connected = False


# =============================================================================
# 后端二：内核原生 SocketCAN + python-can（透明塑封 USB 转 CANFD 设备）
# =============================================================================

class SocketCANTransport(L30Transport):
    """基于内核原生 SocketCAN (python-can) 的 CANFD 传输层（29 位扩展帧, 轮询收发）。

    用于"透明塑封 USB 转 CANFD 设备"：在 Linux 下走标准 SocketCAN, 无需厂商私有库
    (libcanbus.so), 通过内核 can0 接口 + python-can 收发。帧格式与厂商设备一致
    （L30 为 29 位扩展帧），故上层协议无需任何改动。
    """

    def __init__(self, channel: str = "can0", bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True,
                 bitrate_switch: bool = False):
        """
        Args:
            channel: SocketCAN 接口名, 如 "can0"
            bitrate: 仲裁段波特率(默认 1Mbps, 与 L30 libcanbus 一致)
            dbitrate: 数据段波特率(默认 5Mbps)
            auto_setup: 是否自动执行 ip link 配置并拉起接口(需 sudo)
            bitrate_switch: 发送帧是否启用 BRS(数据段切高速)。默认 False——实测
                BRS 开启 + 高速数据段易把总线打入 BUS-OFF, 关闭时数据段保持仲裁
                波特率, 收发更稳定。
        """
        self.channel = channel
        self.bitrate = bitrate
        self.dbitrate = dbitrate
        self.auto_setup = auto_setup
        self.bitrate_switch = bitrate_switch
        self.bus = None
        self.is_connected = False

    def _setup_interface(self):
        """自动配置 can0 接口（需要 sudo 权限）。

        等价于手动执行:
            sudo ip link set can0 down
            sudo ip link set can0 type can bitrate <b> dbitrate <d> fd on
            sudo ip link set can0 up
            sudo ip link set can0 txqueuelen 1000
        逐条容错, 失败不中断（接口可能已由用户提前配置）。
        """
        cmds = [
            (["sudo", "ip", "link", "set", self.channel, "down"], True),
            (["sudo", "ip", "link", "set", self.channel, "type", "can",
              "bitrate", str(self.bitrate), "dbitrate", str(self.dbitrate),
              "fd", "on"], True),
            # restart-ms 部分控制器不支持, 单独下发且允许失败
            (["sudo", "ip", "link", "set", self.channel, "type", "can",
              "restart-ms", "100"], False),
            (["sudo", "ip", "link", "set", self.channel, "up"], True),
            (["sudo", "ip", "link", "set", self.channel, "txqueuelen", "1000"], True),
        ]
        for c, required in cmds:
            try:
                ret = subprocess.run(c, check=False, timeout=5,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if ret.returncode != 0:
                    err = ret.stderr.decode(errors='ignore').strip()
                    if required:
                        print(f"[SocketCAN] 接口配置命令失败: {' '.join(c)}  原因: {err}")
                    else:
                        print(f"[SocketCAN] 提示: 该设备不支持的可选项已跳过: {' '.join(c)} ({err})")
                time.sleep(0.1)  # 给内核时间完成状态切换（尤其 down 之后）
            except Exception as e:
                print(f"[SocketCAN] 接口配置命令异常: {' '.join(c)} -> {e}")

        # 回读实际生效的时序与状态, 便于定位波特率不匹配 / BUS-OFF
        try:
            ret = subprocess.run(["ip", "-details", "link", "show", self.channel],
                                 check=False, timeout=5,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out = ret.stdout.decode(errors='ignore')
            timing = next((l.strip() for l in out.splitlines() if "bitrate" in l), "(未读到)")
            state = next((l.strip() for l in out.splitlines() if "state" in l), "")
            print(f"[SocketCAN] 期望 bitrate={self.bitrate} dbitrate={self.dbitrate}")
            print(f"            实际生效: {timing}")
            print(f"            状态行  : {state}")
            if "BUS-OFF" in state or "ERROR-PASSIVE" in state:
                print("[SocketCAN] ⚠️ 总线处于错误状态, 通常是波特率与设备不一致或接线/终端电阻问题。")
        except Exception:
            pass

    def initialize(self) -> bool:
        """自动拉起接口 -> 校验接口存在 -> 打开 python-can CANFD 总线。"""
        try:
            import can  # noqa: F401
        except ImportError:
            print("❌ 缺少 python-can 库, 请执行: pip install python-can")
            return False

        try:
            import can
            print(f"开始初始化 SocketCAN 设备 (通道: {self.channel})...")

            if self.auto_setup:
                print(f"正在自动配置接口 {self.channel} "
                      f"(bitrate={self.bitrate}, dbitrate={self.dbitrate}, fd on)...")
                self._setup_interface()
                time.sleep(0.2)

            # 打开总线前先确认接口存在, 避免 python-can 抛出难懂异常
            if not os.path.exists(f"/sys/class/net/{self.channel}"):
                print(f"❌ 未找到网络接口 {self.channel}。")
                print("   该透明塑封设备需拨到 Linux 模式才会枚举为原生 CAN 接口；")
                print("   若 lsusb 显示为 'STM32 Virtual ComPort' 则说明仍是串口模式。")
                print("   请：1) 将 type-c 下方开关拨到 Linux 模式  2) 重新插拔USB")
                print(f"      3) 用 `ip -br link show type can` 确认 {self.channel} 出现")
                self.is_connected = False
                self.bus = None
                return False

            self.bus = can.interface.Bus(channel=self.channel,
                                         interface='socketcan', fd=True)
            self.is_connected = True
            print(f"✅ SocketCAN 通道 {self.channel} 打开成功")
            return True
        except Exception as e:
            print(f"❌ SocketCAN 初始化失败: {e}")
            print("   请检查:")
            print("   1. 设备是否接入、type-c 下方开关是否拨到 Linux 模式")
            print(f"   2. 接口 {self.channel} 是否存在 (ip -br link show type can)")
            print("   3. 是否具备 sudo 权限以自动配置接口")
            self.is_connected = False
            self.bus = None
            return False

    def send(self, frame_id: int, payload: bytes) -> bool:
        """以 29 位扩展帧 CANFD 发送 payload（补零到合法帧长）。"""
        if not self.is_connected or self.bus is None:
            return False
        try:
            import can
            data = bytes(payload)
            data_len = min(len(data), 64)
            data = data[:data_len]
            # 补零到合法 CANFD 帧长（0-8,12,16,20,24,32,48,64）
            dlc = get_dlc_from_length(data_len)
            padded_len = DLC_TO_LENGTH[dlc]
            body = data + b'\x00' * (padded_len - data_len)

            msg = can.Message(
                arbitration_id=frame_id,
                is_extended_id=True,          # L30 为 29 位扩展帧
                is_fd=True,
                bitrate_switch=self.bitrate_switch,
                data=body,
            )
            self.bus.send(msg, timeout=0.2)
            return True
        except Exception as e:
            print(f"发送消息异常(SocketCAN): {e}")
            return False

    def receive(self, timeout_ms: int = 3) -> List[Tuple[int, bytes]]:
        """阻塞取第一帧(至多 timeout_ms), 再非阻塞排空缓冲, 返回 (id, data) 列表。"""
        if not self.is_connected or self.bus is None:
            return []
        out: List[Tuple[int, bytes]] = []
        try:
            first = self.bus.recv(timeout=timeout_ms / 1000.0)
            if first is None:
                return out
            out.append((first.arbitration_id, bytes(first.data)))
            # 排空当前已到达的其余帧（非阻塞）
            while True:
                m = self.bus.recv(timeout=0.0)
                if m is None:
                    break
                out.append((m.arbitration_id, bytes(m.data)))
        except Exception:
            pass
        return out

    def close(self) -> None:
        """关闭 SocketCAN 总线。"""
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as e:
                print(f"关闭 SocketCAN 连接失败: {e}")
        self.bus = None
        self.is_connected = False


def create_transport(comm_type: str = "libcanbus", canfd_device: int = 0,
                     channel=0, bitrate: int = 1000000, dbitrate: int = 5000000,
                     auto_setup: bool = True) -> L30Transport:
    """
    按 comm_type 创建对应的 CANFD 传输后端。

    Args:
        comm_type: "libcanbus"(默认, 厂商库) 或 "socketcan"(内核 can0 + python-can)
        canfd_device: 仅 libcanbus——设备(盒子)索引
        channel: libcanbus 下为通道号(int, 默认 0)；socketcan 下为接口名(str, 默认 "can0")
        bitrate/dbitrate/auto_setup: 仅 socketcan——仲裁/数据段波特率与是否自动拉起接口
    Returns:
        L30Transport 子类实例。
    """
    if comm_type == "socketcan":
        ch = channel if isinstance(channel, str) else "can0"
        return SocketCANTransport(channel=ch, bitrate=bitrate,
                                  dbitrate=dbitrate, auto_setup=auto_setup)
    ch = channel if isinstance(channel, int) else 0
    return LibCanBusTransport(canfd_device=canfd_device, channel=ch)

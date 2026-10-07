#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L30灵巧手CANFD通信控制程序
基于新版CANFD协议的17自由度灵巧手控制系统

协议规范:
- 物理接口: CANFD
- 仲裁波特率: 1Mbps (80%)
- 数据波特率: 5Mbps (75%)
- 帧格式: 扩展帧, 数据帧协议

作者: hejianxin
版本: 2.1
日期: 2026-04-14
"""

import sys
import os
import time
import logging
import struct
from typing import List, Optional, Tuple, Dict, Callable
from dataclasses import dataclass, field
from enum import IntEnum
from ctypes import *

import numpy as np

from .linker_hand_l30_canfd_comm import (
    LibCanBusTransport,
    SocketCANTransport,
    create_transport,
)

logger = logging.getLogger(__name__)


# CANFD库常量和结构体定义
STATUS_OK = 0


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
        ("Cantype", c_char)
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
        ("Data", c_ubyte * 64)
    ]


class Dev_Info(Structure):
    _fields_ = [
        ("HW_Type", c_char * 32),
        ("HW_Ser", c_char * 32),
        ("HW_Ver", c_char * 32),
        ("FW_Ver", c_char * 32),
        ("MF_Date", c_char * 32)
    ]


# DLC到实际数据长度的映射表
DLC_TO_LENGTH = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64}

# 非标准DLC映射
NON_STANDARD_DLC_MAP = {0x10: 16, 0x40: 64}


def get_dlc_from_length(length: int) -> int:
    """根据数据长度获取DLC值"""
    if length <= 8:
        return length
    if length <= 12:
        return 9
    if length <= 16:
        return 10
    if length <= 20:
        return 11
    if length <= 24:
        return 12
    if length <= 32:
        return 13
    if length <= 48:
        return 14
    return 15


def get_length_from_dlc(dlc: int) -> int:
    """
    根据DLC值获取实际数据长度

    设备可能返回非标准DLC值(如0x40=64或0x10=16)，需要特殊处理
    """
    if dlc in DLC_TO_LENGTH:
        return DLC_TO_LENGTH[dlc]
    if dlc in NON_STANDARD_DLC_MAP:
        return NON_STANDARD_DLC_MAP[dlc]
    return min(dlc, 64)


# =============================================================================
# 协议常量定义
# =============================================================================

class Priority(IntEnum):
    """CANFDID优先级"""
    HIGH = 0
    MEDIUM = 1
    LOW = 2
    LOWEST = 3


class MessageDirection(IntEnum):
    """消息方向"""
    REQUEST = 0  # 请求/TX
    RESPONSE = 1  # 应答/RX


class ReadWrite(IntEnum):
    """读写标志"""
    READ = 0
    WRITE = 1


class HandType(IntEnum):
    """左右手类型"""
    LEFT = 0
    RIGHT = 1


class StatusCode(IntEnum):
    """返回状态码"""
    OK = 0x00
    HAND_NOT_SET = 0x01
    MOTOR_DISABLED = 0x02
    NOT_CALIBRATED = 0x03
    READ_ERROR = 0xF0


class Command(IntEnum):
    """命令码"""
    JOINT_POSITION = 0x01       # 关节位置读写
    JOINT_TORQUE = 0x02          # 关节扭矩写入
    JOINT_TORQUE_LIMIT = 0x03   # 关节转矩限制
    JOINT_SPEED = 0x05          # 关节速度读写
    JOINT_ACCELERATION = 0x07   # 关节加速度写入
    JOINT_ENABLE = 0x08         # 关节使能读写
    JOINT_TEMPERATURE = 0x33    # 关节温度读取
    JOINT_ERROR_CODE = 0x35      # 关节错误码读取
    JOINT_CURRENT = 0x36         # 关节电流读取
    CALIBRATE_ZERO = 0x37        # 标定零点
    EMERGENCY_STOP = 0x37        # 急停
    THUMB_PRESSURE = 0xB1        # 拇指压力数据
    INDEX_PRESSURE = 0xB2        # 食指压力数据
    MIDDLE_PRESSURE = 0xB3      # 中指压力数据
    RING_PRESSURE = 0xB4        # 无名指压力数据
    PINKY_PRESSURE = 0xB5       # 小指压力数据
    DEVICE_CODE = 0xC0          # 设备编码
    DEVICE_VERSION = 0xC1       # 设备版本
    DEVICE_CANFDID = 0xC3       # CANFDID和左右手
    RESTORE_DEVICE_ID = 0xC4   # 还原设备ID


# =============================================================================
# 关节信息定义
# =============================================================================

@dataclass
class JointInfo:
    """关节信息"""
    id: int
    name: str
    finger: str
    min_pos: int = -32768
    max_pos: int = 32767
    current_pos: int = 0
    target_pos: int = 0
    current_vel: int = 0
    target_vel: int = 0
    target_torque: int = 0
    torque_limit: int = 1000
    acceleration: int = 0
    enabled: bool = False
    temperature: int = 0
    error_code: int = 0
    current: int = 0

    def update_from_reading(self, positions: Optional[List[int]] = None,
                            velocities: Optional[List[int]] = None,
                            currents: Optional[List[int]] = None,
                            temperature: Optional[int] = None,
                            error_code: Optional[int] = None,
                            enabled: Optional[bool] = None) -> None:
        """从读取结果更新关节信息"""
        if positions is not None:
            self.current_pos = positions
            self.target_pos = positions
        if velocities is not None:
            self.current_vel = velocities
            self.target_vel = velocities
        if currents is not None:
            self.current = currents
        if temperature is not None:
            self.temperature = temperature
        if error_code is not None:
            self.error_code = error_code
        if enabled is not None:
            self.enabled = enabled

    @property
    def range(self) -> Tuple[int, int]:
        """获取关节范围"""
        return (self.min_pos, self.max_pos)


# 关节定义 - 按协议顺序排列
# 顺序: 拇指指根弯曲、拇指指尖弯曲、拇指侧摆、拇指旋转、
#       无名指侧摆、无名指指尖弯曲、无名指指根弯曲、
#       中指指根弯曲、中指指尖弯曲、小指根弯曲、小指指尖弯曲、小指侧摆、
#       中指侧摆、食指侧摆、食指指根弯曲、食指指尖弯曲、手腕弯
JOINT_DEFINITIONS = [
    # 拇指 (4 DOF)
    JointInfo(1, "指根弯曲", "拇指", 0, 1000),
    JointInfo(2, "指尖弯曲", "拇指", 0, 1500),
    JointInfo(3, "侧摆", "拇指", 0, 1000),
    JointInfo(4, "旋转", "拇指", 0, 900),
    # 无名指 (3 DOF)
    JointInfo(5, "侧摆", "无名指", -200, 200),
    JointInfo(6, "指尖弯曲", "无名指", 0, 1500),
    JointInfo(7, "指根弯曲", "无名指", 0, 1600),
    # 中指 (3 DOF)
    JointInfo(8, "指根弯曲", "中指", 0, 1600),
    JointInfo(9, "指尖弯曲", "中指", 0, 1500),
    JointInfo(10, "侧摆", "中指", -200, 200),  # 注意：图片中中指侧摆是索引12
    # 小指 (3 DOF)
    JointInfo(11, "指根弯曲", "小指", 0, 1600),
    JointInfo(12, "指尖弯曲", "小指", 0, 1500),
    JointInfo(13, "侧摆", "小指", -200, 200),
    # 食指 (3 DOF)
    JointInfo(14, "侧摆", "食指", -200, 200),
    JointInfo(15, "指根弯曲", "食指", 0, 1600),
    JointInfo(16, "指尖弯曲", "食指", 0, 1500),
    # 手腕 (1 DOF)
    JointInfo(17, "俯仰", "手腕", -1000, 1000),
]

# 关节限位字典
JOINT_LIMITS = {
    1: (0, 1000),      # 0: 拇指指根弯曲
    2: (0, 1500),      # 1: 拇指指尖弯曲
    3: (0, 1000),      # 2: 拇指侧摆
    4: (0, 900),       # 3: 拇指旋转
    5: (-200, 200),    # 4: 无名指侧摆
    6: (0, 1500),      # 5: 无名指指尖弯曲
    7: (0, 1600),      # 6: 无名指指根弯曲
    8: (0, 1600),      # 7: 中指指根弯曲
    9: (0, 1500),      # 8: 中指指尖弯曲
    10: (0, 1600),     # 9: 小指根弯曲
    11: (0, 1500),     # 10: 小指指尖弯曲
    12: (-200, 200),   # 11: 小指侧摆
    13: (-200, 200),   # 12: 中指侧摆
    14: (-200, 200),   # 13: 食指侧摆
    15: (0, 1600),     # 14: 食指指根弯曲
    16: (0, 1500),     # 15: 食指指尖弯曲
    17: (-1000, 1000), # 16: 手腕
}
# 关节名称中英文映射
JOINT_NAME_EN = ["thumb_cmc_pitch", "thumb_ip_pitch", "thumb_cmc_yaw", "thumb_cmc_roll",
                 "ring_mcp_roll", "ring_pip_pitch", "ring_mcp_pitch",
                 "middle_mcp_pitch", "middle_pip_pitch",
                 "pinky_mcp_pitch", "pinky_pip_pitch", "pinky_mcp_roll",
                 "middle_mcp_roll", "index_mcp_roll",
                 "index_mcp_pitch", "index_pip_pitch",
                 "wrist_pitch"]

JOINT_NAME_CN = ["拇指指根弯曲", "拇指指尖弯曲", "拇指侧摆", "拇指旋转",
                 "无名指侧摆", "无名指指尖弯曲", "无名指指根弯曲",
                 "中指指根弯曲", "中指指尖弯曲", "小指根弯曲",
                 "小指指尖弯曲", "小指侧摆", "中指侧摆", "食指侧摆",
                 "食指指根弯曲", "食指指尖弯曲", "手腕"]

# 关节范围映射 (字符串键名版本)
JOINT_RANGE = {
    "thumb_cmc_pitch": (0, 1000),    # 拇指指根弯曲
    "thumb_mcp_pitch": (0, 1500),     # 拇指指尖弯曲
    "thumb_cmc_yaw": (0, 1000),      # 拇指侧摆
    "thumb_cmc_roll": (0, 900),      # 拇指旋转
    "ring_mcp_roll": (-200, 200),    # 无名指侧摆
    "ring_pip_pitch": (0, 1500),     # 无名指指尖弯曲
    "ring_mcp_pitch": (0, 1600),     # 无名指指根弯曲
    "middle_mcp_pitch": (0, 1600),   # 中指指根弯曲
    "middle_pip_pitch": (0, 1500),   # 中指指尖弯曲
    "pinky_mcp_pitch": (0, 1600),    # 小指根弯曲
    "pinky_pip_pitch": (0, 1500),    # 小指指尖弯曲
    "pinky_mcp_roll": (-200, 200),   # 小指侧摆
    "middle_mcp_roll": (-200, 200),  # 中指侧摆
    "index_mcp_roll": (-200, 200),   # 食指侧摆
    "index_mcp_pitch": (0, 1600),    # 食指指根弯曲
    "index_pip_pitch": (0, 1500),    # 食指指尖弯曲
    "wrist": (-1000, 1000)           # 手腕
}


# =============================================================================
# CANFD通信类
# =============================================================================

class L30CANFDProtocol:
    """L30灵巧手CANFD通信协议实现"""

    ARBITRATION_BAUD = 1000000
    DATA_BAUD = 5000000
    DEFAULT_DEVICE_ID = 0x06

    def __init__(self, device_id: int = 0x06, canfd_device=0, channel=0,
                 comm_type: str = "libcanbus", bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True):
        """
        Args:
            device_id: 灵巧手设备 ID
            canfd_device: CANFD 设备(盒子)索引, 仅 libcanbus 后端使用
            channel: 通道——libcanbus 为 int(默认 0), socketcan 为接口名 str(如 "can0")
            comm_type: 通讯后端 —— "libcanbus"(默认, 厂商私有库) 或
                       "socketcan"(内核 can0 + python-can, 透明塑封 USB-CANFD 设备)
            bitrate/dbitrate/auto_setup: 仅 socketcan 使用——仲裁/数据段波特率与
                       是否自动拉起接口
        """
        self.device_id = device_id
        self.canfd_device = canfd_device
        self.channel = channel
        self.comm_type = comm_type
        # 可插拔传输后端：libcanbus(默认) 或 socketcan；实际收发委托给它
        self.transport = create_transport(
            comm_type=comm_type, canfd_device=canfd_device, channel=channel,
            bitrate=bitrate, dbitrate=dbitrate, auto_setup=auto_setup)
        self.is_connected = False
        self.frame_counter = 0
        self._pressure_matrices = {
            'thumb': np.full((12, 6), -1),
            'index': np.full((12, 6), -1),
            'middle': np.full((12, 6), -1),
            'ring': np.full((12, 6), -1),
            'little': np.full((12, 6), -1),
        }

    # =========================================================================
    # 底层通信方法（委托可插拔传输后端）
    # =========================================================================

    def initialize(self) -> bool:
        """初始化CANFD通信（委托传输后端：libcanbus 或 socketcan）"""
        logger.info("正在初始化CANFD通信...")
        ok = self.transport.initialize()
        self.is_connected = ok
        if ok:
            logger.info("CANFD通信初始化完成")
        return ok

    def close(self) -> None:
        """关闭CANFD连接（委托传输后端）"""
        self.transport.close()
        self.is_connected = False
        logger.info("CANFD连接已关闭")

    def _increment_frame_counter(self) -> int:
        """递增帧计数器"""
        self.frame_counter = (self.frame_counter + 1) & 0xF
        return self.frame_counter

    def _build_canfd_id(
        self,
        priority: int = 0,
        direction: int = MessageDirection.REQUEST,
        rw: int = ReadWrite.WRITE,
        device_id: int = None,
        command: int = 0,
        subcommand: int = 0
    ) -> int:
        """
        构建CANFD扩展帧ID
        
        BIT[28:27] - 优先级
        BIT[26:26] - 消息方向 (0:请求, 1:应答)
        BIT[25:25] - R/W (0:读, 1:写)
        BIT[24:20] - 设备ID
        BIT[19:12] - 命令
        BIT[11:08] - 子命令
        BIT[07:00] - 保留
        """
        if device_id is None:
            device_id = self.device_id

        frame_id = 0
        frame_id |= (priority & 0x3) << 27
        frame_id |= (direction & 0x1) << 26
        frame_id |= (rw & 0x1) << 25
        frame_id |= (device_id & 0x1F) << 20
        frame_id |= (command & 0xFF) << 12
        frame_id |= (subcommand & 0xF) << 8
        return frame_id

    def _parse_canfd_id(self, frame_id: int) -> Dict:
        """解析CANFD扩展帧ID"""
        return {
            'priority': (frame_id >> 27) & 0x3,
            'direction': (frame_id >> 26) & 0x1,
            'rw': (frame_id >> 25) & 0x1,
            'device_id': (frame_id >> 20) & 0x1F,
            'command': (frame_id >> 12) & 0xFF,
            'subcommand': (frame_id >> 8) & 0xF,
        }

    def _build_transaction_control(self, total_frames: int = 0, seq_num: int = 0) -> int:
        """
        构建事务控制字节

        BIT[07:04] - 帧计数
        BIT[03:02] - 总帧数 (0:单帧, >0:多帧)
        BIT[01:00] - 多帧序号
        """
        return ((self.frame_counter & 0xF) << 4) | ((total_frames & 0x3) << 2) | (seq_num & 0x3)

    def _parse_transaction_control(self, control: int) -> Dict:
        """解析事务控制字节"""
        return {
            'frame_counter': (control >> 4) & 0xF,
            'total_frames': (control >> 2) & 0x3,
            'seq_num': control & 0x3,
        }

    def _pack_joint_data(self, values: List[int], min_val: int, max_val: int) -> bytes:
        """通用方法：将关节值打包为大端序字节"""
        data = bytearray()
        for val in values:
            clamped = max(min_val, min(max_val, int(val)))
            data.extend(clamped.to_bytes(2, byteorder='big', signed=True))
        return bytes(data)

    def _unpack_joint_data(self, data: bytes, offset: int, count: int) -> List[int]:
        """通用方法：解包关节数据"""
        values = []
        for i in range(count):
            val = int.from_bytes(data[offset + i*2:offset + i*2 + 2], byteorder='big', signed=True)
            values.append(val)
        return values

    def _check_response_status(self, data: bytes, min_len: int) -> Tuple[bool, int]:
        """检查响应状态"""
        if len(data) >= min_len:
            return data[2] == StatusCode.OK, data[2]
        return False, -1

    def send_message(
        self,
        command: int,
        data: bytes,
        is_write: bool = True,
        subcommand: int = 0
    ) -> bool:
        """
        发送CANFD消息（委托传输后端）

        Args:
            command: 命令码
            data: 数据载荷
            is_write: 是否为写操作
            subcommand: 子命令
        """
        if not self.is_connected:
            logger.error("错误: CANFD未连接")
            return False

        try:
            frame_id = self._build_canfd_id(
                priority=Priority.HIGH,
                direction=MessageDirection.REQUEST,
                rw=ReadWrite.WRITE if is_write else ReadWrite.READ,
                device_id=self.device_id,
                command=command,
                subcommand=subcommand
            )

            self._increment_frame_counter()
            data_len = min(len(data), 62)

            # 数据段有效字节: BYTE0 长度 / BYTE1 事务控制 / BYTE2~ 数据
            payload = bytearray(2 + data_len)
            payload[0] = data_len
            payload[1] = self._build_transaction_control()
            payload[2:2 + data_len] = bytes(data[:data_len])

            return self.transport.send(frame_id, bytes(payload))

        except Exception as e:
            logger.error(f"发送消息异常: {e}")
            return False

    def receive_messages(
        self,
        timeout_ms: int = 3,
        filter_device_id: bool = True,
        expected_command: int = None
    ) -> List[Tuple[int, bytes, Dict]]:
        """
        接收CANFD消息（委托传输后端取回原始帧后解析）

        Returns:
            List of (frame_id, data, parsed_info)
        """
        if not self.is_connected:
            return []

        try:
            messages = []
            for frame_id, data in self.transport.receive(timeout_ms):
                parsed = self._parse_canfd_id(frame_id)
                parsed['transaction'] = self._parse_transaction_control(data[1]) if len(data) > 1 else {}

                if filter_device_id and parsed['device_id'] != self.device_id:
                    continue

                if expected_command is not None and parsed['command'] != expected_command:
                    continue

                messages.append((frame_id, data, parsed))

            return messages

        except Exception as e:
            logger.error(f"接收消息异常: {e}")
            return []

    def _wait_for_response(
        self,
        expected_command: int,
        timeout_ms: int = 200,
        expected_rw: int = ReadWrite.READ
    ) -> Optional[Tuple[bytes, int]]:
        """
        等待并获取响应

        Returns:
            (data, status_code) or None
        """
        start_time = time.time()
        while time.time() - start_time < timeout_ms / 1000:
            messages = self.receive_messages(
                timeout_ms=50,
                expected_command=expected_command
            )

            for frame_id, data, parsed in messages:
                if parsed['direction'] == MessageDirection.RESPONSE:
                    if parsed['rw'] == expected_rw:
                        status = data[2] if len(data) > 2 else 0
                        return data, status

            time.sleep(0.005)

        return None

    # =========================================================================
    # 设备操作
    # =========================================================================

    def enable_all_joints(self) -> bool:
        """
        使能所有17个关节

        重要: 在进行其他操作之前必须先调用此函数使能关节!
        """
        return self.set_joint_enable([1] * 17)

    def query_device_type(self) -> Optional[str]:
        """查询设备类型（左手/右手）"""
        if not self.send_message(Command.DEVICE_CANFDID, b'', is_write=False):
            return None

        response = self._wait_for_response(Command.DEVICE_CANFDID)

        if response:
            data, status = response
            if status == 0 and len(data) >= 5:
                device_id = data[3]
                hand_type = "right" if data[4] == HandType.RIGHT else "left"
                logger.info(f"检测到设备ID: {device_id}, 类型: {hand_type}")
                return hand_type

        return None

    def get_device_version(self) -> Optional[Dict]:
        """读取设备版本"""
        if not self.send_message(Command.DEVICE_VERSION, b'', is_write=False):
            return None

        response = self._wait_for_response(Command.DEVICE_VERSION)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 12:
                def parse_version(d, start):
                    return f"{d[start]}.{d[start + 1]}.{d[start + 2]}"

                return {
                    'hardware': parse_version(data, 3),
                    'software': parse_version(data, 6),
                    'mechanical': parse_version(data, 9)
                }

        return None

    def set_device_id(self, device_id: int, hand_type: HandType) -> bool:
        """设置设备ID和左右手"""
        return self.send_message(Command.DEVICE_CANFDID, bytes([device_id, hand_type]), is_write=True)

    def restore_device_id(self) -> bool:
        """还原设备ID为0x01"""
        return self.send_message(Command.RESTORE_DEVICE_ID, b'', is_write=True)

    # =========================================================================
    # 关节控制
    # =========================================================================

    def set_joint_positions(self, positions: List[int]) -> bool:
        """设置17个关节位置"""
        if len(positions) != 17:
            logger.error(f"位置数据长度错误: 期望17个，实际{len(positions)}")
            return False

        data = bytearray()
        for i, pos in enumerate(positions):
            min_val, max_val = JOINT_LIMITS.get(i + 1, (-32768, 32767))
            clamped_pos = max(min_val, min(max_val, int(pos)))
            data.extend(clamped_pos.to_bytes(2, byteorder='big', signed=True))

        return self.send_message(Command.JOINT_POSITION, bytes(data), is_write=True)

    def get_joint_positions(self) -> Optional[List[int]]:
        """读取17个关节位置"""
        if not self.send_message(Command.JOINT_POSITION, b'\x00', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_POSITION)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 36:
                return self._unpack_joint_data(data, 3, 17)

        return None

    def set_joint_torques(self, torques: List[int]) -> bool:
        """设置17个关节扭矩 (范围-2047~2047，单位6.5mA)"""
        if len(torques) != 17:
            logger.error(f"扭矩数据长度错误: 期望17个，实际{len(torques)}")
            return False

        return self.send_message(Command.JOINT_TORQUE,
                                self._pack_joint_data(torques, -2047, 2047),
                                is_write=True)

    def get_joint_torques(self) -> Optional[List[int]]:
        """读取17个关节扭矩"""
        if not self.send_message(Command.JOINT_TORQUE, b'\x00\x10', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_TORQUE, expected_rw=ReadWrite.READ)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 38:
                return self._unpack_joint_data(data, 3, 17)

        return None

    def set_joint_torque_limits(self, limits: List[int]) -> bool:
        """设置17个关节转矩限制 (范围0~1000，单位0.1%)"""
        if len(limits) != 17:
            logger.error(f"转矩限制数据长度错误: 期望17个，实际{len(limits)}")
            return False

        return self.send_message(Command.JOINT_TORQUE_LIMIT,
                                self._pack_joint_data(limits, 0, 1000),
                                is_write=True)

    def set_joint_velocities(self, velocities: List[int]) -> bool:
        """设置17个关节速度 (范围-32767~32767，单位0.732RPM)"""
        if len(velocities) != 17:
            logger.error(f"速度数据长度错误: 期望17个，实际{len(velocities)}")
            return False

        return self.send_message(Command.JOINT_SPEED,
                                self._pack_joint_data(velocities, 0, 150),
                                is_write=True)

    def get_joint_velocities(self) -> Optional[List[int]]:
        """读取17个关节速度"""
        if not self.send_message(Command.JOINT_SPEED, b'\x00', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_SPEED)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 36:
                return self._unpack_joint_data(data, 3, 17)

        return None

    def set_joint_accelerations(self, accelerations: List[int]) -> bool:
        """设置17个关节加速度 (范围0~254，单位8.7度/秒²)"""
        if len(accelerations) != 17:
            logger.error(f"加速度数据长度错误: 期望17个，实际{len(accelerations)}")
            return False

        data = bytes(max(0, min(254, int(acc))) for acc in accelerations)
        return self.send_message(Command.JOINT_ACCELERATION, data, is_write=True)

    def get_joint_accelerations(self) -> Optional[List[int]]:
        """读取17个关节加速度"""
        if not self.send_message(Command.JOINT_ACCELERATION, b'\x00\x10', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_ACCELERATION, expected_rw=ReadWrite.READ)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 20:
                return list(data[3:20])

        return None

    def set_joint_enable(self, enables: List[int]) -> bool:
        """设置17个关节使能状态"""
        if len(enables) != 17:
            logger.error(f"使能数据长度错误: 期望17个，实际{len(enables)}")
            return False

        data = bytes(1 if e else 0 for e in enables)
        return self.send_message(Command.JOINT_ENABLE, data, is_write=True)

    def get_joint_enable(self) -> Optional[List[int]]:
        """读取17个关节使能状态"""
        if not self.send_message(Command.JOINT_ENABLE, b'\x00', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_ENABLE)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 20:
                return list(data[3:20])

        return None

    def get_joint_temperatures(self) -> Optional[List[int]]:
        """读取17个关节温度 (单位°C)"""
        if not self.send_message(Command.JOINT_TEMPERATURE, b'\x00', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_TEMPERATURE)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 20:
                return list(data[3:20])

        return None

    def get_joint_error_codes(self) -> Optional[List[int]]:
        """读取17个关节错误码"""
        if not self.send_message(Command.JOINT_ERROR_CODE, b'\x00', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_ERROR_CODE)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 20:
                return list(data[3:20])

        return None

    def get_joint_currents(self) -> Optional[List[int]]:
        """读取17个关节当前电流 (单位6.5mA)"""
        if not self.send_message(Command.JOINT_CURRENT, b'\x00', is_write=False):
            return None

        response = self._wait_for_response(Command.JOINT_CURRENT)
        if response:
            data, status = response
            if status == StatusCode.OK and len(data) >= 36:
                return self._unpack_joint_data(data, 3, 17)

        return None

    def calibrate_zero_point(self) -> bool:
        """标定零点 - 执行后将手放入模具，等待1秒左右进入失能模式"""
        return self.send_message(Command.CALIBRATE_ZERO, b'\x00', is_write=True)

    def emergency_stop(self) -> bool:
        """急停"""
        return self.send_message(Command.EMERGENCY_STOP, b'\x00', is_write=True)

    # =========================================================================
    # 触觉传感器
    # =========================================================================

    def get_finger_pressure(self, command: Command) -> Optional[List[int]]:
        """
        读取手指压力数据

        Args:
            command: 0xB1-0xB5 对应拇指到小指

        Returns:
            72字节压力数据列表，或 None
        """
        if not self.send_message(command, b'\x00', is_write=False):
            return None

        frame1_data = None
        frame2_data = None
        start_time = time.time()

        while time.time() - start_time < 0.002:
            messages = self.receive_messages(timeout_ms=2, filter_device_id=True, expected_command=command)

            for frame_id, data, parsed in messages:
                if len(data) < 4:
                    continue

                data_len = data[0]

                if data_len == 0x3E and frame1_data is None:
                    frame1_data = list(data[3:])
                elif data_len == 0x0C and frame1_data is not None and frame2_data is None:
                    frame2_data = list(data[3:-2])
                    return frame1_data + frame2_data

            # time.sleep(0.001)

        if frame1_data is not None and frame2_data is not None:
            return frame1_data + frame2_data
        return None

    def _process_pressure_matrix(self, data: Optional[List[int]], key: str) -> np.ndarray:
        """处理压力矩阵数据"""
        if data is not None:
            arr = np.array(data).reshape(12, 6)
            self._pressure_matrices[key] = arr[::-1]
        return self._pressure_matrices[key]

    def get_thumb_pressure(self) -> np.ndarray:
        """读取拇指压力数据"""
        return self._process_pressure_matrix(self.get_finger_pressure(Command.THUMB_PRESSURE), 'thumb')

    def get_index_pressure(self) -> np.ndarray:
        """读取食指压力数据"""
        return self._process_pressure_matrix(self.get_finger_pressure(Command.INDEX_PRESSURE), 'index')

    def get_middle_pressure(self) -> np.ndarray:
        """读取中指压力数据"""
        return self._process_pressure_matrix(self.get_finger_pressure(Command.MIDDLE_PRESSURE), 'middle')

    def get_ring_pressure(self) -> np.ndarray:
        """读取无名指压力数据"""
        return self._process_pressure_matrix(self.get_finger_pressure(Command.RING_PRESSURE), 'ring')

    def get_little_pressure(self) -> np.ndarray:
        """读取小指压力数据"""
        return self._process_pressure_matrix(self.get_finger_pressure(Command.PINKY_PRESSURE), 'little')

    def get_all_pressures(self) -> Dict[str, List[int]]:
        """读取所有手指压力数据"""
        return {
            'thumb_matrix': self.get_thumb_pressure().tolist(),
            'index_matrix': self.get_index_pressure().tolist(),
            'middle_matrix': self.get_middle_pressure().tolist(),
            'ring_matrix': self.get_ring_pressure().tolist(),
            'little_matrix': self.get_little_pressure().tolist()
        }


# =============================================================================
# 灵巧手控制器
# =============================================================================

class L30DexterousHandController:
    """L30灵巧手高级控制器"""

    JOINT_COUNT = 17

    def __init__(self, device_id: int = 0x06, canfd_id=0, comm_type: str = "libcanbus",
                 channel=None, bitrate: int = 1000000, dbitrate: int = 5000000,
                 auto_setup: bool = True):
        """
        Args:
            device_id: 灵巧手设备 ID
            canfd_id: CANFD 设备(盒子)索引
            comm_type: 通讯后端 —— "libcanbus"(默认, 厂商私有库) 或
                       "socketcan"(内核 can0 + python-can, 透明塑封 USB-CANFD 设备)
            channel: 通道——libcanbus 为 int(默认 0), socketcan 为接口名(默认 "can0")
            bitrate/dbitrate/auto_setup: 仅 socketcan 使用
        """
        # channel 未显式指定时按后端给默认值: socketcan -> "can0", libcanbus -> 0
        if channel is None:
            channel = "can0" if comm_type == "socketcan" else 0
        self.protocol = L30CANFDProtocol(device_id, canfd_id, channel=channel,
                                         comm_type=comm_type, bitrate=bitrate,
                                         dbitrate=dbitrate, auto_setup=auto_setup)
        self.device_id = device_id
        self.hand_type: Optional[str] = None
        self.joints = {joint.id: joint for joint in JOINT_DEFINITIONS}

    def connect(self, enable_on_connect=True) -> Tuple[bool, Optional[str]]:
        """连接灵巧手"""
        logger.info("开始连接灵巧手...")

        if not self.protocol.initialize():
            return False, None

        logger.info("使能所有关节...")
        print("使能所有关节...")
        if enable_on_connect and not self.protocol.enable_all_joints():
            logger.warning("关节使能失败, 继续尝试...")
            print("关节使能失败, 继续尝试...")
        time.sleep(0.1)

        # 设备类型查询偶发超时/丢包，重试若干次避免误判为 hand_type 不匹配
        hand_type = None
        for _ in range(5):
            hand_type = self.protocol.query_device_type()
            if hand_type:
                break
            time.sleep(0.05)
        if hand_type:
            self.hand_type = hand_type
            logger.info(f"连接成功: 设备ID={self.device_id}, 类型={hand_type}")
            return True, hand_type

        return True, "未知"

    def disconnect(self) -> None:
        """断开连接"""
        self.protocol.close()

    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.protocol.is_connected

    def set_positions(self, positions: List[int]) -> bool:
        """设置关节位置"""
        if len(positions) != self.JOINT_COUNT:
            raise ValueError(f"需要{self.JOINT_COUNT}个关节值")
        clamped = [self._clamp_joint(i + 1, pos) for i, pos in enumerate(positions)]
        return self.protocol.set_joint_positions(clamped)

    def _clamp_joint(self, joint_id: int, value: int) -> int:
        """将关节值限制在范围内"""
        min_val, max_val = JOINT_LIMITS.get(joint_id, (-32768, 32767))
        return max(min_val, min(max_val, int(value)))

    def get_positions(self) -> Optional[List[int]]:
        """获取关节位置"""
        return self.protocol.get_joint_positions()

    def set_velocities(self, velocities: List[int]) -> bool:
        """设置关节速度"""
        return self.protocol.set_joint_velocities(velocities)

    def get_velocities(self) -> Optional[List[int]]:
        """获取关节速度"""
        return self.protocol.get_joint_velocities()

    def set_torques(self, torques: List[int]) -> bool:
        """设置关节扭矩"""
        return self.protocol.set_joint_torques(torques)

    def get_torques(self) -> Optional[List[int]]:
        """获取关节扭矩"""
        return self.protocol.get_joint_torques()

    def set_enable(self, enables: List[int]) -> bool:
        """设置关节使能"""
        return self.protocol.set_joint_enable(enables)

    def calibrate(self) -> bool:
        """标定零点"""
        return self.protocol.calibrate_zero_point()

    def stop(self) -> bool:
        """急停"""
        return self.protocol.emergency_stop()

    def enable_all(self) -> bool:
        """使能所有关节"""
        return self.protocol.enable_all_joints()

    def get_matrix_touch(self) -> Dict[str, List[int]]:
        """获取触摸矩阵"""
        return self.protocol.get_all_pressures()

    def get_joint_name(self) -> Tuple[List[str], List[str]]:
        """获取关节名称"""
        return JOINT_NAME_EN, JOINT_NAME_CN

    def get_joint_range(self) -> Dict[str, Tuple[int, int]]:
        """获取关节范围"""
        return JOINT_RANGE

    def get_all_state(self,is_touch=False) -> Optional[Dict]:
        """获取完整状态"""
        all_state =  {
            'positions': self.protocol.get_joint_positions(),
            'velocities': self.protocol.get_joint_velocities(),
            'currents': self.protocol.get_joint_currents(),
            'temperatures': self.protocol.get_joint_temperatures(),
            'error_codes': self.protocol.get_joint_error_codes()
        }
        if is_touch == True:
            all_state['matrix_touch'] = self.get_matrix_touch()
        return all_state

    def normalize_positions(self, raw_positions: List[int]) -> List[float]:
        """将原始位置值归一化到0-1范围"""
        normalized = []
        for i, pos in enumerate(raw_positions):
            motor_id = i + 1
            if motor_id in JOINT_LIMITS:
                min_val, max_val = JOINT_LIMITS[motor_id]
                if max_val != min_val:
                    norm = (pos - min_val) / (max_val - min_val)
                    normalized.append(max(0, min(1, norm)))
                else:
                    normalized.append(0.5)
            else:
                normalized.append(0.5)
        return normalized

    def denormalize_positions(self, normalized: List[float]) -> List[int]:
        """将0-1归一化值转换回原始位置"""
        positions = []
        for i, norm in enumerate(normalized):
            motor_id = i + 1
            if motor_id in JOINT_LIMITS:
                min_val, max_val = JOINT_LIMITS[motor_id]
                pos = min_val + norm * (max_val - min_val)
                positions.append(int(round(pos)))
            else:
                positions.append(0)
        return positions

    def update_joint_states(self) -> None:
        """更新所有关节状态"""
        positions = self.get_positions()
        if positions:
            for i, pos in enumerate(positions):
                joint_id = i + 1
                if joint_id in self.joints:
                    self.joints[joint_id].current_pos = pos
                    self.joints[joint_id].target_pos = pos

        currents = self.protocol.get_joint_currents()
        if currents:
            for i, current in enumerate(currents):
                joint_id = i + 1
                if joint_id in self.joints:
                    self.joints[joint_id].current = current

        temperatures = self.protocol.get_joint_temperatures()
        if temperatures:
            for i, temp in enumerate(temperatures):
                joint_id = i + 1
                if joint_id in self.joints:
                    self.joints[joint_id].temperature = temp

        error_codes = self.protocol.get_joint_error_codes()
        if error_codes:
            for i, code in enumerate(error_codes):
                joint_id = i + 1
                if joint_id in self.joints:
                    self.joints[joint_id].error_code = code

    def get_joint_state(self, joint_id: int) -> Optional[Dict]:
        """获取单个关节状态"""
        if joint_id in self.joints:
            return {
                'id': self.joints[joint_id].id,
                'name': self.joints[joint_id].name,
                'finger': self.joints[joint_id].finger,
                'current_pos': self.joints[joint_id].current_pos,
                'target_pos': self.joints[joint_id].target_pos,
                'current': self.joints[joint_id].current,
                'temperature': self.joints[joint_id].temperature,
                'error_code': self.joints[joint_id].error_code,
                'enabled': self.joints[joint_id].enabled,
                'range': self.joints[joint_id].range
            }
        return None

    def __enter__(self) -> 'L30DexterousHandController':
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """上下文管理器出口"""
        self.disconnect()


# =============================================================================
# 便捷函数
# =============================================================================

def create_default_controller(device_id: int = 0x01) -> L30DexterousHandController:
    """创建默认控制器"""
    return L30DexterousHandController(device_id)


def setup_logging(level: int = logging.INFO) -> None:
    """配置日志"""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

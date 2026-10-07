#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L30灵巧手 CANFD 扩展帧通讯控制程序（新版协议 v1.0.6）

依据文档《L30灵巧手CANFD扩展帧通讯协议.md》实现。

协议要点:
- 物理接口: CANFD, 扩展帧, 仲裁段 1Mbps(80%), 数据段 5Mbps(75%)
- 多字节数据统一使用 大端(Big-Endian) 字节序
- CANFDID(29 位)结构:
    BIT28:26 优先级(3) | BIT25 访问类型(1, 0=读/1=写) | BIT24:21 父命令(4)
    | BIT20:13 子命令(8) | BIT12:8 DstID(5) | BIT7:3 SrcID(5) | BIT2:0 保留(3)
    CAN_ID = (Pri<<26)|(Access<<25)|(Parent<<21)|(Sub<<13)|(Dst<<8)|(Src<<3)
- 帧数据段:
    BYTE0 数据长度(请求从BYTE2起算, 应答从BYTE3起算, 不含状态码)
    BYTE1 事务控制 = (N<<4)|seq, 单帧固定 0x00, 多帧实际帧数 K=N+1
    请求 BYTE2~   : 写数据
    应答 BYTE2    : 状态码; BYTE3~ : 读数据

本文件底层 CANFD 通讯方法(初始化/收发)参考旧版 linker_hand_l30_v6_canfd.py。

版本: 1.0
日期: 2026-07-22
"""

import time
import logging
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass
from enum import IntEnum
from ctypes import (
    Structure, CDLL, cdll, cast, byref, RTLD_GLOBAL,
    c_uint, c_ushort, c_char, c_ubyte, c_uint16, POINTER,
)

import numpy as np

from .linker_hand_l30_canfd_comm import (
    LibCanBusTransport,
    SocketCANTransport,
    create_transport,
)

logger = logging.getLogger(__name__)


# =============================================================================
# CANFD 底层库常量与结构体（与旧版一致）
# =============================================================================

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


# DLC 编码 -> 线路字节数（CAN FD 标准表, 见协议 §2.2）
DLC_TO_LENGTH = {
    0x00: 0, 0x01: 1, 0x02: 2, 0x03: 3, 0x04: 4, 0x05: 5, 0x06: 6, 0x07: 7,
    0x08: 8, 0x09: 12, 0x0A: 16, 0x0B: 20, 0x0C: 24, 0x0D: 32, 0x0E: 48, 0x0F: 64,
}
# 少数设备可能回填非标准 DLC 值
NON_STANDARD_DLC_MAP = {0x10: 16, 0x40: 64}


def get_dlc_from_length(length: int) -> int:
    """
    根据实际线路字节数, 向上取最接近的合法 DLC 编码。

    CAN FD 的 DLC 只能取标准值(0~8,12,16,20,24,32,48,64), 当有效字节数
    介于两个标准值之间时, 取更大的那一档, 不足的字节由设备固定补 0x00。

    Args:
        length: 需要传输的线路字节总数(含 BYTE0 数据长度 + BYTE1 事务控制 + 数据)
    Returns:
        对应的 DLC 编码值(0x00~0x0F)
    """
    if length <= 8:
        return length          # 0~8 字节: DLC 编码与字节数一一对应
    if length <= 12:
        return 0x09            # 9~12  字节 -> 12 字节帧
    if length <= 16:
        return 0x0A            # 13~16 字节 -> 16 字节帧
    if length <= 20:
        return 0x0B            # 17~20 字节 -> 20 字节帧
    if length <= 24:
        return 0x0C            # 21~24 字节 -> 24 字节帧
    if length <= 32:
        return 0x0D            # 25~32 字节 -> 32 字节帧
    if length <= 48:
        return 0x0E            # 33~48 字节 -> 48 字节帧
    return 0x0F               # 49~64 字节 -> 64 字节帧


def get_length_from_dlc(dlc: int) -> int:
    """
    根据接收到的 DLC 编码, 反查线路字节数。

    优先查标准表; 若设备回填了非标准值(如 0x10/0x40)则用兼容表;
    再兜底按数值截断到 64。

    Args:
        dlc: 接收帧中的 DLC 字段值
    Returns:
        该帧的线路字节数
    """
    if dlc in DLC_TO_LENGTH:
        return DLC_TO_LENGTH[dlc]
    if dlc in NON_STANDARD_DLC_MAP:
        return NON_STANDARD_DLC_MAP[dlc]
    return min(dlc, 64)


# =============================================================================
# 协议常量定义（新版）
# =============================================================================

class Priority(IntEnum):
    """CANFDID 仲裁优先级, 数值越小优先级越高"""
    HIGHEST = 0
    LOWEST = 7


class Access(IntEnum):
    """访问类型 BIT25"""
    READ = 0
    WRITE = 1


class ParentCmd(IntEnum):
    """父命令 BIT24:21"""
    MULTI_JOINT = 0x1     # 多关节控制
    TOUCH = 0x2           # 触觉传感器
    CONFIG = 0x3          # 配置信息
    PERIODIC = 0x4        # 周期上报
    QUERY = 0x5           # 单次查询
    SINGLE_JOINT = 0x6    # 单关节调试控制
    BOOTLOADER = 0xF      # 固件升级


class MultiJointSub(IntEnum):
    """父命令 0x1 子命令"""
    POSITION = 0x01
    TORQUE = 0x02
    SPEED = 0x03
    STOP = 0x05           # 全局急停(暂不支持)
    PAUSE = 0x06          # 全局暂停(暂不支持)
    ENABLE = 0x07         # 全局使能
    DISABLE = 0x08        # 全局失能


class TouchSub(IntEnum):
    """父命令 0x2 子命令（单指触觉）"""
    THUMB = 0x01
    INDEX = 0x02
    MIDDLE = 0x03
    RING = 0x04
    PINKY = 0x05


class ConfigSub(IntEnum):
    """父命令 0x3 子命令"""
    UNLOCK = 0x01
    DEVICE_INFO = 0x02
    PRODUCT_CODE = 0x03
    NODE_ID = 0x04
    CALIBRATE_ZERO = 0x05
    HAND_TYPE = 0x06


class QuerySub(IntEnum):
    """父命令 0x5 子命令（单次查询）"""
    POSITION = 0x01
    CURRENT = 0x02
    SPEED = 0x03
    TEMPERATURE = 0x04
    ERROR_CODE = 0x05


class PeriodicSub(IntEnum):
    """父命令 0x4 子命令（周期上报）"""
    POSITION = 0x01
    CURRENT = 0x02
    SPEED = 0x03
    TEMPERATURE = 0x04
    ERROR_CODE = 0x05


class SingleJointFunc(IntEnum):
    """父命令 0x6 功能码, 子命令 = (功能码<<5)|关节编号"""
    POSITION = 0x1
    TORQUE = 0x2
    SPEED = 0x3


class HandType(IntEnum):
    """左右手类型"""
    LEFT = 0x00
    RIGHT = 0x01


class StatusCode(IntEnum):
    """通用状态码（父命令 0x1~0x6, 见协议 §8.2）"""
    OK = 0x00
    ERR_DLC = 0x10
    ERR_PARAM = 0x11
    ERR_CMD = 0x12
    ERR_SUBCMD = 0x13
    ERR_FORMAT = 0x14
    ERR_PERMISSION = 0x20      # 权限不足（未解锁 / 密码错误）
    ERR_NOT_CALIBRATED = 0x21
    ERR_MOTOR_DISABLED = 0x22
    ERR_STATE = 0x23
    ERR_NOT_SET = 0x24
    ERR_READ = 0x30
    ERR_WRITE = 0x31
    ERR_MULTIFRAME_TIMEOUT = 0x32
    ERR_MULTIFRAME_INCOMPLETE = 0x33
    ERR_PERIOD_CFG = 0x34
    ERR_PERIOD_RANGE = 0x35
    ERR_MASK = 0x36
    ERR_COMM_TIMEOUT = 0x40
    ERR_COMM_LOST = 0x41
    ERR_BUSY = 0xF0


class Command(IntEnum):
    """
    [兼容旧模块] 旧版 linker_hand_l30_v6_canfd.py 的命令码枚举。

    新版协议实际使用 ParentCmd + 各子命令(MultiJointSub/QuerySub 等)组织命令,
    此枚举仅为保持与旧模块相同的对外导入接口而保留, 使 linker_hand_l30_api.py
    的 `from ...core.xxx import Command` 在两个模块间都能成功导入。
    """
    JOINT_POSITION = 0x01
    JOINT_TORQUE = 0x02
    JOINT_TORQUE_LIMIT = 0x03
    JOINT_SPEED = 0x05
    JOINT_ACCELERATION = 0x07
    JOINT_ENABLE = 0x08
    JOINT_TEMPERATURE = 0x33
    JOINT_ERROR_CODE = 0x35
    JOINT_CURRENT = 0x36
    CALIBRATE_ZERO = 0x37
    THUMB_PRESSURE = 0xB1
    INDEX_PRESSURE = 0xB2
    MIDDLE_PRESSURE = 0xB3
    RING_PRESSURE = 0xB4
    PINKY_PRESSURE = 0xB5
    DEVICE_CODE = 0xC0
    DEVICE_VERSION = 0xC1
    DEVICE_CANFDID = 0xC3
    RESTORE_DEVICE_ID = 0xC4


# 配置解锁默认密码（见协议 §12.1）
UNLOCK_PASSWORD = bytes([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC])

# 关节数量
JOINT_COUNT = 17


# =============================================================================
# 关节定义（左手默认范围, 见协议 §9.1；数值为 int16 大端语义位置量）
# =============================================================================

@dataclass
class JointInfo:
    id: int
    name: str
    finger: str
    min_pos: int
    max_pos: int

    @property
    def range(self) -> Tuple[int, int]:
        """返回该关节的 (最小位置, 最大位置) 元组。"""
        return (self.min_pos, self.max_pos)


JOINT_DEFINITIONS = [
    JointInfo(1,  "拇指指根弯曲", "拇指",   0,  900),
    JointInfo(2,  "拇指指尖弯曲", "拇指",   0, 1200),
    JointInfo(3,  "拇指侧摆",    "拇指",   0,  900),
    JointInfo(4,  "拇指旋转",    "拇指",   0,  800),
    JointInfo(5,  "无名指侧摆",   "无名指", -200,  200),
    JointInfo(6,  "无名指指尖弯曲", "无名指",  0, 1200),
    JointInfo(7,  "无名指指根弯曲", "无名指",  0, 1200),
    JointInfo(8,  "中指指根弯曲", "中指",   0, 1200),
    JointInfo(9,  "中指指尖弯曲", "中指",   0, 1200),
    JointInfo(10, "小指根弯曲",   "小指",   0, 1500),
    JointInfo(11, "小指指尖弯曲", "小指",   0, 1200),
    JointInfo(12, "小指侧摆",    "小指", -200,  200),
    JointInfo(13, "中指侧摆",    "中指", -200,  200),
    JointInfo(14, "食指侧摆",    "食指", -200,  200),
    JointInfo(15, "食指指根弯曲", "食指",   0, 1200),
    JointInfo(16, "食指指尖弯曲", "食指",   0, 1200),
    JointInfo(17, "手腕",       "手腕", -900,  900),
]

# 关节限位字典 {关节编号(1~17): (min, max)}
JOINT_LIMITS = {j.id: (j.min_pos, j.max_pos) for j in JOINT_DEFINITIONS}

# 关节名称（英文 / 中文, 按 J1~J17 顺序）
JOINT_NAME_EN = [
    "thumb_cmc_pitch", "thumb_ip_pitch", "thumb_cmc_yaw", "thumb_cmc_roll",
    "ring_mcp_roll", "ring_pip_pitch", "ring_mcp_pitch",
    "middle_mcp_pitch", "middle_pip_pitch",
    "pinky_mcp_pitch", "pinky_pip_pitch", "pinky_mcp_roll",
    "middle_mcp_roll", "index_mcp_roll",
    "index_mcp_pitch", "index_pip_pitch",
    "wrist_pitch",
]
JOINT_NAME_CN = [j.name for j in JOINT_DEFINITIONS]

# 触觉传感器矩阵规格（华威科 12x6）
TOUCH_ROWS, TOUCH_COLS = 12, 6
TOUCH_BYTES = TOUCH_ROWS * TOUCH_COLS  # 72


# =============================================================================
# CANFD 通讯协议实现
# =============================================================================

class L30CANFDProtocol:
    """L30 灵巧手 CANFD 扩展帧通讯协议（新版 v1.0.6）"""

    ARBITRATION_BAUD = 1000000   # 仲裁段 1Mbps
    DATA_BAUD = 5000000          # 数据段 5Mbps
    DEFAULT_NODE_ID = 1          # 设备默认 NodeID(DstID)
    MASTER_ID = 0                # 主站 SrcID

    def __init__(self, node_id: int = DEFAULT_NODE_ID, src_id: int = MASTER_ID,
                 canfd_device: int = 0, channel=0, comm_type: str = "libcanbus",
                 bitrate: int = 1000000, dbitrate: int = 5000000,
                 auto_setup: bool = True):
        """
        构造协议对象(不会立即连接, 需另行调用 initialize)。

        Args:
            node_id: 设备节点 ID(DstID), 范围 1~31; 收发时用于寻址与应答过滤
            src_id:  主站源 ID(SrcID), 主站默认 0
            canfd_device: CANFD 设备(盒子)索引, 仅 libcanbus 后端使用
            channel: 通道号——libcanbus 为 int(默认 0), socketcan 为接口名 str(如 "can0")
            comm_type: 通讯后端 —— "libcanbus"(默认, 厂商私有库) 或
                       "socketcan"(内核 can0 + python-can, 透明塑封 USB-CANFD 设备)
            bitrate/dbitrate/auto_setup: 仅 socketcan 使用——仲裁/数据段波特率与
                       是否自动拉起接口
        """
        self.node_id = node_id
        self.src_id = src_id
        self.canfd_device = canfd_device
        self.channel = channel
        self.comm_type = comm_type
        # 可插拔传输后端：libcanbus(默认) 或 socketcan；实际收发委托给它
        self.transport = create_transport(
            comm_type=comm_type, canfd_device=canfd_device, channel=channel,
            bitrate=bitrate, dbitrate=dbitrate, auto_setup=auto_setup)
        self.is_connected = False     # 连接状态标志(由 initialize/close 维护)
        # 5 指触觉矩阵缓存(12x6), 初始值 -1 表示尚未读到有效数据
        self._touch_matrices = {
            'thumb': np.full((TOUCH_ROWS, TOUCH_COLS), -1),
            'index': np.full((TOUCH_ROWS, TOUCH_COLS), -1),
            'middle': np.full((TOUCH_ROWS, TOUCH_COLS), -1),
            'ring': np.full((TOUCH_ROWS, TOUCH_COLS), -1),
            'little': np.full((TOUCH_ROWS, TOUCH_COLS), -1),
        }

    # =========================================================================
    # 底层通讯（初始化 / 关闭 / 收发, 委托可插拔传输后端）
    # =========================================================================

    def initialize(self) -> bool:
        """
        初始化 CANFD 通讯（委托传输后端）。

        libcanbus: 加载库 -> 扫描 -> 打开通道 -> CANFD 配置 -> 过滤器；
        socketcan: 自动拉起 can0 接口 -> 打开 python-can 总线。

        Returns:
            初始化成功返回 True, 否则 False。
        """
        logger.info("正在初始化CANFD通信...")
        ok = self.transport.initialize()
        self.is_connected = ok
        if ok:
            logger.info("CANFD通信初始化完成")
        return ok

    def close(self) -> None:
        """关闭 CANFD 连接并复位连接标志（委托传输后端）。"""
        self.transport.close()
        self.is_connected = False
        logger.info("CANFD连接已关闭")

    # =========================================================================
    # CANFDID / 事务控制 编解码
    # =========================================================================

    def _build_canfd_id(self, access: int, parent_cmd: int, sub_cmd: int,
                        dst_id: Optional[int] = None, src_id: Optional[int] = None,
                        priority: int = Priority.HIGHEST) -> int:
        """
        构建 29 位扩展帧 CANFDID。

        位域: CAN_ID = (Pri<<26)|(Access<<25)|(Parent<<21)|(Sub<<13)|(Dst<<8)|(Src<<3)

        Args:
            access: 访问类型(0=读/1=写), 占 BIT25
            parent_cmd: 父命令(0x1~0xF), 占 BIT24:21
            sub_cmd: 子命令(0x00~0xFF), 占 BIT20:13
            dst_id: 目标节点 ID, 默认取本机 node_id, 占 BIT12:8
            src_id: 源节点 ID, 默认取本机 src_id, 占 BIT7:3
            priority: 仲裁优先级(0 最高), 占 BIT28:26
        Returns:
            组装好的 29 位 CANFDID 整数。
        """
        if dst_id is None:
            dst_id = self.node_id
        if src_id is None:
            src_id = self.src_id
        # 逐字段掩码后移位到对应位置, 最后按位或合并
        frame_id = (priority & 0x7) << 26
        frame_id |= (access & 0x1) << 25
        frame_id |= (parent_cmd & 0xF) << 21
        frame_id |= (sub_cmd & 0xFF) << 13
        frame_id |= (dst_id & 0x1F) << 8
        frame_id |= (src_id & 0x1F) << 3
        return frame_id

    def _parse_canfd_id(self, frame_id: int) -> Dict:
        """
        解析 29 位 CANFDID, 拆出各位域。

        Args:
            frame_id: 接收帧的 29 位 ID
        Returns:
            含 priority/access/parent_cmd/sub_cmd/dst_id/src_id 的字典。
        """
        return {
            'priority': (frame_id >> 26) & 0x7,
            'access': (frame_id >> 25) & 0x1,
            'parent_cmd': (frame_id >> 21) & 0xF,
            'sub_cmd': (frame_id >> 13) & 0xFF,
            'dst_id': (frame_id >> 8) & 0x1F,
            'src_id': (frame_id >> 3) & 0x1F,
        }

    @staticmethod
    def _build_transaction(total_field: int = 0, seq: int = 0) -> int:
        """
        构建事务控制字节 BYTE[1] = (N<<4)|seq。

        Args:
            total_field: 高 4 位总帧数字段 N(单帧填 0, 多帧实际帧数 K=N+1)
            seq: 低 4 位帧序号(单帧 0, 多帧 0~N)
        Returns:
            合并后的事务控制字节; 单帧时为 0x00。
        """
        return ((total_field & 0xF) << 4) | (seq & 0xF)

    @staticmethod
    def _parse_transaction(control: int) -> Dict:
        """
        解析事务控制字节。

        Args:
            control: BYTE[1] 事务控制值
        Returns:
            含 total_field(N)、seq(序号)、total_frames(实际帧数 K=N+1) 的字典。
        """
        n = (control >> 4) & 0xF
        return {'total_field': n, 'seq': control & 0xF, 'total_frames': n + 1}

    # =========================================================================
    # 帧收发
    # =========================================================================

    def send_frame(self, access: int, parent_cmd: int, sub_cmd: int,
                   data: bytes = b'', total_field: int = 0, seq: int = 0,
                   priority: int = Priority.HIGHEST) -> bool:
        """
        发送单帧 CANFD 消息（委托传输后端）。

        帧数据段布局: BYTE0=有效数据长度, BYTE1=事务控制, BYTE2~=数据;
        传输后端负责换算 DLC 并补零到合法帧长。

        Args:
            access: Access.READ / Access.WRITE
            parent_cmd: 父命令
            sub_cmd: 子命令
            data: 写数据（放在 BYTE2 起）；读请求为空
            total_field: 事务控制总帧数字段 N（单帧填 0）
            seq: 帧序号（单帧填 0）
            priority: 仲裁优先级(默认最高)
        Returns:
            发送成功返回 True, 否则 False。
        """
        if not self.is_connected:
            logger.error("错误: CANFD未连接")
            return False
        try:
            # 组装 29 位 ID(目标/源使用本机默认值)
            frame_id = self._build_canfd_id(access, parent_cmd, sub_cmd, priority=priority)
            data_len = min(len(data), 62)  # 数据段最多 62 字节(BYTE2~63)

            # 数据段有效字节: BYTE0 长度 / BYTE1 事务控制 / BYTE2~ 数据
            payload = bytearray(2 + data_len)
            payload[0] = data_len
            payload[1] = self._build_transaction(total_field, seq)
            payload[2:2 + data_len] = bytes(data[:data_len])

            return self.transport.send(frame_id, bytes(payload))
        except Exception as e:
            logger.error(f"发送消息异常: {e}")
            return False

    def receive_messages(self, timeout_ms: int = 3, filter_node: bool = True,
                         expected_parent: Optional[int] = None,
                         expected_sub: Optional[int] = None
                         ) -> List[Tuple[int, bytes, Dict]]:
        """
        接收 CANFD 消息（委托传输后端取回原始帧后解析）, 并按来源/父命令/子命令过滤。

        Args:
            timeout_ms: 底层接收阻塞超时(毫秒)
            filter_node: 为 True 时只保留 SrcID==本机 node_id 的帧(即目标设备应答)
            expected_parent: 若指定, 仅保留该父命令的帧
            expected_sub: 若指定, 仅保留该子命令的帧
        Returns:
            (frame_id, data, parsed_info) 列表; parsed_info 含 ID 位域及事务控制解析。
        """
        if not self.is_connected:
            return []
        try:
            messages = []
            for frame_id, data in self.transport.receive(timeout_ms):
                parsed = self._parse_canfd_id(frame_id)
                parsed['transaction'] = (
                    self._parse_transaction(data[1]) if len(data) > 1 else {}
                )

                # 应答帧的 SrcID 应为设备 NodeID, 借此丢弃非目标设备的报文
                if filter_node and parsed['src_id'] != self.node_id:
                    continue
                if expected_parent is not None and parsed['parent_cmd'] != expected_parent:
                    continue
                if expected_sub is not None and parsed['sub_cmd'] != expected_sub:
                    continue

                messages.append((frame_id, data, parsed))
            return messages
        except Exception as e:
            logger.error(f"接收消息异常: {e}")
            return []

    def _wait_for_response(self, parent_cmd: int, sub_cmd: Optional[int] = None,
                           timeout_ms: int = 200) -> Optional[Tuple[bytes, int, Dict]]:
        """
        在超时时间内轮询, 返回匹配的首帧应答。

        Args:
            parent_cmd: 期望应答的父命令
            sub_cmd: 期望应答的子命令(可选)
            timeout_ms: 总等待超时(毫秒)
        Returns:
            (data, status_code, parsed) 三元组; status_code 取应答 BYTE2; 超时返回 None。
        """
        start = time.time()
        while (time.time() - start) < timeout_ms / 1000.0:
            for _, data, parsed in self.receive_messages(
                timeout_ms=20, expected_parent=parent_cmd, expected_sub=sub_cmd
            ):
                status = data[2] if len(data) > 2 else -1  # 应答 BYTE2 为状态码
                return data, status, parsed
            time.sleep(0.002)
        return None

    def _read_response_joints(self, parent_cmd: int, sub_cmd: int, count: int,
                              signed: bool, byte_per_joint: int = 2,
                              timeout_ms: int = 200) -> Optional[List[int]]:
        """
        发送读请求并解析应答中的 N 路关节数据(从 BYTE3 起, 跳过状态码)。

        Args:
            parent_cmd/sub_cmd: 目标读命令
            count: 关节数量(L30 为 17)
            signed: 数据是否为有符号
            byte_per_joint: 每关节字节数(位置/电流/速度=2, 温度/错误码=1)
            timeout_ms: 应答等待超时
        Returns:
            关节值列表; 失败或状态码非 0 时返回 None。
        """
        if not self.send_frame(Access.READ, parent_cmd, sub_cmd):
            return None
        resp = self._wait_for_response(parent_cmd, sub_cmd, timeout_ms)
        if not resp:
            return None
        data, status, _ = resp
        if status != StatusCode.OK:
            logger.warning(f"读取失败 parent=0x{parent_cmd:X} sub=0x{sub_cmd:X} 状态码=0x{status:02X}")
            return None
        return self._unpack_joints(data, 3, count, byte_per_joint, signed)  # BYTE3 起为读数据

    # =========================================================================
    # 数据打包 / 解包
    # =========================================================================

    @staticmethod
    def _pack_joints(values: List[int], signed: bool, byte_per_joint: int = 2,
                     limits: Optional[Dict[int, Tuple[int, int]]] = None,
                     hard_min: Optional[int] = None, hard_max: Optional[int] = None) -> bytes:
        """
        将 17 路关节值按 J1~J17 顺序打包为大端字节, 并按需裁剪范围。

        Args:
            values: 关节值列表(按 J1~J17)
            signed: 是否有符号编码
            byte_per_joint: 每关节字节数
            limits: 逐关节限位字典 {关节号: (min,max)}(如位置控制)
            hard_min/hard_max: 统一上下限(如扭矩 60~800、速度 1~250)
        Returns:
            打包后的大端字节串。
        """
        out = bytearray()
        for i, val in enumerate(values):
            v = int(val)
            if limits is not None:                       # 逐关节限位
                lo, hi = limits.get(i + 1, (-32768, 32767))
                v = max(lo, min(hi, v))
            if hard_min is not None:                     # 统一下限裁剪
                v = max(hard_min, v)
            if hard_max is not None:                     # 统一上限裁剪
                v = min(hard_max, v)
            out.extend(v.to_bytes(byte_per_joint, byteorder='big', signed=signed))
        return bytes(out)

    @staticmethod
    def _unpack_joints(data: bytes, offset: int, count: int,
                       byte_per_joint: int = 2, signed: bool = True) -> List[int]:
        """
        从 data[offset] 起解包 count 个大端关节值。

        当实际数据不足时自动收缩数量, 避免越界。

        Args:
            data: 原始应答字节
            offset: 数据起始偏移(单次查询应答为 3)
            count: 期望关节数量
            byte_per_joint: 每关节字节数
            signed: 是否有符号解析
        Returns:
            解析出的关节值列表。
        """
        values = []
        need = offset + count * byte_per_joint
        if len(data) < need:  # 数据不足时按实际可解析数量收缩
            count = max(0, (len(data) - offset) // byte_per_joint)
        for i in range(count):
            s = offset + i * byte_per_joint
            values.append(int.from_bytes(data[s:s + byte_per_joint], byteorder='big', signed=signed))
        return values

    # =========================================================================
    # 父命令 0x1: 多关节控制
    # =========================================================================

    def set_joint_positions(self, positions: List[int]) -> bool:
        """
        多关节位置控制(子命令 0x01, 无应答, int16 大端)。

        下发 17 路目标位置, 超范围值按 JOINT_LIMITS 逐关节裁剪(设备端亦会按手型限位)。

        Args:
            positions: 长度必须为 17, 按 J1~J17 顺序
        Returns:
            发送成功返回 True; 长度不符或发送失败返回 False。
        """
        if len(positions) != JOINT_COUNT:
            logger.error(f"位置数据长度错误: 期望{JOINT_COUNT}, 实际{len(positions)}")
            return False
        data = self._pack_joints(positions, signed=True, limits=JOINT_LIMITS)
        return self.send_frame(Access.WRITE, ParentCmd.MULTI_JOINT, MultiJointSub.POSITION, data)

    def set_joint_torques(self, torques: List[int]) -> bool:
        """
        多关节扭矩控制(子命令 0x02, 无应答)。

        单位 6.5ma, 有效范围 60~800, 超范围统一裁剪。

        Args:
            torques: 长度必须为 17, 按 J1~J17 顺序
        Returns:
            发送成功返回 True; 长度不符或发送失败返回 False。
        """
        if len(torques) != JOINT_COUNT:
            logger.error(f"扭矩数据长度错误: 期望{JOINT_COUNT}, 实际{len(torques)}")
            return False
        data = self._pack_joints(torques, signed=False, hard_min=60, hard_max=800)
        return self.send_frame(Access.WRITE, ParentCmd.MULTI_JOINT, MultiJointSub.TORQUE, data)

    def set_joint_velocities(self, velocities: List[int]) -> bool:
        """
        多关节速度控制(子命令 0x03, 无应答)。

        单位 0.732rpm, 有效范围 1~250, 超范围统一裁剪。

        Args:
            velocities: 长度必须为 17, 按 J1~J17 顺序
        Returns:
            发送成功返回 True; 长度不符或发送失败返回 False。
        """
        if len(velocities) != JOINT_COUNT:
            logger.error(f"速度数据长度错误: 期望{JOINT_COUNT}, 实际{len(velocities)}")
            return False
        data = self._pack_joints(velocities, signed=False, hard_min=1, hard_max=250)
        return self.send_frame(Access.WRITE, ParentCmd.MULTI_JOINT, MultiJointSub.SPEED, data)

    def _write_with_ack(self, parent_cmd: int, sub_cmd: int, data: bytes = b'',
                        timeout_ms: int = 200) -> Tuple[bool, int]:
        """
        发送写命令并等待应答状态码(适用于有应答的写操作)。

        Args:
            parent_cmd/sub_cmd: 目标命令
            data: 写数据(可空)
            timeout_ms: 应答等待超时
        Returns:
            (是否成功, 状态码); 未收到应答时返回 (False, -1)。
        """
        if not self.send_frame(Access.WRITE, parent_cmd, sub_cmd, data):
            return False, -1
        resp = self._wait_for_response(parent_cmd, sub_cmd, timeout_ms)
        if not resp:
            return False, -1
        _, status, _ = resp
        return status == StatusCode.OK, status

    def enable_all_joints(self) -> bool:
        """全局使能所有关节(子命令 0x07, 有应答)。成功返回 True。"""
        ok, _ = self._write_with_ack(ParentCmd.MULTI_JOINT, MultiJointSub.ENABLE)
        return ok

    def disable_all_joints(self) -> bool:
        """全局失能所有关节(子命令 0x08, 有应答)。成功返回 True。"""
        ok, _ = self._write_with_ack(ParentCmd.MULTI_JOINT, MultiJointSub.DISABLE)
        return ok

    def emergency_stop(self) -> bool:
        """全局急停(子命令 0x05, 固件暂不支持, 有应答)。成功返回 True。"""
        ok, _ = self._write_with_ack(ParentCmd.MULTI_JOINT, MultiJointSub.STOP)
        return ok

    def pause(self) -> bool:
        """全局暂停并保持当前位置(子命令 0x06, 固件暂不支持, 有应答)。成功返回 True。"""
        ok, _ = self._write_with_ack(ParentCmd.MULTI_JOINT, MultiJointSub.PAUSE)
        return ok

    # =========================================================================
    # 父命令 0x2: 触觉传感器（多帧读, 12x6=72 字节）
    # =========================================================================

    def get_finger_touch(self, sub_cmd: int, timeout_ms: int = 20) -> Optional[List[int]]:
        """
        读取单指触觉矩阵(多帧读, 返回 72 字节列表)。

        协议以多帧读方式返回 12x6=72 字节: 第1帧(seq=0)BYTE3 起 61 字节,
        第2帧(seq=1)续 11 字节; 每帧 BYTE2 均为状态码, BYTE0 为本帧有效长度。
        本方法按 seq 顺序拼接各帧 BYTE3 起的数据段。

        Args:
            sub_cmd: 触觉子命令(TouchSub.THUMB~PINKY)
            timeout_ms: 多帧拼包超时(协议建议 10ms 级)
        Returns:
            72 元素的整数列表; 超时/状态异常/数据不完整返回 None。
        """
        if not self.send_frame(Access.READ, ParentCmd.TOUCH, sub_cmd):
            return None

        frames: Dict[int, bytes] = {}          # seq -> 数据段, 用于按序拼接
        expected_frames: Optional[int] = None  # 由事务控制解析出的总帧数 K
        start = time.time()
        while (time.time() - start) < timeout_ms / 1000.0:
            for _, data, parsed in self.receive_messages(
                timeout_ms=5, expected_parent=ParentCmd.TOUCH, expected_sub=sub_cmd
            ):
                if len(data) < 3:
                    continue
                if data[2] != StatusCode.OK:               # BYTE2 状态码
                    logger.warning(f"触觉读取状态异常: 0x{data[2]:02X}")
                    return None
                tc = parsed.get('transaction', {})
                seq = tc.get('seq', 0)
                expected_frames = tc.get('total_frames', expected_frames)
                seg_len = data[0]                          # BYTE0: 本帧有效数据长度(不含状态码)
                frames[seq] = data[3:3 + seg_len]          # BYTE3 起为矩阵数据
                if expected_frames is not None and len(frames) >= expected_frames:
                    break
            if expected_frames is not None and len(frames) >= expected_frames:
                break
            time.sleep(0.001)

        if not frames:
            return None
        # 按 seq 升序拼接得到完整矩阵
        payload = bytearray()
        for seq in sorted(frames.keys()):
            payload.extend(frames[seq])
        if len(payload) < TOUCH_BYTES:
            logger.warning(f"触觉数据不完整: {len(payload)}/{TOUCH_BYTES}")
            return None
        return list(payload[:TOUCH_BYTES])

    def _process_touch_matrix(self, data: Optional[List[int]], key: str) -> np.ndarray:
        """
        将 72 字节触觉数据整形为 12x6 矩阵并缓存。

        上下翻转(arr[::-1])以符合显示方向; 数据无效时返回上次缓存值。

        Args:
            data: get_finger_touch 返回的 72 元素列表(或 None)
            key: 手指键名(thumb/index/middle/ring/little)
        Returns:
            12x6 的 numpy 矩阵。
        """
        if data is not None and len(data) == TOUCH_BYTES:
            arr = np.array(data).reshape(TOUCH_ROWS, TOUCH_COLS)
            self._touch_matrices[key] = arr[::-1]
        return self._touch_matrices[key]

    def get_thumb_touch(self) -> np.ndarray:
        """读取拇指触觉矩阵(子命令 0x01), 返回 12x6 numpy 矩阵。"""
        return self._process_touch_matrix(self.get_finger_touch(TouchSub.THUMB), 'thumb')

    def get_index_touch(self) -> np.ndarray:
        """读取食指触觉矩阵(子命令 0x02), 返回 12x6 numpy 矩阵。"""
        return self._process_touch_matrix(self.get_finger_touch(TouchSub.INDEX), 'index')

    def get_middle_touch(self) -> np.ndarray:
        """读取中指触觉矩阵(子命令 0x03), 返回 12x6 numpy 矩阵。"""
        return self._process_touch_matrix(self.get_finger_touch(TouchSub.MIDDLE), 'middle')

    def get_ring_touch(self) -> np.ndarray:
        """读取无名指触觉矩阵(子命令 0x04), 返回 12x6 numpy 矩阵。"""
        return self._process_touch_matrix(self.get_finger_touch(TouchSub.RING), 'ring')

    def get_little_touch(self) -> np.ndarray:
        """读取小指触觉矩阵(子命令 0x05), 返回 12x6 numpy 矩阵。"""
        return self._process_touch_matrix(self.get_finger_touch(TouchSub.PINKY), 'little')

    def get_all_touch(self) -> Dict[str, list]:
        """依次读取全部 5 指触觉矩阵, 返回 {手指_matrix: 12x6 列表} 字典。"""
        return {
            'thumb_matrix': self.get_thumb_touch().tolist(),
            'index_matrix': self.get_index_touch().tolist(),
            'middle_matrix': self.get_middle_touch().tolist(),
            'ring_matrix': self.get_ring_touch().tolist(),
            'little_matrix': self.get_little_touch().tolist(),
        }

    # =========================================================================
    # 父命令 0x3: 配置信息
    # =========================================================================

    def unlock(self, password: bytes = UNLOCK_PASSWORD) -> bool:
        """
        配置解锁(子命令 0x01)。

        写 DeviceInfo/NodeID/手型/标定零点等操作前必须先解锁。
        连续密码错误达上限(10 次)会锁定, 需重启设备。

        Args:
            password: 6 字节解锁密码, 默认使用协议约定密码
        Returns:
            解锁成功返回 True; 密码错误/权限不足(0x20)返回 False。
        """
        ok, status = self._write_with_ack(ParentCmd.CONFIG, ConfigSub.UNLOCK, bytes(password))
        if not ok:
            logger.error(f"配置解锁失败, 状态码=0x{status:02X}")
        return ok

    def get_device_info(self, timeout_ms: int = 200) -> Optional[Dict]:
        """
        读取 DeviceInfo(子命令 0x02)。

        应答 BYTE3 起为 18 字节结构: product_id / serial_no(4B) / sw(3B) /
        hw(3B) / struct(3B) / node_id / hand_type / sensor_type / origin。

        Args:
            timeout_ms: 应答等待超时
        Returns:
            解析后的设备信息字典; 失败返回 None。
        """
        if not self.send_frame(Access.READ, ParentCmd.CONFIG, ConfigSub.DEVICE_INFO):
            return None
        resp = self._wait_for_response(ParentCmd.CONFIG, ConfigSub.DEVICE_INFO, timeout_ms)
        if not resp:
            return None
        data, status, _ = resp
        if status != StatusCode.OK or len(data) < 21:
            return None
        info = data[3:21]  # BYTE3~20 共 18 字节设备信息
        return {
            'product_id': info[0],                                    # 0x13=L30
            'serial_no': int.from_bytes(info[1:5], 'big'),            # 全局序列号
            'sw_version': f"{info[5]}.{info[6]}.{info[7]}",           # 软件版本
            'hw_version': f"{info[8]}.{info[9]}.{info[10]}",          # 硬件版本
            'struct_version': f"{info[11]}.{info[12]}.{info[13]}",    # 结构/协议版本
            'node_id': info[14],                                      # 当前 NodeID
            'hand_type': 'right' if info[15] == HandType.RIGHT else 'left',
            'sensor_type': info[16],                                  # 传感器类型
            'origin': info[17],                                       # 产地
        }

    def set_device_info(self, serial_no: int, origin: int) -> bool:
        """
        写 DeviceInfo(子命令 0x02, 需先解锁)。

        仅可写 serial_no 与 origin, 其余字段由设备维护。

        Args:
            serial_no: 全局序列号(uint32)
            origin: 产地编码(1=北京自装, 2=大厂, 3=固安)
        Returns:
            写入成功返回 True; 未解锁(0x20)等失败返回 False。
        """
        data = serial_no.to_bytes(4, 'big') + bytes([origin & 0xFF])  # serial_no 大端 + origin
        ok, status = self._write_with_ack(ParentCmd.CONFIG, ConfigSub.DEVICE_INFO, data)
        if not ok:
            logger.error(f"写DeviceInfo失败, 状态码=0x{status:02X}")
        return ok

    def get_product_code(self, timeout_ms: int = 200) -> Optional[str]:
        """
        读取产品编码字符串(子命令 0x03, ASCII, 无需解锁)。

        应答 BYTE0 为字符串长度, BYTE2 为状态码, BYTE3 起为 ASCII 编码
        (如 'LHT30-06-169-L-B-1-A')。

        Args:
            timeout_ms: 应答等待超时
        Returns:
            产品编码字符串; 失败返回 None。
        """
        if not self.send_frame(Access.READ, ParentCmd.CONFIG, ConfigSub.PRODUCT_CODE):
            return None
        resp = self._wait_for_response(ParentCmd.CONFIG, ConfigSub.PRODUCT_CODE, timeout_ms)
        if not resp:
            return None
        data, status, _ = resp
        if status != StatusCode.OK:
            return None
        length = data[0]  # BYTE0: 字符串有效长度
        try:
            return data[3:3 + length].decode('ascii', errors='ignore')
        except Exception:
            return None

    def get_node_id(self, timeout_ms: int = 200) -> Optional[int]:
        """
        读取当前 NodeID(子命令 0x04, 无需解锁)。

        应答 BYTE3 为当前 NodeID。

        Args:
            timeout_ms: 应答等待超时
        Returns:
            当前 NodeID(1~31); 失败返回 None。
        """
        if not self.send_frame(Access.READ, ParentCmd.CONFIG, ConfigSub.NODE_ID):
            return None
        resp = self._wait_for_response(ParentCmd.CONFIG, ConfigSub.NODE_ID, timeout_ms)
        if not resp:
            return None
        data, status, _ = resp
        if status != StatusCode.OK or len(data) < 4:
            return None
        return data[3]

    def set_node_id(self, new_node_id: int) -> bool:
        """
        写 NodeID(子命令 0x04, 需先解锁)。

        修改后立即生效, 设备使用新 NodeID 响应后续帧; 本地 node_id 同步更新
        以便继续正确寻址与过滤应答。

        Args:
            new_node_id: 新节点 ID, 范围 1~31
        Returns:
            写入成功返回 True; 越界/未解锁等失败返回 False。
        """
        if not (1 <= new_node_id <= 31):
            logger.error("NodeID 越界, 应为 1~31")
            return False
        ok, status = self._write_with_ack(ParentCmd.CONFIG, ConfigSub.NODE_ID,
                                          bytes([new_node_id]))
        if ok:
            self.node_id = new_node_id  # 设备已使用新 NodeID 响应后续帧
        else:
            logger.error(f"写NodeID失败, 状态码=0x{status:02X}")
        return ok

    def calibrate_zero(self) -> bool:
        """
        标定全部关节零点(子命令 0x05)。

        前置条件: 须先解锁(unlock)且全局失能(disable_all_joints), 否则设备拒绝。
        成功后零点参考写入 EEPROM。

        Returns:
            标定成功返回 True; 未解锁(0x20)/状态不允许(0x23)/写失败(0x31)返回 False。
        """
        ok, status = self._write_with_ack(ParentCmd.CONFIG, ConfigSub.CALIBRATE_ZERO)
        if not ok:
            logger.error(f"标定零点失败, 状态码=0x{status:02X}")
        return ok

    def get_hand_type(self, timeout_ms: int = 200) -> Optional[str]:
        """
        读取手型(子命令 0x06, 无需解锁)。

        Args:
            timeout_ms: 应答等待超时
        Returns:
            'left'/'right'; 未配置(0x24)或失败返回 None。
        """
        if not self.send_frame(Access.READ, ParentCmd.CONFIG, ConfigSub.HAND_TYPE):
            return None
        resp = self._wait_for_response(ParentCmd.CONFIG, ConfigSub.HAND_TYPE, timeout_ms)
        if not resp:
            return None
        data, status, _ = resp
        if status == StatusCode.ERR_NOT_SET:
            return None  # 手型未配置
        if status != StatusCode.OK or len(data) < 4:
            return None
        return 'right' if data[3] == HandType.RIGHT else 'left'  # BYTE3 为手型

    def set_hand_type(self, hand_type: int) -> bool:
        """
        写手型(子命令 0x06, 需先解锁)。

        写入后刷新关节正反转范围与限位。

        Args:
            hand_type: 0=左手, 非 0=右手
        Returns:
            写入成功返回 True; 未解锁/参数非法等失败返回 False。
        """
        ht = HandType.RIGHT if hand_type else HandType.LEFT
        ok, status = self._write_with_ack(ParentCmd.CONFIG, ConfigSub.HAND_TYPE, bytes([int(ht)]))
        if not ok:
            logger.error(f"写手型失败, 状态码=0x{status:02X}")
        return ok

    # =========================================================================
    # 父命令 0x4: 周期上报
    # =========================================================================

    def config_periodic_report(self, sub_cmd: int, enable: bool,
                               period_ms: int = 20, joint_mask: int = 0x00000000) -> bool:
        """
        配置周期上报(子命令 0x01~0x05, 写)。

        载荷: Enable(1B) + Period(uint32 大端) + 关节位掩码(uint32 大端)。
        掩码 Bit0->J1 ... Bit16->J17, joint_mask=0 表示上报全部 17 关节。
        周期建议 20~1000ms, 超范围设备返回 0x35。

        Args:
            sub_cmd: 上报项子命令(PeriodicSub.*)
            enable: True=使能上报, False=关闭
            period_ms: 上报周期(毫秒)
            joint_mask: 关节选择位掩码
        Returns:
            配置成功返回 True; 失败返回 False。
        """
        data = bytes([0x01 if enable else 0x00])   # Enable
        data += period_ms.to_bytes(4, 'big')        # Period(大端)
        data += joint_mask.to_bytes(4, 'big')       # 关节位掩码(大端)
        ok, status = self._write_with_ack(ParentCmd.PERIODIC, sub_cmd, data)
        if not ok:
            logger.error(f"周期上报配置失败 sub=0x{sub_cmd:X}, 状态码=0x{status:02X}")
        return ok

    def stop_periodic_report(self, sub_cmd: int) -> bool:
        """
        关闭指定子命令的周期上报(Enable=0)。

        Args:
            sub_cmd: 上报项子命令(PeriodicSub.*)
        Returns:
            配置成功返回 True。
        """
        return self.config_periodic_report(sub_cmd, enable=False, period_ms=0, joint_mask=0)

    def read_periodic_report(self, sub_cmd: int, joint_count: int,
                             byte_per_joint: int = 2, signed: bool = True,
                             timeout_ms: int = 50) -> Optional[List[int]]:
        """
        读取一帧设备主动上报数据。

        主动上报帧为 Access=0、单帧、无状态码, 数据从 BYTE2 起按 J1->J17
        顺序紧凑排列(仅含选中关节)。

        Args:
            sub_cmd: 上报项子命令(PeriodicSub.*)
            joint_count: 选中关节数(掩码为 0 时为 17)
            byte_per_joint: 每关节字节数(位置/电流/速度=2, 温度/错误码=1)
            signed: 是否有符号
            timeout_ms: 等待上报帧超时
        Returns:
            关节值列表; 超时返回 None。
        """
        start = time.time()
        while (time.time() - start) < timeout_ms / 1000.0:
            for _, data, parsed in self.receive_messages(
                timeout_ms=10, expected_parent=ParentCmd.PERIODIC, expected_sub=sub_cmd
            ):
                if parsed['access'] != Access.READ or len(data) < 2:
                    continue
                # 主动上报无状态码, 数据自 BYTE2 起
                return self._unpack_joints(data, 2, joint_count, byte_per_joint, signed)
            time.sleep(0.002)
        return None

    # =========================================================================
    # 父命令 0x5: 单次查询
    # =========================================================================

    def get_joint_positions(self) -> Optional[List[int]]:
        """单次查询 17 路当前关节位置(子命令 0x01, int16 大端)。失败返回 None。"""
        return self._read_response_joints(ParentCmd.QUERY, QuerySub.POSITION,
                                          JOINT_COUNT, signed=True, byte_per_joint=2)

    def get_joint_currents(self) -> Optional[List[int]]:
        """单次查询 17 路当前电流(子命令 0x02, 单位 6.5ma)。失败返回 None。"""
        return self._read_response_joints(ParentCmd.QUERY, QuerySub.CURRENT,
                                          JOINT_COUNT, signed=True, byte_per_joint=2)

    def get_joint_torques(self) -> Optional[List[int]]:
        """
        [兼容旧接口] 读取 17 路关节扭矩/电流。

        新版协议单次查询无独立"扭矩"读取, 与旧版 0x02 读取语义一致地
        返回电流查询结果(单位 6.5ma), 以保持与 linker_hand_l30_v6_canfd
        相同的对外方法名, 便于 linker_hand_l30_api.py 通用调用。
        失败返回 None。
        """
        return self.get_joint_currents()

    def get_joint_velocities(self) -> Optional[List[int]]:
        """单次查询 17 路当前速度(子命令 0x03, 单位 0.732rpm)。失败返回 None。"""
        return self._read_response_joints(ParentCmd.QUERY, QuerySub.SPEED,
                                          JOINT_COUNT, signed=True, byte_per_joint=2)

    def get_joint_temperatures(self) -> Optional[List[int]]:
        """单次查询 17 路当前温度(子命令 0x04, 单位 1°C, 每关节 1 字节)。失败返回 None。"""
        return self._read_response_joints(ParentCmd.QUERY, QuerySub.TEMPERATURE,
                                          JOINT_COUNT, signed=False, byte_per_joint=1)

    def get_joint_error_codes(self) -> Optional[List[int]]:
        """单次查询 17 路舵机错误码(子命令 0x05, 每关节 1 字节)。失败返回 None。"""
        return self._read_response_joints(ParentCmd.QUERY, QuerySub.ERROR_CODE,
                                          JOINT_COUNT, signed=False, byte_per_joint=1)

    # =========================================================================
    # 父命令 0x6: 单关节调试控制（仅写, 子命令 = (功能码<<5)|关节编号）
    # =========================================================================

    @staticmethod
    def _single_joint_subcmd(func: int, joint_idx: int) -> int:
        """
        计算单关节子命令 = (功能码<<5)|关节编号。

        Args:
            func: 功能码(1=位置, 2=扭矩, 3=速度)
            joint_idx: 关节编号 1~17(对应 J1~J17)
        Returns:
            子命令值(位置 0x21~0x31 / 扭矩 0x41~0x51 / 速度 0x61~0x71)。
        """
        return ((func & 0x7) << 5) | (joint_idx & 0x1F)

    def set_single_joint_position(self, joint_idx: int, value: int) -> Tuple[bool, int]:
        """
        单关节位置控制(功能码 0x1, 子命令 0x21~0x31, 有应答)。

        Args:
            joint_idx: 关节编号 1~17
            value: 目标位置(int16, 按 JOINT_LIMITS 裁剪)
        Returns:
            (是否成功, 状态码); 关节编号越界返回 (False, -1)。
        """
        if not (1 <= joint_idx <= JOINT_COUNT):
            logger.error("关节编号越界, 应为 1~17")
            return False, -1
        lo, hi = JOINT_LIMITS.get(joint_idx, (-32768, 32767))
        v = max(lo, min(hi, int(value)))
        sub = self._single_joint_subcmd(SingleJointFunc.POSITION, joint_idx)
        return self._write_with_ack(ParentCmd.SINGLE_JOINT, sub,
                                    v.to_bytes(2, 'big', signed=True))

    def set_single_joint_torque(self, joint_idx: int, value: int) -> Tuple[bool, int]:
        """
        单关节扭矩控制(功能码 0x2, 子命令 0x41~0x51, 有应答)。

        单位 6.5ma, 范围 60~800, 超范围裁剪。

        Args:
            joint_idx: 关节编号 1~17
            value: 目标扭矩
        Returns:
            (是否成功, 状态码); 关节编号越界返回 (False, -1)。
        """
        if not (1 <= joint_idx <= JOINT_COUNT):
            logger.error("关节编号越界, 应为 1~17")
            return False, -1
        v = max(60, min(800, int(value)))
        sub = self._single_joint_subcmd(SingleJointFunc.TORQUE, joint_idx)
        return self._write_with_ack(ParentCmd.SINGLE_JOINT, sub, v.to_bytes(2, 'big'))

    def set_single_joint_speed(self, joint_idx: int, value: int) -> Tuple[bool, int]:
        """
        单关节速度控制(功能码 0x3, 子命令 0x61~0x71, 有应答)。

        单位 0.732rpm, 范围 1~250, 超范围裁剪。

        Args:
            joint_idx: 关节编号 1~17
            value: 目标速度
        Returns:
            (是否成功, 状态码); 关节编号越界返回 (False, -1)。
        """
        if not (1 <= joint_idx <= JOINT_COUNT):
            logger.error("关节编号越界, 应为 1~17")
            return False, -1
        v = max(1, min(250, int(value)))
        sub = self._single_joint_subcmd(SingleJointFunc.SPEED, joint_idx)
        return self._write_with_ack(ParentCmd.SINGLE_JOINT, sub, v.to_bytes(2, 'big'))


# =============================================================================
# CRC 算法（Bootloader 升级用, 见协议 §16.6）
# =============================================================================

def crc16_modbus(data: bytes) -> int:
    """
    计算单包 CRC16-Modbus(见协议 §16.6.1, 用于升级子命令 0x02)。

    参数为固件数据字节; 初值 0xFFFF, 多项式 0xA001(反向), 无最终异或。

    Args:
        data: 待校验字节序列
    Returns:
        16 位 CRC 值。
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def crc32_firmware(data: bytes) -> int:
    """
    计算整包 CRC32-IEEE(见协议 §16.6.2, 用于升级子命令 0x03)。

    初值 0xFFFFFFFF, 多项式 0xEDB88320(反向), 结果与 0xFFFFFFFF 异或。

    Args:
        data: 全部固件有效字节(按发包顺序拼接)
    Returns:
        32 位 CRC 值。
    """
    c = 0xFFFFFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if (c & 1) else (c >> 1)
    return (c ^ 0xFFFFFFFF) & 0xFFFFFFFF


# =============================================================================
# 高级控制器
# =============================================================================

class L30DexterousHandController:
    """
    L30 灵巧手高级控制器(新版 CANFD 协议)。

    在 L30CANFDProtocol 之上封装连接管理、批量控制、状态读取与归一化,
    提供更贴近应用层的接口, 并支持 with 上下文自动连接/断开。
    """

    JOINT_COUNT = JOINT_COUNT

    def __init__(self, device_id: int = 1, canfd_id: int = 0, src_id: int = 0,
                 node_id: Optional[int] = None, canfd_device: Optional[int] = None,
                 comm_type: str = "libcanbus", channel=None, bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True):
        """
        构造高级控制器。

        为与旧版 linker_hand_l30_v6_canfd.L30DexterousHandController 保持一致的
        对外接口, 同时兼容两套参数命名:
          - 旧命名: device_id(设备/节点 ID) / canfd_id(CANFD 盒子编号)
          - 新命名: node_id / canfd_device

        Args:
            device_id: 设备节点 ID(DstID), 默认 1; 旧接口沿用此名
            canfd_id: CANFD 设备(盒子)索引, 默认 0; 旧接口沿用此名
            src_id: 主站源 ID(SrcID), 默认 0
            node_id: 新命名的设备节点 ID, 若给定则覆盖 device_id
            canfd_device: 新命名的 CANFD 设备索引, 若给定则覆盖 canfd_id
            comm_type: 通讯后端 —— "libcanbus"(默认, 厂商私有库) 或
                       "socketcan"(内核 can0 + python-can, 透明塑封 USB-CANFD 设备)
            channel: 通道——libcanbus 为 int(默认 0), socketcan 为接口名(默认 "can0")
            bitrate/dbitrate/auto_setup: 仅 socketcan 使用
        """
        # 新命名优先, 否则回退到旧命名, 使两种调用方式都能工作
        nid = node_id if node_id is not None else device_id
        dev = canfd_device if canfd_device is not None else canfd_id
        # channel 未显式指定时按后端给默认值: socketcan -> "can0", libcanbus -> 0
        if channel is None:
            channel = "can0" if comm_type == "socketcan" else 0
        self.protocol = L30CANFDProtocol(nid, src_id, dev, channel=channel,
                                         comm_type=comm_type, bitrate=bitrate,
                                         dbitrate=dbitrate, auto_setup=auto_setup)
        self.node_id = nid
        self.device_id = nid                            # 兼容旧属性名
        self.hand_type: Optional[str] = None            # 连接后填充 left/right
        self.joints = {j.id: j for j in JOINT_DEFINITIONS}  # 关节静态信息表

    # ---- 连接管理 ----
    def connect(self, enable_on_connect=True) -> Tuple[bool, Optional[str]]:
        """
        连接并使能灵巧手, 同时读取手型。

        Returns:
            (是否连接成功, 手型字符串或 None)。
        """
        logger.info("开始连接灵巧手...")
        if not self.protocol.initialize():
            return False, None

        print("使能所有关节...")
        if enable_on_connect and not self.protocol.enable_all_joints():
            logger.warning("关节使能失败, 继续尝试...")
        time.sleep(0.1)

        # 设备类型查询偶发超时/丢包，重试若干次避免误判为 hand_type 不匹配
        self.hand_type = None
        for _ in range(5):
            self.hand_type = self.protocol.get_hand_type()
            if self.hand_type:
                break
            time.sleep(0.05)
        logger.info(f"连接成功: NodeID={self.node_id}, 类型={self.hand_type}")
        return True, self.hand_type

    def disconnect(self) -> None:
        """断开连接(关闭底层 CANFD)。"""
        self.protocol.close()

    @property
    def is_connected(self) -> bool:
        """是否已连接。"""
        return self.protocol.is_connected

    # ---- 关节控制 ----
    def set_positions(self, positions: List[int]) -> bool:
        """设置 17 路关节位置; 长度不符抛 ValueError。"""
        if len(positions) != self.JOINT_COUNT:
            raise ValueError(f"需要{self.JOINT_COUNT}个关节值")
        return self.protocol.set_joint_positions(positions)

    def set_velocities(self, velocities: List[int]) -> bool:
        """设置 17 路关节速度(单位 0.732rpm)。"""
        return self.protocol.set_joint_velocities(velocities)

    def set_torques(self, torques: List[int]) -> bool:
        """设置 17 路关节扭矩(单位 6.5ma)。"""
        return self.protocol.set_joint_torques(torques)

    def get_positions(self) -> Optional[List[int]]:
        """读取 17 路当前关节位置。"""
        return self.protocol.get_joint_positions()

    def get_velocities(self) -> Optional[List[int]]:
        """读取 17 路当前关节速度。"""
        return self.protocol.get_joint_velocities()

    def get_currents(self) -> Optional[List[int]]:
        """读取 17 路当前关节电流。"""
        return self.protocol.get_joint_currents()

    def get_temperatures(self) -> Optional[List[int]]:
        """读取 17 路当前关节温度。"""
        return self.protocol.get_joint_temperatures()

    def get_error_codes(self) -> Optional[List[int]]:
        """读取 17 路舵机错误码。"""
        return self.protocol.get_joint_error_codes()

    def enable_all(self) -> bool:
        """全局使能所有关节。"""
        return self.protocol.enable_all_joints()

    def disable_all(self) -> bool:
        """全局失能所有关节。"""
        return self.protocol.disable_all_joints()

    def stop(self) -> bool:
        """全局急停(固件暂不支持)。"""
        return self.protocol.emergency_stop()

    def calibrate(self) -> bool:
        """标定零点(内部先解锁并失能, 满足标定前置条件)。"""
        self.protocol.unlock()
        self.protocol.disable_all_joints()
        return self.protocol.calibrate_zero()

    # ---- 触觉 ----
    def get_matrix_touch(self) -> Dict[str, list]:
        """读取全部 5 指触觉矩阵。"""
        return self.protocol.get_all_touch()

    # ---- 信息 ----
    def get_joint_name(self) -> Tuple[List[str], List[str]]:
        """获取关节名称(英文列表, 中文列表)。"""
        return JOINT_NAME_EN, JOINT_NAME_CN

    def get_joint_range(self) -> Dict[int, Tuple[int, int]]:
        """获取关节限位字典 {关节号: (min,max)}。"""
        return JOINT_LIMITS

    def get_device_info(self) -> Optional[Dict]:
        """读取设备信息(DeviceInfo)。"""
        return self.protocol.get_device_info()

    def get_all_state(self, is_touch: bool = False) -> Dict:
        """
        获取完整状态(位置/速度/电流/温度/错误码)。

        Args:
            is_touch: 为 True 时附带触觉矩阵(耗时较长)
        Returns:
            状态字典。
        """
        state = {
            'positions': self.protocol.get_joint_positions(),
            'velocities': self.protocol.get_joint_velocities(),
            'currents': self.protocol.get_joint_currents(),
            'temperatures': self.protocol.get_joint_temperatures(),
            'error_codes': self.protocol.get_joint_error_codes(),
        }
        if is_touch:
            state['matrix_touch'] = self.get_matrix_touch()
        return state

    # ---- 归一化 ----
    def normalize_positions(self, raw_positions: List[int]) -> List[float]:
        """
        将原始位置值按各关节限位归一化到 0~1。

        Args:
            raw_positions: 原始位置列表(按 J1~J17)
        Returns:
            归一化后的浮点列表; 限位无效时取 0.5。
        """
        result = []
        for i, pos in enumerate(raw_positions):
            lo, hi = JOINT_LIMITS.get(i + 1, (0, 1))
            result.append(max(0.0, min(1.0, (pos - lo) / (hi - lo))) if hi != lo else 0.5)
        return result

    def denormalize_positions(self, normalized: List[float]) -> List[int]:
        """
        将 0~1 归一化值按各关节限位还原为原始位置。

        Args:
            normalized: 归一化列表(按 J1~J17)
        Returns:
            原始位置整数列表。
        """
        result = []
        for i, norm in enumerate(normalized):
            lo, hi = JOINT_LIMITS.get(i + 1, (0, 0))
            result.append(int(round(lo + norm * (hi - lo))))
        return result

    def __enter__(self) -> 'L30DexterousHandController':
        """上下文管理器入口: 自动连接。"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """上下文管理器出口: 自动断开。"""
        self.disconnect()


# =============================================================================
# 便捷函数
# =============================================================================

def create_default_controller(node_id: int = 1) -> L30DexterousHandController:
    """
    创建默认高级控制器。

    Args:
        node_id: 设备节点 ID, 默认 1
    Returns:
        L30DexterousHandController 实例(尚未连接)。
    """
    return L30DexterousHandController(node_id)


def setup_logging(level: int = logging.INFO) -> None:
    """
    配置根日志格式与级别。

    Args:
        level: 日志级别, 默认 logging.INFO
    """
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    )

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L30 hand API

API for CANFD protocol control
"""
import time,os,sys
from typing import  List, Optional, Tuple, Dict, Any
import numpy as np
# 两个控制类模块：优先旧版 v6_canfd，初始化失败时自动回退 v6_2_canfd
from .core import linker_hand_l30_v6_canfd as hand_canfd_v1
from .core import linker_hand_l30_v6_2_canfd as hand_canfd_v2
# 共用常量/枚举（两个模块导出接口一致，从 v1 引入即可）
from .core.linker_hand_l30_v6_canfd import (
    L30DexterousHandController,
    L30CANFDProtocol,
    Command,
    HandType,
    JOINT_NAME_EN,
    JOINT_NAME_CN,
    StatusCode,
)
from .utils.color_msg import ColorMsg

class LinkerHandL30API:
    # 控制类候选模块，按顺序尝试初始化：优先旧版 v6_canfd，失败则回退 v6_2_canfd
    _CONTROLLER_MODULES = [
        ("V6", hand_canfd_v1),
        ("V6.2", hand_canfd_v2),
    ]

    def __init__(self, hand_joint="L30", hand_type="right", device_id=0x06, canfd_id=0,
                 comm_type="libcanbus", channel=None, bitrate=1000000,
                 dbitrate=5000000, auto_setup=True, enable_on_connect=True) -> None:
        """
        Args:
            hand_joint: 手关节类型，默认L30
            hand_type: 手类型，默认right
            device_id: 设备ID，默认0x06, 灵巧手ID
            canfd_id: CANFD ID，默认0，CANFD盒子id编号，一般第一个插入的盒子id为0，第二个插入的盒子id为1，依此类推
            comm_type: 通信后端 —— "libcanbus"(默认, 蓝色/黑色 CANFD 盒, 厂商私有库) 或
                       "socketcan"(内核 can0 + python-can, 透明塑封 USB-CANFD 设备)
            channel: 仅 socketcan 生效——接口名(默认 "can0")；libcanbus 忽略
            bitrate: 仅 socketcan 生效——仲裁段波特率(须与灵巧手一致)
            dbitrate: 仅 socketcan 生效——数据段波特率
            auto_setup: 仅 socketcan 生效——True 时自动 sudo ip link 拉起接口
        """

        self.enable_on_connect = enable_on_connect
        self.hand_joint = hand_joint
        self.hand_type = hand_type
        self.hand_controller = None
        # 通信后端参数(透传给控制类)；channel 为空时按后端取默认(socketcan->can0, libcanbus->0)
        if channel is None:
            channel = "can0" if comm_type == "socketcan" else 0
        self._comm_kwargs = dict(comm_type=comm_type, channel=channel,
                                 bitrate=bitrate, dbitrate=dbitrate, auto_setup=auto_setup)

        # 依次尝试各控制类模块：仅当 CANFD 连接成功且 hand_type 匹配才算成功；
        # 由于旧版可能只是成功打开 CANFD 盒子、却按旧协议读不到正确 hand_type，
        # 因此把 hand_type 不匹配也视为该控制类初始化失败，自动回退到下一个控制类。
        last_reason = None
        for name, module in self._CONTROLLER_MODULES:
            ColorMsg(msg=f"开始使用 {name} 连接L30...", color="yellow")
            controller, reason = self._try_init_controller(module, device_id, canfd_id)
            if controller is not None:
                self.hand_controller = controller
                ColorMsg(msg=f"{name} 初始化成功", color="green")
                break
            last_reason = reason
            ColorMsg(msg=f"{name} 初始化失败({reason})，尝试下一个控制类...", color="red")
        print(self.hand_type, flush=True)
        if self.hand_controller is None:
            ColorMsg(msg=f"Linker Hand L30 连接失败，所有控制类初始化均失败: {last_reason}", color="red")
            # hand_type 不匹配时保持原有异常类型，其余情况视为初始化失败
            if last_reason and "hand_type" in last_reason:
                raise ValueError("Hand type is incorrect")
            raise RuntimeError("Linker Hand L30 CANFD 初始化失败")

        time.sleep(0.001)
        ColorMsg(msg=f"连接成功,Linker Hand {self.hand_joint} - {self.hand_type}在CANFD编号为{canfd_id}设备上", color="green")

        self.joint_name_en = JOINT_NAME_EN
        self.joint_name_cn = JOINT_NAME_CN

    def _try_init_controller(self, module, device_id, canfd_id):
        """
        尝试用指定控制类模块创建控制器并连接。

        成功标准：CANFD 连接成功 且 读取到的 hand_type 与期望一致。

        Args:
            module: 控制类模块，须提供 L30DexterousHandController
            device_id: 设备ID(灵巧手ID)
            canfd_id: CANFD 盒子编号
        Returns:
            (controller, reason):
              - 成功返回 (控制器实例, None)
              - 失败返回 (None, 失败原因字符串)
        """
        try:
            controller = module.L30DexterousHandController(
                device_id=device_id, canfd_id=canfd_id, **self._comm_kwargs)
            result = controller.connect(enable_on_connect=self.enable_on_connect)
            # connect 可能返回 (bool, hand_type) 元组或 bool，统一取成功标志
            ok = result[0] if isinstance(result, tuple) else bool(result)
            if not ok:
                controller.disconnect()
                return None, "CANFD连接失败"
            # 校验左右手类型：不匹配则视为该控制类失败，交由外层回退
            if self.hand_type != controller.hand_type:
                ColorMsg(msg=f"hand_type不匹配(期望={self.hand_type}, 实际={controller.hand_type})", color="red")
                controller.disconnect()
                return None, f"hand_type不匹配(期望={self.hand_type}, 实际={controller.hand_type})"
            return controller, None
        except Exception as e:
            ColorMsg(msg=f"控制类初始化异常: {e}", color="red")
            return None, f"异常: {e}"


        

    def set_joint_torques(self, torques: List[int] = [1024] * 17)->None:
        """
        设置17个关节扭矩，默认1024
        
        Args:
            torques: 17个关节扭矩值，范围-2047~2047，单位6.5mA
        """
        if not isinstance(torques, list) or len(torques) != 17:
            raise ValueError(f"需要17个长度的list，当前为{len(torques) if isinstance(torques, list) else type(torques)}")
        
        tor = []
        for i, val in enumerate(torques):
            if not isinstance(val, int):
                raise TypeError(f"第{i}个值必须是int类型，当前为{type(val)}: {val}")
            tor.append(max(-2047, min(2047, val)))
        #print(f"设置扭矩:{tor}")
        self.hand_controller.set_torques(tor)

    def set_velocities(self, velocities: List[int] = [150] * 17)->None:
        """
        设置17个关节速度,默认150
        
        Args:
            velocities: 17个关节速度值，范围0~150，单位0.732RPM
        """
        if not isinstance(velocities, list) or len(velocities) != 17:
            raise ValueError(f"需要17个长度的list，当前为{len(velocities) if isinstance(velocities, list) else type(velocities)}")
        
        vel = []
        for i, val in enumerate(velocities):
            if not isinstance(val, int):
                raise TypeError(f"第{i}个值必须是int类型，当前为{type(val)}: {val}")
            vel.append(max(0, min(150, val)))
        #print(f"设置速度:{vel}")
        self.hand_controller.set_velocities(vel)

    def set_positions(self, positions: List[int] = [0] * 17)->None:
        """
        设置17个关节位置,默认0
        
        Args:
            positions: 17个关节位置值，范围-2047~2047，单位0.732RPM
        """
        # positions的数据已经在L30DexterousHandController类中进行验证
        self.hand_controller.set_positions(positions)


    def get_all_state(self,is_touch=False)->Dict[str, Any]:
        ''' 获取所有状态 
        
        Returns:
            Dict
            all_state: 所有状态，包括：
                - positions: 关节位置
                - velocities: 关节速度
                - temperatures: 关节温度
                - currents: 关节电流
                - error_codes: 错误码
                - matrix_touch: 五指指尖压感矩阵
                
        '''
        all_state = self.hand_controller.get_all_state(is_touch=is_touch)
        if all_state is None:
            raise ValueError("获取所有状态失败")
        return all_state

    def get_joint_state(self)->Optional[List[int]]:
        """
        获取17个关节状态
        
        Returns:
            List[int]
            17个关节状态
        """
        joint_state = self.hand_controller.protocol.get_joint_positions()
        if joint_state is None:
            raise ValueError("获取关节状态失败")
        return joint_state

    def get_joint_velocities(self)->Optional[List[int]]:
        """
        获取17个关节速度
        
        Returns:
            List[int]
            17个关节速度
        """
        joint_velocities = self.hand_controller.protocol.get_joint_velocities()
        if joint_velocities is None:
            raise ValueError("获取关节速度失败")
        return joint_velocities

    def get_joint_torques(self)->Optional[List[int]]:
        """
        获取17个关节扭矩
        
        Returns:
            List[int]
            17个关节扭矩
        """
        joint_torques = self.hand_controller.protocol.get_joint_torques()
        if joint_torques is None:
            raise ValueError("获取关节扭矩失败")
        return joint_torques

    def get_joint_temperatures(self)->Optional[List[int]]:
        """
        获取17个关节温度
        
        Returns:
            List[int]
            17个关节温度
        """
        joint_temperatures = self.hand_controller.protocol.get_joint_temperatures()
        if joint_temperatures is None:
            raise ValueError("获取关节温度失败")
        return joint_temperatures

    def get_joint_currents(self)->Optional[List[int]]:
        """
        获取17个关节电流
        
        Returns:
            List[int]
            17个关节电流
        """
        joint_currents = self.hand_controller.protocol.get_joint_currents()
        if joint_currents is None:
            raise ValueError("获取关节电流失败")

    def get_matrix_touch(self)->Dict[str, List[int]]:
        """
        获取五指指尖压感矩阵
        
        Returns:
            List[int]
            五指指尖压感矩阵
        """
        matrix_touch = self.hand_controller.get_matrix_touch()
        if matrix_touch is None:
            raise ValueError("获取五指指尖压感矩阵失败")
        return matrix_touch

    def disconnect(self):
        self.hand_controller.disconnect()

    def __del__(self) -> None:
        # 初始化失败时 hand_controller 可能为 None，判空避免析构噪音
        if self.hand_controller is not None:
            self.hand_controller.disconnect()
            time.sleep(0.001)

if __name__ == "__main__":
    api = LinkerHandL30API(hand_joint="L30", hand_type="right", device_id=0x06)
    api.set_positions(positions=[1000, 2, 145, -13, 3, 2, -3, 7, 3, 5, 12, -3, 1, 1, 11, 9, 137])

    
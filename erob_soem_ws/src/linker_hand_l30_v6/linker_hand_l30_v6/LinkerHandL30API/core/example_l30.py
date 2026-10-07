#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L30灵巧手使用示例

演示基于新CANFD协议的控制类使用方式
"""

import time

from linker_hand_l30_v6_canfd import (
    L30DexterousHandController,
    L30CANFDProtocol,
    Command,
    HandType,
    JOINT_DEFINITIONS,
    JOINT_LIMITS,
    StatusCode,
)


def example_basic_connection():
    """基本连接示例"""
    print("=" * 50)
    print("示例1: 基本连接")
    print("=" * 50)

    controller = L30DexterousHandController(device_id=0x06)

    # 连接 (会自动使能所有关节)
    success, hand_type = controller.connect()
    if not success:
        print("连接失败!")
        return

    print(f"设备类型: {hand_type}")

    # 断开
    controller.disconnect()


def example_low_level_protocol():
    """底层协议使用示例 - 正确的初始化流程"""
    print("=" * 50)
    print("示例2: 底层协议使用 (正确初始化流程)")
    print("=" * 50)

    protocol = L30CANFDProtocol(device_id=0x06)

    if not protocol.initialize():
        return

    # 步骤1: 必须先使能所有关节!
    print("\n步骤1: 使能所有关节")
    if protocol.enable_all_joints():
        print("  关节使能成功")
    else:
        print("  关节使能失败")

    time.sleep(0.1)  # 等待使能生效

    # 步骤2: 查询设备类型
    print("\n步骤2: 查询设备类型")
    hand_type = protocol.query_device_type()
    print(f"  设备类型: {hand_type}")

    # 步骤3: 获取设备版本
    print("\n步骤3: 获取设备版本")
    version = protocol.get_device_version()
    if version:
        print(f"  硬件版本: {version['hardware']}")
        print(f"  软件版本: {version['software']}")
        print(f"  机械版本: {version['mechanical']}")

    # 步骤4: 读取关节位置
    print("\n步骤4: 读取关节位置")
    positions = protocol.get_joint_positions()
    if positions:
        print(f"  关节位置: {positions}")

    protocol.close()


def example_joint_control():
    """关节控制示例"""
    print("=" * 50)
    print("示例3: 关节控制")
    print("=" * 50)

    controller = L30DexterousHandController(device_id=0x06)

    if not controller.connect()[0]:
        return

    # 设置所有关节到中位 (归一化值 0.5)
    mid_positions = controller.denormalize_positions([0.5] * 17)

    print(f"设置中位位置: {mid_positions}")
    controller.set_positions(mid_positions)

    # 读取当前位置
    positions = controller.get_positions()
    if positions:
        print(f"当前角度值: {positions}")

    controller.disconnect()


def example_grasp_sequence():
    """抓取序列示例"""
    print("=" * 50)
    print("示例4: 抓取序列")
    print("=" * 50)

    controller = L30DexterousHandController(device_id=0x06)

    if not controller.connect()[0]:
        return

    import time

    # 打开手
    open_pos = controller.denormalize_positions([0.0] * 17)
    controller.set_positions(open_pos)
    print("手已打开")
    time.sleep(1)

    # 半握
    half_grasp = controller.denormalize_positions([0.5] * 17)
    controller.set_positions(half_grasp)
    print("半握状态")
    time.sleep(1)

    # 握拳
    fist = controller.denormalize_positions([1.0] * 17)
    controller.set_positions(fist)
    print("握拳状态")
    time.sleep(1)

    # 打开
    controller.set_positions(open_pos)
    print("手已打开")

    controller.disconnect()


def example_read_all_states():
    """读取所有状态示例"""
    print("=" * 50)
    print("示例5: 读取所有状态")
    print("=" * 50)

    controller = L30DexterousHandController(device_id=0x06)

    if not controller.connect()[0]:
        return

    state = controller.get_state()

    if state:
        print("\n=== 位置 ===")
        if state['positions']:
            for i, pos in enumerate(state['positions']):
                joint = JOINT_DEFINITIONS[i]
                print(f"  关节{i+1} ({joint.finger}-{joint.name}): {pos}")

        print("\n=== 速度 ===")
        if state['velocities']:
            for i, vel in enumerate(state['velocities']):
                joint = JOINT_DEFINITIONS[i]
                rpm = vel * 0.732
                print(f"  关节{i+1} ({joint.finger}-{joint.name}): {vel} ({rpm:.2f} RPM)")

        print("\n=== 温度 ===")
        if state['temperatures']:
            for i, temp in enumerate(state['temperatures']):
                joint = JOINT_DEFINITIONS[i]
                print(f"  关节{i+1} ({joint.finger}-{joint.name}): {temp}°C")

        print("\n=== 错误码 ===")
        if state['error_codes']:
            for i, err in enumerate(state['error_codes']):
                joint = JOINT_DEFINITIONS[i]
                print(f"  关节{i+1} ({joint.finger}-{joint.name}): 0x{err:02X}")

    controller.disconnect()


def example_protocol_operations():
    """底层协议操作示例"""
    print("=" * 50)
    print("示例5: 底层协议操作")
    print("=" * 50)

    protocol = L30CANFDProtocol(device_id=0x06)

    if not protocol.initialize():
        return

    # 使能所有关节 (必须先执行!)
    print("使能所有关节...")
    protocol.enable_all_joints()
    time.sleep(0.1)

    # 查询设备类型
    hand_type = protocol.query_device_type()
    print(f"设备类型: {hand_type}")

    # 获取设备版本
    version = protocol.get_device_version()
    if version:
        print(f"硬件版本: {version['hardware']}")
        print(f"软件版本: {version['software']}")
        print(f"机械版本: {version['mechanical']}")

    # 读取关节位置
    positions = protocol.get_joint_positions()
    if positions:
        print(f"关节位置: {positions}")

    protocol.close()


def example_pressure_sensor():
    """触觉传感器示例"""
    print("=" * 50)
    print("示例6: 触觉传感器")
    print("=" * 50)

    controller = L30DexterousHandController(device_id=0x06)

    if not controller.connect()[0]:
        return

    # 读取拇指压力
    thumb_pressure = controller.protocol.get_thumb_pressure()
    # get_finger_pressure 返回合并后的单一列表（72字节）
    print(f"拇指压力数据: {thumb_pressure}")

    # 可视化压力数据
    import numpy as np
    all_data = np.array(thumb_pressure)
    print(f"压力数据统计: min={all_data.min()}, max={all_data.max()}, mean={all_data.mean():.1f}")

    controller.disconnect()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="L30灵巧手控制示例")
    parser.add_argument(
        "--example",
        type=int,
        default=6,
        choices=[1, 2, 3, 4, 5, 6],
        help="选择示例 (1-6)"
    )
    parser.add_argument(
        "--device-id",
        type=int,
        default=6,
        help="设备ID (1-31), L30默认ID为6"
    )

    args = parser.parse_args()

    # 根据设备ID创建控制器
    if args.example == 1:
        example_basic_connection()
    elif args.example == 2:
        example_joint_control()
    elif args.example == 3:
        example_grasp_sequence()
    elif args.example == 4:
        example_read_all_states()
    elif args.example == 5:
        example_protocol_operations()
    elif args.example == 6:
        example_pressure_sensor()

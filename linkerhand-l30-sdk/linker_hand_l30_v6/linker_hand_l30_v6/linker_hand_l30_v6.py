#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy,time, threading, json, sys
from rclpy.node import Node                     # ROS2 节点类
from rclpy.clock import Clock
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from .LinkerHandL30API.utils.color_msg import ColorMsg
from .LinkerHandL30API.linker_hand_l30_api import LinkerHandL30API


class LinkerHandL30(Node):
    def __init__(self):
        super().__init__('linker_hand_l30_node')
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'L30')
        self.declare_parameter('canfd_id', 0)
        self.declare_parameter('is_touch', False)
        # 通信后端相关参数
        self.declare_parameter('comm_type', 'libcanbus')  # libcanbus | socketcan
        self.declare_parameter('channel', 'can0')          # 仅 socketcan: 接口名
        self.declare_parameter('bitrate', 1000000)         # 仅 socketcan: 仲裁段波特率
        self.declare_parameter('dbitrate', 5000000)        # 仅 socketcan: 数据段波特率
        self.declare_parameter('auto_setup', True)         # 仅 socketcan: 自动拉起接口

        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.canfd_id = self.get_parameter('canfd_id').value
        self.is_touch = self.get_parameter('is_touch').value
        self.comm_type = self.get_parameter('comm_type').value
        self.channel = self.get_parameter('channel').value
        self.bitrate = self.get_parameter('bitrate').value
        self.dbitrate = self.get_parameter('dbitrate').value
        self.auto_setup = self.get_parameter('auto_setup').value
        self.hand_setting_sub = self.create_subscription(String,'/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        self.hand_api = LinkerHandL30API(hand_joint=self.hand_joint, hand_type=self.hand_type,
                                         device_id=0x06, canfd_id=self.canfd_id,
                                         comm_type=self.comm_type, channel=self.channel,
                                         bitrate=self.bitrate, dbitrate=self.dbitrate,
                                         auto_setup=self.auto_setup)
        self.last_pose = []
        self.last_vel = []
        self.set_speed = []
        self.set_torque = []
        self.lock = True
        # ros时间获取
        self.stamp_clock = Clock()
        
        self.publisher_hand_state_ = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state', 10)
        if self.is_touch == True:
            self.publisher_hand_matrix_touch_ = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
            self.publisher_hand_matrix_touch_mass_ = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
        self.publisher_hand_info_ = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        self.control_topic = f'/cb_{self.hand_type}_hand_control_cmd'
        self.subscription = self.create_subscription(
            JointState,
            self.control_topic,
            self.hand_callback,
            10)
        # 设置运动速度（0-1000）
        
        # 获取所有关节名称和对应的电机ID
        self.joint_names = self.hand_api.joint_name_en
        ColorMsg(msg=f"关节名称: {self.joint_names}", color="green")
        # 设置扭矩
        self.hand_api.set_joint_torques([2047] * 17)
        ColorMsg(msg=f"设置扭矩: 2047", color="green")
        # 设置速度
        self.hand_api.set_velocities([250] * 17)
        ColorMsg(msg=f"设置速度: 250", color="green")
        time.sleep(1)  # 等待手部设备准备就绪
        self.pub_thread = threading.Thread(target=self.pub_hand_state)
        self.pub_thread.daemon = True  # 设置为守护线程
        self.pub_thread.start()
        

    def pub_hand_state(self):
        count = 0
        while rclpy.ok():
            if len(self.set_speed) == 17:
                self.hand_api.set_velocities(self.set_speed)
                self.set_speed = []
            if len(self.set_torque) == 17:
                self.hand_api.set_joint_torques(self.set_torque)
                self.set_torque = []
            if len(self.last_pose) == 17:
                self.hand_api.set_positions(self.last_pose)
                self.last_pose = []
            joint_state = self.hand_api.get_joint_state() # 电机当前实时状态
            joint_velocities = self.hand_api.get_joint_velocities() # 电机当前实时速度
            if self.is_touch == True:
                matrix_touch = self.hand_api.get_matrix_touch() # 手部当前实时压力矩阵
            if count % 9 == 0: # 非必要信息减少请求频率
                info_msg = {}
                info_msg["torques"] = self.hand_api.get_joint_torques() # 电机当前实时扭矩
                info_msg["temperatures"] = self.hand_api.get_joint_temperatures() # 电机当前实时温度
                info_msg["currents"] = self.hand_api.get_joint_currents() # 电机当前实时电流
                self.pub_info(info_msg)

                
            state_msg = self.joint_msg(positions=joint_state, joint_names=self.hand_api.joint_name_en, velocity=joint_velocities)
            self.publisher_hand_state_.publish(state_msg)
            if self.is_touch == True:
                self.pub_matrix_dic(matrix_touch)
                self.pub_matrix_mass(matrix_touch)
            count += 1

    def pub_matrix_dic(self,matrix_dic):
        """发布矩阵数据JSON格式"""
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        matrix_dic["stamp"] = {"secs": t_secs, "nsecs": t_nsecs}
        msg.data = json.dumps(matrix_dic)
        self.publisher_hand_matrix_touch_.publish(msg)

    def pub_matrix_mass(self, dic):
        """发布矩阵数据合值 单位g 克 JSON格式"""
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        dic_mass = {}
        dic_mass["stamp"] = {"secs": t_secs, "nsecs": t_nsecs}
        dic_mass["unit"] = "g"
        dic_mass["thumb_mass"] = sum(sum(row) for row in dic["thumb_matrix"])
        dic_mass["index_mass"] = sum(sum(row) for row in dic["index_matrix"])
        dic_mass["middle_mass"] = sum(sum(row) for row in dic["middle_matrix"])
        dic_mass["ring_mass"] = sum(sum(row) for row in dic["ring_matrix"])
        dic_mass["little_mass"] = sum(sum(row) for row in dic["little_matrix"])
        msg.data = json.dumps(dic_mass)
        self.publisher_hand_matrix_touch_mass_.publish(msg)

    def pub_info(self, info_msg):
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        info_msg["stamp"] = {"secs": t_secs, "nsecs": t_nsecs}
        msg.data = json.dumps(info_msg)
        self.publisher_hand_info_.publish(msg)


    def hand_callback(self, msg):
        positions = list(msg.position)
        velocity = list(msg.velocity)
        if not positions:
            return
        if len(positions) == 17:
            self.last_pose = positions
        if len(positions) != 17 and len(positions) > 0:
            ColorMsg(msg=f"警告：positions长度不正确: {len(positions)}", color="red")

        if len(velocity) == 17:
            self.last_vel = velocity
        if len(velocity) != 17 and len(velocity) > 0:
            ColorMsg(msg=f"警告：velocity长度不正确: {len(velocity)}", color="red")
            



    
    

    def joint_msg(self,positions=[0] * 17,joint_names=[],velocity=[0] * 17,effort=[0] * 17):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = joint_names
        joint_state.position = [float(x) for x in positions]
        joint_state.velocity = [float(x) for x in velocity]
        joint_state.effort = [float(x) for x in effort]
        return joint_state
    
    

    def hand_setting_cb(self,msg):
        '''控制命令回调'''
        data = json.loads(msg.data)
        print(f"Received setting command: {data['setting_cmd']}",flush=True)
        try:
            if data["setting_cmd"] == "set_speed": # Set speed
                speed = list(data["params"]["speed"])
                if len(speed) != 17:
                    ColorMsg(msg=f"警告：speed长度不正确: {len(speed)}", color="red")
                    return
                self.set_speed = speed
                ColorMsg(msg=f"Setting Speed: {speed}", color="green")
            if data["setting_cmd"] == "set_max_torque_limits": # Set torque
                torque = list(data["params"]["torque"])
                if len(torque) != 17:
                    ColorMsg(msg=f"警告：torque长度不正确: {len(torque)}", color="red")
                    return
                self.set_torque = torque
                ColorMsg(msg=f"Setting Torque: {torque}", color="green")
        except Exception as e:
            print(f"Error in hand_setting_cb: {e}", flush=True)
            ColorMsg(msg=f"Error in hand_setting_cb: {e}", color="red")

def main(args=None):
    rclpy.init(args=args)
    node = LinkerHandL30()
    try:
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:      # 关闭 CAN 或其他硬件资源
        node.hand_api.disconnect()
        #node.destroy_node()      # 销毁 ROS 节点
        print("程序已退出。")
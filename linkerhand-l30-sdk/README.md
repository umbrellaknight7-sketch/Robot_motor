<img  src="resource/logo.png" width="800">

# LinkerHand灵巧手ROS2 SDK For L30

# 1. **概述**
LinkerHand灵巧手ROS SDK 是灵心巧手(北京)科技有限公司开发，用于L30等LinkerHand灵巧手的驱动软件和功能示例源码。可用于真机与仿真器使用。
LinkerHandROS2 SDK当前支持Ubuntu22.04 ROS humble Python3.10 及以上环境



| Name | Version | Link |
| --- | --- | --- |
| ROS2 SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.11](https://img.shields.io/badge/Python-3.11-3776AB?style=flat-square&logo=python&logoColor=white) ![Ubuntu 24.04](https://img.shields.io/badge/OS-Ubuntu%2024.04-E95420?style=flat-square&logo=ubuntu&logoColor=white) ![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-00B3E6?style=flat-square&logo=ros) ![Windows 11](https://img.shields.io/badge/OS-Windows%2011-0078D4?style=flat-square&logo=windows&logoColor=white) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linkerhand-l30-sdk.git) |


# 2. **警告**

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。

2. 执行动作前请务必进行安全评估，以防止发生碰撞。

3. 请保护好灵巧手。


# 3. **版本说明**
V3.1.0

1. 支持 L30 版 Linker Hand ROS2 驱动（HOP v0.0.3 协议）。
2. 支持 `libcanbus`（金属 CANFD 盒）与 `socketcan`（透明塑封 USB-CANFD）两种通信后端。
3. GUI 控制界面，带有手指舞、数字手势等预设动作。
4. 可选指尖触觉传感器数据发布。

V3.0.2
1. 根据掌型，电机运动范围改变



V3.0.1a
1. 硬件升级到V6版本，优化电机温度。
2. 增加指尖矩阵压感功能。
3. GUI控制可显示矩阵指尖压感热力图和波形图


V2.0.1
1. 更新GUI控制和动作序列


V1.0.2
1. 支持L30线驱版本LinkeerHand灵巧手左手和右手
2. 支持手指运行速度设置

V1.0.1
1. 支持L30线驱版本LinkeerHand灵巧手
2. GUI控制界面





## 4.0 系统与硬件需求
支持以下两类 CANFD 设备，对应不同的通信后端 `comm_type`：

| CANFD 设备 | comm_type | 说明 |
| --- | --- | --- |
| 蓝色 / 黑色金属 CANFD 盒 | `libcanbus` | 使用厂商私有库 `libcanbus.so`，需按 5.2 解压安装 |
| 透明塑封 USB-CANFD 设备 | `socketcan` | 内核原生 SocketCAN（`can0` + python-can），无需安装驱动 |

* 操作系统：Ubuntu24.04

* ROS2版本：Jazzy

* Python版本：V3.12

* 硬件：amd64_x86/arm64 配备 5v标准USB接口
## 4.1 金属 CANFD 盒配置 (comm_type = libcanbus)
将libcanbus用命令解压到/usr/local/lib/目录下面

tar -xvf libcanbus.tar -C /usr/local/lib/

配置环境变量:      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

libcanbus.a为静态库，libcanbus.so为共享库

libcanbus_arm        编译器为    arm-linux-gnueabihf-gcc
libcanbus_arm64      编译器为    aarch64-linux-gnu-gcc
libcanbus(ubuntu20)  编译器为    gcc version 9.4.0
libcanbus(ubuntu22)  编译器为    gcc version 11.3.0


编译时先加载libcanbus库，再加载libusb库(源码注释里面有编译方法)。

如果编译提示pthread相关错误，说明libcanbus.tar
自带的libusb库不匹配， 请安装libusb库        sudo apt-get install libusb-1.0-0-dev 并删掉/usr/local/lib目录下的libusb相关库文件

如果提示ludev错误，请安装libudev-dev：       sudo apt-get install libudev-dev

如果编译过程中找不到cc1plus 请安装           sudo apt-get install --reinstall build-essential


注意：root用户可直接读写usbcan设备，非root用户，需要修改usbcan模块的操作权限，可百度搜索修改方法。
或者尝试将99-canfd.rules文件放到 /etc/udev/rules.d/， 然后执行
```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
重启系统。

## 4.1.1 透明塑封 USB-CANFD 设备配置 (comm_type = socketcan)

1. 确保 type-c 接口下方的开关拨到 **Linux 模式**（若 `lsusb` 显示为 `STM32 Virtual ComPort`，说明仍是串口模式）。
2. 重新插拔 USB，确认接口枚举为原生 CAN：
3. 免驱

## 硬件V5版以前早期CANFD版本请跳转至[L30 CANFD](https://github.com/linker-bot/linkerhand-l30-sdk/tree/v2.0.0)

## 4.2 下载 

```bash
$ mkdir -p linker_hand_l30_sdk/src    #创建目录
$ cd linker_hand_l30_sdk/src    #进入目录
$ git clone https://github.com/linker-bot/linkerhand-l30-sdk.git    #获取SDK
```

## 4.3 安装依赖与编译

```bash
$ cd linker_hand_l30_sdk/src/linker_hand_l30_sdk    #进入目录
$ pip install -r requirements.txt    #安装所需依赖
$ cd linker_hand_l30_sdk # 回到工程目录
$ colcon build --symlink-install    #编译和构建ROS包
```

# 5 启动SDK
- 修改src/linker_hand_l30_v6/launch/linker_hand_l30_v6.launch.py文件，按照文件内参数说明进行修改
```bash
$ cd linker_hand_l30_sdk/
$ source ./install/setup.bash
$ ros2 launch linker_hand_l30_v6 linker_hand_l30_v6.launch.py
$ [linker_hand_l30_v6-1] 2026-04-17 10:10:42  开始连接L30...
$ [linker_hand_l30_v6-1] 找到 1 个设备
$ [linker_hand_l30_v6-1] 设备通道 0 打开成功
$ [linker_hand_l30_v6-1] 使能所有关节...
$ [linker_hand_l30_v6-1] 2026-04-17 10:10:43  连接成功,Linker Hand L30 - right在CANFD编号为0设备上
$ [linker_hand_l30_v6-1] 2026-04-17 10:10:43  关节名称: ['thumb_cmc_pitch', 'thumb_ip_pitch', 'thumb_cmc_yaw', 'thumb_cmc_roll', 'ring_mcp_roll', 'ring_pip_pitch', 'ring_mcp_pitch', 'middle_mcp_pitch', 'middle_pip_pitch', 'pinky_mcp_pitch', 'pinky_pip_pitch', 'pinky_mcp_roll', 'middle_mcp_roll', 'index_mcp_roll', 'index_mcp_pitch', 'index_pip_pitch', 'wrist_pitch']
$ [linker_hand_l30_v6-1] 2026-04-17 10:10:43  设置扭矩: 2047
$ [linker_hand_l30_v6-1] 2026-04-17 10:10:43  设置扭矩: 32767
$ #显示以上信息则SDK启动成功
```

## 5.1启动GUI控制L30灵巧手
 - 启动SDK后，新开一个终端。注:由于有界面，不能使用ssh远程连接开启GUI 或者使用X11 终端
 - 修改gui_control/launch/gui_control.launch.py 按照参数说明，修改左手 or 右手
```bash
$ cd linker_hand_l30_sdk
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```
<img  src="resource/gui.png" width="550">

 - 启动后自带指尖矩阵压感热力图和波形图

<img  src="resource/matrix_touch.png" width="550">

## 6 L30手指控制顺序
["拇指指根弯曲", "拇指指尖弯曲", "拇指侧摆", "拇指旋转", "无名指侧摆", "无名指指尖弯曲", "无名指指根弯曲","中指指根弯曲", "中指指尖弯曲", "小指根弯曲","小指指尖弯曲", "小指侧摆", "中指侧摆", "食指侧摆","食指指根弯曲", "食指指尖弯曲", "手腕"]

["thumb_cmc_pitch", "thumb_ip_pitch", "thumb_cmc_yaw", "thumb_cmc_roll", "ring_mcp_roll", "ring_pip_pitch", "ring_mcp_pitch", "middle_mcp_pitch", "middle_pip_pitch", "pinky_mcp_pitch", "pinky_pip_pitch", "pinky_mcp_roll", "middle_mcp_roll", "index_mcp_roll", "index_mcp_pitch", "index_pip_pitch", "wrist_pitch"]

## 6.1 控制范围限制，对应控制顺序 电机ID:(运动范围)
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

## 6.2 TOPIC说明
/cb_hand_setting_cmd  # 设置扭矩、速度控制
/cb_{left|right}_hand_control_cmd  # 控制position list len 17 16手指电机+1手腕电机
/cb_{left|right}_hand_info # 显示温度、电流、电机故障码等数据
/cb_{left|right}_hand_matrix_touch # 五指指尖矩阵压感，12行6列
/cb_{left|right}_hand_matrix_touch_mass # 五指指尖矩阵压感和值
/cb_{left|right}_hand_state # 电机状态 list len 17 16手指电机+1手腕电机

## 6.3 TOPIC数据结构
```bash
$ ros2 topic echo /cb_hand_setting_cmd -f
data: '{"setting_cmd": "set_speed", "params": {"hand_type": "right", "speed": [32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767]}}'
```

```bash
$ ros2 topic echo /cb_right_hand_info -f
data: '{"torques": [32, 0, 28, 40, 36, 28, 36, 28, 28, 28, 28, 32, 32, 32, 0, 32, 44], "temperatures": [45, 45, 43, 46, 44, 42, 44, 42, 46, 44, 45, 48, 45, 44, 42, 44, 42], "currents": null, "stamp": {"secs": 1776678491, "nsecs": 243660021}}'
```

```bash
$ ros2 topic echo /cb_right_hand_control_cmd --flow-style
header:
  stamp:
    sec: 1776678323
    nanosec: 901946398
  frame_id: ''
name: [thumb_cmc_pitch, thumb_ip_pitch, thumb_cmc_yaw, thumb_cmc_roll, ring_mcp_roll, ring_pip_pitch, ring_mcp_pitch, middle_mcp_pitch, middle_pip_pitch, pinky_mcp_pitch, pinky_pip_pitch, pinky_mcp_roll, middle_mcp_roll, index_mcp_roll, index_mcp_pitch, index_pip_pitch, wrist_pitch]
position: [175.0, 1.0, 187.0, 0.0, 0.0, 2.0, 0.0, 8.0, 3.0, 8.0, 12.0, -3.0, 0.0, 1.0, 11.0, 10.0, 137.0]
velocity: [32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0, 32767.0]
effort: []
---
```

```bash
$ ros2 topic echo /cb_right_hand_matrix_touch -f
data: '{"thumb_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "index_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "middle_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "ring_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "little_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 6, 0, 0], [0, 0, 0, 1, 1, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "stamp": {"secs": 1776678587, "nsecs": 562649939}}'
```

```bash
$ ros2 topic echo /cb_right_hand_matrix_touch_mass -f
data: '{"stamp": {"secs": 1776678632, "nsecs": 509134375}, "unit": "g", "thumb_mass": 0, "index_mass": 0, "middle_mass": 1, "ring_mass": 0, "little_mass": 8}'
```



## 7 左右双手使用
 - 将两个CANFD设备插入USB上。终端查看两个设备是否存在
```bash
$ lsusb
$ Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
$ Bus 001 Device 002: ID 048d:5702 Integrated Technology Express, Inc. RGB LED Controller
$ Bus 001 Device 003: ID a8fa:8598 Com Equipment CANFD Analyser
$ Bus 001 Device 004: ID 05e3:0608 Genesys Logic, Inc. Hub
$ Bus 001 Device 005: ID 8087:0aaa Intel Corp. Bluetooth 9460/9560 Jefferson Peak (JfP)
$ Bus 001 Device 006: ID a8fa:8598 Com Equipment CANFD Analyser
$ Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# 可以看到两个CANFD设备
```
 - 查看CANFD设备详细信息
```bash
$ lsusb -v -d a8fa:8598
Bus 001 Device 003: ID a8fa:8598 Com Equipment CANFD Analyser
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 [unknown]
  bDeviceSubClass         0 [unknown]
  bDeviceProtocol         0
  bMaxPacketSize0        64
  idVendor           0xa8fa Com Equipment
  idProduct          0x8598 CANFD Analyser
  bcdDevice            3.00
  iManufacturer           1 Com Equipment
  iProduct                2 CANFD Analyser
  iSerial                 3 F0802061329D3641
......
Bus 001 Device 006: ID a8fa:8598 Com Equipment CANFD Analyser
Couldn't open device, some information will be missing
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 [unknown]
  bDeviceSubClass         0 [unknown]
  bDeviceProtocol         0
  bMaxPacketSize0        64
  idVendor           0xa8fa Com Equipment
  idProduct          0x8598 CANFD Analyser
  bcdDevice            3.00
  iManufacturer           1 Com Equipment
  iProduct                2 CANFD Analyser
  iSerial                 3 F080205B34AB4B31
# 可以查看到两个CANFD的iSerial序列号
```
 - 修改99-double-canfd.rules
```bash
# 设备 1 (F0802061329D3641) 左手
SUBSYSTEM=="usb", ATTRS{idVendor}=="a8fa", ATTRS{idProduct}=="8598", ATTRS{serial}=="左手iSerial序列号", MODE="0666", GROUP="plugdev", SYMLINK+="L30_LEFT"

# 设备 2 (F080205B34AB4B31) 右手
SUBSYSTEM=="usb", ATTRS{idVendor}=="a8fa", ATTRS{idProduct}=="8598", ATTRS{serial}=="右手iSerial序列号", MODE="0666", GROUP="plugdev", SYMLINK+="L30_RIGHT"
```
 - 将99-double-canfd.rules复制到/etc/udev/rules.d/目录
```bash
$ sudo cp 99-double-canfd.rules /etc/udev/rules.d/99-double-canfd.rules
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
 - 查看是否配置成功
```bash
$ ls /dev
# 查看是否有L30_LEFT和L30_RIGHT
```
 - 修改src/linker_hand_l30_v6/launch/linker_hand_l30_v6_double.launch.py文件，按照文件内参数说明进行修改

#!/usr/bin/env python3
"""Check the isolated display, authenticated WebSocket/VNC path and GUI sliders."""
import base64
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import time
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from cryptography.hazmat.primitives.ciphers import Cipher, modes
try:
    from cryptography.hazmat.decrepit.ciphers.algorithms import TripleDES
except ImportError:
    from cryptography.hazmat.primitives.ciphers.algorithms import TripleDES

ROOT = Path(__file__).resolve().parents[3]
ARTIFACTS = ROOT/'erob_soem_ws/artifacts/dev/rviz_viewer'
LOGS = ROOT/'erob_soem_ws/log/dev/rviz_viewer'


def exact(sock, count):
    data = b''
    while len(data) < count:
        part = sock.recv(count-len(data))
        if not part:
            raise RuntimeError('Unexpected connection close')
        data += part
    return data


class WebSocket:
    def __init__(self):
        self.sock = socket.create_connection(('127.0.0.1', 6080), timeout=10)
        self.buffer = b''
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(('GET /websockify HTTP/1.1\r\nHost: localhost:6080\r\n'
            'Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\n'
            'Sec-WebSocket-Protocol: binary\r\nSec-WebSocket-Key: '+key+'\r\n\r\n').encode())
        header = b''
        while not header.endswith(b'\r\n\r\n'):
            header += exact(self.sock, 1)
        assert b'101 Switching Protocols' in header
        expected = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
        assert expected in header

    def send(self, data):
        assert len(data) < 126
        mask = os.urandom(4)
        self.sock.sendall(bytes([0x82, 0x80 | len(data)])+mask+bytes(v ^ mask[i%4] for i,v in enumerate(data)))

    def read(self, count):
        while len(self.buffer) < count:
            a,b = exact(self.sock, 2)
            size = b & 127
            if size == 126: size = struct.unpack('!H',exact(self.sock,2))[0]
            elif size == 127: size = struct.unpack('!Q',exact(self.sock,8))[0]
            assert not b & 128
            payload = exact(self.sock,size)
            assert a & 15 in (0,2), 'Unexpected WebSocket opcode'
            self.buffer += payload
        result,self.buffer = self.buffer[:count],self.buffer[count:]
        return result


def check_network():
    for resource in ('vnc.html','app/ui.js','core/rfb.js'):
        with urllib.request.urlopen('http://127.0.0.1:6080/'+resource,timeout=10) as response:
            assert response.status == 200 and len(response.read()) > 100
    ws = WebSocket()
    assert ws.read(12) == b'RFB 003.008\n'
    ws.send(b'RFB 003.008\n')
    kinds = ws.read(ws.read(1)[0]); assert 2 in kinds
    ws.send(b'\x02')
    challenge = ws.read(16)
    password = (ARTIFACTS/'run/vnc_password.txt').read_bytes().strip()[:8].ljust(8,b'\0')
    key = bytes(int(f'{b:08b}'[::-1],2) for b in password)
    encryptor = Cipher(TripleDES(key), modes.ECB()).encryptor()
    ws.send(encryptor.update(challenge)+encryptor.finalize())
    assert ws.read(4) == b'\0'*4, 'VNC authentication failed'
    ws.send(b'\x01')
    init = ws.read(24)
    width,height = struct.unpack('!HH',init[:4])
    assert (width,height) == (1600,1000)
    ws.read(struct.unpack('!I',init[20:24])[0])
    ws.sock.close()
    return {'websocket_vnc_authenticated': True, 'display_size': [width,height]}


class Observer(Node):
    def __init__(self):
        super().__init__('kinematics_viewer_acceptance')
        self.joints = {}
        self.transforms = {}
        self.create_subscription(JointState,'/joint_states',self.joint_state,10)
        self.create_subscription(TFMessage,'/tf',self.tf,10)

    def joint_state(self,msg):
        self.joints.update(zip(msg.name,msg.position))

    def tf(self,msg):
        self.transforms.update({t.child_frame_id:t.transform for t in msg.transforms})

    def wait(self,condition,timeout=15):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if condition(): return
        raise RuntimeError('Timed out waiting for viewer feedback')


def check_sliders():
    x11 = ctypes.CDLL('libX11.so.6')
    xtest = ctypes.CDLL('libXtst.so.6')
    x11.XOpenDisplay.argtypes=[ctypes.c_char_p]; x11.XOpenDisplay.restype=ctypes.c_void_p
    x11.XFlush.argtypes=[ctypes.c_void_p]; x11.XCloseDisplay.argtypes=[ctypes.c_void_p]
    xtest.XTestFakeMotionEvent.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_ulong]
    xtest.XTestFakeButtonEvent.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]
    display=x11.XOpenDisplay(None); assert display
    def motion(x,y):
        xtest.XTestFakeMotionEvent(display,-1,x,y,0); x11.XFlush(display)
    def button(pressed):
        xtest.XTestFakeButtonEvent(display,1,pressed,0); x11.XFlush(display)
    rclpy.init()
    node=Observer()
    try:
        node.wait(lambda: len(node.joints)==19 and len(node.transforms)==19)
        nodes={name for name,_ in node.get_node_names_and_namespaces()}
        assert not nodes & {'controller_manager','wheel_human_commander','wheel_human_wrist_adapter'}
        # Coordinates correspond to the startup 1600x1000 layout; run before user rearrangement.
        before=node.transforms['Left_Arm_Link1'].rotation.z
        motion(1248,603); button(1); time.sleep(.1); motion(1280,603); time.sleep(.1); button(0)
        node.wait(lambda: abs(node.joints['Left_Shoulder_Joint'])>.2 and
                  abs(node.transforms['Left_Arm_Link1'].rotation.z-before)>.01)
        moved=node.joints['Left_Shoulder_Joint']
        motion(1350,52); button(1); button(0)  # Center button restores the zero pose.
        node.wait(lambda: all(abs(q)<1e-6 for q in node.joints.values()))
        return {'joint_count':19,'transform_count':19,'slider_test_angle_rad':moved,'restored_zero_pose':True}
    finally:
        node.destroy_node();rclpy.shutdown();x11.XCloseDisplay(display)


if __name__=='__main__':
    result=check_network()
    result.update(check_sliders())
    (LOGS/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

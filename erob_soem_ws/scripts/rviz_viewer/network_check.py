#!/usr/bin/env python3
"""Check the localhost noVNC endpoint and authenticated WebSocket/VNC path."""

import base64
import hashlib
import os
from pathlib import Path
import socket
import struct
import urllib.request

from cryptography.hazmat.primitives.ciphers import Cipher, modes
try:
    from cryptography.hazmat.decrepit.ciphers.algorithms import TripleDES
except ImportError:
    from cryptography.hazmat.primitives.ciphers.algorithms import TripleDES

WORKSPACE = Path(__file__).resolve().parents[2]
ARTIFACTS = WORKSPACE/'artifacts/dev/rviz_viewer'
LOGS = WORKSPACE/'log/dev/rviz_viewer'


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

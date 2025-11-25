# sf_can_controller.py

import socket
import struct
import sys
import select
from dataclasses import dataclass
from typing import List, Optional, Tuple

# CAN 帧格式和 ID 定义
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

HEARTBEAT_FUNC_ID = 0x780
FUNC_ID_NMT = 0x000
FUNC_ID_RPDO1 = 0x200


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """浮点数转定点整数。"""
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """定点整数转浮点数。"""
    span = x_max - x_min
    return float(x_int) * span / ((1 << bits) - 1) + x_min


@dataclass
class MITState:
    """SF 电机状态和限制。"""
    pos: float = 0.0
    vel: float = 0.0
    tor: float = 0.0
    posMin: float = -3.14
    posMax: float = 3.14
    velMin: float = -45.0
    velMax: float = 45.0
    kpMin: float = 0.0
    kpMax: float = 500.0
    kdMin: float = 0.0
    kdMax: float = 5.0
    torMin: float = -18.0
    torMax: float = 18.0


class SocketCAN:
    """SocketCAN 封装类。"""
    def __init__(self, interface: str):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setblocking(False)
        self.sock.bind((interface,))

    def send(self, can_id: int, data: bytes) -> None:
        """发送 CAN 帧。"""
        payload = data.ljust(8, b"\x00")
        frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), payload)
        self.sock.send(frame)

    def recv(self) -> Optional[Tuple[int, int, bytes]]:
        """接收 CAN 帧 (非阻塞)。"""
        try:
            frame = self.sock.recv(CAN_FRAME_SIZE)
        except BlockingIOError:
            return None
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
        return can_id, dlc, data

    def close(self) -> None:
        """关闭 SocketCAN 连接。"""
        self.sock.close()


class MotorController:
    """SF 电机控制逻辑类。"""
    def __init__(self, interface: str, motor_id: int):
        self.can = SocketCAN(interface)
        self.motor_id = motor_id
        # 存储所有电机的状态
        self.devices_state: List[MITState] = [MITState() for _ in range(4)]
        self.command = MITState() # 用于封包限制

    def _send_cmd(self, base_id: int, data: bytes) -> None:
        """发送 CAN 命令的私有方法。"""
        can_id = (base_id + self.motor_id) & socket.CAN_SFF_MASK
        self.can.send(can_id, data)

    # --- NMT 控制命令 ---
    def enable(self) -> None:
        """使能电机。"""
        data = bytes([0xFF] * 7 + [0xFC])
        self._send_cmd(FUNC_ID_NMT, data)

    def disable(self) -> None:
        """失能电机。"""
        data = bytes([0xFF] * 7 + [0xFD])
        self._send_cmd(FUNC_ID_NMT, data)

    # --- MIT 控制命令 ---
    def send_mit_command(self, pos: float, vel: float, kp: float, kd: float, tor: float) -> None:
        """打包并发送 MIT 控制命令。"""
        cmd = self.command
        
        pos_int = float_to_uint(pos, cmd.posMin, cmd.posMax, 16)
        vel_int = float_to_uint(vel, cmd.velMin, cmd.velMax, 12)
        kp_int = float_to_uint(kp, cmd.kpMin, cmd.kpMax, 12)
        kd_int = float_to_uint(kd, cmd.kdMin, cmd.kdMax, 12)
        tor_int = float_to_uint(tor, cmd.torMin, cmd.torMax, 12)

        MITcommand = bytearray(8)

        MITcommand[0] = (pos_int >> 8)
        MITcommand[1] = (pos_int & 0xFF)
        MITcommand[2] = (vel_int >> 4)
        MITcommand[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8)
        MITcommand[4] = (kp_int & 0xFF)
        MITcommand[5] = (kd_int >> 4)
        MITcommand[6] = ((kd_int & 0xF) << 4) | (tor_int >> 8)
        MITcommand[7] = (tor_int & 0xFF)

        self._send_cmd(FUNC_ID_RPDO1, bytes(MITcommand))

    # --- 接收和状态处理 ---
    def handle_frame(self, can_id: int, dlc: int, data: bytes) -> Optional[MITState]:
        """处理收到的单个 CAN 帧，更新电机状态。"""
        func_id = can_id & 0x780 
        node_id = can_id & 0x7F 

        if func_id != HEARTBEAT_FUNC_ID or node_id == 0:
            return None
            
        index = node_id - 1
        if index >= len(self.devices_state):
            return None

        # 解包数据
        pos_int = (data[1] << 8) | data[2]
        vel_int = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
        tor_int = ((data[4] & 0x0F) << 8) | data[5]

        # 更新状态
        state = self.devices_state[index]
        state.pos = uint_to_float(pos_int, state.posMin, state.posMax, 16)
        state.vel = uint_to_float(vel_int, state.velMin, state.velMax, 12)
        state.tor = uint_to_float(tor_int, state.torMin, state.torMax, 12)
        
        return state

    def poll_rx(self) -> None:
        """轮询接收 CAN 帧，直到接收不到数据，更新内部状态。"""
        while True:
            frame = self.can.recv()
            if frame is None:
                break
            self.handle_frame(*frame)

    def get_motor_state(self):
        """获取目标电机的当前状态。"""
        return self.devices_state[self.motor_id - 1]

    def close(self) -> None:
        """关闭 CAN 连接。"""
        self.can.close()
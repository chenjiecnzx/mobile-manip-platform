"""幻尔 RRC 控制板（STM32）串口协议的帧编解码。

只做字节层面的编码和解析：不打开串口，不依赖 ROS，在任何机器上都能做单元测试。

依据：厂家随车源码包 V1.0.0（2026-06-01）里主控端 Python SDK 的静态阅读，
尚未用实机抓包核对。实机核对后，把抓到的报文补进 tests/test_codec.py。

帧格式（主控发给控制板、控制板发给主控都一样）：

    0xAA 0x55 | 功能码 1 字节 | 数据长度 n 1 字节 | 数据 n 字节 | CRC8 1 字节

CRC8 覆盖功能码、数据长度和数据，不含帧头。多字节数值都是小端。
串口设备 /dev/rrc，波特率 1 000 000。
"""
from __future__ import annotations

import enum
import struct
from typing import Iterable, List, Optional, Tuple

FRAME_HEAD = b"\xAA\x55"  # 帧头
MAX_PAYLOAD = 255         # 数据最多 255 字节（长度字段只有 1 字节）
SERVO_POS_MIN = 0         # 总线舵机位置原始值下限
SERVO_POS_MAX = 1000      # 总线舵机位置原始值上限


class Func(enum.IntEnum):
    """功能码（帧的第 3 字节）。"""

    SYS = 0        # 系统：电池电压上报
    LED = 1
    BUZZER = 2
    MOTOR = 3      # 底盘电机
    PWM_SERVO = 4
    BUS_SERVO = 5  # 总线舵机（机械臂和夹爪）
    KEY = 6
    IMU = 7        # IMU 上报
    GAMEPAD = 8
    SBUS = 9
    OLED = 10


class ServoRead(enum.IntEnum):
    """总线舵机的读取子命令。"""

    POSITION = 0x05  # 位置，原始值 0~1000
    VIN = 0x07       # 输入电压，毫伏（单位待实机核对）
    TEMP = 0x09      # 温度，摄氏度（单位待实机核对）


def crc8(data: bytes) -> int:
    """CRC-8/MAXIM：多项式 x^8+x^5+x^4+1（反射形式 0x8C），初值 0。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def encode_frame(func: int, payload: bytes = b"") -> bytes:
    """把功能码和数据打包成一帧。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"数据 {len(payload)} 字节，超过上限 {MAX_PAYLOAD}")
    body = bytes([int(func), len(payload)]) + bytes(payload)
    return FRAME_HEAD + body + bytes([crc8(body)])


class FrameParser:
    """流式解析器：串口读到多少字节就喂多少，凑齐一帧且校验通过才输出。"""

    def __init__(self) -> None:
        self._buf = bytearray()  # 还没凑成帧的字节
        self.crc_errors = 0      # 校验失败的帧数，用来观察串口质量

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        """喂入新收到的字节，返回这次凑齐的帧，每帧是 (功能码, 数据)。"""
        buf = self._buf
        buf.extend(data)
        frames: List[Tuple[int, bytes]] = []
        while True:
            start = buf.find(FRAME_HEAD)
            if start < 0:
                # 没有完整帧头；末尾若是 0xAA，可能是半个帧头，留着
                keep = 1 if buf.endswith(b"\xAA") else 0
                del buf[: len(buf) - keep]
                break
            del buf[:start]
            if len(buf) < 4:  # 帧头 2 字节 + 功能码 + 长度
                break
            func, n = buf[2], buf[3]
            if func >= len(Func):
                del buf[:2]  # 功能码非法：这不是真帧头，跳过后继续找
                continue
            if len(buf) < 5 + n:  # 一整帧 = 4 + n 字节数据 + 1 字节校验
                break
            if crc8(bytes(buf[2 : 4 + n])) == buf[4 + n]:
                frames.append((func, bytes(buf[4 : 4 + n])))
                del buf[: 5 + n]
            else:
                self.crc_errors += 1
                del buf[:2]  # 校验失败：跳过这个帧头，从后面重新找
        return frames


# ---------- 主控发给控制板的指令 ----------


def motor_speeds(speeds: Iterable[Tuple[int, float]]) -> bytes:
    """底盘轮速指令。

    speeds: [(电机号 1~4, 轮子转速)]，转速单位是转每秒，正负号表示转向。
    线上的电机号从 0 起，所以这里减 1。
    """
    items = list(speeds)
    payload = bytes([0x01, len(items)])
    for motor_id, rps in items:
        if not 1 <= motor_id <= 4:
            raise ValueError(f"电机号 {motor_id} 不在 1~4")
        payload += struct.pack("<Bf", motor_id - 1, rps)
    return encode_frame(Func.MOTOR, payload)


def bus_servo_set_positions(
    duration_ms: int, positions: Iterable[Tuple[int, int]]
) -> bytes:
    """总线舵机位置指令。

    duration_ms: 从当前位置走到目标位置的时长，毫秒，0~65535。
    positions: [(舵机 ID, 目标位置原始值 0~1000)]。
    """
    if not 0 <= duration_ms <= 0xFFFF:
        raise ValueError(f"时长 {duration_ms} ms 不在 0~65535")
    items = list(positions)
    payload = struct.pack("<BHB", 0x01, duration_ms, len(items))
    for servo_id, pos in items:
        if not SERVO_POS_MIN <= pos <= SERVO_POS_MAX:
            raise ValueError(f"舵机 {servo_id} 的位置 {pos} 不在 0~1000")
        payload += struct.pack("<BH", servo_id, pos)
    return encode_frame(Func.BUS_SERVO, payload)


def bus_servo_stop(servo_ids: Iterable[int]) -> bytes:
    """让指定的总线舵机停在当前位置。"""
    ids = list(servo_ids)
    return encode_frame(Func.BUS_SERVO, bytes([0x03, len(ids)] + ids))


def bus_servo_torque(servo_id: int, enable: bool) -> bytes:
    """总线舵机力矩开关：enable 为真上力矩，为假卸力矩（可以用手掰动）。"""
    return encode_frame(Func.BUS_SERVO, bytes([0x0B if enable else 0x0C, servo_id]))


def bus_servo_read(servo_id: int, item: ServoRead) -> bytes:
    """请求读取一个总线舵机的位置、电压或温度。一问一答，每次只能问一个舵机。"""
    return encode_frame(Func.BUS_SERVO, bytes([int(item), servo_id]))


# ---------- 控制板发给主控的上报 ----------

_SERVO_REPLY_FORMAT = {
    ServoRead.POSITION: "<h",  # 位置原始值，有符号 2 字节
    ServoRead.VIN: "<H",       # 电压，无符号 2 字节
    ServoRead.TEMP: "<B",      # 温度，无符号 1 字节
}


def parse_bus_servo_reply(payload: bytes) -> Tuple[int, int, Optional[int]]:
    """解析总线舵机的读取应答，返回 (舵机 ID, 子命令, 数值)。

    应答的前 3 字节是舵机 ID、子命令、状态（0 表示成功）。
    舵机报告失败，或子命令不在 ServoRead 里时，数值为 None。
    """
    servo_id, cmd, status = struct.unpack_from("<BBb", payload)
    if status != 0 or cmd not in _SERVO_REPLY_FORMAT:
        return servo_id, cmd, None
    (value,) = struct.unpack_from(_SERVO_REPLY_FORMAT[ServoRead(cmd)], payload, 3)
    return servo_id, cmd, value


def parse_imu(payload: bytes) -> Tuple[float, float, float, float, float, float]:
    """解析 IMU 上报，返回 (ax, ay, az, gx, gy, gz)。

    ax~az 是三轴加速度，单位 g；gx~gz 是三轴角速度，单位度每秒；6 个 4 字节浮点数。
    单位是按厂家节点的换算（加速度乘重力加速度、角速度转弧度）反推的，待实机核对：
    车静止放平时 az 应接近 1。
    """
    return struct.unpack("<6f", payload)


def parse_battery_mv(payload: bytes) -> Optional[int]:
    """解析系统上报里的电池电压，单位毫伏；不是电压上报时返回 None。"""
    if len(payload) == 3 and payload[0] == 0x04:
        return struct.unpack_from("<H", payload, 1)[0]
    return None

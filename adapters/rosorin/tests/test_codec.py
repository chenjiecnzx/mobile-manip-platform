"""rrc_protocol.codec 的单元测试。

标准报文的来历：按厂家 SDK 的字节顺序组帧，CRC 用厂家的 256 项查表算法独立算出，
不经过本仓库的 crc8()。它们来自静态阅读，实机抓到真实报文后要补进来。
"""
import unittest

from rrc_protocol import codec
from rrc_protocol.codec import Func, ServoRead

# 主控发给控制板
MOTOR_1_FWD = bytes.fromhex("aa5503070101000000803fff")  # 1 号电机 1.0 转每秒
MOTOR_4_STOP = bytes.fromhex(
    "aa5503160104000000000001000000000200000000030000000007"
)  # 4 个电机全部 0
SERVO_SET = bytes.fromhex("aa55050a01e8030201f4010abc0232")  # 1000 ms，1 号到 500，10 号到 700
SERVO_STOP = bytes.fromhex("aa550504030201024b")  # 1、2 号舵机停
TORQUE_OFF = bytes.fromhex("aa5505020c0361")  # 3 号舵机卸力矩
READ_POS = bytes.fromhex("aa55050205028d")  # 读 2 号舵机位置

# 控制板发给主控
REPLY_POS = bytes.fromhex("aa55050502050000029c")  # 2 号舵机位置 512
REPLY_VIN = bytes.fromhex("aa5505050307007c2e2d")  # 3 号舵机电压 11900
REPLY_TEMP = bytes.fromhex("aa5505040409002436")  # 4 号舵机温度 36
REPLY_ERR = bytes.fromhex("aa5505050205ff0000f2")  # 2 号舵机读位置失败
IMU = bytes.fromhex(
    "aa55071800000000000000000000803f0000003f000080be000000400c"
)  # 加速度 (0, 0, 1)，角速度 (0.5, -0.25, 2)
BATTERY = bytes.fromhex("aa55000304182eb0")  # 电池 11800 mV
EMPTY = bytes.fromhex("aa550600aa")  # 数据长度为 0 的帧


class Crc8Test(unittest.TestCase):
    def test_standard_check_value(self):
        # CRC-8/MAXIM 的公开校验值
        self.assertEqual(codec.crc8(b"123456789"), 0xA1)

    def test_empty(self):
        self.assertEqual(codec.crc8(b""), 0)


class EncodeTest(unittest.TestCase):
    def test_motor(self):
        self.assertEqual(codec.motor_speeds([(1, 1.0)]), MOTOR_1_FWD)
        self.assertEqual(
            codec.motor_speeds([(i, 0.0) for i in range(1, 5)]), MOTOR_4_STOP
        )

    def test_motor_id_out_of_range(self):
        for bad in (0, 5):
            with self.assertRaises(ValueError):
                codec.motor_speeds([(bad, 0.0)])

    def test_servo_set_positions(self):
        self.assertEqual(
            codec.bus_servo_set_positions(1000, [(1, 500), (10, 700)]), SERVO_SET
        )

    def test_servo_position_out_of_range(self):
        for bad in (-1, 1001):
            with self.assertRaises(ValueError):
                codec.bus_servo_set_positions(1000, [(1, bad)])

    def test_servo_duration_out_of_range(self):
        for bad in (-1, 65536):
            with self.assertRaises(ValueError):
                codec.bus_servo_set_positions(bad, [(1, 500)])

    def test_servo_stop_torque_read(self):
        self.assertEqual(codec.bus_servo_stop([1, 2]), SERVO_STOP)
        self.assertEqual(codec.bus_servo_torque(3, False), TORQUE_OFF)
        self.assertEqual(codec.bus_servo_read(2, ServoRead.POSITION), READ_POS)

    def test_payload_too_long(self):
        with self.assertRaises(ValueError):
            codec.encode_frame(Func.OLED, bytes(256))


class ParserTest(unittest.TestCase):
    def test_whole_frames(self):
        parser = codec.FrameParser()
        frames = parser.feed(BATTERY + EMPTY + REPLY_POS)
        self.assertEqual([f for f, _ in frames], [Func.SYS, Func.KEY, Func.BUS_SERVO])
        self.assertEqual(frames[1][1], b"")
        self.assertEqual(parser.crc_errors, 0)

    def test_byte_by_byte(self):
        parser = codec.FrameParser()
        frames = []
        for byte in IMU + BATTERY:
            frames += parser.feed(bytes([byte]))
        self.assertEqual([f for f, _ in frames], [Func.IMU, Func.SYS])

    def test_garbage_between_frames(self):
        parser = codec.FrameParser()
        frames = parser.feed(b"\x00\xAA\x13" + BATTERY + b"\xAA\x55\x63" + REPLY_POS)
        self.assertEqual([f for f, _ in frames], [Func.SYS, Func.BUS_SERVO])

    def test_bad_crc_is_dropped_and_counted(self):
        parser = codec.FrameParser()
        broken = BATTERY[:-1] + bytes([BATTERY[-1] ^ 0xFF])
        frames = parser.feed(broken + REPLY_POS)
        self.assertEqual([f for f, _ in frames], [Func.BUS_SERVO])
        self.assertEqual(parser.crc_errors, 1)

    def test_encode_then_parse(self):
        parser = codec.FrameParser()
        payload = bytes(range(200))
        self.assertEqual(
            parser.feed(codec.encode_frame(Func.OLED, payload)),
            [(Func.OLED, payload)],
        )


class ReportTest(unittest.TestCase):
    @staticmethod
    def payload(frame):
        return codec.FrameParser().feed(frame)[0][1]

    def test_servo_replies(self):
        self.assertEqual(
            codec.parse_bus_servo_reply(self.payload(REPLY_POS)), (2, 0x05, 512)
        )
        self.assertEqual(
            codec.parse_bus_servo_reply(self.payload(REPLY_VIN)), (3, 0x07, 11900)
        )
        self.assertEqual(
            codec.parse_bus_servo_reply(self.payload(REPLY_TEMP)), (4, 0x09, 36)
        )

    def test_servo_reply_failure(self):
        self.assertEqual(
            codec.parse_bus_servo_reply(self.payload(REPLY_ERR)), (2, 0x05, None)
        )

    def test_imu(self):
        self.assertEqual(
            codec.parse_imu(self.payload(IMU)), (0.0, 0.0, 1.0, 0.5, -0.25, 2.0)
        )

    def test_battery(self):
        self.assertEqual(codec.parse_battery_mv(self.payload(BATTERY)), 11800)
        self.assertIsNone(codec.parse_battery_mv(b"\x01\x00\x00"))


if __name__ == "__main__":
    unittest.main()

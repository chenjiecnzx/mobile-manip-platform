"""mmp_core.intents 的单元测试：合法调用放行，其余每一种都要被拒绝。"""
import unittest

from mmp_core.intents import RobotState, validate

AUTO = RobotState(mode="auto", estop=False)
MANUAL = RobotState(mode="manual", estop=False)
ESTOP = RobotState(mode="auto", estop=True)

MOVE = {"skill": "move_relative", "args": {"dx_m": 0.5, "dy_m": 0, "dyaw_deg": -90}}
STOP = {"skill": "stop", "args": {}}


class AcceptTest(unittest.TestCase):
    def test_all_four_skills(self):
        for call in (
            MOVE,
            {"skill": "arm_named_pose", "args": {"pose": "home"}},
            {"skill": "gripper", "args": {"action": "open"}},
            STOP,
        ):
            verdict = validate(call, AUTO)
            self.assertTrue(verdict.ok, verdict.reason)
            self.assertEqual(verdict.intent, call)

    def test_json_string(self):
        self.assertTrue(validate('{"skill": "stop", "args": {}}', AUTO).ok)

    def test_range_ends_are_allowed(self):
        call = {"skill": "move_relative",
                "args": {"dx_m": 1.0, "dy_m": -1.0, "dyaw_deg": 180}}
        self.assertTrue(validate(call, AUTO).ok)


class RejectFormatTest(unittest.TestCase):
    def assert_rejected(self, raw, state=AUTO):
        verdict = validate(raw, state)
        self.assertFalse(verdict.ok, f"应当被拒绝：{raw!r}")
        self.assertTrue(verdict.reason)
        self.assertIsNone(verdict.intent)

    def test_not_json(self):
        self.assert_rejected("向前走一米")
        self.assert_rejected('{"skill": "stop", "args": {}')

    def test_not_an_object(self):
        self.assert_rejected('["stop"]')
        self.assert_rejected('"stop"')

    def test_missing_or_extra_top_level_keys(self):
        self.assert_rejected({"skill": "stop"})
        self.assert_rejected({"skill": "stop", "args": {}, "then": STOP})

    def test_unknown_skill(self):
        self.assert_rejected({"skill": "shell", "args": {"cmd": "rm -rf /"}})
        self.assert_rejected({"skill": "set_motor", "args": {"rps": 10}})
        self.assert_rejected({"skill": ["stop"], "args": {}})

    def test_missing_or_extra_args(self):
        self.assert_rejected({"skill": "move_relative", "args": {"dx_m": 0.5}})
        self.assert_rejected(
            {"skill": "gripper", "args": {"action": "open", "force": 100}}
        )
        self.assert_rejected({"skill": "stop", "args": {"reason": "x"}})
        self.assert_rejected({"skill": "stop", "args": []})

    def test_number_out_of_range(self):
        for dx in (1.01, -5, 100):
            self.assert_rejected(
                {"skill": "move_relative",
                 "args": {"dx_m": dx, "dy_m": 0, "dyaw_deg": 0}}
            )

    def test_number_wrong_type(self):
        for dx in ("0.5", True, None, [0.5]):
            self.assert_rejected(
                {"skill": "move_relative",
                 "args": {"dx_m": dx, "dy_m": 0, "dyaw_deg": 0}}
            )

    def test_nan_and_infinity(self):
        # Python 的 json 默认会把 NaN、Infinity 解析成浮点数，必须拦下
        for token in ("NaN", "Infinity", "-Infinity"):
            self.assert_rejected(
                '{"skill": "move_relative", "args": '
                '{"dx_m": %s, "dy_m": 0, "dyaw_deg": 0}}' % token
            )

    def test_choice_not_in_list(self):
        self.assert_rejected({"skill": "arm_named_pose", "args": {"pose": "wave"}})
        self.assert_rejected({"skill": "gripper", "args": {"action": 1}})
        self.assert_rejected({"skill": "gripper", "args": {"action": ["open"]}})


class StateGateTest(unittest.TestCase):
    def test_motion_needs_auto_mode(self):
        verdict = validate(MOVE, MANUAL)
        self.assertFalse(verdict.ok)
        self.assertIn("manual", verdict.reason)

    def test_estop_blocks_motion(self):
        verdict = validate(MOVE, ESTOP)
        self.assertFalse(verdict.ok)
        self.assertIn("急停", verdict.reason)

    def test_stop_is_always_allowed(self):
        for state in (AUTO, MANUAL, ESTOP, RobotState(mode="stopped", estop=True)):
            self.assertTrue(validate(STOP, state).ok)


if __name__ == "__main__":
    unittest.main()

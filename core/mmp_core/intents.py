"""意图校验器。

大模型（或任何上层客户端）只能提交一条“技能调用”，格式是：

    {"skill": "技能名", "args": {"参数名": 参数值, ...}}

校验器逐项检查：格式、技能是否在白名单里、参数是否齐全且在允许范围内、
机器人当前状态是否允许执行。全部通过才交给执行层，任何一项不过就拒绝并给出原因。

校验是确定性的：同样的输入永远得到同样的结果。纯 Python，不依赖 ROS 和具体硬件。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple, Union


@dataclass(frozen=True)
class Num:
    """数值参数的允许范围（含两端）。"""

    low: float
    high: float


@dataclass(frozen=True)
class Choice:
    """枚举参数的允许取值。"""

    options: Tuple[str, ...]


ArgSpec = Union[Num, Choice]

# 竖切阶段的四个技能。参数名带单位：_m 是米，_deg 是度。
SLICE_SKILLS: Dict[str, Dict[str, ArgSpec]] = {
    # 底盘相对当前位置移动：dx_m 向前为正，dy_m 向左为正，dyaw_deg 逆时针为正
    "move_relative": {
        "dx_m": Num(-1.0, 1.0),
        "dy_m": Num(-1.0, 1.0),
        "dyaw_deg": Num(-180.0, 180.0),
    },
    # 机械臂走到事先示教并命名的位姿（名字清单在做机械臂时按实际示教结果改）
    "arm_named_pose": {"pose": Choice(("home", "ready", "stow"))},
    # 夹爪开合
    "gripper": {"action": Choice(("open", "close"))},
    # 停止：没有参数，任何状态下都允许
    "stop": {},
}

ALWAYS_ALLOWED = ("stop",)  # 不受模式和急停限制的技能


@dataclass(frozen=True)
class RobotState:
    """校验时用到的机器人状态，由监督层提供。"""

    mode: str     # "manual" 手动、"auto" 自动、"stopped" 停止
    estop: bool   # 急停是否按下


@dataclass(frozen=True)
class Verdict:
    """校验结果。"""

    ok: bool                         # 是否允许执行
    reason: str                      # 拒绝原因；通过时为空
    intent: Optional[Dict[str, Any]] = None  # 通过时给出整理后的技能调用


def _reject(reason: str) -> Verdict:
    return Verdict(False, reason)


def validate(
    raw: Union[str, Mapping[str, Any]],
    state: RobotState,
    skills: Mapping[str, Mapping[str, ArgSpec]] = SLICE_SKILLS,
) -> Verdict:
    """校验一条技能调用。raw 可以是 JSON 字符串，也可以是已解析的字典。"""
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except json.JSONDecodeError:
            return _reject("不是合法的 JSON")
    if not isinstance(raw, Mapping) or set(raw) != {"skill", "args"}:
        return _reject('必须是只含 "skill" 和 "args" 两项的对象')

    skill, args = raw["skill"], raw["args"]
    if not isinstance(skill, str) or skill not in skills:
        return _reject(f"技能不在白名单里：{skill!r}")
    spec = skills[skill]
    if not isinstance(args, Mapping) or set(args) != set(spec):
        return _reject(f"{skill} 的参数必须正好是 {sorted(spec)}")

    for name, rule in spec.items():
        value = args[name]
        if isinstance(rule, Num):
            # bool 是 int 的子类，要单独排除；NaN 和无穷大也不接受
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return _reject(f"{name} 必须是数值")
            if not math.isfinite(value) or not rule.low <= value <= rule.high:
                return _reject(f"{name}={value} 不在 [{rule.low}, {rule.high}]")
        elif value not in rule.options:
            return _reject(f"{name}={value!r} 不在 {list(rule.options)}")

    if skill not in ALWAYS_ALLOWED:
        if state.estop:
            return _reject("急停已按下")
        if state.mode != "auto":
            return _reject(f"当前是 {state.mode} 模式，只有 auto 模式才执行")

    return Verdict(True, "", {"skill": skill, "args": dict(args)})

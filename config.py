"""
arm_config.yaml 로더.
get_config() 로 ArmConfig 싱글턴 반환.
"""
import os
from dataclasses import dataclass

import yaml

_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "arm_config.yaml")


@dataclass
class CanConfig:
    channel: str
    bustype: str
    bitrate: int


@dataclass
class ControlConfig:
    default_kp: float
    default_kd: float


@dataclass
class MotorLimits:
    p_min: float;  p_max: float
    v_min: float;  v_max: float
    kp_min: float; kp_max: float
    kd_min: float; kd_max: float
    t_min: float;  t_max: float


@dataclass
class JointConfig:
    can_id:     int
    limit_min:  float   # rad
    limit_max:  float   # rad


@dataclass
class ArmConfig:
    can:          CanConfig
    joints:       dict[str, JointConfig]
    control:      ControlConfig
    motor_limits: MotorLimits


_config: ArmConfig | None = None


def get_config() -> ArmConfig:
    global _config
    if _config is None:
        _config = _load(_CONFIG_PATH)
    return _config


def _load(path: str) -> ArmConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    c = raw["can"]
    can_cfg = CanConfig(channel=c["channel"], bustype=c["bustype"], bitrate=int(c["bitrate"]))

    import math
    joints = {
        name: JointConfig(
            can_id=int(info["can_id"]),
            limit_min=math.radians(float(info["limit_min"])),
            limit_max=math.radians(float(info["limit_max"])),
        )
        for name, info in raw["joints"].items()
    }

    ctrl = raw["control"]
    control_cfg = ControlConfig(default_kp=float(ctrl["default_kp"]), default_kd=float(ctrl["default_kd"]))

    ml = raw["motor_limits"]
    limits_cfg = MotorLimits(
        p_min=float(ml["position"]["min"]),  p_max=float(ml["position"]["max"]),
        v_min=float(ml["velocity"]["min"]),  v_max=float(ml["velocity"]["max"]),
        kp_min=float(ml["kp"]["min"]),       kp_max=float(ml["kp"]["max"]),
        kd_min=float(ml["kd"]["min"]),       kd_max=float(ml["kd"]["max"]),
        t_min=float(ml["torque"]["min"]),    t_max=float(ml["torque"]["max"]),
    )

    return ArmConfig(can=can_cfg, joints=joints, control=control_cfg, motor_limits=limits_cfg)

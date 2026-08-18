"""
arm_config.yaml 로더.
get_config() 로 ArmConfig 싱글턴 반환.
"""
import os
from dataclasses import dataclass

import yaml

_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_config.yaml")


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
    kp_min: float; kp_max: float
    kd_min: float; kd_max: float


@dataclass
class MotorModelSpec:
    v_min: float; v_max: float
    t_min: float; t_max: float


@dataclass
class JointConfig:
    can_id:      int
    motor_model: str    # 'RS02' or 'RS06'
    limit_min:   float  # rad
    limit_max:   float  # rad
    kp:          float  # position gain (overrides control.default_kp)
    kd:          float  # velocity gain (overrides control.default_kd)
    sign:        float  # +1/-1, 모터 회전 방향 보정


@dataclass
class ArmConfig:
    can:          CanConfig
    joints:       dict[str, JointConfig]
    control:      ControlConfig
    motor_limits: MotorLimits
    motor_models: dict[str, MotorModelSpec]


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
    ctrl = raw["control"]
    control_cfg = ControlConfig(
        default_kp=float(ctrl["default_kp"]),
        default_kd=float(ctrl["default_kd"]),
    )

    joints = {
        name: JointConfig(
            can_id=int(info["can_id"]),
            motor_model=str(info.get("motor_model", "RS02")),
            limit_min=math.radians(float(info["limit_min"])),
            limit_max=math.radians(float(info["limit_max"])),
            kp=float(info["kp"]) if "kp" in info else control_cfg.default_kp,
            kd=float(info["kd"]) if "kd" in info else control_cfg.default_kd,
            sign=float(info.get("sign", 1.0)),
        )
        for name, info in raw["joints"].items()
    }

    ml = raw["motor_limits"]
    limits_cfg = MotorLimits(
        kp_min=float(ml["kp"]["min"]), kp_max=float(ml["kp"]["max"]),
        kd_min=float(ml["kd"]["min"]), kd_max=float(ml["kd"]["max"]),
    )

    motor_models = {
        model: MotorModelSpec(
            v_min=float(spec["velocity"]["min"]),
            v_max=float(spec["velocity"]["max"]),
            t_min=float(spec["torque"]["min"]),
            t_max=float(spec["torque"]["max"]),
        )
        for model, spec in raw["motor_models"].items()
    }

    return ArmConfig(can=can_cfg, joints=joints, control=control_cfg, motor_limits=limits_cfg, motor_models=motor_models)

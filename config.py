"""
arm_config.yaml 로더.
get_config() 로 ArmConfig 싱글턴 반환. 모든 모듈이 이 함수를 통해 설정을 읽음.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

_HERE        = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_HERE, "config", "arm_config.yaml")


# ── 설정 데이터 클래스 ──────────────────────────────────────────────────────────

@dataclass
class CanConfig:
    channel:   str
    bustype:   str
    bitrate:   int
    master_id: int


@dataclass
class ControlConfig:
    default_kp: float
    default_kd: float


@dataclass
class TrajectoryConfig:
    frequency: int


@dataclass
class IKConfig:
    ee_frame:   str
    n_steps:    int
    n_restarts: int
    pos_tol:    float
    rot_tol:    float
    damping:    float
    dq_max:     float


@dataclass
class MotorLimits:
    p_min:  float
    p_max:  float
    v_min:  float
    v_max:  float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float
    t_min:  float
    t_max:  float


@dataclass
class ArmConfig:
    can:          CanConfig
    joints:       dict[str, int]   # joint_name → can_id
    control:      ControlConfig
    trajectory:   TrajectoryConfig
    ik:           IKConfig
    motor_limits: MotorLimits


# ── 싱글턴 로더 ────────────────────────────────────────────────────────────────

_config: ArmConfig | None = None


def get_config() -> ArmConfig:
    """arm_config.yaml 을 파싱해 ArmConfig 싱글턴 반환."""
    global _config
    if _config is None:
        _config = _load(_CONFIG_PATH)
    return _config


def _load(path: str) -> ArmConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    c = raw["can"]
    can_cfg = CanConfig(
        channel=c["channel"],
        bustype=c["bustype"],
        bitrate=int(c["bitrate"]),
        master_id=int(c["master_id"], 16) if isinstance(c["master_id"], str) else int(c["master_id"]),
    )

    joints = {name: int(info["can_id"]) for name, info in raw["joints"].items()}

    ctrl = raw["control"]
    control_cfg = ControlConfig(
        default_kp=float(ctrl["default_kp"]),
        default_kd=float(ctrl["default_kd"]),
    )

    traj_cfg = TrajectoryConfig(frequency=int(raw["trajectory"]["frequency"]))

    ik = raw["ik"]
    ik_cfg = IKConfig(
        ee_frame=ik["ee_frame"],
        n_steps=int(ik["n_steps"]),
        n_restarts=int(ik["n_restarts"]),
        pos_tol=float(ik["pos_tol"]),
        rot_tol=float(ik["rot_tol"]),
        damping=float(ik["damping"]),
        dq_max=float(ik["dq_max"]),
    )

    ml = raw["motor_limits"]
    limits_cfg = MotorLimits(
        p_min=float(ml["position"]["min"]), p_max=float(ml["position"]["max"]),
        v_min=float(ml["velocity"]["min"]), v_max=float(ml["velocity"]["max"]),
        kp_min=float(ml["kp"]["min"]),     kp_max=float(ml["kp"]["max"]),
        kd_min=float(ml["kd"]["min"]),     kd_max=float(ml["kd"]["max"]),
        t_min=float(ml["torque"]["min"]),  t_max=float(ml["torque"]["max"]),
    )

    return ArmConfig(
        can=can_cfg,
        joints=joints,
        control=control_cfg,
        trajectory=traj_cfg,
        ik=ik_cfg,
        motor_limits=limits_cfg,
    )

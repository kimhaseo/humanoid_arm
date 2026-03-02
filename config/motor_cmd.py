from dataclasses import dataclass


MOTOR_MAPPING = {
    "left_joint1": 0x141,
    "left_joint2": 0x142,
    "left_joint3": 0x143,
    "left_joint4": 0x144,
    "left_joint5": 0x145,
    "left_joint6": 0x146,
    "left_joint7": 0x147
}


def get_can_id(motor_name: str) -> int:
    can_id = MOTOR_MAPPING.get(motor_name)
    if can_id is None:
        raise ValueError(f"Unknown motor name: '{motor_name}'. "
                         f"Available: {list(MOTOR_MAPPING.keys())}")
    return can_id


@dataclass
class AngleCommand:
    motor_name: str
    angle: float
    speed: int = 400

    def __post_init__(self):
        self.can_id = get_can_id(self.motor_name)


@dataclass
class AccelCommand:
    motor_name: str
    accel: int

    def __post_init__(self):
        self.can_id = get_can_id(self.motor_name)


@dataclass
class PidCommand:
    motor_name: str
    p_gain: int
    i_gain: int

    def __post_init__(self):
        self.can_id = get_can_id(self.motor_name)

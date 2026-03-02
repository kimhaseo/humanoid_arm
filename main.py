"""
main.py — IK 계산 후 각 관절값을 모터로 전송.
"""

import logging
import numpy as np

from ik_solver import IKSolver
from motor_controller import MotorController
from config.motor_cmd import AngleCommand

log = logging.getLogger(__name__)

# ── URDF 조인트 이름 → 모터 이름 매핑 (URDF 조인트명에 맞게 수정) ──
JOINT_TO_MOTOR = {
    "joint_1": "left_joint1",
    "joint_2": "left_joint2",
    "joint_3": "left_joint3",
    "joint_4": "left_joint4",
    "joint_5": "left_joint5",
    "joint_6": "left_joint6",
    "joint_7": "left_joint7",
}


def ik_to_commands(q_dict: dict, speed: int = 400) -> list[AngleCommand]:
    """IK 결과 dict → AngleCommand 리스트 변환."""
    commands = []
    for urdf_joint, angle_deg in q_dict.items():
        motor_name = JOINT_TO_MOTOR.get(urdf_joint)
        if motor_name is None:
            log.warning("매핑 없는 조인트 건너뜀: %s", urdf_joint)
            continue
        commands.append(AngleCommand(motor_name, angle_deg, speed))
    return commands


def main():
    # ── 목표 위치·자세 설정 ──────────────────────────────────────
    target_pos = np.array([0.3, 0.0, 0.2])           # x, y, z (m)
    target_rot = IKSolver.euler_to_rotation(0, 0, 0)  # roll, pitch, yaw (deg)
    # ────────────────────────────────────────────────────────────

    # 1. IK 계산
    log.info("IK 계산 중...")
    solver = IKSolver()
    q_dict, pos_err, rot_err = solver.solve(target_pos, target_rot)

    if q_dict is None:
        log.error("IK 실패 — 위치오차: %.2f mm, 자세오차: %.3f°",
                  pos_err * 1000, np.degrees(rot_err))
        return

    log.info("IK 성공 — 위치오차: %.2f mm, 자세오차: %.3f°",
             pos_err * 1000, np.degrees(rot_err))
    for name, angle in q_dict.items():
        log.info("  %-20s: %8.3f °", name, angle)

    # 2. AngleCommand 변환
    commands = ik_to_commands(q_dict)
    if not commands:
        log.error("전송할 명령이 없습니다. JOINT_TO_MOTOR 매핑을 확인하세요.")
        return

    # 3. 모터 전송
    with MotorController() as mc:
        mc.move_motors(commands)
        log.info("모터 명령 전송 완료.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
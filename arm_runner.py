import time
import threading
import numpy as np
import can

from rob_motor_controller import RobStrideMITMotor
from trajectory import generate_trajectory, FREQ, DT
from ik_solver import IKSolver

# ── 조인트 → CAN ID 매핑 ──────────────────────────────────────────────────────
JOINT_CAN_MAP = {
    "left_joint1": 0x01,
    "left_joint2": 0x02,
    "left_joint3": 0x03,
    "left_joint4": 0x04,
    "left_joint5": 0x05,
    "left_joint6": 0x06,
    "left_joint7": 0x07,
}

DEFAULT_KP = 10.0
DEFAULT_KD = 0.5


class ArmRunner:
    """
    7DOF 로봇팔 MIT 제어 통합 클래스.

    흐름:
        move_to_pose(pos, rot, duration)
            └─ 모터 피드백으로 start_q 동기화
            └─ IKSolver.solve() → goal_q [rad]
            └─ run_trajectory() → quintic 궤적 생성
                └─ 200Hz 루프 → RobStrideMITMotor.control() × 7

    Args:
        channel: CAN 인터페이스 (예: 'can0', 'COM3')
        bustype: python-can 버스 타입 (예: 'socketcan', 'slcan')
        bitrate: 비트레이트 (기본 1Mbps)
        kp:      위치 게인 (공통 float 또는 조인트별 dict)
        kd:      속도 게인 (공통 float 또는 조인트별 dict)
    """

    def __init__(
        self,
        channel: str = 'can0',
        bustype: str = 'socketcan',
        bitrate: int = 1_000_000,
        kp: float | dict = DEFAULT_KP,
        kd: float | dict = DEFAULT_KD,
    ):
        self.bus = can.interface.Bus(channel=channel, interface=bustype, bitrate=bitrate)

        self.motors: dict[str, RobStrideMITMotor] = {
            name: RobStrideMITMotor(can_id=can_id, bus=self.bus)
            for name, can_id in JOINT_CAN_MAP.items()
        }

        self.kp: dict[str, float] = kp if isinstance(kp, dict) else {n: kp for n in JOINT_CAN_MAP}
        self.kd: dict[str, float] = kd if isinstance(kd, dict) else {n: kd for n in JOINT_CAN_MAP}

        self.ik = IKSolver()

        self._stop_event = threading.Event()
        self._rx_thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ── 수신 루프 ────────────────────────────────────────────────────────────

    def _rx_loop(self):
        """백그라운드에서 CAN 수신 → 각 모터 state 업데이트."""
        while not self._stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is not None:
                for motor in self.motors.values():
                    motor.parse(msg)

    # ── 활성화 / 비활성화 ────────────────────────────────────────────────────

    def enable_all(self):
        for motor in self.motors.values():
            motor.enable()
            time.sleep(0.002)

    def disable_all(self):
        for motor in self.motors.values():
            motor.disable()
            time.sleep(0.002)

    # ── Cartesian 목표 이동 ───────────────────────────────────────────────────

    def move_to_pose(
        self,
        pos:      np.ndarray,
        rot:      np.ndarray,
        duration: float,
        tau_ff:   float = 0.0,
    ) -> bool:
        """
        목표 위치/자세 → IK → quintic 궤적 → MIT 명령 전송.

        Args:
            pos:      목표 위치 [m], shape (3,)
            rot:      목표 자세 회전행렬 shape (3,3)  ← IKSolver.euler_to_rotation() 사용
            duration: 이동 시간 [초]
            tau_ff:   feedforward 토크 [Nm]

        Returns:
            True: 성공, False: IK 실패
        """
        # 모터 피드백으로 IK 초기값 동기화
        start_q_rad = np.array([self.motors[name].state.angle for name in JOINT_CAN_MAP])
        self.ik.q_current = start_q_rad.copy()

        goal_q_rad, pe, re = self.ik.solve(pos, rot)

        if goal_q_rad is None:
            print(f"[ArmRunner] IK 실패  pos_err={pe*1000:.1f}mm  rot_err={np.degrees(re):.2f}deg")
            return False

        print(f"[ArmRunner] IK 성공  pos_err={pe*1000:.1f}mm  rot_err={np.degrees(re):.2f}deg")
        self.run_trajectory(list(start_q_rad), list(goal_q_rad), duration, tau_ff)
        return True

    # ── 관절 공간 궤적 실행 ───────────────────────────────────────────────────

    def run_trajectory(
        self,
        start_rad: list[float],
        goal_rad:  list[float],
        duration:  float,
        tau_ff:    float = 0.0,
    ):
        """
        quintic 궤적 생성 후 200Hz 루프로 MIT 명령 전송.

        Args:
            start_rad: 시작 관절각 [rad] (7개)
            goal_rad:  목표 관절각 [rad] (7개)
            duration:  이동 시간 [초]
            tau_ff:    feedforward 토크 [Nm]
        """
        steps = generate_trajectory(start_rad, goal_rad, duration)
        print(f"[ArmRunner] 궤적 시작: {len(steps)} 스텝 @ {FREQ}Hz  duration={duration}s")

        for step in steps:
            t0 = time.perf_counter()

            for joint in step["joints"]:
                self.motors[joint["name"]].control(
                    angle  = joint["pos"],
                    speed  = joint["vel"],
                    kp     = self.kp[joint["name"]],
                    kd     = self.kd[joint["name"]],
                    torque = tau_ff,
                )

            sleep_t = DT - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

        print("[ArmRunner] 궤적 완료")

    # ── 정리 ─────────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop_event.set()
        self._rx_thread.join(timeout=1.0)
        self.disable_all()
        self.bus.shutdown()


# ── 실행 예시 ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    runner = ArmRunner(channel='COM3', bustype='slcan')

    try:
        runner.enable_all()
        time.sleep(0.1)

        pos = np.array([0.3, 0.0, 0.4])
        rot = IKSolver.euler_to_rotation(0, 0, 0)
        runner.move_to_pose(pos, rot, duration=3.0)

    except KeyboardInterrupt:
        print("\n중단")
    finally:
        runner.shutdown()

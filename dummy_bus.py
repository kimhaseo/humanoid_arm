"""
DummyBus - 하드웨어 없이 JointController 테스트용 가짜 CAN 버스

Extended Frame 프로토콜을 시뮬레이션한다:
- COMM_MOTOR_REQUEST 수신 시 → 현재 시뮬 상태로 즉시 응답
- COMM_MOTION_CTRL 수신 시   → 시뮬 각도를 명령값으로 즉시 반영
"""

import queue
import can
from robstride_motor_controller import (
    float_to_uint, uint_to_float,
    COMM_MOTOR_REQUEST, COMM_MOTION_CTRL,
    MASTER_CAN_ID, P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX,
)


class DummyBus:
    """
    가짜 CAN 버스.

    send() 가 COMM_MOTOR_REQUEST 프레임을 받으면 해당 모터의 시뮬 상태를
    Extended Frame 포맷으로 즉시 rx 큐에 넣어 fetch_state() 가 타임아웃
    없이 반환되도록 한다.

    COMM_MOTION_CTRL 프레임을 받으면 명령 각도를 시뮬 상태에 반영한다.
    """

    def __init__(self, verbose: bool = False):
        self.sent_messages: list[can.Message] = []
        self._rx_queue: queue.Queue = queue.Queue()
        self._motor_angles: dict[int, float] = {}   # motor_id → simulated angle [rad]
        self._verbose = verbose

    # ── can.interface.Bus 인터페이스 ──────────────────────────────────────────

    def send(self, msg: can.Message, timeout=None):
        self.sent_messages.append(msg)
        if self._verbose:
            print(f"[DummyBus] TX  id=0x{msg.arbitration_id:08X}  {bytes(msg.data).hex(' ')}")
        if msg.is_extended_id:
            self._dispatch(msg)

    def recv(self, timeout: float = 0.1) -> can.Message | None:
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        if self._verbose:
            print("[DummyBus] shutdown")

    # ── 프로토콜 디스패처 ─────────────────────────────────────────────────────

    def _dispatch(self, msg: can.Message):
        ext_id    = msg.arbitration_id
        comm_type = (ext_id >> 24) & 0x3F
        motor_id  = ext_id & 0xFF

        if comm_type == COMM_MOTOR_REQUEST:
            self._respond_state(motor_id)

        elif comm_type == COMM_MOTION_CTRL:
            # 명령 각도를 시뮬 상태에 즉시 반영
            data = bytes(msg.data)
            angle_enc = (data[0] << 8) | data[1]
            self._motor_angles[motor_id] = uint_to_float(angle_enc, P_MIN, P_MAX, 16)
            # 실제 모터처럼 제어 명령마다 상태 에코백 → motor.state 실시간 갱신
            self._respond_state(motor_id)

    def _respond_state(self, motor_id: int):
        """COMM_MOTOR_REQUEST 응답 프레임 생성 후 rx 큐에 삽입."""
        angle = self._motor_angles.get(motor_id, 0.0)

        angle_enc  = float_to_uint(angle, P_MIN, P_MAX, 16)
        speed_enc  = float_to_uint(0.0,   V_MIN, V_MAX, 16)
        torque_enc = float_to_uint(0.0,   T_MIN, T_MAX, 16)
        temp_enc   = int(25.0 / 0.1)   # 25.0°C

        data = bytearray(8)
        data[0] = (angle_enc  >> 8) & 0xFF
        data[1] =  angle_enc        & 0xFF
        data[2] = (speed_enc  >> 8) & 0xFF
        data[3] =  speed_enc        & 0xFF
        data[4] = (torque_enc >> 8) & 0xFF
        data[5] =  torque_enc       & 0xFF
        data[6] = (temp_enc   >> 8) & 0xFF
        data[7] =  temp_enc         & 0xFF

        # src_id = motor_id (파서가 (ext_id >> 8) & 0xFF 로 읽음)
        resp_id = (COMM_MOTOR_REQUEST << 24) | (motor_id << 8) | MASTER_CAN_ID
        self._rx_queue.put(can.Message(
            arbitration_id=resp_id,
            data=bytes(data),
            is_extended_id=True,
        ))

    # ── 테스트 헬퍼 ──────────────────────────────────────────────────────────

    def set_motor_angle(self, motor_id: int, angle: float):
        """시뮬 모터 각도를 직접 설정 (테스트용)."""
        self._motor_angles[motor_id] = angle


# ── 사용 예시 ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import time
    import can as _can
    from joint_controller import JointController

    _can.interface.Bus = lambda **kwargs: DummyBus(verbose=True)

    jc = JointController()

    try:
        jc.enable_all()

        print("\n=== move_joints_traj: 원점 → 목표 ===")
        jc.move_joints_traj(
            angles={'joint1': 0.5, 'joint2': 0.3, 'joint3': -0.4, 'joint4': 0.2},
            max_vel=0.5, max_acc=0.5,
        )
        jc.print_states()

        print("\n=== move_joint: joint1 단독 이동 ===")
        jc.move_joint('joint1', -0.3, max_vel=0.5, max_acc=1.0)
        jc.print_states()

    finally:
        jc.disable_all()
        jc.shutdown()

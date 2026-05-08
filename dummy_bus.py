"""
DummyBus - 하드웨어 없이 JointController 테스트용 가짜 CAN 버스

can.interface.Bus의 send / recv / shutdown 만 흉내낸다.
"""

import queue
import can
from robstride_motor_controller import (
    float_to_uint, P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX,
)


class DummyBus:
    """
    가짜 CAN 버스.

    - send()  : 보낸 메시지를 sent_messages 리스트에 기록
    - recv()  : inject_response()로 넣어둔 메시지를 반환, 없으면 None
    - shutdown(): 아무것도 안 함
    """

    def __init__(self):
        self.sent_messages: list[can.Message] = []
        self._rx_queue: queue.Queue = queue.Queue()

    def send(self, msg: can.Message, timeout=None):
        self.sent_messages.append(msg)
        print(f"[DummyBus] SEND  id=0x{msg.arbitration_id:03X}  data={bytes(msg.data).hex(' ')}")

    def recv(self, timeout: float = 0.1) -> can.Message | None:
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def inject_response(self, msg: can.Message):
        """recv()가 반환할 메시지를 직접 넣는다."""
        self._rx_queue.put(msg)

    def inject_motor_feedback(
        self,
        motor_id: int,
        angle:  float = 0.0,
        speed:  float = 0.0,
        torque: float = 0.0,
        temp:   float = 25.0,
    ):
        """
        MIT 피드백 프레임을 조립해서 큐에 넣는다.
        joint_controller.py의 _MITMotor.parse() 포맷과 동일.
        """
        angle_enc  = float_to_uint(angle,  P_MIN, P_MAX, 16)
        speed_enc  = float_to_uint(speed,  V_MIN, V_MAX, 12)
        torque_enc = float_to_uint(torque, T_MIN, T_MAX, 12)
        temp_enc   = int(temp / 0.1)

        data = bytearray(8)
        data[0] = 0x00
        data[1] = (angle_enc >> 8) & 0xFF
        data[2] =  angle_enc       & 0xFF
        data[3] = (speed_enc >> 4) & 0xFF
        data[4] = ((speed_enc & 0x0F) << 4) | ((torque_enc >> 8) & 0x0F)
        data[5] =  torque_enc      & 0xFF
        data[6] = (temp_enc >> 8)  & 0xFF
        data[7] =  temp_enc        & 0xFF

        msg = can.Message(
            arbitration_id=motor_id,
            data=bytes(data),
            is_extended_id=False,
        )
        self._rx_queue.put(msg)

    def shutdown(self):
        print("[DummyBus] shutdown")


# ── 사용 예시 ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import time
    import can as _can
    from joint_controller import JointController

    # can.interface.Bus 생성을 DummyBus로 교체
    _can.interface.Bus = lambda **kwargs: DummyBus()

    jc = JointController()

    # 테스트 1: enable_all → 7개 메시지 전송 확인
    print("=== enable_all ===")
    jc.enable_all()
    print(f"전송된 메시지 수: {len(jc.bus.sent_messages)}\n")  # 기대값: 7

    # 테스트 2: MIT 제어 명령
    print("=== set_joints ===")
    jc.bus.sent_messages.clear()
    jc.set_joints({f'joint{i}': 0.1 * i for i in range(1, 8)})
    print(f"전송된 메시지 수: {len(jc.bus.sent_messages)}\n")  # 기대값: 7

    # 테스트 3: 피드백 주입 후 상태 확인
    print("=== 피드백 시뮬레이션 ===")
    for i in range(1, 8):
        jc.bus.inject_motor_feedback(motor_id=i, angle=0.1*i, speed=0.0, torque=0.5, temp=30.0)
    time.sleep(0.3)  # RX 스레드 파싱 대기
    jc.print_states()

    jc.disable_all()
    jc.shutdown()

"""
LK-TECH 모터 기반 그리퍼 제어 (토크(Force) 폐루프 제어, CAN PROTOCOL V2.35 0xA1 명령 사용)

CanHandler(can_handler.py)로 CAN 프레임을 직접 주고받는 방식은 기존 motor_controller.py와 동일하다.

사용 예시:
    gripper = Gripper(motor_id=1, channel="COM5")
    gripper.enable()
    gripper.open()               # 위치 제어로 0도(open_angle_deg)까지 이동
    print(gripper.state.angle_deg)
    gripper.close(force_nm=2.0)  # 힘 제어로 파지
    gripper.shutdown()
"""

import time
from dataclasses import dataclass
from typing import Optional

from can_handler import CanHandler

# ── LK-TECH CAN PROTOCOL V2.35 명령 바이트 (단일 모터, ID = 0x140 + motor_id) ──
ID_BASE = 0x140

CMD_MOTOR_OFF         = 0x80
CMD_MOTOR_ON          = 0x88
CMD_MOTOR_STOP        = 0x81
CMD_WRITE_ZERO        = 0x19   # 현재 위치를 ROM에 모터 영점(원점)으로 기록 — 적용에는 재전원(재부팅) 필요
CMD_TORQUE_CONTROL    = 0xA1   # 토크(전류) 폐루프 제어 — MF/MH/MG 시리즈 전용
CMD_POS_CONTROL_2     = 0xA4   # 위치 폐루프 제어 2 — 멀티턴 절대각(int32, 0.01deg/LSB), 방향은 목표-현재 차이로 자동 결정
CMD_READ_SINGLE_ANGLE = 0x94   # 싱글턴 절대 각도 읽기 (0.01deg/LSB, 0~35999)

IQ_MIN, IQ_MAX = -2048, 2048    # iqControl 정수 범위 (프로토콜 규격)

# iqControl=±2048 에 대응하는 모터 시리즈별 실제 최대 전류(A) — 프로토콜 문서 표기값
MOTOR_SERIES_MAX_CURRENT_A = {
    "MF": 16.5,
    "MG": 33.0,
}


def _to_signed16(low: int, high: int) -> int:
    raw = low | (high << 8)
    return raw - 65536 if raw > 32767 else raw


@dataclass
class GripperState:
    angle_deg: float = 0.0       # 현재 싱글턴 각도 (deg)
    current_a: float = 0.0       # 실제 토크 전류 (A)
    torque_nm: float = 0.0       # 추정 토크 (Nm) = current_a * torque_constant
    speed_dps: float = 0.0       # 속도 (deg/s)
    enabled: bool = False


class Gripper:
    """
    LK-TECH 모터 1개로 구동하는 그리퍼. 토크 폐루프 제어(0xA1)로 정/역방향
    힘을 걸어 열고 닫으며, 명령 응답 프레임에서 현재 각도를 피드백 받는다.

    힘(토크) 제어이므로 위치가 아니라 "얼마나 세게 미는가"를 지정한다. 열림/닫힘
    방향으로 open_angle_deg / close_angle_deg(기계적 스토퍼 또는 목표 각도)에 도달할
    때까지, 혹은 물체를 잡아 각도가 더 이상 변하지 않을 때까지 지정한 힘을 유지한다.

    Args:
        motor_id: LK 모터 CAN ID (1~32)
        channel/interface/bitrate: CanHandler에 그대로 전달되는 CAN 연결 파라미터
        can_handler: 기존 CanHandler를 다른 모터와 공유하려면 전달 (없으면 새로 연다)
        motor_series: 'MF' 또는 'MG' — iqControl -> 전류(A) 환산 기준. max_current_a로 직접 덮어쓸 수 있음.
        max_current_a: iqControl=±2048에 대응하는 실제 최대 전류(A). None이면 motor_series 값 사용.
        torque_constant_nm_per_a: 모터 토크 상수 Kt (Nm/A). 프로토콜 문서에 없는 모터 고유값이므로
                                   반드시 데이터시트/실측값으로 보정할 것.
        open_angle_deg: 오픈 시 위치 제어(0xA4, 멀티턴)로 이동할 목표 각도(deg), 기본 0.0
        close_angle_deg: 완전 닫힘으로 볼 참고 각도(deg) — close()는 여전히 힘 제어라 직접 명령되지는 않음
        open_direction: 양의 힘(force_nm > 0)을 걸었을 때 그리퍼가 "열리는" 방향이면 +1, 반대면 -1 (close()에서 사용)
    """

    MAX_FORCE_NM = 2.5   # 하드 리밋 — open()/close()/set_force()에 전달된 force_nm은 이 값을 넘지 않는다

    def __init__(
        self,
        motor_id: int,
        channel: str = "COM5",
        interface: str = "slcan",
        bitrate: int = 1_000_000,
        can_handler: Optional[CanHandler] = None,
        motor_series: str = "MG",
        max_current_a: Optional[float] = None,
        torque_constant_nm_per_a: float = 0.7,  # MG4010E-i10v3: 모터축 0.07 N·m/A × 감속비 10
        open_angle_deg: float = 0.0,
        close_angle_deg: float = 90.0,
        open_direction: int = 1,
    ):
        if not (1 <= motor_id <= 32):
            raise ValueError("motor_id는 1~32 범위여야 합니다.")

        self.motor_id = motor_id
        self.arb_id   = ID_BASE + motor_id

        self._owns_can = can_handler is None
        self.can = can_handler or CanHandler(channel=channel, interface=interface, bitrate=bitrate)

        self.max_current_a   = max_current_a or MOTOR_SERIES_MAX_CURRENT_A[motor_series]
        self.torque_constant = torque_constant_nm_per_a
        self.open_angle_deg  = open_angle_deg
        self.close_angle_deg = close_angle_deg
        self.open_direction  = 1 if open_direction >= 0 else -1

        self.state = GripperState()

    # ── Nm <-> iqControl 변환 ────────────────────────────────────────────────

    def _nm_to_iq(self, force_nm: float) -> int:
        force_nm = max(-self.MAX_FORCE_NM, min(self.MAX_FORCE_NM, force_nm))
        current_a = force_nm / self.torque_constant
        iq = round(current_a / self.max_current_a * IQ_MAX)
        return max(IQ_MIN, min(IQ_MAX, iq))

    def _iq_to_a(self, iq: int) -> float:
        return iq / IQ_MAX * self.max_current_a

    # ── 응답 프레임 파싱 ─────────────────────────────────────────────────────

    def _handle_response(self, response):
        if response is None:
            return
        data = list(response.data)
        if len(data) < 8:
            return
        cmd = data[0]

        if cmd == CMD_TORQUE_CONTROL or cmd == CMD_POS_CONTROL_2:
            iq      = _to_signed16(data[2], data[3])
            speed   = _to_signed16(data[4], data[5])
            encoder = data[6] | (data[7] << 8)              # 14bit: 0~16383

            self.state.current_a = self._iq_to_a(iq)
            self.state.torque_nm = self.state.current_a * self.torque_constant
            self.state.speed_dps = float(speed)
            self.state.angle_deg = encoder / 16384.0 * 360.0

        elif cmd == CMD_READ_SINGLE_ANGLE:
            circle_angle = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)  # 0.01deg/LSB
            self.state.angle_deg = circle_angle / 100.0

    # ── 기본 제어 ─────────────────────────────────────────────────────────────

    def enable(self, timeout: float = 0.1):
        self.can.send_message(self.arb_id, [CMD_MOTOR_ON, 0, 0, 0, 0, 0, 0, 0])
        self.can.receive_message(timeout=timeout)  # ACK를 드레인해 다음 명령 응답과 엇갈리지 않게 함
        self.state.enabled = True

    def disable(self, timeout: float = 0.1):
        self.can.send_message(self.arb_id, [CMD_MOTOR_OFF, 0, 0, 0, 0, 0, 0, 0])
        self.can.receive_message(timeout=timeout)
        self.state.enabled = False

    def stop(self, timeout: float = 0.1):
        self.can.send_message(self.arb_id, [CMD_MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0])
        self.can.receive_message(timeout=timeout)

    def set_zero(self, timeout: float = 0.1) -> bool:
        """
        현재 위치를 모터 영점(원점)으로 ROM에 기록한다 (0x19).
        LK-TECH 프로토콜 규격상 새 영점은 모터 재전원(재부팅) 후에 적용된다.
        반환값: 응답 프레임이 0x19 ACK였는지 여부(그 전 명령의 지연 응답과 섞인 경우 False).
        """
        self.can.send_message(self.arb_id, [CMD_WRITE_ZERO, 0, 0, 0, 0, 0, 0, 0])
        response = self.can.receive_message(timeout=timeout)
        acked = response is not None and list(response.data)[0] == CMD_WRITE_ZERO
        if not acked:
            print(f"[set_zero] 0x19 ACK를 받지 못함 (응답: {response})")
        return acked

    def read_angle(self, timeout: float = 0.1) -> float:
        """싱글턴 절대 각도를 요청하고 state.angle_deg를 갱신 후 반환한다."""
        self.can.send_message(self.arb_id, [CMD_READ_SINGLE_ANGLE, 0, 0, 0, 0, 0, 0, 0])
        self._handle_response(self.can.receive_message(timeout=timeout))
        return self.state.angle_deg

    def set_force(self, force_nm: float, timeout: float = 0.1) -> float:
        """
        지정한 토크(Nm, 부호=방향)를 즉시 명령한다. 절대값은 MAX_FORCE_NM(2.5Nm)으로
        자동 클램프된다. 응답 프레임으로 현재 각도/전류/속도를 갱신 후 현재 각도를 반환한다.
        """
        iq = self._nm_to_iq(force_nm)
        print(iq)
        data = [
            CMD_TORQUE_CONTROL, 0, 0, 0,
            iq & 0xFF, (iq >> 8) & 0xFF,
            0, 0,
        ]
        self.can.send_message(self.arb_id, data)
        # self._handle_response(self.can.receive_message(timeout=timeout))
        # return self.state.angle_deg

    def move_to_angle(self, angle_deg: float, max_speed_dps: float = 100.0, timeout: float = 0.1) -> float:
        """
        멀티턴 절대 각도로 위치 폐루프 제어한다 (0xA4).
        angle_deg는 누적 다중회전 기준 목표 각도이며, 회전 방향은 목표-현재 위치 차이로
        모터가 자동 결정한다. set_zero()로 잡은 원점(0도) 기준으로 호출하면 된다.
        """
        angle_enc = round(angle_deg * 100) & 0xFFFFFFFF   # int32, 0.01deg/LSB (2의 보수)
        speed_enc = max(0, min(0xFFFF, int(max_speed_dps)))
        data = [
            CMD_POS_CONTROL_2, 0,
            speed_enc & 0xFF, (speed_enc >> 8) & 0xFF,
            angle_enc & 0xFF, (angle_enc >> 8) & 0xFF,
            (angle_enc >> 16) & 0xFF, (angle_enc >> 24) & 0xFF,
        ]
        self.can.send_message(self.arb_id, data)
        self._handle_response(self.can.receive_message(timeout=timeout))
        return self.state.angle_deg

    # ── 그리퍼 개폐 ──────────────────────────────────────────────────────────

    def open(self, speed_dps: float = 100.0, timeout: float = 3.0):
        """위치 제어(0xA4)로 open_angle_deg(기본 0도)까지 이동하고 timeout만큼 대기한다."""
        self.move_to_angle(self.open_angle_deg, max_speed_dps=speed_dps)
        time.sleep(timeout)

    def close(self, force_nm: float = 1.0, timeout: float = 3.0):
        """open_direction 반대 방향으로 force_nm 크기 힘을 한 번 명령하고 timeout만큼 대기한다."""
        self._drive(-self.open_direction * abs(force_nm), timeout)

    def _drive(self, signed_force: float, timeout: float):
        self.set_force(signed_force)
        time.sleep(timeout)

    # ── 정리 ──────────────────────────────────────────────────────────────────

    def shutdown(self):
        if self._owns_can:
            self.can.close()


# ── 사용 예시 ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    gripper = Gripper(
        motor_id=7,
        channel="COM3",
        interface="slcan",
        open_direction=1,
    )
    # gripper.enable()
    # time.sleep(1)
    # gripper.set_zero()

    try:
        gripper.enable()
        print("[열기] 위치 제어 0도")
        gripper.open(speed_dps=500, timeout=5)
        print(gripper.read_angle())

        gripper.close(force_nm=0.75, timeout=5)
        print(gripper.read_angle())


    except KeyboardInterrupt:
        print("사용자 중단")
    finally:
        gripper.stop()
        gripper.disable()
        gripper.shutdown()

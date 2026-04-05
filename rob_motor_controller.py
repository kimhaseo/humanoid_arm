"""
RobStride RS01 Motor CAN 제어 Python 라이브러리
python-can 라이브러리 기반

설치:
    pip install python-can

사용 예시:
    motor = RobStrideMotor(can_id=0x7F, channel='can0', bustype='socketcan')
    motor.enable()
    motor.move_control(torque=0.0, angle=1.57, speed=5.0, kp=10.0, kd=0.5)
    motor.disable()
    motor.shutdown()
"""

import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable
import can

# ── 프로토콜 상수 ──────────────────────────────────────────────────────────────
P_MIN, P_MAX = -12.5, 12.5      # 각도 범위 (rad)
V_MIN, V_MAX = -44.0, 44.0      # 속도 범위 (rad/s)
KP_MIN, KP_MAX = 0.0, 500.0     # Kp 범위
KD_MIN, KD_MAX = 0.0, 5.0       # Kd 범위
T_MIN, T_MAX = -17.0, 17.0      # 토크 범위 (Nm)

MASTER_CAN_ID = 0xFD            # 마스터 CAN ID (기본값)

# 통신 타입 (Standard Protocol - Extended Frame)
COMM_GET_ID           = 0x00
COMM_MOTION_CTRL      = 0x01
COMM_MOTOR_REQUEST    = 0x02
COMM_MOTOR_ENABLE     = 0x03
COMM_MOTOR_STOP       = 0x04
COMM_SET_POS_ZERO     = 0x06
COMM_SET_CAN_ID       = 0x07
COMM_GET_PARAM        = 0x11
COMM_SET_PARAM        = 0x12
COMM_ERROR_FEEDBACK   = 0x15
COMM_DATA_SAVE        = 0x16
COMM_BAUD_CHANGE      = 0x17
COMM_PROACTIVE_SET    = 0x18
COMM_MOTOR_MODE_SET   = 0x19

# 운전 모드
MODE_MOVE    = 0    # 토크+속도+각도 복합 제어
MODE_POS_PP  = 1    # PP 위치 제어
MODE_SPEED   = 2    # 속도 제어
MODE_CURRENT = 3    # 전류 제어
MODE_ZERO    = 4    # 영점 설정
MODE_POS_CSP = 5    # CSP 위치 제어

# 파라미터 인덱스
IDX_RUN_MODE    = 0x7005
IDX_IQ_REF      = 0x7006
IDX_SPD_REF     = 0x700A
IDX_LMT_TORQUE  = 0x700B
IDX_CUR_KP      = 0x7010
IDX_CUR_KI      = 0x7011
IDX_CUR_FILT    = 0x7014
IDX_LOC_REF     = 0x7016
IDX_LIMIT_SPD   = 0x7017
IDX_LIMIT_CUR   = 0x7018
IDX_MECH_POS    = 0x7019   # 읽기 전용
IDX_IQF         = 0x701A   # 읽기 전용
IDX_MECH_VEL    = 0x701B   # 읽기 전용
IDX_VBUS        = 0x701C   # 읽기 전용
IDX_ROTATION    = 0x701D   # 읽기 전용
IDX_PP_LIMIT_SPD = 0x7024  # PP 모드 속도 제한
IDX_PP_ACC      = 0x7025   # PP 모드 가속도
IDX_SPD_ACC     = 0x7022   # 속도 모드 가속도


# ── 데이터 변환 함수 ───────────────────────────────────────────────────────────

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """float → unsigned int (bits 비트로 양자화)"""
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)

def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """unsigned int → float (역양자화)"""
    span = (1 << bits) - 1
    x &= span
    return (x_max - x_min) * x / span + x_min

def pack_float(value: float) -> bytes:
    """float → 4바이트 little-endian"""
    return struct.pack('<f', value)

def unpack_float(data: bytes, offset: int = 0) -> float:
    """4바이트 little-endian → float"""
    return struct.unpack_from('<f', data, offset)[0]


# ── 모터 상태 데이터클래스 ────────────────────────────────────────────────────

@dataclass
class MotorState:
    angle:   float = 0.0    # 각도 (rad)
    speed:   float = 0.0    # 속도 (rad/s)
    torque:  float = 0.0    # 토크 (Nm)
    temp:    float = 0.0    # 온도 (°C)
    pattern: int   = 0      # 운전 패턴 (0=정지, 1=대기, 2=운전 중)
    error:   int   = 0      # 에러 코드
    enabled: bool  = False  # 활성화 여부


# ── 메인 클래스 ───────────────────────────────────────────────────────────────

class RobStrideMotor:
    """
    RobStride RS01 모터 CAN 제어 클래스 (Standard Protocol)

    Args:
        can_id:    모터 CAN ID (기본값: 0x7F)
        channel:   CAN 인터페이스 (예: 'can0', 'PCAN_USBBUS1', 'COM3')
        bustype:   python-can 버스 타입 (예: 'socketcan', 'pcan', 'slcan', 'kvaser')
        bitrate:   비트레이트 (기본값: 1_000_000 = 1Mbps)
        master_id: 마스터 CAN ID (기본값: 0xFD)
        callback:  수신 콜백 함수 (state: MotorState) → 상태 업데이트 시 호출
    """

    def __init__(
        self,
        can_id: int = 0x7F,
        channel: str = 'can0',
        bustype: str = 'socketcan',
        bitrate: int = 1_000_000,
        master_id: int = MASTER_CAN_ID,
        callback: Optional[Callable[['MotorState'], None]] = None,
    ):
        self.can_id    = can_id
        self.master_id = master_id
        self.state     = MotorState()
        self._callback = callback
        self._run_mode_cache = -1   # 현재 설정된 운전 모드 캐시

        self.bus = can.interface.Bus(channel=channel, interface=bustype, bitrate=bitrate)

        # 수신 스레드 시작
        self._stop_event = threading.Event()
        self._rx_thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ── 내부 유틸 ─────────────────────────────────────────────────────────────

    def _ext_id(self, comm_type: int, data_field: int = 0) -> int:
        """Extended Frame ID 생성: comm_type[31:24] | data[23:8] | can_id[7:0]"""
        return (comm_type << 24) | (data_field << 8) | self.can_id

    def _send_ext(self, comm_type: int, data: bytes, data_field: int = 0):
        """Extended Frame 전송"""
        ext_id = (comm_type << 24) | (data_field << 8) | self.can_id
        msg = can.Message(
            arbitration_id=ext_id,
            data=list(data),
            is_extended_id=True,
        )
        self.bus.send(msg)

    def _send_std(self, std_id: int, data: bytes):
        """Standard Frame 전송 (MIT 모드용)"""
        msg = can.Message(
            arbitration_id=std_id,
            data=list(data),
            is_extended_id=False,
        )
        self.bus.send(msg)

    def _rx_loop(self):
        """수신 루프 (백그라운드 스레드)"""
        while not self._stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is not None:
                self._parse_message(msg)

    def _parse_message(self, msg: can.Message):
        """수신 메시지 파싱 (Standard Protocol)"""
        if not msg.is_extended_id:
            return
        ext_id = msg.arbitration_id
        data   = bytes(msg.data)

        comm_type = (ext_id >> 24) & 0x3F
        src_id    = (ext_id >> 8)  & 0xFF

        # 모터 ID 필터
        if src_id != self.can_id:
            return

        if comm_type == COMM_MOTOR_REQUEST:
            # 위치/속도/토크/온도 응답
            self.state.angle   = uint_to_float((data[0] << 8) | data[1], P_MIN, P_MAX, 16)
            self.state.speed   = uint_to_float((data[2] << 8) | data[3], V_MIN, V_MAX, 16)
            self.state.torque  = uint_to_float((data[4] << 8) | data[5], T_MIN, T_MAX, 16)
            self.state.temp    = ((data[6] << 8) | data[7]) * 0.1
            self.state.error   = (ext_id >> 16) & 0x3F
            self.state.pattern = (ext_id >> 22) & 0x3
            if self._callback:
                self._callback(self.state)

        elif comm_type == COMM_GET_PARAM:
            # 파라미터 읽기 응답
            index = (data[1] << 8) | data[0]
            if index == IDX_RUN_MODE:
                self._run_mode_cache = data[4]

    # ── 기본 제어 ─────────────────────────────────────────────────────────────

    def enable(self):
        """모터 활성화 (Standard Protocol)"""
        data = bytes(8)
        self._send_ext(COMM_MOTOR_ENABLE, data, self.master_id)
        self.state.enabled = True

    def disable(self, clear_error: bool = False):
        """모터 비활성화"""
        data = bytearray(8)
        data[0] = 1 if clear_error else 0
        self._send_ext(COMM_MOTOR_STOP, bytes(data), self.master_id)
        self.state.enabled = False

    def get_device_id(self):
        """장치 ID 요청 (통신 타입 0x00)"""
        self._send_ext(COMM_GET_ID, bytes(8), self.master_id)

    def request_state(self):
        """현재 상태 요청 (통신 타입 0x02)"""
        self._send_ext(COMM_MOTOR_REQUEST, bytes(8), self.master_id)

    def set_zero(self):
        """현재 위치를 영점(0)으로 설정"""
        self.disable()
        time.sleep(0.01)
        data = bytearray(8)
        data[0] = 1
        self._send_ext(COMM_SET_POS_ZERO, bytes(data), self.master_id)
        time.sleep(0.01)
        self.enable()

    def save_parameters(self):
        """파라미터 영구 저장 (통신 타입 0x16)"""
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        self._send_ext(COMM_DATA_SAVE, data, self.master_id)

    # ── 파라미터 읽기/쓰기 ────────────────────────────────────────────────────

    def set_param_float(self, index: int, value: float):
        """float 파라미터 쓰기 (통신 타입 0x12)"""
        data = bytearray(8)
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        data[2] = 0x00
        data[3] = 0x00
        struct.pack_into('<f', data, 4, value)
        self._send_ext(COMM_SET_PARAM, bytes(data), self.master_id)

    def set_param_mode(self, index: int, value: int):
        """모드(uint8) 파라미터 쓰기"""
        data = bytearray(8)
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        data[2] = 0x00
        data[3] = 0x00
        data[4] = value & 0xFF
        self._send_ext(COMM_SET_PARAM, bytes(data), self.master_id)

    def get_param(self, index: int):
        """파라미터 읽기 요청 (통신 타입 0x11)"""
        data = bytearray(8)
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        self._send_ext(COMM_GET_PARAM, bytes(data), self.master_id)

    def _ensure_mode(self, target_mode: int):
        """운전 모드 전환 (필요 시에만)"""
        if self._run_mode_cache != target_mode:
            self.set_param_mode(IDX_RUN_MODE, target_mode)
            self.get_param(IDX_RUN_MODE)
            self.enable()
            self._run_mode_cache = target_mode
            time.sleep(0.002)

    # ── 운전 모드별 제어 ──────────────────────────────────────────────────────

    def move_control(
        self,
        torque: float = 0.0,
        angle: float  = 0.0,
        speed: float  = 0.0,
        kp: float     = 10.0,
        kd: float     = 0.5,
    ):
        """
        복합 제어 (Move Control, Mode 0) — 통신 타입 0x01
        ExtId 상위 16비트에 토크값 인코딩, 데이터에 각도/속도/Kp/Kd 인코딩

        Args:
            torque: 목표 토크 (-17 ~ 17 Nm)
            angle:  목표 각도 (-12.5 ~ 12.5 rad)
            speed:  목표 속도 (-44 ~ 44 rad/s)
            kp:     위치 비례 게인 (0 ~ 500)
            kd:     위치 미분 게인 (0 ~ 5)
        """
        self._ensure_mode(MODE_MOVE)
        if not self.state.enabled:
            self.enable()

        torque_enc = float_to_uint(torque, T_MIN, T_MAX, 16)
        ext_id = (COMM_MOTION_CTRL << 24) | (torque_enc << 8) | self.can_id

        data = bytearray(8)
        angle_enc = float_to_uint(angle, P_MIN, P_MAX, 16)
        speed_enc = float_to_uint(speed, V_MIN, V_MAX, 16)
        kp_enc    = float_to_uint(kp,    KP_MIN, KP_MAX, 16)
        kd_enc    = float_to_uint(kd,    KD_MIN, KD_MAX, 16)

        data[0] = (angle_enc >> 8) & 0xFF
        data[1] = angle_enc & 0xFF
        data[2] = (speed_enc >> 8) & 0xFF
        data[3] = speed_enc & 0xFF
        data[4] = (kp_enc >> 8) & 0xFF
        data[5] = kp_enc & 0xFF
        data[6] = (kd_enc >> 8) & 0xFF
        data[7] = kd_enc & 0xFF

        msg = can.Message(arbitration_id=ext_id, data=list(data), is_extended_id=True)
        self.bus.send(msg)

    def position_control(
        self,
        angle_rad: float,
        speed_rad_s: float = 5.0,
        acceleration: float = 10.0,
    ):
        """
        PP 위치 제어 (Mode 1) — 파라미터 쓰기 방식

        Args:
            angle_rad:    목표 각도 (rad)
            speed_rad_s:  이동 속도 제한 (rad/s, 0 ~ 30)
            acceleration: 가속도 제한
        """
        self._ensure_mode(MODE_POS_PP)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_PP_LIMIT_SPD, speed_rad_s)
        self.set_param_float(IDX_PP_ACC, acceleration)
        time.sleep(0.001)
        self.set_param_float(IDX_LOC_REF, angle_rad)

    def csp_position_control(self, angle_rad: float, limit_speed: float = 5.0):
        """
        CSP 위치 제어 (Mode 5) — 연속 위치 명령에 적합

        Args:
            angle_rad:    목표 각도 (rad)
            limit_speed:  속도 제한 (rad/s, 0 ~ 44)
        """
        self._ensure_mode(MODE_POS_CSP)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_LIMIT_SPD, limit_speed)
        time.sleep(0.001)
        self.set_param_float(IDX_LOC_REF, angle_rad)

    def speed_control(self, speed_rad_s: float, limit_current: float = 5.0):
        """
        속도 제어 (Mode 2)

        Args:
            speed_rad_s:   목표 속도 (-30 ~ 30 rad/s)
            limit_current: 전류 제한 (0 ~ 23 A)
        """
        self._ensure_mode(MODE_SPEED)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_LIMIT_CUR, limit_current)
        self.set_param_float(IDX_SPD_ACC, 10.0)
        self.set_param_float(IDX_SPD_REF, speed_rad_s)

    def current_control(self, current_a: float):
        """
        전류 제어 (Mode 3)

        Args:
            current_a: 목표 전류 (-23 ~ 23 A)
        """
        self._ensure_mode(MODE_CURRENT)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_IQ_REF, current_a)

    # ── CAN ID / 보레이트 변경 ────────────────────────────────────────────────

    def change_can_id(self, new_id: int):
        """CAN ID 변경 (주의: 변경 후 재연결 필요)"""
        self.disable()
        time.sleep(0.01)
        ext_id = (COMM_SET_CAN_ID << 24) | (new_id << 16) | (self.master_id << 8) | self.can_id
        msg = can.Message(arbitration_id=ext_id, data=[0]*8, is_extended_id=True)
        self.bus.send(msg)

    def change_baudrate(self, rate_code: int):
        """
        보레이트 변경 (재전원 후 적용)

        Args:
            rate_code: 0x01=1Mbps, 0x02=500K, 0x03=250K, 0x04=125K
        """
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, rate_code, 0x08])
        self._send_ext(COMM_BAUD_CHANGE, data, self.master_id)

    # ── 정리 ──────────────────────────────────────────────────────────────────

    def shutdown(self):
        """CAN 버스 종료"""
        self._stop_event.set()
        self._rx_thread.join(timeout=1.0)
        self.bus.shutdown()


# ── MIT 모드 클래스 (보조) ────────────────────────────────────────────────────

class RobStrideMITMotor:
    """
    RobStride RS01 MIT Protocol 제어 클래스
    Standard CAN Frame 사용

    Args:
        can_id:   모터 CAN ID
        bus:      공유 CAN bus 인스턴스 (can.interface.Bus)
    """

    def __init__(
        self,
        can_id: int,
        bus: can.interface.Bus,
    ):
        self.can_id = can_id
        self.state  = MotorState()
        self.bus    = bus

    def _send(self, std_id: int, data: bytes):
        msg = can.Message(arbitration_id=std_id, data=list(data), is_extended_id=False)
        self.bus.send(msg)

    def parse(self, msg: can.Message):
        """수신 메시지에서 이 모터의 상태를 파싱. ArmRunner의 rx 루프에서 호출."""
        if msg.is_extended_id:
            return
        data = bytes(msg.data)
        # 응답 ID의 하위 바이트가 이 모터의 can_id와 일치하는지 확인
        src = msg.arbitration_id & 0xFF
        if src != self.can_id:
            return
        # 일반 응답 (각도/속도/토크/온도)
        if not (data[3] == 0 and data[4] == 0 and data[5] == 0 and data[6] == 0 and data[7] == 0):
            self.state.angle  = uint_to_float((data[1] << 8) | data[2], P_MIN, P_MAX, 16)
            self.state.speed  = uint_to_float((data[3] << 4) | (data[4] >> 4), V_MIN, V_MAX, 12)
            self.state.torque = uint_to_float(((data[4] & 0x0F) << 8) | data[5], T_MIN, T_MAX, 12)
            self.state.temp   = ((data[6] << 8) | data[7]) * 0.1

    def enable(self):
        """MIT 활성화"""
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]))
        self.state.enabled = True

    def disable(self):
        """MIT 비활성화"""
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]))
        self.state.enabled = False

    def set_zero(self):
        """영점 설정"""
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]))

    def clear_error(self, f_cmd: int = 0x00):
        """에러 클리어"""
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, f_cmd, 0xFB])
        self._send(self.can_id, data)

    def control(
        self,
        angle: float  = 0.0,
        speed: float  = 0.0,
        kp: float     = 10.0,
        kd: float     = 0.5,
        torque: float = 0.0,
    ):
        """
        MIT 복합 제어 (각도+속도+Kp+Kd+토크)

        Args:
            angle:  목표 각도 (-12.5 ~ 12.5 rad)
            speed:  목표 속도 (-44 ~ 44 rad/s)
            kp:     위치 Kp (0 ~ 500)
            kd:     위치 Kd (0 ~ 5)
            torque: 피드포워드 토크 (-17 ~ 17 Nm)
        """
        angle_enc  = float_to_uint(angle,  P_MIN,  P_MAX,  16)
        speed_enc  = float_to_uint(speed,  V_MIN,  V_MAX,  12)
        kp_enc     = float_to_uint(kp,     KP_MIN, KP_MAX, 12)
        kd_enc     = float_to_uint(kd,     KD_MIN, KD_MAX, 12)
        torque_enc = float_to_uint(torque, T_MIN,  T_MAX,  12)

        data = bytearray(8)
        data[0] = (angle_enc >> 8) & 0xFF
        data[1] = angle_enc & 0xFF
        data[2] = (speed_enc >> 4) & 0xFF
        data[3] = ((speed_enc & 0x0F) << 4) | ((kp_enc >> 8) & 0x0F)
        data[4] = kp_enc & 0xFF
        data[5] = (kd_enc >> 4) & 0xFF
        data[6] = ((kd_enc & 0x0F) << 4) | ((torque_enc >> 8) & 0x0F)
        data[7] = torque_enc & 0xFF

        self._send(self.can_id, bytes(data))

    def position_control(self, position_rad: float, speed_rad_s: float = 5.0):
        """MIT 위치 제어 (StdId = (1<<8) | can_id)"""
        std_id = (1 << 8) | self.can_id
        data = struct.pack('<ff', position_rad, speed_rad_s)
        self._send(std_id, data)

    def speed_control(self, speed_rad_s: float, current_limit: float = 5.0):
        """MIT 속도 제어 (StdId = (2<<8) | can_id)"""
        std_id = (2 << 8) | self.can_id
        data = struct.pack('<ff', speed_rad_s, current_limit)
        self._send(std_id, data)

    def shutdown(self):
        """모터 비활성화. bus 종료는 ArmRunner에서 담당."""
        self.disable()


# ── 사용 예시 ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    # ── 설정 ──────────────────────────────────────────────────────────────────
    # Windows PCAN-USB 예시:
    #   channel='PCAN_USBBUS1', bustype='pcan'
    # Linux SocketCAN 예시:
    #   channel='can0', bustype='socketcan'
    # SLCAN (USB-CAN 동글) 예시:
    #   channel='COM3', bustype='slcan'  (Windows)
    #   channel='/dev/ttyUSB0', bustype='slcan'  (Linux)

    CHANNEL = 'COM3'
    BUSTYPE = 'slcan'
    MOTOR_ID = 0x7F

    def on_state_update(state: MotorState):
        print(f'  각도: {state.angle:7.3f} rad | 속도: {state.speed:7.3f} rad/s | '
              f'토크: {state.torque:6.3f} Nm | 온도: {state.temp:.1f}°C | '
              f'패턴: {state.pattern} | 에러: {state.error}')

    print('=== RobStride RS01 모터 제어 예시 ===')
    print(f'CAN 채널: {CHANNEL} ({BUSTYPE}), 모터 ID: 0x{MOTOR_ID:02X}')

    motor = RobStrideMotor(
        can_id=MOTOR_ID,
        channel=CHANNEL,
        bustype=BUSTYPE,
        bitrate=1_000_000,
        callback=on_state_update,
    )

    try:
        # 1. 모터 활성화
        print('\n[1] 모터 활성화...')
        motor.enable()
        time.sleep(0.1)

        # 2. 복합 제어 (각도 이동)
        print('\n[2] 복합 제어: 각도 1.0 rad, 속도 5 rad/s, Kp=10, Kd=0.5')
        motor.move_control(torque=0.0, angle=2.0, speed=10.0, kp=2.0, kd=1.0)
        time.sleep(3.0)
        #
        # 3. CSP 위치 제어
        # print('\n[3] CSP 위치 제어: 0.0 rad')
        # motor.position_control(angle_rad=-2.0, speed_rad_s=1.0)
        # time.sleep(2.0)

        # # 4. 속도 제어
        # print('\n[4] 속도 제어: 3 rad/s, 전류 제한 5A')
        # motor.speed_control(speed_rad_s=3.0, limit_current=5.0)
        # time.sleep(2.0)
        #
        # # 5. 정지
        # print('\n[5] 속도 0으로 감속 후 정지')
        # motor.speed_control(speed_rad_s=0.0, limit_current=5.0)
        # time.sleep(1.0)
        #
        # # 6. 현재 상태 요청
        print('\n[6] 상태 요청:')
        motor.request_state()
        time.sleep(0.1)

    except KeyboardInterrupt:
        print('\n\n사용자 중단')
    finally:
        print('\n모터 비활성화 및 CAN 종료')
        motor.disable(clear_error=True)
        time.sleep(0.05)
        motor.shutdown()

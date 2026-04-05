import struct
import time
import threading
from dataclasses import dataclass
from typing import Optional, Callable
import can

# ── 프로토콜 상수 ──────────────────────────────────────────────────────────────
P_MIN,  P_MAX  = -12.5, 12.5   # 각도 범위 (rad)
V_MIN,  V_MAX  = -44.0, 44.0   # 속도 범위 (rad/s)
KP_MIN, KP_MAX =   0.0, 500.0  # Kp 범위
KD_MIN, KD_MAX =   0.0,   5.0  # Kd 범위
T_MIN,  T_MAX  = -17.0,  17.0  # 토크 범위 (Nm)

MASTER_CAN_ID = 0xFD

# 통신 타입 (Standard Protocol - Extended Frame)
COMM_GET_ID          = 0x00
COMM_MOTION_CTRL     = 0x01
COMM_MOTOR_REQUEST   = 0x02
COMM_MOTOR_ENABLE    = 0x03
COMM_MOTOR_STOP      = 0x04
COMM_SET_POS_ZERO    = 0x06
COMM_SET_CAN_ID      = 0x07
COMM_GET_PARAM       = 0x11
COMM_SET_PARAM       = 0x12
COMM_ERROR_FEEDBACK  = 0x15
COMM_DATA_SAVE       = 0x16
COMM_BAUD_CHANGE     = 0x17
COMM_PROACTIVE_SET   = 0x18
COMM_MOTOR_MODE_SET  = 0x19

# 운전 모드
MODE_MOVE    = 0  # 토크+속도+각도 복합 제어
MODE_POS_PP  = 1  # PP 위치 제어
MODE_SPEED   = 2  # 속도 제어
MODE_CURRENT = 3  # 전류 제어
MODE_ZERO    = 4  # 영점 설정
MODE_POS_CSP = 5  # CSP 위치 제어

# 파라미터 인덱스
IDX_RUN_MODE     = 0x7005
IDX_IQ_REF       = 0x7006
IDX_SPD_REF      = 0x700A
IDX_LMT_TORQUE   = 0x700B
IDX_CUR_KP       = 0x7010
IDX_CUR_KI       = 0x7011
IDX_CUR_FILT     = 0x7014
IDX_LOC_REF      = 0x7016
IDX_LIMIT_SPD    = 0x7017
IDX_LIMIT_CUR    = 0x7018
IDX_MECH_POS     = 0x7019  # 읽기 전용
IDX_IQF          = 0x701A  # 읽기 전용
IDX_MECH_VEL     = 0x701B  # 읽기 전용
IDX_VBUS         = 0x701C  # 읽기 전용
IDX_ROTATION     = 0x701D  # 읽기 전용
IDX_PP_LIMIT_SPD = 0x7024  # PP 모드 속도 제한
IDX_PP_ACC       = 0x7025  # PP 모드 가속도
IDX_SPD_ACC      = 0x7022  # 속도 모드 가속도


# ── 데이터 변환 ────────────────────────────────────────────────────────────────

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """float → unsigned int (bits 비트로 양자화)"""
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """unsigned int → float (역양자화)"""
    return (x_max - x_min) * (x & ((1 << bits) - 1)) / ((1 << bits) - 1) + x_min

def pack_float(value: float) -> bytes:
    """float → 4바이트 little-endian"""
    return struct.pack('<f', value)

def unpack_float(data: bytes, offset: int = 0) -> float:
    """4바이트 little-endian → float"""
    return struct.unpack_from('<f', data, offset)[0]


# ── 모터 상태 ─────────────────────────────────────────────────────────────────

@dataclass
class MotorState:
    angle:   float = 0.0   # 현재 각도 (rad)
    speed:   float = 0.0   # 현재 속도 (rad/s)
    torque:  float = 0.0   # 현재 토크 (Nm)
    temp:    float = 0.0   # 온도 (°C)
    pattern: int   = 0     # 운전 패턴 (0=정지, 1=대기, 2=운전 중)
    error:   int   = 0     # 에러 코드
    enabled: bool  = False


# ── Standard Protocol 클래스 (모터 설정용) ────────────────────────────────────

class RobStrideMotor:
    """
    RobStride RS01 Standard Protocol 제어 클래스.
    모터 초기 설정 (CAN ID 변경, 보레이트, 파라미터 저장, 영점 등)에 사용.

    Args:
        can_id:    모터 CAN ID
        channel:   CAN 인터페이스 (예: 'can0', 'COM3')
        bustype:   python-can 버스 타입 (예: 'socketcan', 'slcan')
        bitrate:   비트레이트
        master_id: 마스터 CAN ID
        callback:  상태 업데이트 콜백 (state: MotorState)
    """

    def __init__(
        self,
        can_id:    int = 0x7F,
        channel:   str = 'can0',
        bustype:   str = 'socketcan',
        bitrate:   int = 1_000_000,
        master_id: int = MASTER_CAN_ID,
        callback:  Optional[Callable[['MotorState'], None]] = None,
    ):
        self.can_id    = can_id
        self.master_id = master_id
        self.state     = MotorState()
        self._callback = callback
        self._run_mode_cache = -1

        self.bus = can.interface.Bus(channel=channel, interface=bustype, bitrate=bitrate)

        self._stop_event = threading.Event()
        self._rx_thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ── 내부 ──────────────────────────────────────────────────────────────────

    def _send_ext(self, comm_type: int, data: bytes, data_field: int = 0):
        ext_id = (comm_type << 24) | (data_field << 8) | self.can_id
        msg = can.Message(arbitration_id=ext_id, data=list(data), is_extended_id=True)
        self.bus.send(msg)

    def _rx_loop(self):
        while not self._stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is not None:
                self._parse_message(msg)

    def _parse_message(self, msg: can.Message):
        if not msg.is_extended_id:
            return
        ext_id = msg.arbitration_id
        data   = bytes(msg.data)

        comm_type = (ext_id >> 24) & 0x3F
        src_id    = (ext_id >> 8)  & 0xFF
        if src_id != self.can_id:
            return

        if comm_type == COMM_MOTOR_REQUEST:
            self.state.angle   = uint_to_float((data[0] << 8) | data[1], P_MIN, P_MAX, 16)
            self.state.speed   = uint_to_float((data[2] << 8) | data[3], V_MIN, V_MAX, 16)
            self.state.torque  = uint_to_float((data[4] << 8) | data[5], T_MIN, T_MAX, 16)
            self.state.temp    = ((data[6] << 8) | data[7]) * 0.1
            self.state.error   = (ext_id >> 16) & 0x3F
            self.state.pattern = (ext_id >> 22) & 0x3
            if self._callback:
                self._callback(self.state)

        elif comm_type == COMM_GET_PARAM:
            index = (data[1] << 8) | data[0]
            if index == IDX_RUN_MODE:
                self._run_mode_cache = data[4]

    def _ensure_mode(self, target_mode: int):
        if self._run_mode_cache != target_mode:
            self.set_param_mode(IDX_RUN_MODE, target_mode)
            self.get_param(IDX_RUN_MODE)
            self.enable()
            self._run_mode_cache = target_mode
            time.sleep(0.002)

    # ── 기본 제어 ─────────────────────────────────────────────────────────────

    def enable(self):
        self._send_ext(COMM_MOTOR_ENABLE, bytes(8), self.master_id)
        self.state.enabled = True

    def disable(self, clear_error: bool = False):
        data = bytearray(8)
        data[0] = 1 if clear_error else 0
        self._send_ext(COMM_MOTOR_STOP, bytes(data), self.master_id)
        self.state.enabled = False

    def get_device_id(self):
        self._send_ext(COMM_GET_ID, bytes(8), self.master_id)

    def request_state(self):
        self._send_ext(COMM_MOTOR_REQUEST, bytes(8), self.master_id)

    def set_zero(self):
        self.disable()
        time.sleep(0.01)
        data = bytearray(8)
        data[0] = 1
        self._send_ext(COMM_SET_POS_ZERO, bytes(data), self.master_id)
        time.sleep(0.01)
        self.enable()

    def save_parameters(self):
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        self._send_ext(COMM_DATA_SAVE, data, self.master_id)

    # ── 파라미터 읽기/쓰기 ────────────────────────────────────────────────────

    def set_param_float(self, index: int, value: float):
        data = bytearray(8)
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        struct.pack_into('<f', data, 4, value)
        self._send_ext(COMM_SET_PARAM, bytes(data), self.master_id)

    def set_param_mode(self, index: int, value: int):
        data = bytearray(8)
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        data[4] = value & 0xFF
        self._send_ext(COMM_SET_PARAM, bytes(data), self.master_id)

    def get_param(self, index: int):
        data = bytearray(8)
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        self._send_ext(COMM_GET_PARAM, bytes(data), self.master_id)

    # ── 운전 모드별 제어 ──────────────────────────────────────────────────────

    def move_control(self, torque=0.0, angle=0.0, speed=0.0, kp=10.0, kd=0.5):
        self._ensure_mode(MODE_MOVE)
        if not self.state.enabled:
            self.enable()
        torque_enc = float_to_uint(torque, T_MIN, T_MAX, 16)
        ext_id = (COMM_MOTION_CTRL << 24) | (torque_enc << 8) | self.can_id
        data = bytearray(8)
        data[0] = (float_to_uint(angle, P_MIN, P_MAX, 16) >> 8) & 0xFF
        data[1] =  float_to_uint(angle, P_MIN, P_MAX, 16)       & 0xFF
        data[2] = (float_to_uint(speed, V_MIN, V_MAX, 16) >> 8) & 0xFF
        data[3] =  float_to_uint(speed, V_MIN, V_MAX, 16)       & 0xFF
        data[4] = (float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8)  & 0xFF
        data[5] =  float_to_uint(kp, KP_MIN, KP_MAX, 16)        & 0xFF
        data[6] = (float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8)  & 0xFF
        data[7] =  float_to_uint(kd, KD_MIN, KD_MAX, 16)        & 0xFF
        self.bus.send(can.Message(arbitration_id=ext_id, data=list(data), is_extended_id=True))

    def position_control(self, angle_rad: float, speed_rad_s: float = 5.0, acceleration: float = 10.0):
        self._ensure_mode(MODE_POS_PP)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_PP_LIMIT_SPD, speed_rad_s)
        self.set_param_float(IDX_PP_ACC, acceleration)
        time.sleep(0.001)
        self.set_param_float(IDX_LOC_REF, angle_rad)

    def csp_position_control(self, angle_rad: float, limit_speed: float = 5.0):
        self._ensure_mode(MODE_POS_CSP)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_LIMIT_SPD, limit_speed)
        time.sleep(0.001)
        self.set_param_float(IDX_LOC_REF, angle_rad)

    def speed_control(self, speed_rad_s: float, limit_current: float = 5.0):
        self._ensure_mode(MODE_SPEED)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_LIMIT_CUR, limit_current)
        self.set_param_float(IDX_SPD_ACC, 10.0)
        self.set_param_float(IDX_SPD_REF, speed_rad_s)

    def current_control(self, current_a: float):
        self._ensure_mode(MODE_CURRENT)
        if not self.state.enabled:
            self.enable()
        self.set_param_float(IDX_IQ_REF, current_a)

    # ── CAN ID / 보레이트 변경 ────────────────────────────────────────────────

    def change_can_id(self, new_id: int):
        self.disable()
        time.sleep(0.01)
        ext_id = (COMM_SET_CAN_ID << 24) | (new_id << 16) | (self.master_id << 8) | self.can_id
        self.bus.send(can.Message(arbitration_id=ext_id, data=[0]*8, is_extended_id=True))

    def change_baudrate(self, rate_code: int):
        """rate_code: 0x01=1Mbps, 0x02=500K, 0x03=250K, 0x04=125K"""
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, rate_code, 0x08])
        self._send_ext(COMM_BAUD_CHANGE, data, self.master_id)

    # ── 정리 ──────────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop_event.set()
        self._rx_thread.join(timeout=1.0)
        self.bus.shutdown()


# ── MIT Protocol 클래스 (제어용) ──────────────────────────────────────────────

class RobStrideMITMotor:
    """
    RobStride RS01 MIT Protocol 제어 클래스.
    CAN bus는 ArmRunner에서 생성해서 주입 (7개 모터가 버스 공유).

    Args:
        can_id: 모터 CAN ID
        bus:    공유 can.interface.Bus 인스턴스
    """

    def __init__(self, can_id: int, bus: can.interface.Bus):
        self.can_id = can_id
        self.state  = MotorState()
        self.bus    = bus

    # ── 내부 송신 ─────────────────────────────────────────────────────────────

    def _send(self, std_id: int, data: bytes):
        msg = can.Message(arbitration_id=std_id, data=list(data), is_extended_id=False)
        self.bus.send(msg)

    # ── 수신 파싱 (ArmRunner의 rx 루프에서 호출) ──────────────────────────────

    def parse(self, msg: can.Message):
        if msg.is_extended_id:
            return
        data = bytes(msg.data)
        if (msg.arbitration_id & 0xFF) != self.can_id:
            return
        if any(data[3:8]):
            self.state.angle  = uint_to_float((data[1] << 8) | data[2], P_MIN, P_MAX, 16)
            self.state.speed  = uint_to_float((data[3] << 4) | (data[4] >> 4), V_MIN, V_MAX, 12)
            self.state.torque = uint_to_float(((data[4] & 0x0F) << 8) | data[5], T_MIN, T_MAX, 12)
            self.state.temp   = ((data[6] << 8) | data[7]) * 0.1

    # ── 기본 제어 ─────────────────────────────────────────────────────────────

    def enable(self):
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]))
        self.state.enabled = True

    def disable(self):
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]))
        self.state.enabled = False

    def set_zero(self):
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]))

    def clear_error(self):
        self._send(self.can_id, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFB]))

    # ── MIT 복합 제어 ─────────────────────────────────────────────────────────

    def control(
        self,
        angle:  float = 0.0,
        speed:  float = 0.0,
        kp:     float = 10.0,
        kd:     float = 0.5,
        torque: float = 0.0,
    ):
        """
        MIT 복합 제어. 드라이버 내부에서 τ = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff 계산.

        Args:
            angle:  목표 각도 [rad]  (-12.5 ~ 12.5)
            speed:  목표 속도 [rad/s] (-44 ~ 44)
            kp:     위치 게인 (0 ~ 500)
            kd:     속도 게인 (0 ~ 5)
            torque: feedforward 토크 [Nm] (-17 ~ 17)
        """
        angle_enc  = float_to_uint(angle,  P_MIN,  P_MAX,  16)
        speed_enc  = float_to_uint(speed,  V_MIN,  V_MAX,  12)
        kp_enc     = float_to_uint(kp,     KP_MIN, KP_MAX, 12)
        kd_enc     = float_to_uint(kd,     KD_MIN, KD_MAX, 12)
        torque_enc = float_to_uint(torque, T_MIN,  T_MAX,  12)

        data = bytearray(8)
        data[0] = (angle_enc >> 8) & 0xFF
        data[1] =  angle_enc       & 0xFF
        data[2] = (speed_enc >> 4) & 0xFF
        data[3] = ((speed_enc & 0x0F) << 4) | ((kp_enc >> 8) & 0x0F)
        data[4] =  kp_enc          & 0xFF
        data[5] = (kd_enc >> 4)    & 0xFF
        data[6] = ((kd_enc & 0x0F) << 4) | ((torque_enc >> 8) & 0x0F)
        data[7] =  torque_enc      & 0xFF

        self._send(self.can_id, bytes(data))

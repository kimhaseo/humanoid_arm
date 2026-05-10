"""
JointController - 7축 로봇팔 Standard Protocol 제어
robstride_motor_controller.py와 동일한 Extended Frame 프로토콜 사용.
CAN 버스 1개를 7개 모터가 공유하는 구조.
"""

import time
import threading
import can
from robstride_motor_controller import (
    float_to_uint, uint_to_float, MotorState,
    COMM_MOTOR_ENABLE, COMM_MOTOR_STOP, COMM_SET_POS_ZERO,
    COMM_MOTION_CTRL, COMM_MOTOR_REQUEST,
    COMM_SET_PARAM, IDX_RUN_MODE, MODE_MOVE,
    MASTER_CAN_ID,
)
from config import get_config

_cfg = get_config()

CAN_CHANNEL = _cfg.can.channel
CAN_BUSTYPE = _cfg.can.bustype
CAN_BITRATE = _cfg.can.bitrate

DEFAULT_KP = _cfg.control.default_kp
DEFAULT_KD = _cfg.control.default_kd

JOINT_CONFIGS = _cfg.joints  # dict[str, JointConfig]

P_MIN,  P_MAX  = _cfg.motor_limits.p_min,  _cfg.motor_limits.p_max
V_MIN,  V_MAX  = _cfg.motor_limits.v_min,  _cfg.motor_limits.v_max
KP_MIN, KP_MAX = _cfg.motor_limits.kp_min, _cfg.motor_limits.kp_max
KD_MIN, KD_MAX = _cfg.motor_limits.kd_min, _cfg.motor_limits.kd_max
T_MIN,  T_MAX  = _cfg.motor_limits.t_min,  _cfg.motor_limits.t_max


class _Motor:
    """
    공유 버스를 사용하는 Standard Protocol 모터 래퍼.
    robstride_motor_controller.RobStrideMotor와 동일한 Extended Frame 방식.
    """

    MASTER_ID = MASTER_CAN_ID  # 0xFD

    def __init__(self, can_id: int, bus: can.interface.Bus,
                 limit_min: float = P_MIN, limit_max: float = P_MAX):
        self.can_id     = can_id
        self.limit_min  = limit_min
        self.limit_max  = limit_max
        self.state  = MotorState()
        self.bus    = bus
        self._run_mode_cache = -1

    def _send_ext(self, comm_type: int, data: bytes, data_field: int = 0):
        ext_id = (comm_type << 24) | (data_field << 8) | self.can_id
        msg = can.Message(arbitration_id=ext_id, data=list(data), is_extended_id=True)
        self.bus.send(msg)

    def parse(self, msg: can.Message):
        """JointController의 RX 스레드에서 호출."""
        if not msg.is_extended_id:
            return
        ext_id    = msg.arbitration_id
        comm_type = (ext_id >> 24) & 0x3F
        src_id    = (ext_id >> 8)  & 0xFF
        if src_id != self.can_id:
            return
        if comm_type == COMM_MOTOR_REQUEST:
            data = bytes(msg.data)
            self.state.angle  = uint_to_float((data[0] << 8) | data[1], P_MIN, P_MAX, 16)
            self.state.speed  = uint_to_float((data[2] << 8) | data[3], V_MIN, V_MAX, 16)
            self.state.torque = uint_to_float((data[4] << 8) | data[5], T_MIN, T_MAX, 16)
            self.state.temp   = ((data[6] << 8) | data[7]) * 0.1
            self.state.error  = (ext_id >> 16) & 0x3F

    def enable(self):
        self._send_ext(COMM_MOTOR_ENABLE, bytes(8), self.MASTER_ID)
        self.state.enabled = True

    def disable(self, clear_error: bool = False):
        data = bytearray(8)
        data[0] = 1 if clear_error else 0
        self._send_ext(COMM_MOTOR_STOP, bytes(data), self.MASTER_ID)
        self.state.enabled = False

    def set_zero(self):
        data = bytearray(8)
        data[0] = 1
        self._send_ext(COMM_SET_POS_ZERO, bytes(data), self.MASTER_ID)

    def _set_run_mode(self, mode: int):
        if self._run_mode_cache != mode:
            data = bytearray(8)
            data[0] = IDX_RUN_MODE & 0xFF
            data[1] = (IDX_RUN_MODE >> 8) & 0xFF
            data[4] = mode & 0xFF
            self._send_ext(COMM_SET_PARAM, bytes(data), self.MASTER_ID)
            self._run_mode_cache = mode
            time.sleep(0.002)

    def control(
        self,
        angle:  float = 0.0,
        speed:  float = 0.0,
        kp:     float = DEFAULT_KP,
        kd:     float = DEFAULT_KD,
        torque: float = 0.0,
    ):
        """
        Extended Frame 복합 제어. τ = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff

        Args:
            angle:  목표 각도 [rad]   (-12.5 ~ 12.5)
            speed:  목표 속도 [rad/s] (-44 ~ 44)
            kp:     위치 게인         (0 ~ 500)
            kd:     속도 게인         (0 ~ 5)
            torque: feedforward 토크 [Nm] (-17 ~ 17)
        """
        angle = max(self.limit_min, min(self.limit_max, angle))

        self._set_run_mode(MODE_MOVE)
        if not self.state.enabled:
            self.enable()

        torque_enc = float_to_uint(torque, T_MIN, T_MAX, 16)
        ext_id = (COMM_MOTION_CTRL << 24) | (torque_enc << 8) | self.can_id

        angle_enc = float_to_uint(angle, P_MIN, P_MAX, 16)
        speed_enc = float_to_uint(speed, V_MIN, V_MAX, 16)
        kp_enc    = float_to_uint(kp,    KP_MIN, KP_MAX, 16)
        kd_enc    = float_to_uint(kd,    KD_MIN, KD_MAX, 16)

        data = bytearray(8)
        data[0] = (angle_enc >> 8) & 0xFF
        data[1] =  angle_enc       & 0xFF
        data[2] = (speed_enc >> 8) & 0xFF
        data[3] =  speed_enc       & 0xFF
        data[4] = (kp_enc    >> 8) & 0xFF
        data[5] =  kp_enc          & 0xFF
        data[6] = (kd_enc    >> 8) & 0xFF
        data[7] =  kd_enc          & 0xFF

        msg = can.Message(arbitration_id=ext_id, data=list(data), is_extended_id=True)
        self.bus.send(msg)


class JointController:
    """
    7개 조인트 Standard Protocol 컨트롤러.
    CAN 버스 1개 + RX 스레드 1개를 7개 모터가 공유한다.

    사용 예시:
        ctrl = JointController()
        ctrl.enable_all()
        ctrl.set_joints({'joint1': 0.5, 'joint2': -0.3})
        ctrl.disable_all()
        ctrl.shutdown()
    """

    def __init__(
        self,
        channel: str = CAN_CHANNEL,
        bustype: str = CAN_BUSTYPE,
        bitrate: int = CAN_BITRATE,
    ):
        self.bus = can.interface.Bus(channel=channel, interface=bustype, bitrate=bitrate)

        self.motors: dict[str, _Motor] = {
            name: _Motor(
                can_id=jcfg.can_id,
                bus=self.bus,
                limit_min=jcfg.limit_min,
                limit_max=jcfg.limit_max,
            )
            for name, jcfg in JOINT_CONFIGS.items()
        }

        self._stop_event = threading.Event()
        self._rx_thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _rx_loop(self):
        while not self._stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            for motor in self.motors.values():
                motor.parse(msg)

    # ── 활성화 / 비활성화 ──────────────────────────────────────────────────────

    def enable_all(self):
        for motor in self.motors.values():
            motor.enable()
        print("[JointController] 전체 모터 활성화")

    def disable_all(self, clear_error: bool = False):
        for motor in self.motors.values():
            motor.disable(clear_error=clear_error)
        print("[JointController] 전체 모터 비활성화")

    def enable(self, joint_name: str):
        self.motors[joint_name].enable()

    def disable(self, joint_name: str, clear_error: bool = False):
        self.motors[joint_name].disable(clear_error=clear_error)

    # ── MIT 제어 명령 ──────────────────────────────────────────────────────────

    def set_joints(
        self,
        angles:  dict[str, float],
        speeds:  dict[str, float] | None = None,
        kp:      float | dict[str, float] = DEFAULT_KP,
        kd:      float | dict[str, float] = DEFAULT_KD,
        torques: dict[str, float] | None = None,
    ):
        """
        조인트별 복합 제어 명령 전송.

        Args:
            angles:  {'joint1': rad, ...} 목표 각도
            speeds:  {'joint1': rad/s, ...} 목표 속도 (기본 0.0)
            kp:      위치 게인 - 단일 float 또는 조인트별 dict
            kd:      속도 게인 - 단일 float 또는 조인트별 dict
            torques: {'joint1': Nm, ...} feedforward 토크 (기본 0.0)
        """
        for name, motor in self.motors.items():
            if name not in angles:
                continue
            motor.control(
                angle  = angles[name],
                speed  = (speeds  or {}).get(name, 0.0),
                kp     = kp[name] if isinstance(kp, dict) else kp,
                kd     = kd[name] if isinstance(kd, dict) else kd,
                torque = (torques or {}).get(name, 0.0),
            )

    def set_joint(
        self,
        joint_name: str,
        angle:  float,
        speed:  float = 0.0,
        kp:     float = DEFAULT_KP,
        kd:     float = DEFAULT_KD,
        torque: float = 0.0,
    ):
        """단일 조인트 복합 제어 명령 전송."""
        self.motors[joint_name].control(angle=angle, speed=speed, kp=kp, kd=kd, torque=torque)

    # ── 영점 설정 ──────────────────────────────────────────────────────────────

    def set_zero_all(self):
        for motor in self.motors.values():
            motor.set_zero()
        print("[JointController] 전체 모터 영점 설정")

    # ── 상태 조회 ──────────────────────────────────────────────────────────────

    def get_states(self) -> dict[str, dict]:
        return {
            name: {
                'angle':  motor.state.angle,
                'speed':  motor.state.speed,
                'torque': motor.state.torque,
                'temp':   motor.state.temp,
                'error':  motor.state.error,
            }
            for name, motor in self.motors.items()
        }

    def print_states(self):
        states = self.get_states()
        print(f"{'Joint':<10} {'Angle(rad)':>12} {'Speed(rad/s)':>14} {'Torque(Nm)':>12} {'Temp(°C)':>10} {'Error':>7}")
        print("-" * 70)
        for name, s in states.items():
            print(f"{name:<10} {s['angle']:>12.4f} {s['speed']:>14.4f} {s['torque']:>12.4f} {s['temp']:>10.1f} {s['error']:>7}")

    # ── 종료 ───────────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop_event.set()
        self._rx_thread.join(timeout=1.0)
        self.bus.shutdown()
        print("[JointController] 종료")


# ── 사용 예시 ──────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import can as _can
    from dummy_bus import DummyBus
    _can.interface.Bus = lambda **kwargs: DummyBus()

    jc = JointController()

    try:
        jc.enable_all()
        time.sleep(0.5)

        jc.set_joints(
            angles={f'joint{i}': 0.0 for i in range(1, 5)},
            speeds={f'joint{i}': 0.0 for i in range(1, 5)},
        )
        time.sleep(2.0)
        jc.print_states()

        jc.set_joints(
            angles={'joint1':  0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0},
            speeds={'joint1':  0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0},
            kp=5.0, kd=1.0,
        )
        time.sleep(2.0)
        jc.print_states()

    except KeyboardInterrupt:
        print('\n사용자 중단')
    finally:
        jc.disable_all(clear_error=True)
        time.sleep(0.2)
        jc.shutdown()

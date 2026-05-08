"""
JointController - 7축 로봇팔 MIT 제어
robstride_motor_controller.py의 상수/함수를 그대로 사용하되,
CAN 버스 1개를 7개 모터가 공유하는 구조로 직접 구현한다.
"""

import time
import threading
import can
from robstride_motor_controller import float_to_uint, uint_to_float, MotorState
from config import get_config

_cfg = get_config()

CAN_CHANNEL = _cfg.can.channel
CAN_BUSTYPE = _cfg.can.bustype
CAN_BITRATE = _cfg.can.bitrate

DEFAULT_KP = _cfg.control.default_kp
DEFAULT_KD = _cfg.control.default_kd

JOINT_IDS: dict[str, int] = _cfg.joints

P_MIN,  P_MAX  = _cfg.motor_limits.p_min,  _cfg.motor_limits.p_max
V_MIN,  V_MAX  = _cfg.motor_limits.v_min,  _cfg.motor_limits.v_max
KP_MIN, KP_MAX = _cfg.motor_limits.kp_min, _cfg.motor_limits.kp_max
KD_MIN, KD_MAX = _cfg.motor_limits.kd_min, _cfg.motor_limits.kd_max
T_MIN,  T_MAX  = _cfg.motor_limits.t_min,  _cfg.motor_limits.t_max


class _MITMotor:
    """
    공유 버스를 사용하는 MIT 모터 래퍼.
    버스와 RX 스레드는 JointController가 관리하고,
    이 클래스는 송신 및 상태 파싱만 담당한다.
    """

    def __init__(self, can_id: int, bus: can.interface.Bus):
        self.can_id = can_id
        self.state  = MotorState()
        self.bus    = bus

    def _send(self, data: bytes):
        msg = can.Message(arbitration_id=self.can_id, data=list(data), is_extended_id=False)
        self.bus.send(msg)

    def parse(self, msg: can.Message):
        """JointController의 RX 스레드에서 호출."""
        if msg.is_extended_id:
            return
        data = bytes(msg.data)
        # 이 모터에서 온 응답인지 확인 (arbitration_id 하위 바이트 = motor can_id)
        if (msg.arbitration_id & 0xFF) != self.can_id:
            return
        if not (data[3] == 0 and data[4] == 0 and data[5] == 0 and data[6] == 0 and data[7] == 0):
            self.state.angle  = uint_to_float((data[1] << 8) | data[2], P_MIN, P_MAX, 16)
            self.state.speed  = uint_to_float((data[3] << 4) | (data[4] >> 4), V_MIN, V_MAX, 12)
            self.state.torque = uint_to_float(((data[4] & 0x0F) << 8) | data[5], T_MIN, T_MAX, 12)
            self.state.temp   = ((data[6] << 8) | data[7]) * 0.1

    def enable(self):
        self._send(bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]))
        self.state.enabled = True

    def disable(self):
        self._send(bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]))
        self.state.enabled = False

    def set_zero(self):
        self._send(bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]))

    def clear_error(self):
        self._send(bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFB]))

    def control(
        self,
        angle:  float = 0.0,
        speed:  float = 0.0,
        kp:     float = DEFAULT_KP,
        kd:     float = DEFAULT_KD,
        torque: float = 0.0,
    ):
        """
        MIT 복합 제어. τ = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff

        Args:
            angle:  목표 각도 [rad]   (-12.5 ~ 12.5)
            speed:  목표 속도 [rad/s] (-44 ~ 44)
            kp:     위치 게인         (0 ~ 500)
            kd:     속도 게인         (0 ~ 5)
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

        self._send(bytes(data))


class JointController:
    """
    7개 조인트 MIT 제어 컨트롤러.
    CAN 버스 1개 + RX 스레드 1개를 7개 모터가 공유한다.

    사용 예시:
        ctrl = JointController()
        ctrl.enable_all()
        ctrl.set_joints({'joint1': 0.5, 'joint2': -0.3, ...})
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

        self.motors: dict[str, _MITMotor] = {
            name: _MITMotor(can_id=motor_id, bus=self.bus)
            for name, motor_id in JOINT_IDS.items()
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

    def disable_all(self):
        for motor in self.motors.values():
            motor.disable()
        print("[JointController] 전체 모터 비활성화")

    def enable(self, joint_name: str):
        self.motors[joint_name].enable()

    def disable(self, joint_name: str):
        self.motors[joint_name].disable()

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
        조인트별 MIT 제어 명령 전송.

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
        """단일 조인트 MIT 제어 명령 전송."""
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
            }
            for name, motor in self.motors.items()
        }

    def print_states(self):
        states = self.get_states()
        print(f"{'Joint':<10} {'Angle(rad)':>12} {'Speed(rad/s)':>14} {'Torque(Nm)':>12} {'Temp(°C)':>10}")
        print("-" * 62)
        for name, s in states.items():
            print(f"{name:<10} {s['angle']:>12.4f} {s['speed']:>14.4f} {s['torque']:>12.4f} {s['temp']:>10.1f}")

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
            angles={f'joint{i}': 0.0 for i in range(1, 8)},
            speeds={f'joint{i}': 0.0 for i in range(1, 8)},
        )
        time.sleep(2.0)
        jc.print_states()

        jc.set_joints(
            angles={
                'joint1':  0.5, 'joint2': -0.3, 'joint3':  0.8,
                'joint4': -0.5, 'joint5':  0.3, 'joint6': -0.2,
                'joint7':  0.1,
            },
            speeds={
                'joint1':  1.0, 'joint2':  1.0, 'joint3':  1.0,
                'joint4':  1.0, 'joint5':  1.0, 'joint6':  1.0,
                'joint7':  1.0,
            },
            kp=15.0, kd=0.8,
        )
        time.sleep(2.0)
        jc.print_states()

    finally:
        jc.disable_all()
        time.sleep(0.2)
        jc.shutdown()

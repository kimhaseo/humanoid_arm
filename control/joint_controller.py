"""
JointController - 7축 로봇팔 Standard Protocol 제어
robstride_motor_controller.py와 동일한 Extended Frame 프로토콜 사용.
CAN 버스 1개를 7개 모터가 공유하는 구조.
"""

import math
import time
import threading
import can
from motors.robstride_motor_controller import (
    float_to_uint, uint_to_float, MotorState,
    COMM_MOTOR_ENABLE, COMM_MOTOR_STOP, COMM_SET_POS_ZERO,
    COMM_MOTION_CTRL, COMM_MOTOR_REQUEST,
    COMM_SET_PARAM, IDX_RUN_MODE, MODE_MOVE,
    MASTER_CAN_ID, P_MIN, P_MAX, KP_MIN, KP_MAX, KD_MIN, KD_MAX,
)
from config import get_config

_cfg = get_config()

CAN_CHANNEL = _cfg.can.channel
CAN_BUSTYPE = _cfg.can.bustype
CAN_BITRATE = _cfg.can.bitrate

DEFAULT_KP = _cfg.control.default_kp
DEFAULT_KD = _cfg.control.default_kd

JOINT_CONFIGS = _cfg.joints  # dict[str, JointConfig]

# P_MIN/P_MAX는 CAN 프로토콜의 16비트 각도 인코딩 범위 — 실제 모터 펌웨어 스펙에 고정된
# 값이므로 robstride_motor_controller에서 그대로 가져온다 (config 값으로 바꾸면 안 됨:
# 인코딩 시 여기 범위로 압축했는데 펌웨어가 -12.5~12.5로 풀면 각도가 실제보다 부풀려진다).
KP_MIN, KP_MAX = _cfg.motor_limits.kp_min, _cfg.motor_limits.kp_max
KD_MIN, KD_MAX = _cfg.motor_limits.kd_min, _cfg.motor_limits.kd_max

MOTOR_MODELS = _cfg.motor_models  # dict[str, MotorModelSpec]


class _Motor:
    """
    공유 버스를 사용하는 Standard Protocol 모터 래퍼.
    robstride_motor_controller.RobStrideMotor와 동일한 Extended Frame 방식.
    """

    MASTER_ID = MASTER_CAN_ID  # 0xFD

    def __init__(self, can_id: int, bus: can.interface.Bus,
                 limit_min: float = P_MIN, limit_max: float = P_MAX,
                 kp: float = DEFAULT_KP, kd: float = DEFAULT_KD,
                 v_min: float = -44.0, v_max: float = 44.0,
                 t_min: float = -17.0, t_max: float = 17.0,
                 sign: int = 1):
        self.can_id     = can_id
        self.limit_min  = limit_min
        self.limit_max  = limit_max
        self.kp         = kp
        self.kd         = kd
        self.v_min      = v_min
        self.v_max      = v_max
        self.t_min      = t_min
        self.t_max      = t_max
        self.sign       = sign   # 1 or -1
        self.state  = MotorState()
        self.bus    = bus
        self._run_mode_cache = -1
        self._state_event = threading.Event()

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
            self.state.angle  = uint_to_float((data[0] << 8) | data[1], P_MIN, P_MAX, 16) * self.sign
            self.state.speed  = uint_to_float((data[2] << 8) | data[3], self.v_min, self.v_max, 16) * self.sign
            self.state.torque = uint_to_float((data[4] << 8) | data[5], self.t_min, self.t_max, 16) * self.sign
            self.state.temp   = ((data[6] << 8) | data[7]) * 0.1
            self.state.error  = (ext_id >> 16) & 0x3F
            self._state_event.set()

    def request_state(self):
        """현재 상태 요청 (COMM_MOTOR_REQUEST 0x02)."""
        self._send_ext(COMM_MOTOR_REQUEST, bytes(8), self.MASTER_ID)

    def fetch_state(self, timeout: float = 0.2) -> MotorState:
        """상태 요청 후 응답이 올 때까지 대기. timeout 초과 시 TimeoutError."""
        self._state_event.clear()
        self.request_state()
        if not self._state_event.wait(timeout):
            raise TimeoutError(f"모터 {self.can_id} 상태 요청 응답 없음 (timeout={timeout}s)")
        return self.state

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
        kp:     float | None = None,
        kd:     float | None = None,
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
        if kp is None:
            kp = self.kp
        if kd is None:
            kd = self.kd
        angle = max(self.limit_min, min(self.limit_max, angle))

        self._set_run_mode(MODE_MOVE)
        if not self.state.enabled:
            self.enable()

        torque_enc = float_to_uint(torque * self.sign, self.t_min, self.t_max, 16)
        ext_id = (COMM_MOTION_CTRL << 24) | (torque_enc << 8) | self.can_id

        angle_enc = float_to_uint(angle * self.sign, P_MIN, P_MAX, 16)
        speed_enc = float_to_uint(speed * self.sign, self.v_min, self.v_max, 16)
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


def _trapezoid_profile(
    q0: float,
    q1: float,
    max_vel: float,
    max_acc: float,
    dt: float = 0.02,
) -> list[tuple[float, float]]:
    """
    사다리꼴 속도 프로파일로 q0 → q1 궤적 생성.

    Returns:
        [(position, velocity), ...] at dt 간격.
        마지막 웨이포인트는 항상 (q1, 0.0).
    """
    if max_vel <= 0:
        raise ValueError(f"max_vel은 양수여야 합니다: {max_vel}")
    if max_acc <= 0:
        raise ValueError(f"max_acc은 양수여야 합니다: {max_acc}")

    d = q1 - q0
    if abs(d) < 1e-6:
        return [(q0, 0.0)]

    sign = 1.0 if d > 0 else -1.0
    dist = abs(d)

    t_acc = max_vel / max_acc
    d_acc = 0.5 * max_acc * t_acc ** 2

    if 2 * d_acc >= dist:
        # 삼각형 프로파일 (최대속도 도달 전에 감속)
        t_acc = math.sqrt(dist / max_acc)
        v_peak = max_acc * t_acc
        t_flat = 0.0
    else:
        v_peak = max_vel
        t_flat = (dist - 2 * d_acc) / max_vel

    t_total = 2 * t_acc + t_flat
    waypoints: list[tuple[float, float]] = []
    t = 0.0
    while t < t_total:
        if t < t_acc:
            v = max_acc * t * sign
            p = q0 + sign * 0.5 * max_acc * t ** 2
        elif t < t_acc + t_flat:
            dt_f = t - t_acc
            v = v_peak * sign
            p = q0 + sign * (0.5 * max_acc * t_acc ** 2 + v_peak * dt_f)
        else:
            dt_d = t - t_acc - t_flat
            v = (v_peak - max_acc * dt_d) * sign
            p = q0 + sign * (
                0.5 * max_acc * t_acc ** 2
                + v_peak * t_flat
                + v_peak * dt_d - 0.5 * max_acc * dt_d ** 2
            )
        waypoints.append((p, v))
        t += dt

    waypoints.append((q1, 0.0))
    return waypoints


def _ease_profile(
    q0: float,
    q1: float,
    duration: float,
    dt: float = 0.005,
) -> list[tuple[float, float]]:
    """
    코사인 이즈인아웃 프로파일로 q0 → q1을 정확히 duration초에 이동.
    시작/끝 속도가 0으로 수렴하며, 여러 조인트를 같은 박자(시간)에 맞춰
    동시 도착시키고 싶을 때 _trapezoid_profile 대신 사용한다.

    Returns:
        [(position, velocity), ...] at dt 간격. 마지막은 항상 (q1, 0.0).
    """
    if duration <= 0:
        raise ValueError(f"duration은 양수여야 합니다: {duration}")

    d = q1 - q0
    n_steps = max(1, round(duration / dt))
    waypoints: list[tuple[float, float]] = []
    for step in range(n_steps):
        t = step * dt
        phase = math.pi * t / duration
        p = q0 + d * (1 - math.cos(phase)) / 2
        v = d * (math.pi / (2 * duration)) * math.sin(phase)
        waypoints.append((p, v))
    waypoints.append((q1, 0.0))
    return waypoints


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

        # 중력 보상 (URDF 질량/관성 기반, RNEA)
        ctrl.enable_gravity_compensation(scale=1.0)   # 켜기 (배율 1.0 = 100% 보상)
        ctrl.set_gravity_scale(0.5)                   # 배율만 조절 (50% 보상)
        ctrl.disable_gravity_compensation()           # 끄기
    """

    def __init__(
        self,
        channel:    str = CAN_CHANNEL,
        bustype:    str = CAN_BUSTYPE,
        bitrate:    int = CAN_BITRATE,
        urdf_path:  str | None = None,
        visualizer=None,
        bus=None,
    ):
        self.bus = bus if bus is not None else can.interface.Bus(
            channel=channel, interface=bustype, bitrate=bitrate)

        self.motors: dict[str, _Motor] = {
            name: _Motor(
                can_id=jcfg.can_id,
                bus=self.bus,
                limit_min=jcfg.limit_min,
                limit_max=jcfg.limit_max,
                kp=jcfg.kp,
                kd=jcfg.kd,
                v_min=MOTOR_MODELS[jcfg.motor_model].v_min,
                v_max=MOTOR_MODELS[jcfg.motor_model].v_max,
                t_min=MOTOR_MODELS[jcfg.motor_model].t_min,
                t_max=MOTOR_MODELS[jcfg.motor_model].t_max,
                sign=jcfg.sign,
            )
            for name, jcfg in JOINT_CONFIGS.items()
        }

        self._ik_solver      = self._build_ik_solver(urdf_path)
        self._gravity_comp   = self._build_gravity_compensator(urdf_path)
        self.gravity_enabled = False
        self.gravity_scale   = 1.0
        self._visualizer     = visualizer

        self._stop_event = threading.Event()
        self._rx_thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # MeshCat 렌더링 전용 스레드 (모터 루프와 완전 분리, 30Hz)
        self._vis_angles: dict[str, float] | None = None
        self._vis_lock   = threading.Lock()
        self._vis_thread: threading.Thread | None = None
        if self._visualizer is not None:
            self._vis_thread = threading.Thread(target=self._vis_loop, daemon=True)
            self._vis_thread.start()

    def _build_ik_solver(self, urdf_path: str | None):
        from kinematics.ik_solver import IKSolver
        kwargs = {} if urdf_path is None else {"urdf_path": urdf_path}
        return IKSolver(active_joints=list(self.motors.keys()), **kwargs)

    def _build_gravity_compensator(self, urdf_path: str | None):
        from dynamics.gravity_compensation import GravityCompensator
        kwargs = {} if urdf_path is None else {"urdf_path": urdf_path}
        return GravityCompensator(active_joints=list(self.motors.keys()), **kwargs)

    def _rx_loop(self):
        while not self._stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            for motor in self.motors.values():
                motor.parse(msg)

    def _vis_loop(self):
        """30Hz 고정 주기로 최신 joint angles를 MeshCat에 렌더링."""
        interval = 1.0 / 30
        t_next = time.perf_counter()
        while not self._stop_event.is_set():
            with self._vis_lock:
                angles = dict(self._vis_angles) if self._vis_angles is not None else None
            if angles is not None:
                self._visualizer.display(angles)
            t_next += interval
            remaining = t_next - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                t_next = time.perf_counter()  # 렌더링이 밀렸으면 기준 시각 재설정

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

    # ── 중력 보상 ─────────────────────────────────────────────────────────────

    def enable_gravity_compensation(self, scale: float = 1.0):
        """
        중력 보상 활성화. 이후 모든 이동 명령(set_joints/move_joint*)에 URDF
        질량/관성 기반 RNEA 중력 토크가 feedforward로 자동 추가된다.

        Args:
            scale: 보상 배율 (1.0 = 100% 보상, 0.5 = 절반만 보상, 0.0 = 무보상)
        """
        self.gravity_enabled = True
        self.gravity_scale   = scale
        print(f"[JointController] 중력 보상 ON (배율 {scale:.2f})")

    def disable_gravity_compensation(self):
        """중력 보상 비활성화 (feedforward 토크 0)."""
        self.gravity_enabled = False
        print("[JointController] 중력 보상 OFF")

    def set_gravity_scale(self, scale: float):
        """중력 보상 on/off 상태는 유지한 채 배율만 변경."""
        self.gravity_scale = scale

    def _current_full_angles(self) -> dict[str, float]:
        """전체 조인트의 최근 피드백 각도 스냅샷 (중력 토크 계산용 기준 자세)."""
        return {name: motor.state.angle for name, motor in self.motors.items()}

    def _gravity_feedforward(self, target_angles: dict[str, float]) -> dict[str, float]:
        """
        target_angles의 조인트들에 대한 중력 보상 토크 계산.
        나머지 조인트는 최근 피드백 각도를 그대로 사용해 전체 팔 자세를 구성한다.
        중력 보상이 꺼져 있으면 빈 dict 반환.
        """
        if not self.gravity_enabled:
            return {}
        full_angles = self._current_full_angles()
        full_angles.update(target_angles)
        tau = self._gravity_comp.gravity_torque(full_angles)
        return {name: t * self.gravity_scale for name, t in tau.items() if name in target_angles}

    # ── MIT 제어 명령 ──────────────────────────────────────────────────────────

    def set_joints(
        self,
        angles:  dict[str, float],
        speeds:  dict[str, float] | None = None,
        kp:      float | dict[str, float] | None = None,
        kd:      float | dict[str, float] | None = None,
        torques: dict[str, float] | None = None,
    ):
        """
        조인트별 복합 제어 명령 전송.

        Args:
            angles:  {'joint1': rad, ...} 목표 각도
            speeds:  {'joint1': rad/s, ...} 목표 속도 (기본 0.0)
            kp:      위치 게인 - 단일 float, 조인트별 dict, 또는 None(config 값 사용)
            kd:      속도 게인 - 단일 float, 조인트별 dict, 또는 None(config 값 사용)
            torques: {'joint1': Nm, ...} feedforward 토크 (기본 0.0) — 중력 보상이
                     켜져 있으면 이 값에 중력 보상 토크가 더해져서 전송된다.
        """
        grav = self._gravity_feedforward(angles)
        for name, motor in self.motors.items():
            if name not in angles:
                continue
            motor.control(
                angle  = angles[name],
                speed  = (speeds  or {}).get(name, 0.0),
                kp     = kp.get(name) if isinstance(kp, dict) else kp,
                kd     = kd.get(name) if isinstance(kd, dict) else kd,
                torque = (torques or {}).get(name, 0.0) + grav.get(name, 0.0),
            )

    def set_joint(
        self,
        joint_name: str,
        angle:  float,
        speed:  float = 0.0,
        kp:     float | None = None,
        kd:     float | None = None,
        torque: float = 0.0,
    ):
        """단일 조인트 복합 제어 명령 전송. kp/kd 미지정 시 config 값 사용."""
        self.set_joints(
            angles  = {joint_name: angle},
            speeds  = {joint_name: speed},
            kp      = {joint_name: kp} if kp is not None else None,
            kd      = {joint_name: kd} if kd is not None else None,
            torques = {joint_name: torque},
        )

    # ── 영점 설정 ──────────────────────────────────────────────────────────────

    def set_zero_all(self):
        for motor in self.motors.values():
            motor.set_zero()
        print("[JointController] 전체 모터 영점 설정")

    # ── 가감속 궤적 이동 ───────────────────────────────────────────────────────

    def fetch_current_positions(self, timeout: float = 0.2) -> dict[str, float]:
        """
        모든 조인트 현재 위치를 모터에서 직접 읽어 반환 [rad].
        현재 위치가 joint limit을 벗어나면 ValueError 발생.
        """
        positions: dict[str, float] = {}
        for name, motor in self.motors.items():
            state = motor.fetch_state(timeout=timeout)
            angle = state.angle
            if angle < motor.limit_min or angle > motor.limit_max:
                raise ValueError(
                    f"[{name}] 현재 위치 {math.degrees(angle):.2f}° 가 "
                    f"리밋 범위 [{math.degrees(motor.limit_min):.1f}°, "
                    f"{math.degrees(motor.limit_max):.1f}°] 를 벗어났습니다."
                )
            positions[name] = angle
        return positions

    def move_joints_traj(
        self,
        angles:   dict[str, float],
        max_vel:  float | dict[str, float] = 1.0,
        max_acc:  float | dict[str, float] = 2.0,
        dt:          float = 0.005,
        kp:          float | dict[str, float] | None = None,
        kd:          float | dict[str, float] | None = None,
        settle_time: float = 0.1,
        wait:        bool = True,
    ):
        """
        현재 위치를 읽고 사다리꼴 가감속 궤적으로 목표 위치까지 이동.

        Args:
            angles:      {'joint1': rad, ...} 목표 각도
            max_vel:     최대 속도 [rad/s] — 단일 값 또는 조인트별 dict
            max_acc:     최대 가속도 [rad/s²] — 단일 값 또는 조인트별 dict
            dt:          제어 주기 [s] (기본 5ms = 200Hz)
            kp/kd:       게인 (None 이면 config 값 사용)
            settle_time: 궤적 완료 후 PD 수렴 대기 시간 [s] (기본 0.1s)
            wait:        True 이면 이동 완료 후 반환
        """
        def _get(param, name):
            return param.get(name) if isinstance(param, dict) else param

        current = self.fetch_current_positions()

        # 조인트별 궤적 생성
        trajs: dict[str, list[tuple[float, float]]] = {}
        for name in angles:
            if name not in self.motors:
                raise KeyError(f"알 수 없는 조인트: {name}")
            q0 = current[name]
            q1 = angles[name]
            motor = self.motors[name]
            q1 = max(motor.limit_min, min(motor.limit_max, q1))
            mv = _get(max_vel, name) or 1.0
            ma = _get(max_acc, name) or 2.0
            trajs[name] = _trapezoid_profile(q0, q1, mv, ma, dt)

        self._execute_traj(trajs, dt=dt, kp=kp, kd=kd, settle_time=settle_time, wait=wait)

    def move_joints_timed(
        self,
        angles:       dict[str, float],
        duration_sec: float,
        dt:           float = 0.005,
        kp:           float | dict[str, float] | None = None,
        kd:           float | dict[str, float] | None = None,
        settle_time:  float = 0.0,
        wait:         bool = True,
    ):
        """
        현재 위치에서 목표 위치까지 모든 조인트가 정확히 duration_sec 후에
        동시 도착하도록 이동 (코사인 이즈인아웃). 음악 박자에 맞춘 안무 재생용 —
        각 조인트마다 이동 거리가 달라도 도착 시각이 항상 같다.

        Args:
            angles:       {'joint1': rad, ...} 목표 각도
            duration_sec: 이번 동작에 걸릴 시간 [s]
            dt:           제어 주기 [s]
            kp/kd:        게인 (None 이면 config 값 사용)
            settle_time:  이동 완료 후 대기 시간 [s]
            wait:         True 이면 이동 완료 후 반환
        """
        current = self.fetch_current_positions()

        trajs: dict[str, list[tuple[float, float]]] = {}
        for name in angles:
            if name not in self.motors:
                raise KeyError(f"알 수 없는 조인트: {name}")
            q0 = current[name]
            motor = self.motors[name]
            q1 = max(motor.limit_min, min(motor.limit_max, angles[name]))
            trajs[name] = _ease_profile(q0, q1, duration_sec, dt)

        self._execute_traj(trajs, dt=dt, kp=kp, kd=kd, settle_time=settle_time, wait=wait)

    def _execute_traj(
        self,
        trajs:       dict[str, list[tuple[float, float]]],
        dt:          float,
        kp:          float | dict[str, float] | None,
        kd:          float | dict[str, float] | None,
        settle_time: float,
        wait:        bool,
    ):
        """trajs의 (position, velocity) 스텝들을 dt 간격으로 실시간 재생."""
        def _get(param, name):
            return param.get(name) if isinstance(param, dict) else param

        # 가장 긴 궤적 길이에 맞춰 나머지를 마지막 값으로 패딩
        max_len = max(len(t) for t in trajs.values())
        for name in trajs:
            last = trajs[name][-1]
            while len(trajs[name]) < max_len:
                trajs[name].append(last)

        def _precise_sleep(deadline: float):
            # Windows에서 time.sleep 정밀도가 ~15ms이므로
            # 잔여 2ms 이상은 sleep, 이하는 busy-wait
            remaining = deadline - time.perf_counter()
            if remaining > 0.002:
                time.sleep(remaining - 0.002)
            while time.perf_counter() < deadline:
                pass

        def _run():
            # 절대 시각 기준으로 sleep → 드리프트 누적 방지
            t_next = time.perf_counter()
            for step in range(max_len):
                step_angles = {name: waypoints[step][0] for name, waypoints in trajs.items()}
                grav = self._gravity_feedforward(step_angles)
                for name, waypoints in trajs.items():
                    pos, vel = waypoints[step]
                    self.motors[name].control(
                        angle  = pos,
                        speed  = vel,
                        kp     = _get(kp, name),
                        kd     = _get(kd, name),
                        torque = grav.get(name, 0.0),
                    )
                if self._visualizer is not None:
                    with self._vis_lock:
                        self._vis_angles = step_angles
                t_next += dt
                _precise_sleep(t_next)

            if settle_time > 0:
                time.sleep(settle_time)

        if wait:
            _run()
        else:
            threading.Thread(target=_run, daemon=True).start()

    def move_joint(
        self,
        joint_name:  str,
        angle:       float,
        max_vel:     float = 1.0,
        max_acc:     float = 2.0,
        dt:          float = 0.005,
        kp:          float | None = None,
        kd:          float | None = None,
        settle_time: float = 0.1,
        wait:        bool = True,
    ):
        """단일 조인트 가감속 궤적 이동."""
        self.move_joints_traj(
            angles       = {joint_name: angle},
            max_vel      = {joint_name: max_vel},
            max_acc      = {joint_name: max_acc},
            dt           = dt,
            kp           = {joint_name: kp} if kp is not None else None,
            kd           = {joint_name: kd} if kd is not None else None,
            settle_time  = settle_time,
            wait         = wait,
        )

    def move_pose(
        self,
        position:    list[float],
        orientation: list[float] | None = None,
        max_vel:     float | dict[str, float] = 1.0,
        max_acc:     float | dict[str, float] = 2.0,
        dt:          float = 0.005,
        kp:          float | dict[str, float] | None = None,
        kd:          float | dict[str, float] | None = None,
        settle_time: float = 0.1,
        wait:        bool = True,
    ):
        """
        목표 위치/자세로 역기구학(IK) 궤적 이동.

        Args:
            position:    [x, y, z] 목표 위치 [mm]
            orientation: [roll, pitch, yaw] 목표 자세 [rad] (ZYX 순서) —
                         None 이면 위치만 제어 (자세는 IK 가 자유롭게 선택)
            max_vel:     최대 속도 [rad/s] — 단일 값 또는 조인트별 dict
            max_acc:     최대 가속도 [rad/s²] — 단일 값 또는 조인트별 dict
            dt:          제어 주기 [s]
            kp/kd:       게인 (None 이면 config 값 사용)
            settle_time: 궤적 완료 후 PD 수렴 대기 시간 [s] (기본 0.1s)
            wait:        True 이면 이동 완료 후 반환
        """
        angles = self._ik_solver.ik(position, orientation)
        self.move_joints_traj(
            angles=angles,
            max_vel=max_vel, max_acc=max_acc,
            dt=dt, kp=kp, kd=kd, settle_time=settle_time, wait=wait,
        )

    # ── 상태 조회 ──────────────────────────────────────────────────────────────

    def get_ee_pose(self, angles: dict[str, float] | None = None) -> tuple[list[float], list[float]]:
        """
        현재(또는 지정한) 조인트 각도에서의 엔드이펙터 포즈를 순기구학으로 계산.
        angles 미지정 시 모터에서 현재 위치를 직접 읽어온다 (응답 없으면 중립 자세로 대체).

        Returns:
            (position [x,y,z] mm, orientation [roll,pitch,yaw] deg)
        """
        if angles is None:
            try:
                angles = self.fetch_current_positions()
            except Exception:
                angles = {}
        pos, rot = self._ik_solver.fk(angles)
        rpy = self._ik_solver.matrix_to_rpy(rot)
        return pos.tolist(), [math.degrees(r) for r in rpy]

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

    # ── 액션 ───────────────────────────────────────────────────────────────────

    def action_1(self):
        """액션 1: 원점 → 목표 → 반대 → 반대 → 원점 복귀."""
        try:
            self._action_1_body()
        except ValueError as e:
            print(f"[action_1] 리밋 초과로 실행 취소: {e}")
        except Exception as e:
            print(f"[action_1] 오류: {e}")

    def _action_1_body(self):
        print("[1] 원점으로 가감속 이동")
        self.move_joints_traj(
            angles={f'joint{i}': 0.0 for i in range(1, 6)},
            max_vel=0.5, max_acc=0.5,
        )
        self.print_states()

    # ── 종료 ───────────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop_event.set()
        self._rx_thread.join(timeout=1.0)
        if self._vis_thread is not None:
            self._vis_thread.join(timeout=1.0)
        self.bus.shutdown()
        print("[JointController] 종료")


# ── 사용 예시 ──────────────────────────────────────────────────────────────────

if __name__ == '__main__':

    # import can as _can
    # from dummy_bus import DummyBus
    # _can.interface.Bus = lambda **kwargs: DummyBus()

    from robot_visualizer import RobotVisualizer
    vis = RobotVisualizer()
    jc = JointController(visualizer=vis)

    try:
        jc.enable_all()
        time.sleep(0.5)
        for i in range(10):
            jc.action_1()
        time.sleep(10)
    except KeyboardInterrupt:
        print('\n사용자 중단')
    finally:
        jc.disable_all(clear_error=True)
        time.sleep(0.2)
        jc.shutdown()

"""
중력보상 오프라인 검증 (실기 없이 실행).

    python tests/test_gravity_compensation.py   (repo root에서 실행)

1) GravityCompensator 단독 - 계산된 토크가 물리적으로 말이 되는지 확인.
   URDF 질량/관성이 바뀌면 회귀 기준값도 같이 갱신해야 한다.
2) JointController + DummyBus 통합 - gravity_enabled/gravity_scale이
   실제 CAN 프레임(torque 필드)까지 반영되는지 확인.
"""
import math

import can as _can

from dynamics.gravity_compensation import GravityCompensator
from motors.robstride_motor_controller import uint_to_float, COMM_MOTION_CTRL


def test_gravity_torque_at_neutral_pose():
    comp = GravityCompensator()
    tau = comp.gravity_torque({})
    assert set(tau.keys()) == {f'joint{i}' for i in range(1, 8)}
    assert all(math.isfinite(v) for v in tau.values())
    # 영점 자세에서 팔이 수평으로 뻗어 있어 base/shoulder 조인트에 유의미한
    # 정적 토크가 걸려야 한다 (0 근처면 URDF 질량이 안 잡혔다는 신호).
    assert abs(tau['joint1']) > 0.5, f"joint1 gravity torque suspiciously small: {tau['joint1']}"


def test_gravity_torque_changes_with_pose():
    comp = GravityCompensator()
    tau_a = comp.gravity_torque({'joint2': 0.0})
    tau_b = comp.gravity_torque({'joint2': math.pi / 2})
    assert tau_a['joint2'] != tau_b['joint2']


def _last_motion_ctrl_frame(bus, can_id):
    matches = [
        m for m in bus.sent_messages
        if (m.arbitration_id & 0xFF) == can_id
        and ((m.arbitration_id >> 24) & 0x3F) == COMM_MOTION_CTRL
    ]
    assert matches, f"can_id={can_id}에 대한 COMM_MOTION_CTRL 프레임이 없습니다"
    return matches[-1]


def _decode_torque(msg, t_min, t_max):
    torque_bits = (msg.arbitration_id >> 8) & 0xFFFF
    return uint_to_float(torque_bits, t_min, t_max, 16)


def test_gravity_disabled_by_default_sends_no_feedforward_torque():
    from canbus.dummy_bus import DummyBus
    from control.joint_controller import JointController, MOTOR_MODELS, JOINT_CONFIGS

    _can.interface.Bus = lambda **kwargs: DummyBus()

    jc = JointController()
    try:
        assert jc.gravity_enabled is False   # 안전을 위해 항상 꺼진 채 시작

        jc.set_joint('joint2', 0.4)

        jcfg = JOINT_CONFIGS['joint2']
        msg  = _last_motion_ctrl_frame(jc.bus, jcfg.can_id)
        spec = MOTOR_MODELS[jcfg.motor_model]
        sent_torque = _decode_torque(msg, spec.t_min, spec.t_max)

        assert abs(sent_torque) < 1e-3
    finally:
        jc.shutdown()


def test_gravity_feedforward_reaches_can_frame():
    from canbus.dummy_bus import DummyBus
    from control.joint_controller import JointController, MOTOR_MODELS, JOINT_CONFIGS

    _can.interface.Bus = lambda **kwargs: DummyBus()

    jc = JointController()
    try:
        jc.enable_gravity_compensation(scale=1.0)

        angle = 0.4
        jc.set_joint('joint2', angle)

        jcfg = JOINT_CONFIGS['joint2']
        msg  = _last_motion_ctrl_frame(jc.bus, jcfg.can_id)
        spec = MOTOR_MODELS[jcfg.motor_model]
        sent_torque = _decode_torque(msg, spec.t_min, spec.t_max)

        expected = GravityCompensator(active_joints=list(jc.motors.keys())) \
            .gravity_torque({'joint2': angle})['joint2']
        expected_clamped = max(spec.t_min, min(spec.t_max, expected))

        assert abs(sent_torque - expected_clamped) < 0.05, (sent_torque, expected_clamped)
    finally:
        jc.shutdown()


def test_gravity_scale_reduces_feedforward_torque():
    from canbus.dummy_bus import DummyBus
    from control.joint_controller import JointController, MOTOR_MODELS, JOINT_CONFIGS

    _can.interface.Bus = lambda **kwargs: DummyBus()

    jc = JointController()
    try:
        jc.enable_gravity_compensation(scale=0.5)

        angle = 0.4
        jc.set_joint('joint2', angle)

        jcfg = JOINT_CONFIGS['joint2']
        msg  = _last_motion_ctrl_frame(jc.bus, jcfg.can_id)
        spec = MOTOR_MODELS[jcfg.motor_model]
        sent_torque = _decode_torque(msg, spec.t_min, spec.t_max)

        full_tau = GravityCompensator(active_joints=list(jc.motors.keys())) \
            .gravity_torque({'joint2': angle})['joint2']
        expected_clamped = max(spec.t_min, min(spec.t_max, full_tau * 0.5))

        assert abs(sent_torque - expected_clamped) < 0.05, (sent_torque, expected_clamped)
    finally:
        jc.shutdown()


if __name__ == '__main__':
    tests = [obj for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    for t in tests:
        t()
        print(f'OK  {t.__name__}')
    print(f'\n{len(tests)} passed')

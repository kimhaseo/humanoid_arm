import numpy as np

JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
]

FREQ = 200          # Hz
DT   = 1.0 / FREQ  # 5 ms


def quintic_coeffs(q0: float, qf: float, T: float) -> np.ndarray:
    """
    5차 다항식(minimum jerk) 계수 반환.
    경계 조건: q(0)=q0, q(T)=qf, dq(0)=dq(T)=0, ddq(0)=ddq(T)=0
    단위: 입력 단위 그대로 (rad 입력 → rad 출력)
    반환: [a0, a1, a2, a3, a4, a5]
    """
    a0 = q0
    a1 = 0.0
    a2 = 0.0
    a3 = 10 * (qf - q0) / T**3
    a4 = -15 * (qf - q0) / T**4
    a5 =   6 * (qf - q0) / T**5
    return np.array([a0, a1, a2, a3, a4, a5])


def eval_quintic(coeffs: np.ndarray, t: float):
    """t 시점의 위치(rad), 속도(rad/s), 가속도(rad/s²) 반환"""
    a = coeffs
    pos = a[0] + a[1]*t   +  a[2]*t**2 +  a[3]*t**3 +  a[4]*t**4 +  a[5]*t**5
    vel = a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4
    acc = 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3
    return pos, vel, acc


def generate_trajectory(
    start_rad: list[float],
    goal_rad:  list[float],
    duration:  float,
) -> list[dict]:
    """
    start → goal 까지 5차 다항식 궤적을 200Hz로 생성.

    Args:
        start_rad: 시작 관절각 [rad] (7개)
        goal_rad:  목표 관절각 [rad] (7개)
        duration:  이동 시간 [초]

    Returns:
        steps: 각 스텝마다 {"step", "t", "joints": [{"name", "pos"(rad), "vel"(rad/s), "acc"(rad/s²)}, ...]}
    """
    assert len(start_rad) == 7 and len(goal_rad) == 7, "조인트 7개 필요"

    coeffs_list = [
        quintic_coeffs(s, g, duration)
        for s, g in zip(start_rad, goal_rad)
    ]

    n_steps = round(duration * FREQ)
    steps = []
    for i in range(n_steps + 1):
        t = min(i * DT, duration)
        joints = []
        for name, coeffs in zip(JOINT_NAMES, coeffs_list):
            pos, vel, acc = eval_quintic(coeffs, t)
            joints.append({"name": name, "pos": pos, "vel": vel, "acc": acc})
        steps.append({"step": i, "t": t, "joints": joints})

    return steps


# ── 실행 예시 ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    start = np.deg2rad([0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0]).tolist()
    goal  = np.deg2rad([30.0, -15.0, 20.0, 10.0, 25.0, 45.0, 30.0]).tolist()
    dur   = 3.0

    steps = generate_trajectory(start, goal, dur)
    total = len(steps)

    print(f"총 {total} 스텝 @ {FREQ}Hz  (duration={dur:.2f}s)\n")
    for s in steps[::60]:  # 0.3초마다 출력
        cols = "  ".join(
            f"{j['name']}=({np.degrees(j['pos']):6.2f}deg, {np.degrees(j['vel']):6.2f}deg/s)"
            for j in s["joints"]
        )
        print(f"step={s['step']:4d}  t={s['t']:.3f}s  {cols}")

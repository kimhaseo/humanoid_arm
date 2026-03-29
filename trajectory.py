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
    """t 시점의 위치(deg), 속도(deg/s), 가속도(deg/s²) 반환"""
    a = coeffs
    pos = a[0] + a[1]*t   +  a[2]*t**2 +  a[3]*t**3 +  a[4]*t**4 +  a[5]*t**5
    vel = a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4
    acc = 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3
    return pos, vel, acc


def generate_trajectory(
    start_angles: list[float],
    goal_angles:  list[float],
    duration:     float,
    use_rad:      bool = False,
) -> list[dict]:
    """
    start → goal 까지 5차 다항식 궤적을 200Hz로 생성.

    Args:
        use_rad: True면 pos(rad), vel(rad/s), acc(rad/s²) 로 변환해서 반환

    Returns:
        steps: 각 스텝마다 {"step", "t", "joints": [{"name", "pos", "vel", "acc"}, ...]}
    """
    assert len(start_angles) == 7 and len(goal_angles) == 7, "조인트 7개 필요"

    coeffs_list = [
        quintic_coeffs(s, g, duration)
        for s, g in zip(start_angles, goal_angles)
    ]

    n_steps = round(duration * FREQ)
    steps = []
    for i in range(n_steps + 1):
        t = min(i * DT, duration)
        joints = []
        for name, coeffs in zip(JOINT_NAMES, coeffs_list):
            pos, vel, acc = eval_quintic(coeffs, t)
            if use_rad:
                pos = np.deg2rad(pos)
                vel = np.deg2rad(vel)
                acc = np.deg2rad(acc)
            joints.append({"name": name, "pos": pos, "vel": vel, "acc": acc})
        steps.append({"step": i, "t": t, "joints": joints})

    return steps


def print_trajectory(
    start_angles: list[float],
    goal_angles:  list[float],
    duration:     float,
    print_every:  int = 1,
) -> None:
    """
    각 스텝별 목표 각도(deg)와 목표 속도(deg/s)를 출력.

    Args:
        print_every: N 스텝마다 한 줄 출력 (기본 1 = 전체 출력)
    """
    steps = generate_trajectory(start_angles, goal_angles, duration)
    total = len(steps)

    # 헤더
    joint_header = "  ".join(f"{n:>22}" for n in JOINT_NAMES)
    print(f"{'step':>6}  {'t(s)':>6}  {joint_header}")
    print("-" * (6 + 2 + 6 + 2 + 24 * 7))

    for s in steps:
        if s["step"] % print_every != 0 and s["step"] != total - 1:
            continue
        joint_cols = "  ".join(
            f"({j['pos']:>8.3f},{j['vel']:>9.3f})" for j in s["joints"]
        )
        print(f"{s['step']:>6}  {s['t']:>6.3f}  {joint_cols}")

    print(f"\n총 {total} 스텝 @ {FREQ}Hz  (duration={duration:.2f}s)")


# ──────────────────────────────────────────
# 실행 예시
# ──────────────────────────────────────────
if __name__ == "__main__":
    start  = [0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
    goal   = [30.0, -15.0, 20.0, 10.0, 25.0, 45.0, 30.0]
    dur    = 3.0   # 3초

    # 전체 출력하면 너무 많으므로 10스텝마다 출력 (1로 바꾸면 전체)
    print_trajectory(start, goal, duration=dur, print_every=10)

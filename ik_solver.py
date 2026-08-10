"""
IKSolver - URDF 기반 순/역기구학 솔버 (Pinocchio 백엔드)
"""

import os
import math
import numpy as np

_DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "right_urdf", "right_urdf.urdf",
)


class IKSolver:
    """
    URDF 기반 순/역기구학 솔버 (Pinocchio 백엔드).

    active_joints 에 지정된 조인트만 IK 자유도로 사용하고 나머지는 0 고정.
    내부적으로 pin.buildReducedModel 로 비활성 조인트를 잠근 축소 모델을 생성한다.
    URDF 조인트명(joint_N) ↔ config 명(jointN) 자동 매핑.

    사용 예시::

        solver = IKSolver(active_joints=['joint1', 'joint2', 'joint3', 'joint4'])

        # 역기구학 (위치만)
        angles = solver.ik([0.3, 0.0, 0.4])
        # 역기구학 (위치 + 자세)
        angles = solver.ik([0.3, 0.0, 0.4], orientation=[0.0, 0.5, 0.0])
        # 초기값 힌트 제공
        angles = solver.ik([0.3, 0.0, 0.4], seed={'joint1': 0.1})

        # 순기구학
        pos, rot = solver.fk(angles)
        rpy = IKSolver.matrix_to_rpy(rot)

        # 도달 가능 여부 검증
        ok, pos_err, rot_err = solver.check_reachability([0.3, 0.0, 0.4])
    """

    _EE_FRAME = "link_end"

    def __init__(
        self,
        urdf_path: str = _DEFAULT_URDF,
        active_joints: list[str] | None = None,
    ):
        """
        Args:
            urdf_path:     URDF 파일 경로 (기본: 패키지 동봉 URDF)
            active_joints: IK 에서 자유도로 쓸 조인트 이름 목록 (config 명, 예: ['joint1', ...])
                           None 이면 URDF 의 모든 관절 사용
        """
        try:
            import pinocchio as pin
        except ImportError:
            raise ImportError("pinocchio 패키지가 필요합니다: pip install pin")

        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")

        self.urdf_path = urdf_path
        self._pin = pin

        # 전체 모델 로드
        model_full = pin.buildModelFromUrdf(urdf_path)

        # URDF 조인트명 → config 명 매핑 (universe 제외)
        # model.names: ['universe', 'joint_1', 'joint_2', ...]
        all_cfg: dict[str, int] = {}   # cfg_name → joint_id
        for jid in range(1, model_full.njoints):
            urdf_name = model_full.names[jid]
            cfg_name  = urdf_name.replace('_', '')      # "joint_1" → "joint1"
            if cfg_name.startswith('joint') and cfg_name[5:].isdigit():
                all_cfg[cfg_name] = jid

        if active_joints is None:
            active_joints = sorted(all_cfg.keys(), key=lambda n: int(n[5:]))
        self.active_joints: list[str] = active_joints

        # 비활성 조인트를 0 으로 잠근 축소 모델 생성
        lock_ids = [jid for cfg, jid in all_cfg.items() if cfg not in active_joints]
        q_ref = pin.neutral(model_full)
        self.model = pin.buildReducedModel(model_full, lock_ids, q_ref) if lock_ids else model_full
        self.data  = self.model.createData()

        # 엔드 이펙터 프레임 ID
        if not self.model.existFrame(self._EE_FRAME):
            available = [self.model.frames[i].name for i in range(self.model.nframes)]
            raise RuntimeError(
                f"URDF 에서 '{self._EE_FRAME}' 프레임을 찾을 수 없습니다. "
                f"사용 가능한 프레임: {available}"
            )
        self._ee_id = self.model.getFrameId(self._EE_FRAME)

        # 축소 모델의 조인트명 → q 인덱스 재매핑
        self._cfg_to_qidx: dict[str, int] = {}
        for jid in range(1, self.model.njoints):
            urdf_name = self.model.names[jid]
            cfg_name  = urdf_name.replace('_', '')
            if cfg_name in active_joints:
                self._cfg_to_qidx[cfg_name] = self.model.idx_qs[jid]

    # ── 순기구학 ─────────────────────────────────────────────────────────────

    def fk(self, angles: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        """
        순기구학: 조인트 각도 → 엔드 이펙터 위치/자세.

        Args:
            angles: {'joint1': rad, ...}

        Returns:
            position: shape (3,)    엔드 이펙터 위치 [mm]
            rotation: shape (3, 3)  엔드 이펙터 회전 행렬 (ZYX 기준)
        """
        pin = self._pin
        q   = self._to_q(angles)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        T = self.data.oMf[self._ee_id]
        return T.translation.copy() * 1000.0, T.rotation.copy()

    # ── 역기구학 ─────────────────────────────────────────────────────────────

    def ik(
        self,
        position:    list[float],
        orientation: list[float] | None = None,
        seed:        dict[str, float] | None = None,
        max_iter:    int   = 300,
        eps:         float = 1e-6,
        step:        float = 0.1,
        damp:        float = 1e-6,
    ) -> dict[str, float]:
        """
        역기구학: 목표 위치/자세 → 조인트 각도 (Jacobian 댐핑 반복법).

        오차와 Jacobian 모두 LOCAL_WORLD_ALIGNED (월드 프레임) 기준으로 통일.
        - 위치 오차: target.t - T_curr.t
        - 자세 오차: pin.log3(R_target @ R_curr^T)  ← 월드 프레임 회전 오차
        - 업데이트:  dq = J^T (J J^T + λI)^{-1} err  (양수 방향)

        Args:
            position:    [x, y, z] 목표 위치 [mm]
            orientation: [roll, pitch, yaw] 목표 자세 [rad] (ZYX 순서) —
                         None 이면 위치만 제어
            seed:        초기 관절 각도 힌트 {'joint1': rad, ...} — None 이면 영점
            max_iter:    최대 반복 횟수
            eps:         수렴 판정 오차 [m 또는 rad]
            step:        업데이트 스텝 크기
            damp:        댐핑 계수 (특이점 근방 수치 안정성)

        Returns:
            {'joint1': rad, ...}
        """
        pin = self._pin

        target = pin.SE3.Identity()
        target.translation = np.array(position, dtype=float) / 1000.0
        if orientation is not None:
            target.rotation = self.rpy_to_matrix(*orientation)

        q = self._to_q(seed) if seed is not None else pin.neutral(self.model)

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            T_curr = self.data.oMf[self._ee_id]

            # 오차 — 위치 + (선택) 자세 모두 월드 프레임 기준
            err_t = target.translation - T_curr.translation          # (3,)
            if orientation is not None:
                err_r = pin.log3(target.rotation @ T_curr.rotation.T)  # (3,)
                err   = np.concatenate([err_t, err_r])                  # (6,)
                n     = 6
            else:
                err = err_t                                            # (3,)
                n   = 3

            if np.linalg.norm(err) < eps:
                break

            # LOCAL_WORLD_ALIGNED Jacobian — 오차와 동일한 프레임
            J = pin.getFrameJacobian(
                self.model, self.data, self._ee_id, pin.LOCAL_WORLD_ALIGNED
            )[:n, :]                                                    # (n, nv)

            # 댐핑 Pseudo-inverse: dq = J^T (J J^T + λI)^{-1} err
            dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(n), err)
            q  = pin.integrate(self.model, q, step * dq)
            q  = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        return self._from_q(q)

    # ── 도달 가능성 검증 ──────────────────────────────────────────────────────

    def check_reachability(
        self,
        position:    list[float],
        orientation: list[float] | None = None,
        pos_tol:     float = 1.0,
        rot_tol:     float = 1e-2,
    ) -> tuple[bool, float, float]:
        """
        IK → FK 재검증으로 목표 자세 도달 가능 여부 확인.

        Args:
            position: [x, y, z] [mm]
            pos_tol:  허용 위치 오차 [mm]

        Returns:
            (reachable, pos_error [mm], rot_error [rad])
        """
        angles         = self.ik(position, orientation)
        fk_pos, fk_rot = self.fk(angles)

        pos_err = float(np.linalg.norm(np.array(position) - fk_pos))
        rot_err = 0.0
        if orientation is not None:
            R_target = self.rpy_to_matrix(*orientation)
            trace    = float(np.trace(R_target @ fk_rot.T))
            rot_err  = abs(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))

        return (pos_err <= pos_tol) and (rot_err <= rot_tol), pos_err, rot_err

    # ── 정적 유틸리티 ─────────────────────────────────────────────────────────

    @staticmethod
    def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """RPY 오일러 각도 → 3×3 회전 행렬 (ZYX: R = Rz·Ry·Rx)."""
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)
        return np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,             cp*cr            ],
        ])

    @staticmethod
    def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
        """3×3 회전 행렬 → (roll, pitch, yaw) [rad] (ZYX 순서, Gimbal lock 처리 포함)."""
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        if sy > 1e-6:
            roll  = math.atan2( R[2, 1],  R[2, 2])
            pitch = math.atan2(-R[2, 0],  sy)
            yaw   = math.atan2( R[1, 0],  R[0, 0])
        else:
            roll  = math.atan2(-R[1, 2],  R[1, 1])
            pitch = math.atan2(-R[2, 0],  sy)
            yaw   = 0.0
        return roll, pitch, yaw

    @staticmethod
    def quaternion_to_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
        """단위 쿼터니언 (w, x, y, z) → 3×3 회전 행렬."""
        return np.array([
            [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)    ],
            [2*(qx*qy + qz*qw),       1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)    ],
            [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)],
        ])

    # ── 내부 헬퍼 ────────────────────────────────────────────────────────────

    def _to_q(self, angles: dict[str, float] | None) -> np.ndarray:
        """{'joint1': rad, ...} → pinocchio q 벡터 (미지정은 0)."""
        q = self._pin.neutral(self.model)
        if angles:
            for cfg, val in angles.items():
                idx = self._cfg_to_qidx.get(cfg)
                if idx is not None:
                    q[idx] = val
        return q

    def _from_q(self, q: np.ndarray) -> dict[str, float]:
        """pinocchio q 벡터 → {'joint1': rad, ...} (active_joints 만)."""
        return {cfg: float(q[idx]) for cfg, idx in self._cfg_to_qidx.items()}


if __name__ == '__main__':
    import math

    solver = IKSolver()

    # ── FK 영점 확인 ──────────────────────────────────────────────────────────
    pos0, rot0 = solver.fk({})
    print(f'FK(q=0)  pos={pos0.round(4)}  rpy={tuple(round(r,4) for r in IKSolver.matrix_to_rpy(rot0))}')
    print()

    # ── 위치-only IK ─────────────────────────────────────────────────────────
    # 알려진 관절각 → FK → 그 위치를 IK로 역산하여 오차 검증
    test_configs = [
        {'joint1':  0.5},
        {'joint1':  0.5, 'joint2':  0.3},
        {'joint1': -0.5, 'joint2':  0.5, 'joint3':  0.3},
        {'joint1':  0.3, 'joint2':  0.3, 'joint3':  0.3, 'joint4':  0.3},
    ]
    print('[ 위치 IK ]')
    print(f'  {"target pos (mm)":38}  pos_err(mm)')
    for cfg in test_configs:
        tgt_pos, _ = solver.fk(cfg)          # [mm]
        _, pos_err, _ = solver.check_reachability(tgt_pos.tolist(), pos_tol=1.0)
        flag = 'OK' if pos_err < 1.0 else 'FAIL'
        print(f'  {str(tgt_pos.round(1)):38}  {pos_err:.2e}  {flag}')

    # ── 위치 + 자세 IK ────────────────────────────────────────────────────────
    print()
    print('[ 위치 + 자세 IK ]')
    for cfg in test_configs[:3]:
        tgt_pos, tgt_rot = solver.fk(cfg)    # pos [mm]
        rpy = list(IKSolver.matrix_to_rpy(tgt_rot))
        ok, pos_err, rot_err = solver.check_reachability(
            tgt_pos.tolist(), rpy, pos_tol=1.0, rot_tol=1e-2
        )
        print(f'  pos={tgt_pos.round(1)} mm  pos_err={pos_err:.2e} mm  rot_err={rot_err:.2e} rad  {"OK" if ok else "FAIL"}')
"""
GravityCompensator - URDF 질량/관성 기반 관절 중력 보상 토크 계산 (Pinocchio RNEA 백엔드)
"""

import os

from paths import URDF_RIGHT_ARM as _DEFAULT_URDF


class GravityCompensator:
    """
    현재 관절각에서 정적으로 버티기 위해 필요한 중력 보상 토크를 계산한다.

    IKSolver(kinematics/ik_solver.py)와 동일하게 URDF 조인트명(joint_N) ↔
    config 명(jointN) 매핑을 쓰고, pin.computeGeneralizedGravity(q̇=0, q̈=0인
    RNEA 특수해)로 정적 중력 토크를 구한다.

    사용 예시::

        comp = GravityCompensator(active_joints=['joint1', ..., 'joint7'])
        tau = comp.gravity_torque({'joint1': 0.3, 'joint2': 0.5})
        # {'joint1': Nm, 'joint2': Nm, ...} — 미지정 관절은 0 rad으로 취급
    """

    def __init__(
        self,
        urdf_path: str = _DEFAULT_URDF,
        active_joints: list[str] | None = None,
    ):
        """
        Args:
            urdf_path:     URDF 파일 경로 (기본: 패키지 동봉 URDF)
            active_joints: 중력 토크를 계산할 조인트 이름 목록 (config 명, 예: ['joint1', ...])
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

        model_full = pin.buildModelFromUrdf(urdf_path)

        # URDF 조인트명 → config 명 매핑 (universe 제외)
        all_cfg: dict[str, int] = {}   # cfg_name → joint_id
        for jid in range(1, model_full.njoints):
            urdf_name = model_full.names[jid]
            cfg_name  = urdf_name.replace('_', '')      # "joint_1" → "joint1"
            if cfg_name.startswith('joint') and cfg_name[5:].isdigit():
                all_cfg[cfg_name] = jid

        if active_joints is None:
            active_joints = sorted(all_cfg.keys(), key=lambda n: int(n[5:]))
        self.active_joints: list[str] = active_joints

        # 비활성 조인트를 0 으로 잠근 축소 모델 생성 (IKSolver와 동일 방식)
        lock_ids = [jid for cfg, jid in all_cfg.items() if cfg not in active_joints]
        q_ref = pin.neutral(model_full)
        self.model = pin.buildReducedModel(model_full, lock_ids, q_ref) if lock_ids else model_full
        self.data  = self.model.createData()

        # 축소 모델의 조인트명 → (q 인덱스, v/tau 인덱스) 재매핑.
        # 회전 조인트는 nq == nv == 1 이라 보통 같은 값이지만, tau는 nv 기준
        # 벡터이므로 idx_qs가 아니라 idx_vs로 인덱싱해야 맞다.
        self._cfg_to_qidx: dict[str, int] = {}
        self._cfg_to_vidx: dict[str, int] = {}
        for jid in range(1, self.model.njoints):
            urdf_name = self.model.names[jid]
            cfg_name  = urdf_name.replace('_', '')
            if cfg_name in active_joints:
                self._cfg_to_qidx[cfg_name] = self.model.idx_qs[jid]
                self._cfg_to_vidx[cfg_name] = self.model.idx_vs[jid]

    def gravity_torque(self, angles: dict[str, float]) -> dict[str, float]:
        """
        중력 보상 토크 계산.

        Args:
            angles: {'joint1': rad, ...} — active_joints 중 미지정 관절은 0 rad으로 취급

        Returns:
            {'joint1': Nm, ...} — 이 토크를 feedforward로 더해주면 중력에 의한
            처짐을 상쇄한다 (모터 CAN 프로토콜의 부호/축 방향 관례를 그대로 따름).
        """
        pin = self._pin
        q = pin.neutral(self.model)
        for cfg, val in angles.items():
            idx = self._cfg_to_qidx.get(cfg)
            if idx is not None:
                q[idx] = val

        tau = pin.computeGeneralizedGravity(self.model, self.data, q)
        return {cfg: float(tau[idx]) for cfg, idx in self._cfg_to_vidx.items()}

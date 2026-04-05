"""
IKSolver: pinocchio 기반 7DOF 로봇팔 역기구학 (시각화 없음)
"""

import os
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

_HERE         = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_URDF = os.path.join(_HERE, "7dof_arm_urdf", "7dof_arm_urdf.urdf")
_DEFAULT_MESH = os.path.join(_HERE, "7dof_arm_urdf")


class IKSolver:
    """
    Args:
        urdf_path:   URDF 파일 경로
        ee_frame:    엔드이펙터 프레임 이름
        n_steps:     IK 반복 횟수
        n_restarts:  IK 실패 시 랜덤 재시작 횟수
        pos_tol:     위치 허용 오차 [m]
        rot_tol:     자세 허용 오차 [rad]
        damping:     댐핑 계수 (특이점 회피)
        dq_max:      스텝당 최대 관절 변화량 [rad]
    """

    def __init__(
        self,
        urdf_path:  str   = _DEFAULT_URDF,
        ee_frame:   str   = "link_end",
        n_steps:    int   = 500,
        n_restarts: int   = 15,
        pos_tol:    float = 1e-3,
        rot_tol:    float = 1e-2,
        damping:    float = 1e-4,
        dq_max:     float = 0.02,
    ):
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, _DEFAULT_MESH
        )
        self.data       = self.model.createData()
        self.ee_id      = self.model.getFrameId(ee_frame)
        self.joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]

        self.n_steps    = n_steps
        self.n_restarts = n_restarts
        self.pos_tol    = pos_tol
        self.rot_tol    = rot_tol
        self.damping    = damping
        self.dq_max     = dq_max

        self.q_current  = pin.neutral(self.model)

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def solve(
        self,
        p_target: np.ndarray,
        R_target: np.ndarray,
    ) -> tuple[list[float] | None, float, float]:
        """
        보간 없이 즉시 IK.

        Returns:
            (q_deg_list, pos_err, rot_err)
            q_deg_list: 관절각 [deg] 7개 리스트, 실패 시 None
        """
        best_q, best_pe, best_re = None, np.inf, np.inf

        starts = [self.q_current.copy()] + [
            pin.randomConfiguration(self.model) for _ in range(self.n_restarts)
        ]

        for q0 in starts:
            q, pe, re = self._ik_single(q0, p_target, R_target)
            if pe < self.pos_tol and re < self.rot_tol:
                self.q_current = q.copy()
                return list(np.degrees(q)), pe, re
            if pe + re < best_pe + best_re:
                best_q, best_pe, best_re = q, pe, re

        if best_q is not None:
            self.q_current = best_q.copy()

        return None, best_pe, best_re

    # ── 유틸 ──────────────────────────────────────────────────────────────────

    @staticmethod
    def euler_to_rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        return R.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()

    def get_current_q_deg(self) -> list[float]:
        return list(np.degrees(self.q_current))

    # ── 내부 ──────────────────────────────────────────────────────────────────

    def _get_ee_pose(self) -> tuple[np.ndarray, np.ndarray]:
        pin.forwardKinematics(self.model, self.data, self.q_current)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def _ik_single(
        self,
        q0:       np.ndarray,
        p_target: np.ndarray,
        R_target: np.ndarray,
    ) -> tuple[np.ndarray, float, float]:
        q = q0.copy()
        for _ in range(self.n_steps):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            oMf   = self.data.oMf[self.ee_id]
            pe_vec = p_target - oMf.translation
            re_vec = pin.log3(R_target @ oMf.rotation.T)

            pe = float(np.linalg.norm(pe_vec))
            re = float(np.linalg.norm(re_vec))

            if pe < self.pos_tol and re < self.rot_tol:
                return q, pe, re

            J  = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            JT = J.T
            A  = J @ JT + self.damping * np.eye(6)
            dq = JT @ np.linalg.solve(A, np.hstack([pe_vec, re_vec]))
            q  = pin.integrate(self.model, q, np.clip(dq, -self.dq_max, self.dq_max))
            q  = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_id]
        pe  = float(np.linalg.norm(p_target - oMf.translation))
        re  = float(np.linalg.norm(pin.log3(R_target @ oMf.rotation.T)))
        return q, pe, re


# ── 실행 예시 ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    solver = IKSolver()

    p_target = np.array([0.1, 0.2, -0.3])
    R_target = IKSolver.euler_to_rotation(0, 0, 0)

    q_deg, pe, re = solver.solve(p_target, R_target)

    if q_deg is not None:
        print("IK 성공")
        for name, angle in zip(solver.joint_names, q_deg):
            print(f"  {name:20s}: {angle:8.3f} deg")
        print(f"  pos err: {pe*1000:.2f} mm")
        print(f"  rot err: {np.degrees(re):.3f} deg")
    else:
        print(f"IK 실패  pos err: {pe*1000:.2f} mm  rot err: {np.degrees(re):.3f} deg")

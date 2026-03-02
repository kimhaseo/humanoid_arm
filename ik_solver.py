"""
IK Solver 클래스 — 다른 코드에서 import하여 사용.

사용 예:
    from ik_solver import IKSolver
    solver = IKSolver()
    q_deg, pos_err, rot_err = solver.solve(target_pos, target_rot)
"""

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R


class IKSolver:
    def __init__(
        self,
        urdf_path: str  = "/Users/kimhaseo/workspace/humanoid_arm/7dof_urdf/7dof_urdf.urdf",
        ee_frame:  str  = "end_effector-v1",
        n_steps:   int  = 500,
        n_restarts: int = 15,
        pos_tol:  float = 1e-3,
        rot_tol:  float = 1e-2,
        damping:  float = 1e-4,
        dq_max:   float = 0.05,
    ):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.ee_id = self.model.getFrameId(ee_frame)
        self.joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]

        self.n_steps    = n_steps
        self.n_restarts = n_restarts
        self.pos_tol    = pos_tol
        self.rot_tol    = rot_tol
        self.damping    = damping
        self.dq_max     = dq_max

    # ── 외부에서 쓰는 메서드 ───────────────────────────────────────
    def solve(
        self,
        p_target: np.ndarray,   # [x, y, z] (m)
        R_target: np.ndarray,   # 3x3 rotation matrix
    ) -> tuple[dict | None, float, float]:
        """
        Returns:
            q_dict  (dict | None): {"joint1": deg, "joint2": deg, ...}, 실패 시 None
            pos_err (float)      : 위치 오차 (m)
            rot_err (float)      : 자세 오차 (rad)
        """
        best_q, best_pe, best_re = None, np.inf, np.inf

        starts = [pin.neutral(self.model)] + [
            pin.randomConfiguration(self.model) for _ in range(self.n_restarts)
        ]

        for q0 in starts:
            q, pe, re = self._ik_single(q0, p_target, R_target)
            if pe < self.pos_tol and re < self.rot_tol:
                return self._to_dict(np.degrees(q)), pe, re
            if pe + re < best_pe + best_re:
                best_q, best_pe, best_re = q, pe, re

        return None, best_pe, best_re

    @staticmethod
    def euler_to_rotation(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
        """Roll/Pitch/Yaw (degree) → 3x3 rotation matrix (XYZ 순서)."""
        return R.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()

    # ── 내부 메서드 ───────────────────────────────────────────────
    def _ik_single(self, q0, p_target, R_target):
        q = q0.copy()
        for _ in range(self.n_steps):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            oMf    = self.data.oMf[self.ee_id]
            pe_vec = p_target - oMf.translation
            re_vec = pin.log3(R_target @ oMf.rotation.T)

            pe = float(np.linalg.norm(pe_vec))
            re = float(np.linalg.norm(re_vec))

            if pe < self.pos_tol and re < self.rot_tol:
                return q, pe, re

            J  = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
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

    def _to_dict(self, q_deg: np.ndarray) -> dict:
        return dict(zip(self.joint_names, q_deg))


# ── 단독 실행 시 동작 확인용 ──────────────────────────────────────
if __name__ == "__main__":
    solver = IKSolver()

    target_pos = np.array([0.3, 0.0, 0.2])
    target_rot = IKSolver.euler_to_rotation(0, 0, 0)

    q_dict, pos_err, rot_err = solver.solve(target_pos, target_rot)

    if q_dict is not None:
        print("=== IK 성공 ===")
        for name, angle in q_dict.items():
            print(f"  {name:20s}: {angle:8.3f} °")
        print(f"\n위치 오차 : {pos_err*1000:.2f} mm")
        print(f"자세 오차 : {np.degrees(rot_err):.3f} °")
    else:
        print("=== IK 실패 ===")
        print(f"위치 오차 : {pos_err*1000:.2f} mm")
        print(f"자세 오차 : {np.degrees(rot_err):.3f} °")
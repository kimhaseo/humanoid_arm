import os
import time
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R, Slerp   # Slerp 추가
from pinocchio.visualize import MeshcatVisualizer

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_URDF = os.path.join(_HERE, "7dof_arm_urdf", "7dof_arm_urdf.urdf")
_DEFAULT_MESH_DIR = os.path.join(_HERE, "7dof_arm_urdf")


class IKSolver:
    def __init__(
        self,
        urdf_path: str = _DEFAULT_URDF,
        ee_frame: str = "link_end",
        n_steps: int = 500,
        n_restarts: int = 15,
        pos_tol: float = 1e-3,
        rot_tol: float = 1e-2,
        damping: float = 1e-4,
        dq_max: float = 0.02,
        meshcat: bool = True,
        mesh_dir: str | None = _DEFAULT_MESH_DIR,
    ):
        self.urdf_path = urdf_path
        self.mesh_dir = mesh_dir

        if mesh_dir is not None:
            self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                urdf_path, mesh_dir
            )
        else:
            self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                urdf_path
            )

        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId(ee_frame)
        self.joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]

        self.n_steps = n_steps
        self.n_restarts = n_restarts
        self.pos_tol = pos_tol
        self.rot_tol = rot_tol
        self.damping = damping
        self.dq_max = dq_max

        # ★ 현재 관절 상태 추적 (보간 연속성을 위해)
        self.q_current = pin.neutral(self.model)

        self.viz = None
        if meshcat:
            self._init_meshcat()

    def _init_meshcat(self):
        self.viz = MeshcatVisualizer(
            self.model,
            self.collision_model,
            self.visual_model,
        )
        self.viz.initViewer(open=True)
        self.viz.loadViewerModel()

        R = pin.rpy.rpyToMatrix(0, 0, 0)

        T_base = np.eye(4)
        T_base[:3, :3] = R
        T_base[2, 3] = 0.5
        self.viz.viewer[self.viz.viewerRootNodeName].set_transform(T_base)
        self.viz.display(self.q_current)

    def display_q_rad(self, q_rad: np.ndarray):
        if self.viz is not None:
            self.viz.display(q_rad)

    def display_q_deg_dict(self, q_deg_dict: dict):
        q_rad = pin.neutral(self.model)
        for i, name in enumerate(self.joint_names):
            if name in q_deg_dict:
                q_rad[i] = np.radians(q_deg_dict[name])
        self.display_q_rad(q_rad)

    # ─────────────────────────────────────────────────────────────
    # ★ 핵심 추가: Cartesian 보간 이동
    # ─────────────────────────────────────────────────────────────

    def _get_current_ee_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """현재 q_current 기준 EE 위치·회전 반환"""
        pin.forwardKinematics(self.model, self.data, self.q_current)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def move_to(
        self,
        p_target: np.ndarray,
        R_target: np.ndarray,
        n_interp: int = 60,
        dt: float = 0.03,
        step_callback=None,
    ) -> tuple[dict | None, float, float]:
        """
        현재 EE 포즈 → 목표까지 Cartesian 보간(LERP + SLERP)으로
        부드럽게 이동. 각 웨이포인트마다 IK를 계산해 시각화.

        Parameters
        ----------
        n_interp : 보간 스텝 수 (많을수록 부드러움, 느려짐)
        dt       : 프레임 간격 [초]
        """
        # ★ 실패 시 복원을 위해 현재 상태 저장
        q_saved = self.q_current.copy()

        p_start, R_start = self._get_current_ee_pose()

        # ★ 시작 전 meshcat을 q_current와 동기화 (이전 애니메이션 끝 자세와 불일치 방지)
        if self.viz is not None:
            self.viz.display(self.q_current)

        # SLERP 준비 (scipy Slerp: 두 키프레임 사이를 보간)
        rot_start = R.from_matrix(R_start)
        rot_end   = R.from_matrix(R_target)
        slerp_fn  = Slerp([0.0, 1.0], R.concatenate([rot_start, rot_end]))

        q = self.q_current.copy()
        pe, re = np.inf, np.inf

        for i in range(1, n_interp + 1):
            t = i / n_interp

            # 위치: 선형 보간 (LERP)
            p_interp = (1.0 - t) * p_start + t * p_target

            # 회전: 구면 선형 보간 (SLERP)
            R_interp = slerp_fn(t).as_matrix()

            # Warm start — 직전 웨이포인트 결과를 초기값으로 사용
            q, pe, re = self._ik_single(q, p_interp, R_interp)

            # 매 스텝 관절각 콜백 (모터 실시간 전달 등)
            if step_callback is not None:
                step_callback(self._to_dict(np.degrees(q)))

            if self.viz is not None:
                self.viz.display(q)
            time.sleep(dt)

        # 루프 끝 후 추가 display 없음 — 마지막 프레임(q)이 곧 q_current
        # (추가 display가 있으면 best_q로 snap-back되어 순간이동처럼 보임)
        if pe < self.pos_tol and re < self.rot_tol:
            self.q_current = q.copy()
            return self._to_dict(np.degrees(q)), pe, re

        # 실패: q_current를 호출 전 상태로 복원 (오염 방지)
        self.q_current = q_saved
        if self.viz is not None:
            self.viz.display(self.q_current)
        return None, pe, re

    # ─────────────────────────────────────────────────────────────

    def solve(
        self,
        p_target: np.ndarray,
        R_target: np.ndarray,
    ) -> tuple[dict | None, float, float]:
        """보간 없이 즉시 IK (q_current 기준 초기값 사용)"""
        best_q, best_pe, best_re = None, np.inf, np.inf

        # neutral 대신 q_current를 첫 번째 초기값으로 사용
        starts = [self.q_current.copy()] + [
            pin.randomConfiguration(self.model) for _ in range(self.n_restarts)
        ]

        for q0 in starts:
            q, pe, re = self._ik_single(q0, p_target, R_target)
            if pe < self.pos_tol and re < self.rot_tol:
                self.q_current = q.copy()
                if self.viz is not None:
                    self.viz.display(q)
                return self._to_dict(np.degrees(q)), pe, re

            if pe + re < best_pe + best_re:
                best_q, best_pe, best_re = q, pe, re

        if best_q is not None:
            self.q_current = best_q.copy()
            if self.viz is not None:
                self.viz.display(best_q)

        return None, best_pe, best_re

    @staticmethod
    def euler_to_rotation(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
        return R.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()

    def _ik_single(self, q0, p_target, R_target):
        q = q0.copy()
        for _ in range(self.n_steps):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            oMf = self.data.oMf[self.ee_id]
            pe_vec = p_target - oMf.translation
            re_vec = pin.log3(R_target @ oMf.rotation.T)

            pe = float(np.linalg.norm(pe_vec))
            re = float(np.linalg.norm(re_vec))

            if pe < self.pos_tol and re < self.rot_tol:
                return q, pe, re

            J = pin.computeFrameJacobian(
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
        pe = float(np.linalg.norm(p_target - oMf.translation))
        re = float(np.linalg.norm(pin.log3(R_target @ oMf.rotation.T)))
        return q, pe, re

    def _to_dict(self, q_deg: np.ndarray) -> dict:
        return dict(zip(self.joint_names, q_deg))


if __name__ == "__main__":
    solver = IKSolver(meshcat=True)

    target_pos = np.array([0.1, 0.2, -0.3])
    target_rot = IKSolver.euler_to_rotation(0, 0, 0)

    print("=== 보간 이동 시작 ===")
    q_dict, pos_err, rot_err = solver.move_to(
        target_pos, target_rot,
        n_interp=60,  # 웨이포인트 수 (많을수록 부드러움)
        dt=0.03,      # 프레임 간격 [초]
    )

    if q_dict is not None:
        print("=== IK 성공 ===")
        for name, angle in q_dict.items():
            print(f"{name:20s}: {angle:8.3f} deg")
        print(f"pos err : {pos_err*1000:.2f} mm")
        print(f"rot err : {np.degrees(rot_err):.3f} deg")
    else:
        print("=== IK 실패 ===")
        print(f"pos err : {pos_err*1000:.2f} mm")
        print(f"rot err : {np.degrees(rot_err):.3f} deg")

    input("\nMeshCat 시각화 중... Enter를 누르면 종료합니다.")
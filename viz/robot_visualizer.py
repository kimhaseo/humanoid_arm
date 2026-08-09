"""
RobotVisualizer - MeshCat 기반 URDF 로봇 실시간 시각화
"""

import os
import numpy as np

from paths import URDF_RIGHT_ARM as _DEFAULT_URDF


class RobotVisualizer:
    """
    MeshCat 기반 URDF 로봇 실시간 시각화.

    단독으로 FK 확인용으로 쓰거나, JointController 에 넘기면
    move_joint / move_pose 실행 중 궤적을 실시간으로 렌더링한다.

    사용 예시::

        # 단독 사용
        vis = RobotVisualizer()
        vis.display({'joint1': 0.5, 'joint2': 0.3})
        print(vis.url)   # 브라우저에서 열 URL

        # JointController 연결
        vis  = RobotVisualizer(open_browser=False)
        ctrl = JointController(visualizer=vis)
        ctrl.move_joint('joint1', 0.5)   # 하드웨어 + 시각화 동시
    """

    def __init__(
        self,
        urdf_path: str = _DEFAULT_URDF,
        open_browser: bool = True,
    ):
        """
        Args:
            urdf_path:    URDF 파일 경로
            open_browser: True 이면 초기화 시 브라우저 자동 오픈
        """
        try:
            import pinocchio as pin
            from pinocchio.visualize import MeshcatVisualizer
        except ImportError:
            raise ImportError("pinocchio 패키지가 필요합니다: pip install pin")

        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")

        self._pin     = pin
        urdf_dir      = os.path.dirname(os.path.abspath(urdf_path))
        package_dirs  = [urdf_dir]

        # 키네마틱 모델 + 시각 모델 로드
        self.model        = pin.buildModelFromUrdf(urdf_path)
        self.visual_model = pin.buildGeomFromUrdf(
            self.model, urdf_path,
            pin.GeometryType.VISUAL,
            package_dirs=package_dirs,
        )
        self.data        = self.model.createData()
        self.visual_data = self.visual_model.createData()

        # config명(jointN) → q 인덱스 매핑
        self._cfg_to_qidx: dict[str, int] = {}
        for jid in range(1, self.model.njoints):
            urdf_name = self.model.names[jid]
            cfg_name  = urdf_name.replace('_', '')
            if cfg_name.startswith('joint') and cfg_name[5:].isdigit():
                self._cfg_to_qidx[cfg_name] = self.model.idx_qs[jid]

        # MeshCat 뷰어 초기화
        self._vis = MeshcatVisualizer(
            self.model,
            self.visual_model,   # collision 자리에 visual 재사용 (표시 전용)
            self.visual_model,
        )
        self._vis.initViewer(open=open_browser)
        self._vis.loadViewerModel()
        self._q = pin.neutral(self.model)   # 현재 표시 상태 유지
        self._vis.display(self._q)

    # ── 공개 API ──────────────────────────────────────────────────────────────

    @property
    def url(self) -> str:
        """MeshCat 뷰어 URL (브라우저에서 직접 열 수 있음)."""
        raw = self._vis.viewer.url()
        return raw if isinstance(raw, str) else str(raw)

    def display(self, angles: dict[str, float]):
        """
        조인트 각도로 로봇 자세 업데이트.

        Args:
            angles: {'joint1': rad, ...}  — 미지정 조인트는 이전 상태 유지
        """
        for cfg, val in angles.items():
            idx = self._cfg_to_qidx.get(cfg)
            if idx is not None:
                self._q[idx] = val
        self._vis.display(self._q)

    def set_background(
        self,
        top:    tuple[float, float, float] = (0.1, 0.1, 0.1),
        bottom: tuple[float, float, float] = (0.05, 0.05, 0.05),
    ):
        """
        배경색 설정. RGB 값은 0.0 ~ 1.0.

        Args:
            top:    상단 색상 (기본: 어두운 회색)
            bottom: 하단 색상 (기본: 더 어두운 회색)
        """
        self._vis.viewer["/Background"].set_property("top_color",    list(top))
        self._vis.viewer["/Background"].set_property("bottom_color", list(bottom))

    def reset(self):
        """모든 조인트를 영점으로 리셋."""
        self._q = self._pin.neutral(self.model)
        self._vis.display(self._q)



if __name__ == '__main__':
    import time
    import math

    vis = RobotVisualizer()
    print(f'MeshCat URL: {vis.url}')

    # joint1 을 -0.5 ~ 0.5 rad 사이로 스윕
    print('joint1 스윕 시작...')
    for i in range(200):
        t = i / 100 * math.pi
        vis.display({'joint1': math.sin(t) * 0.5})
        time.sleep(0.02)

    vis.reset()
    print('완료.')

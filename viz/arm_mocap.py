"""
arm_mocap.py — 카메라로 오른팔 동작을 캡처해 7-DOF 로봇 팔로 MeshCat 시각화.

필요 패키지:
    pip install mediapipe opencv-python numpy

사용:
    python arm_mocap.py
    브라우저에서 출력된 MeshCat URL을 열면 실시간 시각화가 보임.
"""

import math
import time
import numpy as np
import cv2

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    from mediapipe.tasks.python.components import containers as mp_containers
except ImportError:
    raise ImportError("mediapipe 패키지 필요: pip install mediapipe")

from viz.robot_visualizer import RobotVisualizer


# ── 각도 유틸 ─────────────────────────────────────────────────────────────────

def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """두 단위벡터 사이 각도 [rad]."""
    return float(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1.0, 1.0)))


def signed_angle(v: np.ndarray, ref: np.ndarray, axis: np.ndarray) -> float:
    """
    axis를 기준으로 v → ref 방향 부호 있는 각도 [rad].
    (axis 방향에서 봤을 때 반시계=양수)
    """
    cross = np.cross(unit(v), unit(ref))
    sign  = np.sign(np.dot(cross, unit(axis)))
    return float(sign * np.arccos(np.clip(np.dot(unit(v), unit(ref)), -1.0, 1.0)))


# ── 7-DOF 관절각 변환 ──────────────────────────────────────────────────────────

def compute_7dof_angles(lm) -> dict[str, float]:
    """
    MediaPipe world_landmarks → 7-DOF 관절각 [rad].

    URDF 조인트 의미 (axis 기반 근사):
      joint1: 어깨 수평 회전 (adduction/abduction in transverse plane)
      joint2: 어깨 수직 회전 (flexion/extension)
      joint3: 어깨 내/외 회전 (internal/external rotation, arm roll)
      joint4: 팔꿈치 굴곡 (elbow flexion — 항상 >= 0)
      joint5: 전완 회내/회외 (pronation/supination)
      joint6: 손목 굴곡 (wrist flexion/extension)
      joint7: 손목 요골/척골 편위 (radial/ulnar deviation)

    MediaPipe world 좌표계: x=오른쪽, y=위, z=카메라 방향(앞)
    """
    # ── 랜드마크 추출 (world 좌표 [m]) ────────────────────────────────────────
    def pt(idx):
        p = lm[idx]
        return np.array([p.x, p.y, p.z], dtype=float)

    r_shoulder = pt(12)
    r_elbow    = pt(14)
    r_wrist    = pt(16)
    r_index    = pt(20)
    r_pinky    = pt(18)
    r_thumb    = pt(22)
    l_shoulder = pt(11)

    # ── 기준 좌표계 (torso) ───────────────────────────────────────────────────
    # 어깨 중심에서 오른쪽 어깨를 가리키는 벡터를 오른팔 기준 x축으로 사용
    torso_x = unit(r_shoulder - l_shoulder)  # 오른쪽
    torso_y = np.array([0.0, 1.0, 0.0])      # 위 (중력 반대)
    torso_z = unit(np.cross(torso_x, torso_y))  # 앞쪽

    # ── 상완 벡터 (어깨→팔꿈치) ───────────────────────────────────────────────
    upper_arm  = r_elbow - r_shoulder
    ua_unit    = unit(upper_arm)

    # ── joint1: 어깨 수평 abduction (transverse plane, torso_y 기준 회전) ────
    # 상완을 torso_x–torso_z 수평면에 투영, 참조벡터(torso_z=앞) 대비 각도
    ua_horiz  = unit(np.array([ua_unit[0], 0.0, ua_unit[2]]))  # y성분 제거
    j1 = signed_angle(np.array([0.0, 0.0, -1.0]), ua_horiz, torso_y)
    # 오른팔이 옆으로 뻗으면 ~90°, 앞이면 ~0°

    # ── joint2: 어깨 수직 elevation (frontal plane) ──────────────────────────
    # 상완 단위벡터의 y성분 → elevation angle
    j2 = float(np.arcsin(np.clip(-ua_unit[1], -1.0, 1.0)))
    # 팔이 아래=음수, 수평=0, 위=양수. 아래로 내리면 음수니까 부호 맞춤

    # ── joint3: 어깨 internal/external rotation (arm roll) ──────────────────
    # 상완 축을 기준으로 전완 방향의 roll
    forearm     = r_wrist - r_elbow
    fa_unit     = unit(forearm)

    # 상완 축에 수직인 평면에서 전완 방향 추적
    # 참조: 상완 축(ua_unit)에 수직이고 torso_y에 가까운 벡터
    up_perp    = unit(torso_y - ua_unit * np.dot(torso_y, ua_unit))
    fa_perp    = unit(fa_unit - ua_unit * np.dot(fa_unit, ua_unit))
    j3 = signed_angle(up_perp, fa_perp, ua_unit)

    # ── joint4: 팔꿈치 굴곡 ─────────────────────────────────────────────────
    j4 = math.pi - angle_between(upper_arm, forearm)
    # 펴진 상태=0, 완전 굴곡~π

    # ── 전완 축 및 손 벡터 ────────────────────────────────────────────────────
    hand_mid  = (r_index + r_pinky) * 0.5
    hand_vec  = unit(hand_mid - r_wrist)

    # ── joint5: 전완 회내/회외 (pronation/supination) ────────────────────────
    # 전완 축 기준, 엄지 방향의 roll
    thumb_vec  = unit(r_thumb - r_wrist)
    fa_perp2   = unit(thumb_vec - fa_unit * np.dot(thumb_vec, fa_unit))

    # 전완 수직 참조 (fa_unit ⊥ 전완방향으로 torso_y 투영)
    up_perp2   = unit(torso_y - fa_unit * np.dot(torso_y, fa_unit))
    if np.linalg.norm(fa_perp2) > 1e-9:
        j5 = signed_angle(up_perp2, fa_perp2, fa_unit)
    else:
        j5 = 0.0

    # ── joint6: 손목 굴곡 (wrist flexion/extension) ─────────────────────────
    j6 = angle_between(forearm, hand_vec) - math.pi / 2
    # 손이 전완과 일직선=0, 굴곡=음수, 신전=양수

    # ── joint7: 손목 radial/ulnar deviation ─────────────────────────────────
    # 엄지 vs 새끼 손가락 높이 차이를 이용한 lateral deviation
    # thumb이 위면 radial deviation(양수), pinky가 위면 ulnar deviation(음수)
    knuckle_vec = unit(r_index - r_pinky)
    hand_normal = unit(np.cross(fa_unit, knuckle_vec))
    j7 = signed_angle(fa_unit, hand_vec, hand_normal) - math.pi / 2

    # ── 클리핑 (-π ~ π) ──────────────────────────────────────────────────────
    angles = {'joint1': j1, 'joint2': j2, 'joint3': j3,
              'joint4': j4, 'joint5': j5, 'joint6': j6, 'joint7': j7}
    for k, v in angles.items():
        angles[k] = float(np.clip(v, -math.pi, math.pi))
    return angles


# ── 스무딩 필터 ───────────────────────────────────────────────────────────────

class ExponentialFilter:
    """1차 지수 이동평균 스무딩 (alpha=1이면 필터 없음)."""

    def __init__(self, alpha: float = 0.25):
        self._alpha = alpha
        self._state: dict[str, float] | None = None

    def update(self, angles: dict[str, float]) -> dict[str, float]:
        if self._state is None:
            self._state = dict(angles)
            return dict(angles)
        a = self._alpha
        self._state = {k: a * v + (1 - a) * self._state.get(k, v)
                       for k, v in angles.items()}
        return dict(self._state)


# ── 캘리브레이션 ──────────────────────────────────────────────────────────────

class Calibrator:
    """현재 자세를 0도 기준으로 저장하고 이후 각도에서 빼준다."""

    def __init__(self):
        self._offset: dict[str, float] | None = None

    def calibrate(self, angles: dict[str, float]):
        """현재 각도를 0도 기준으로 설정."""
        self._offset = dict(angles)

    def apply(self, angles: dict[str, float]) -> dict[str, float]:
        """offset을 뺀 상대 각도 반환 (캘리브레이션 전이면 그대로)."""
        if self._offset is None:
            return angles
        import math
        result = {}
        for k, v in angles.items():
            diff = v - self._offset.get(k, 0.0)
            # -π ~ π 범위로 wrap
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            result[k] = diff
        return result

    @property
    def is_set(self) -> bool:
        return self._offset is not None


# ── 모델 다운로드 ─────────────────────────────────────────────────────────────

def _ensure_pose_model() -> str:
    """pose_landmarker_full.task 모델을 없으면 다운로드해서 경로 반환."""
    import urllib.request, os
    from paths import POSE_MODEL_PATH as model_path
    if not os.path.isfile(model_path):
        url = ("https://storage.googleapis.com/mediapipe-models/"
               "pose_landmarker/pose_landmarker_full/float16/latest/"
               "pose_landmarker_full.task")
        print(f"모델 다운로드 중: {url}")
        urllib.request.urlretrieve(url, model_path)
        print("다운로드 완료.")
    return model_path


# ── 메인 루프 ─────────────────────────────────────────────────────────────────

def main(camera_index: int = 0, alpha: float = 0.3):
    """
    Args:
        camera_index: cv2.VideoCapture 카메라 인덱스
        alpha:        스무딩 계수 (0=최대 스무딩, 1=스무딩 없음)
    """
    # MeshCat 초기화
    print("MeshCat 초기화 중...")
    vis   = RobotVisualizer(open_browser=True)
    filt  = ExponentialFilter(alpha=alpha)
    calib = Calibrator()
    print(f"MeshCat URL: {vis.url}")
    print("브라우저에서 위 URL을 여세요.")
    print("단축키: [c] 현재 자세를 0도 기준으로 설정  [r] 캘리브레이션 초기화  [q] 종료\n")

    # MediaPipe Pose Landmarker 모델 다운로드 (최초 1회)
    model_path = _ensure_pose_model()

    # PoseLandmarker (Tasks API) 초기화
    base_opts = mp_python.BaseOptions(model_asset_path=model_path)
    opts = mp_vision.PoseLandmarkerOptions(
        base_options=base_opts,
        running_mode=mp_vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.6,
        min_pose_presence_confidence=0.6,
        min_tracking_confidence=0.6,
        output_segmentation_masks=False,
    )
    landmarker = mp_vision.PoseLandmarker.create_from_options(opts)

    # 카메라 열기 (Windows: DSHOW 백엔드가 안정적)
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"카메라 {camera_index}를 열 수 없습니다.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 카메라 워밍업 (초기 몇 프레임 버림)
    for _ in range(5):
        cap.read()

    print("카메라 시작. 'q' 키를 누르면 종료합니다.\n")

    prev_time = time.time()
    ts_ms     = 0
    key       = 0

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("프레임 읽기 실패 — 재시도 중...")
            time.sleep(0.05)
            continue

        # MediaPipe는 원본(비반전) 프레임으로 처리 → 좌/우 올바르게 인식
        rgb    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Tasks API: mp.Image + 타임스탬프
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        ts_ms += 33   # 약 30fps 가정 (단조 증가 필요)
        result = landmarker.detect_for_video(mp_img, ts_ms)

        if result.pose_world_landmarks:
            wlm = result.pose_world_landmarks[0]   # 첫 번째 사람

            try:
                raw_angles    = compute_7dof_angles(wlm)
                smooth_angles = filt.update(raw_angles)
                display_angles = calib.apply(smooth_angles)
                vis.display(display_angles)

                # 화면에 관절값 표시
                y = 20
                for jname, val in display_angles.items():
                    text = f"{jname}: {math.degrees(val):+6.1f} deg"
                    cv2.putText(frame, text, (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 80), 1)
                    y += 18

                # 캘리브레이션 상태 표시
                status = "CAL SET" if calib.is_set else "NO CAL  [c]=set"
                color  = (0, 200, 255) if calib.is_set else (0, 140, 255)
                cv2.putText(frame, status, (10, y + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # 'c' 키: 현재 스무딩된 각도를 0도 기준으로 저장
                if key == ord('c'):
                    calib.calibrate(smooth_angles)
                    print("캘리브레이션 완료 — 현재 자세가 0도 기준입니다.")

                # 'r' 키: 캘리브레이션 초기화
                if key == ord('r'):
                    calib._offset = None
                    print("캘리브레이션 초기화.")

            except Exception as e:
                cv2.putText(frame, f"Err: {e}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        else:
            cv2.putText(frame, "포즈 감지 안됨 - 카메라 앞에 서주세요",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

        # FPS 표시
        now  = time.time()
        fps  = 1.0 / max(now - prev_time, 1e-9)
        prev_time = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (frame.shape[1] - 100, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 표시할 때만 거울 반전 (자연스러운 셀카 뷰)
        frame = cv2.flip(frame, 1)

        # 스켈레톤 오버레이 — flip 후 x좌표 반전
        if result.pose_landmarks:
            ilm = result.pose_landmarks[0]
            h, w = frame.shape[:2]
            pts = {i: (w - 1 - int(lm.x * w), int(lm.y * h)) for i, lm in enumerate(ilm)}
            arm_connections = [(11,12),(12,14),(14,16),(16,18),(16,20),(16,22),(18,20)]
            for a, b in arm_connections:
                if a in pts and b in pts:
                    cv2.line(frame, pts[a], pts[b], (200, 80, 200), 2)
            for idx in [11,12,14,16,18,20,22]:
                if idx in pts:
                    cv2.circle(frame, pts[idx], 4, (80, 200, 80), -1)

        cv2.imshow("Arm MoCap  (q: quit)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    print("종료.")


if __name__ == "__main__":
    main(camera_index=0, alpha=0.3)

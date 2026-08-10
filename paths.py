"""
프로젝트 전역 절대 경로 상수.

모듈이 서브패키지(canbus/, motors/, kinematics/, ...)로 옮겨져도
__file__ 기준 상대 경로가 깨지지 않도록, 데이터 파일 경로는 항상 여기서 가져온다.
"""
import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

URDF_RIGHT_ARM = os.path.join(PROJECT_ROOT, "right_urdf", "right_urdf.urdf")

SCENARIOS_DIR   = os.path.join(PROJECT_ROOT, "scenarios")
POSE_MODEL_PATH = os.path.join(PROJECT_ROOT, "pose_landmarker_full.task")

"""
STT 텍스트 → 로봇 명령 파서
직접 명령(regex)과 LLM CMD 블록 파싱을 모두 담당한다.
"""

import json
import math
import re
from typing import Optional

# ── 조인트 별칭 ────────────────────────────────────────────────────────────────

_ALIAS = {
    '1': 'joint1', '2': 'joint2', '3': 'joint3',
    '4': 'joint4', '5': 'joint5', '6': 'joint6', '7': 'joint7',
    '일': 'joint1', '이': 'joint2', '삼': 'joint3',
    '사': 'joint4', '오': 'joint5', '육': 'joint6', '칠': 'joint7',
    '첫': 'joint1', '첫번째': 'joint1',
}

# ── 정규식 패턴 ────────────────────────────────────────────────────────────────

# "1번 관절 45도" / "관절 2번 -30도"
_JOINT_ANGLE = re.compile(
    r'(?:(\d+|일|이|삼|사|오|육|칠)\s*번?\s*관절|관절\s*(\d+|일|이|삼|사|오|육|칠)\s*번?)'
    r'\s*(마이너스|minus|-|음수)?\s*(\d+(?:\.\d+)?)\s*도',
    re.IGNORECASE,
)

_ENABLE  = re.compile(r'(모터|관절|팔|로봇)\s*(켜|활성화|시작|on)', re.IGNORECASE)
_DISABLE = re.compile(r'(모터|관절|팔|로봇)\s*(꺼|비활성화|정지|멈춰|멈춤|off)', re.IGNORECASE)
_ZERO    = re.compile(r'(영점|제로)\s*(설정|잡|초기화)?', re.IGNORECASE)
_HOME    = re.compile(r'홈|초기\s*자세|원점\s*자세|기본\s*자세|집\s*자세', re.IGNORECASE)

# LLM이 출력하는 CMD 블록: [CMD:{"joint1": 0.785}]
_CMD_BLOCK = re.compile(r'\[CMD:\s*(\{.*?\})\]', re.DOTALL)


# ── 공개 API ───────────────────────────────────────────────────────────────────

def parse_voice(text: str) -> Optional[dict]:
    """
    STT 텍스트에서 직접 명령을 파싱한다.
    반환 형식:
        {'type': 'set_joints',  'angles': {'joint1': rad, ...}}
        {'type': 'enable_all'}
        {'type': 'disable_all'}
        {'type': 'set_zero_all'}
        {'type': 'home'}
    매칭 없으면 None.
    """
    matches = _JOINT_ANGLE.findall(text)
    if matches:
        angles = {}
        for g1, g2, neg, deg_str in matches:
            key   = g1 or g2
            jname = _ALIAS.get(key, f'joint{key}')
            deg   = float(deg_str) * (-1 if neg else 1)
            angles[jname] = math.radians(deg)
        if angles:
            return {'type': 'set_joints', 'angles': angles}

    if _ENABLE.search(text):
        return {'type': 'enable_all'}
    if _DISABLE.search(text):
        return {'type': 'disable_all'}
    if _ZERO.search(text):
        return {'type': 'set_zero_all'}
    if _HOME.search(text):
        return {'type': 'home'}

    return None


def parse_llm(text: str) -> tuple[str, Optional[dict]]:
    """
    LLM 응답에서 [CMD:{...}] 블록을 추출한다.
    반환: (speech_text, cmd_dict | None)
    speech_text는 CMD 블록이 제거된 순수 발화 텍스트.
    """
    m = _CMD_BLOCK.search(text)
    if not m:
        return text, None
    try:
        cmd = json.loads(m.group(1))
        speech = _CMD_BLOCK.sub('', text).strip()
        # angles 키가 있으면 set_joints, 아니면 type 키로 판단
        if 'angles' in cmd:
            cmd.setdefault('type', 'set_joints')
        return speech, cmd
    except json.JSONDecodeError:
        return text, None


def execute(robot, cmd: dict) -> str:
    """
    파싱된 명령을 JointController에 실행하고 음성 응답 문자열 반환.
    robot이 None이면 '로봇 연결 안 됨' 반환.
    """
    if robot is None:
        return "로봇이 연결돼 있지 않아."

    t = cmd.get('type')

    if t == 'set_joints':
        angles = cmd.get('angles', {})
        if not angles:
            return "어느 관절인지 못 알아들었어."
        robot.set_joints(angles=angles)
        parts = [f"{j} {math.degrees(a):.0f}도" for j, a in angles.items()]
        return f"{', '.join(parts)} 이동했어."

    if t == 'enable_all':
        robot.enable_all()
        return "전체 모터 켰어."

    if t == 'disable_all':
        robot.disable_all()
        return "전체 모터 껐어."

    if t == 'set_zero_all':
        robot.set_zero_all()
        return "영점 설정 완료."

    if t == 'home':
        robot.set_joints(angles={f'joint{i}': 0.0 for i in range(1, 8)})
        return "홈 포지션으로 이동했어."

    return "명령을 실행했어."

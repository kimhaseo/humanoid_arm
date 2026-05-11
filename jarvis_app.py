"""
J.A.R.V.I.S  —  Full-Screen HUD
Backend : whisper + ollama + pyaudio  (separate process)
Frontend: PyQt6 + QPainter animations
"""

import multiprocessing as mp
import sys, os, math, random

# ══════════════════════════════════════════════════════════════════════════════
#  BACKEND PROCESS
# ══════════════════════════════════════════════════════════════════════════════

def backend_main(ui_q: mp.Queue, _cmd_q: mp.Queue):
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
    _d = os.path.join(os.environ.get("LOCALAPPDATA",""), "Programs","Ollama")
    if _d not in os.environ.get("PATH",""):
        os.environ["PATH"] = _d + os.pathsep + os.environ.get("PATH","")

    import asyncio, re, tempfile, threading, time, logging, traceback
    from collections import deque
    from datetime import datetime

    import edge_tts, numpy as np, ollama, sounddevice as sd
    from faster_whisper import WhisperModel

    logging.basicConfig(
        filename="jarvis.log",
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        encoding="utf-8",
    )
    log = logging.getLogger("jarvis")

    SR=16000; CH=1; CHUNK=int(SR*0.1)
    THR=0.005; SIL=0.7; MIN_DUR=0.3; MAX_REC=30.0
    MIC="UM-200"; WM="small"; VOICE="ko-KR-HyunsuNeural"; RATE="+25%"; LLM="qwen2.5:14b"
    PROMPT=("너는 자비스야. 나한테 완전 친한 친구처럼 말해.\n"
            "반말 쓰고, 편하게 툭툭 던지듯이 얘기해줘.\n"
            "이모지, 이모티콘, 특수문자 절대 쓰지 마.\n"
            "2문장 이내로 짧고 빠릿하게 답해. 영어 쓰지 마.\n"
            "로봇팔 조인트(joint1~joint4)를 라디안 단위로 제어할 수 있어. "
            "팔 움직임 명령이 오면 move_joints 도구를 사용해. "
            "enable_motors, disable_motors로 모터 켜고 끄기 가능.")

    # ── 로봇팔 Tool 정의 ──────────────────────────────────────────────────────────
    ARM_TOOLS = [
        {
            "type": "function",
            "function": {
                "name": "move_joints",
                "description": "로봇팔 조인트를 목표 각도로 이동. 각도 단위: 라디안.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "angles": {
                            "type": "object",
                            "description": "조인트 이름과 목표 각도(rad). 예: {\"joint1\": 0.5}",
                            "additionalProperties": {"type": "number"}
                        }
                    },
                    "required": ["angles"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "enable_motors",
                "description": "로봇팔 전체 모터 활성화.",
                "parameters": {"type": "object", "properties": {}}
            }
        },
        {
            "type": "function",
            "function": {
                "name": "disable_motors",
                "description": "로봇팔 전체 모터 비활성화.",
                "parameters": {"type": "object", "properties": {}}
            }
        },
        {
            "type": "function",
            "function": {
                "name": "get_joint_states",
                "description": "현재 조인트 상태(각도, 속도, 토크, 온도) 조회.",
                "parameters": {"type": "object", "properties": {}}
            }
        },
    ]

    def dispatch_tool(tc, arm):
        name = tc.function.name
        args = tc.function.arguments if hasattr(tc.function, "arguments") else {}
        if name == "move_joints":
            angles = args.get("angles", {})
            arm.set_joints(angles=angles)
            readable = ", ".join(f"{k}={v:.3f}rad" for k, v in angles.items())
            log.info(f"move_joints 실행: {readable}")
            return f"이동 완료: {readable}"
        elif name == "enable_motors":
            arm.enable_all()
            log.info("enable_motors 실행")
            return "모터 활성화 완료"
        elif name == "disable_motors":
            arm.disable_all()
            log.info("disable_motors 실행")
            return "모터 비활성화 완료"
        elif name == "get_joint_states":
            states = arm.get_states()
            lines = [f"{n}: {s['angle']:.3f}rad" for n, s in states.items()]
            result = "현재 상태: " + ", ".join(lines)
            log.info(result)
            return result
        return "알 수 없는 명령"

    import re as _re
    _EMOJI_RE = _re.compile(
        "[""\U0001F600-\U0001FFFF""\U00002700-\U000027BF"
        "\U0000200D""\U00002500-\U00002BEF""☀-⛿""]+"
    )
    def clean(t):
        t = _EMOJI_RE.sub("", t)
        t = _re.sub(r"[*_~`#>|\\]", "", t)
        return _re.sub(r"\s{2,}", " ", t).strip()
    EXIT={"종료","꺼줘","시스템 종료","exit","quit","bye"}
    SENT=re.compile(r'(?<=[.?!~。？！\n])\s*')

    def send(m): ui_q.put(m)
    def rms(c):  return float(np.sqrt(np.mean(c**2)))

    class TTS:
        def __init__(self):
            self._q=deque(); self._lk=threading.Lock(); self._busy=False
            threading.Thread(target=self._run,daemon=True).start()
        def put(self,t):
            with self._lk: self._q.append(t)
        def done(self):
            while True:
                with self._lk:
                    if not self._q and not self._busy: return
                time.sleep(0.05)
        def _run(self):
            while True:
                t=None
                with self._lk:
                    if self._q: t=self._q.popleft(); self._busy=True
                if t:
                    try: asyncio.run(self._speak(t))
                    except: pass
                    with self._lk: self._busy=False
                else: time.sleep(0.02)
        @staticmethod
        async def _speak(text):
            text=clean(text)
            if not text: return
            comm=edge_tts.Communicate(text,VOICE,rate=RATE)
            with tempfile.NamedTemporaryFile(suffix=".mp3",delete=False) as f: tmp=f.name
            await comm.save(tmp)
            import pygame
            pygame.mixer.init(); pygame.mixer.music.load(tmp); pygame.mixer.music.play()
            while pygame.mixer.music.get_busy(): await asyncio.sleep(0.05)
            pygame.mixer.quit(); os.unlink(tmp)

    def find_mic():
        devices = sd.query_devices()
        candidates = [(i, d) for i, d in enumerate(devices) if d["max_input_channels"] > 0]
        log.info(f"입력 장치 목록: {[(i, d['name']) for i, d in candidates]}")
        for i, d in candidates:
            if MIC.lower() in d["name"].lower():
                log.info(f"마이크 선택: [{i}] {d['name']}")
                return i
        log.warning(f"'{MIC}' 없음, 기본 장치 사용")
        send({"t":"status","v":f"MIC '{MIC}' 없음 - 기본 장치 사용"})
        default = sd.default.device[0]
        if isinstance(default, int) and 0 <= default < len(devices):
            return default
        return candidates[0][0] if candidates else None

    send({"t":"status","v":"INIT"})
    whisper=WhisperModel(WM,device="cpu",compute_type="int8")
    client=ollama.Client(); mic=find_mic()
    tts=TTS(); hist=[]

    # ── 로봇팔 초기화 ──────────────────────────────────────────────────────────
    arm = None
    try:
        from joint_controller import JointController
        arm = JointController()
        arm.enable_all()
        log.info("JointController 초기화 성공")
        send({"t":"status","v":"ARM_READY"})
    except Exception as _arm_err:
        log.warning(f"JointController 초기화 실패: {_arm_err}")
        send({"t":"status","v":f"ARM_OFF:{_arm_err}"})

    now=lambda: __import__('datetime').datetime.now().strftime("%H:%M:%S")
    greeting="yo"
    send({"t":"ai","ts":now(),"text":greeting})
    tts.put(greeting); tts.done(); send({"t":"status","v":"WAIT"})

    while True:
        try:
            send({"t":"status","v":"WAIT"})
            buf=[]; sil_t=None; speaking=False; t0=None
            _open_dev = mic
            try:
                _test = sd.InputStream(samplerate=SR,channels=CH,dtype="float32",
                                       blocksize=CHUNK,device=_open_dev)
                _test.close()
            except Exception as _mic_open_err:
                log.warning(f"마이크 [{mic}] 열기 실패: {_mic_open_err} → 기본 장치로 재시도")
                mic = None; _open_dev = None
            with sd.InputStream(samplerate=SR,channels=CH,dtype="float32",
                                blocksize=CHUNK,device=_open_dev) as stream:
                while not speaking:
                    c,_=stream.read(CHUNK); c=c[:,0]
                    send({"t":"mic","v":rms(c)})
                    if rms(c)>=THR:
                        speaking=True; t0=__import__('time').time()
                        buf.append(c.copy()); send({"t":"status","v":"REC"})
                while True:
                    c,_=stream.read(CHUNK); c=c[:,0]
                    send({"t":"mic","v":rms(c)}); buf.append(c.copy())
                    if rms(c)<THR:
                        if sil_t is None: sil_t=__import__('time').time()
                        elif __import__('time').time()-sil_t>=SIL: break
                    else: sil_t=None
                    if __import__('time').time()-t0>MAX_REC: break
            send({"t":"mic","v":0.})

            audio=np.concatenate(buf)
            if len(audio)/SR<MIN_DUR: continue

            send({"t":"status","v":"STT"})
            segs,_=whisper.transcribe(audio,language="ko",beam_size=5,vad_filter=True,
                                      initial_prompt="안녕하세요. 한국어로 말하고 있습니다.")
            text=" ".join(s.text for s in segs).strip()
            if not text: continue
            send({"t":"user","ts":now(),"text":text})
            if any(k in text.lower() for k in EXIT):
                fw="응 알겠어. 나중에 또 불러."
                send({"t":"ai","ts":now(),"text":fw}); tts.put(fw); tts.done()
                send({"t":"done"}); break

            hist.append({"role":"user","content":text})
            msgs=[{"role":"system","content":PROMPT}]+hist
            send({"t":"status","v":"LLM"}); send({"t":"stream","v":""})
            log.info(f"사용자 입력: {text}")

            log.debug(f"LLM 호출 (tools={'ON' if arm else 'OFF'})")
            resp = client.chat(
                model=LLM,
                messages=msgs,
                tools=ARM_TOOLS if arm else None,
            )
            log.debug(f"LLM 응답: content={repr(resp.message.content)}, tool_calls={resp.message.tool_calls}")

            if arm and resp.message.tool_calls:
                log.info(f"Tool 호출: {[tc.function.name for tc in resp.message.tool_calls]}")
                tool_msgs = msgs + [resp.message]
                for tc in resp.message.tool_calls:
                    result = dispatch_tool(tc, arm)
                    tool_msgs.append({"role": "tool", "content": result, "name": tc.function.name})
                resp = client.chat(model=LLM, messages=tool_msgs)

            full = resp.message.content or ""
            full_clean = full.strip()
            send({"t":"stream","v":full_clean})

            # 문장 단위로 TTS 전송
            first = True
            for s in SENT.split(full_clean):
                if s.strip():
                    tts.put(s.strip())
                    if first: send({"t":"status","v":"TTS"}); first=False

            hist.append({"role":"assistant","content":full})
            send({"t":"stream","v":""}); send({"t":"ai","ts":now(),"text":full_clean})
            tts.done(); send({"t":"status","v":"WAIT"})
        except Exception as e:
            log.error(f"메인 루프 오류:\n{traceback.format_exc()}")
            if "PortAudioError" in type(e).__name__ or "sounddevice" in str(e).lower():
                log.warning("마이크 오류 — 장치 재탐색")
                mic = find_mic()
            send({"t":"status","v":f"ERR:{e})"}); __import__('time').sleep(1)

    # 루프 종료 후 팔 정리
    if arm:
        try:
            arm.disable_all(clear_error=True)
            arm.shutdown()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
#  UI PROCESS
# ══════════════════════════════════════════════════════════════════════════════

# legacy colour strings kept for backend status labels (not used in UI painting)
GREEN = "#4AFFA0"
RED   = "#FF4D6A"
AMBER = "#FFB347"


def start_ui(ui_q: mp.Queue):
    os.environ["QT_OPENGL"] = "software"
    from PyQt6.QtCore    import Qt, QTimer, QRectF, QPointF
    from PyQt6.QtGui     import (QPainter, QPen, QColor, QBrush, QFont,
                                  QLinearGradient, QRadialGradient, QPolygonF, QKeyEvent)
    from PyQt6.QtWidgets import QApplication, QWidget
    from datetime        import datetime

    # ── JARVIS colour palette ─────────────────────────────────────────────────
    C1  = QColor(103, 199, 235)        # #67C7EB  primary cyan
    C2  = QColor( 62, 154, 184)        # #3E9AB8  secondary
    C3  = QColor( 26,  80, 112)        # #1A5070  dim
    CA  = QColor(139, 211, 251)        # #8BD3FB  accent bright
    CW  = QColor(200, 238, 255)        # near-white
    CG  = QColor( 74, 255, 160)        # green
    CR  = QColor(255,  77, 106)        # red
    CAM = QColor(255, 179,  71)        # amber
    BK  = QColor(  2,   8,  16)        # background

    class HUDCanvas(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("J.A.R.V.I.S")
            self.setStyleSheet(f"background:rgb(2,8,16);")
            self._t        = 0.0
            self._scan_y   = 0
            self._wave     = [0.0] * 140
            self._hex_data = [[f"{random.randint(0,0xFFFF):04X}" for _ in range(4)] for _ in range(30)]
            self._status     = "STANDBY"
            self._status_col = CG
            self._metrics    = {"POWER":0.88,"NEURAL":0.72,"SYNC":0.65,"AUDIO":0.30}

            self._anim = QTimer(self); self._anim.timeout.connect(self._tick); self._anim.start(33)
            self._poll = QTimer(self); self._poll.timeout.connect(self._drain); self._poll.start(30)

        def _drain(self):
            for _ in range(20):
                try: msg = ui_q.get_nowait()
                except: break
                t = msg["t"]
                if t == "status":
                    mp = {"INIT":("INITIALIZING",C2),"WAIT":("STANDBY",CG),
                          "REC": ("RECORDING",   CR),"STT":("PROCESSING",CAM),
                          "LLM": ("ANALYZING",   C1),"TTS":("TRANSMITTING",CA)}
                    self._status, self._status_col = mp.get(msg["v"], (msg["v"], C1))
                elif t == "mic":
                    v = msg["v"]
                    self._wave.append(v)
                    if len(self._wave) > 140: self._wave.pop(0)
                    self._metrics["AUDIO"] = min(1.0, v * 10 + 0.2)
                elif t == "done":
                    self.close()

        def _tick(self):
            self._t      += 0.033
            self._scan_y  = (self._scan_y + 2) % max(self.height(), 1)
            self._metrics["POWER"]  = 0.82 + math.sin(self._t*0.4)*0.12
            self._metrics["NEURAL"] = 0.68 + math.sin(self._t*0.9)*0.18
            self._metrics["SYNC"]   = 0.60 + math.sin(self._t*1.3)*0.22
            for _ in range(4):
                r = random.randint(0, len(self._hex_data)-1)
                c = random.randint(0, len(self._hex_data[0])-1)
                self._hex_data[r][c] = f"{random.randint(0,0xFFFF):04X}"
            self.update()

        def paintEvent(self, _):
            p = QPainter(self)
            p.setRenderHint(QPainter.RenderHint.Antialiasing)
            w, h = self.width(), self.height()
            cx, cy = w // 2, h // 2
            R = min(w, h) // 2 - 28

            p.fillRect(0, 0, w, h, BK)
            self._vignette(p, cx, cy, w, h)
            self._hex_grid(p, w, h)
            self._scan(p, w)
            self._spokes(p, cx, cy, R)
            self._ref_circles(p, cx, cy, R)
            self._radar_sweep(p, cx, cy, R, 1.0)
            self._radar_sweep(p, cx, cy, int(R*0.52), -0.55)
            self._rings(p, cx, cy, R)
            self._arc_segments(p, cx, cy, R)
            self._target_brackets(p, cx, cy, R)
            self._center_core(p, cx, cy)
            self._waveform(p, cx, cy, h, R)
            self._side_panel(p, w, h, cx, cy, R, left=True)
            self._side_panel(p, w, h, cx, cy, R, left=False)
            self._top_bar(p, w, cx)
            self._bottom_bar(p, w, h, cx)
            self._corners(p, w, h)
            p.end()

        # ── vignette ─────────────────────────────────────────────────

        def _vignette(self, p, cx, cy, w, h):
            g = QRadialGradient(cx, cy, max(w, h) * 0.7)
            g.setColorAt(0.0, QColor(10, 30, 55, 0))
            g.setColorAt(0.6, QColor(2,   8, 16, 0))
            g.setColorAt(1.0, QColor(0,   0,  0, 180))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(g))
            p.drawRect(0, 0, w, h)
            p.setBrush(Qt.BrushStyle.NoBrush)

        # ── hex grid (center-area only) ───────────────────────────────

        def _hex_grid(self, p, w, h):
            sz = 52
            pen = QPen(QColor(26, 80, 112, 22)); pen.setWidth(1); p.setPen(pen)
            for row in range(-1, int(h/(sz*0.866))+2):
                for col in range(-1, int(w/(sz*1.5))+2):
                    hx = col * sz * 1.5
                    hy = row * sz * 1.732 + (sz*0.866 if col%2 else 0)
                    pts = QPolygonF([QPointF(hx + sz*0.48*math.cos(math.pi/3*i),
                                             hy + sz*0.48*math.sin(math.pi/3*i)) for i in range(6)])
                    p.drawPolygon(pts)

        # ── scan line ────────────────────────────────────────────────

        def _scan(self, p, w):
            sy = self._scan_y
            c1 = C1; c1a = QColor(c1); c1a.setAlpha(0)
            c1b = QColor(c1); c1b.setAlpha(12)
            g = QLinearGradient(0, sy-70, 0, sy+70)
            g.setColorAt(0.0, c1a); g.setColorAt(0.5, c1b); g.setColorAt(1.0, c1a)
            p.fillRect(0, sy-70, w, 140, g)

        # ── radial spokes ─────────────────────────────────────────────

        def _spokes(self, p, cx, cy, R):
            n = 48
            for i in range(n):
                a = math.radians(i*(360/n) + self._t*2)
                major = (i % 6 == 0)
                c = QColor(C2); c.setAlpha(40 if major else 12)
                pen = QPen(c); pen.setWidth(1); p.setPen(pen)
                r_in  = R * (0.18 if major else 0.28)
                r_out = R * (0.96 if major else 0.68)
                p.drawLine(QPointF(cx+r_in*math.cos(a),  cy+r_in*math.sin(a)),
                           QPointF(cx+r_out*math.cos(a), cy+r_out*math.sin(a)))

        # ── faint reference circles ───────────────────────────────────

        def _ref_circles(self, p, cx, cy, R):
            for frac, alpha in [(1.0,14),(0.76,18),(0.54,22),(0.32,28)]:
                c = QColor(C2); c.setAlpha(alpha)
                pen = QPen(c); pen.setWidth(1); p.setPen(pen)
                p.drawEllipse(QPointF(cx, cy), R*frac, R*frac)

        # ── radar sweep (reusable) ────────────────────────────────────

        def _radar_sweep(self, p, cx, cy, R, speed=1.0):
            deg = (self._t * 48 * speed) % 360
            trail, steps = 80, 24
            for i in range(steps):
                a = deg - i*(trail/steps)
                al = int((1-i/steps)**2.5 * 45)
                c = QColor(C1); c.setAlpha(al)
                p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(c))
                p.drawPie(QRectF(cx-R,cy-R,R*2,R*2), int(a*16), int(-(trail/steps)*16))
            p.setBrush(Qt.BrushStyle.NoBrush)
            sr = math.radians(deg)
            lc = QColor(CA if speed > 0 else C2); lc.setAlpha(160 if speed > 0 else 100)
            pen = QPen(lc); pen.setWidth(1); p.setPen(pen)
            p.drawLine(QPointF(cx,cy), QPointF(cx+R*math.cos(sr), cy+R*math.sin(sr)))

        # ── rotating arcs ─────────────────────────────────────────────

        def _rings(self, p, cx, cy, R):
            # (radius_frac, linewidth, speed_deg/s, arc_span_deg, color, n_ticks)
            cfgs = [
                (1.00, 2,  18, 260, C1,  20),
                (0.84, 1, -13, 185, C2,   0),
                (0.70, 2,  30, 140, C1,  12),
                (0.57, 1, -48, 100, C2,   0),
                (0.44, 2,  72,  75, C1,   8),
                (0.32, 1,-115,  52, C3,   0),
                (0.21, 1, 170,  36, C1,   0),
            ]
            for frac, lw, spd, span, col, n_ticks in cfgs:
                r   = R * frac
                deg = (self._t * spd) % 360
                # subtle glow
                gc = QColor(col); gc.setAlpha(12)
                gpen = QPen(gc); gpen.setWidth(lw+5)
                gpen.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(gpen)
                p.drawArc(QRectF(cx-r,cy-r,r*2,r*2), int(deg*16), int(span*16))
                # main arc
                mc = QColor(col); mc.setAlpha(200)
                mpen = QPen(mc); mpen.setWidth(lw)
                mpen.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(mpen)
                p.drawArc(QRectF(cx-r,cy-r,r*2,r*2), int(deg*16), int(span*16))
                # counter arc
                cc = QColor(col); cc.setAlpha(70)
                cpen = QPen(cc); cpen.setWidth(1)
                cpen.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(cpen)
                p.drawArc(QRectF(cx-r,cy-r,r*2,r*2), int((deg+180)*16), int(span*0.3*16))
                # tick marks
                if n_ticks:
                    tc = QColor(col); tc.setAlpha(150)
                    tpen = QPen(tc); tpen.setWidth(1); p.setPen(tpen)
                    for i in range(n_ticks):
                        a = math.radians(deg + i*(360/n_ticks))
                        p.drawLine(QPointF(cx+(r-8)*math.cos(a), cy+(r-8)*math.sin(a)),
                                   QPointF(cx+(r+8)*math.cos(a), cy+(r+8)*math.sin(a)))

        # ── outer segmented ARC-reactor ring ─────────────────────────

        def _arc_segments(self, p, cx, cy, R):
            n   = 36
            r   = R * 1.06
            charge = math.sin(self._t * 1.1) * 0.5 + 0.5
            lit    = int(n * charge)
            for i in range(n):
                a1 = math.radians(-90 + i*(360/n))
                a2 = math.radians(-90 + (i+0.65)*(360/n))
                span = int((a2-a1)*180/math.pi*16)
                col = QColor(CA if i < lit else C3)
                col.setAlpha(200 if i < lit else 35)
                pen = QPen(col); pen.setWidth(3 if i < lit else 1)
                pen.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(pen)
                p.drawArc(QRectF(cx-r,cy-r,r*2,r*2), int(a1*180/math.pi*16), span)

        # ── 4-corner targeting brackets ───────────────────────────────

        def _target_brackets(self, p, cx, cy, R):
            slide = abs(math.sin(self._t*0.7)) * 14 + 4
            r  = R * 0.70 + slide
            sz = 20
            c  = QColor(CA); c.setAlpha(220)
            pen = QPen(c); pen.setWidth(2); p.setPen(pen)
            for ang in [45, 135, 225, 315]:
                a  = math.radians(ang)
                bx = cx + r * math.cos(a)
                by = cy + r * math.sin(a)
                for da in [-28, 28]:
                    ta = math.radians(ang + da)
                    p.drawLine(QPointF(bx, by),
                               QPointF(bx + sz*math.cos(ta), by + sz*math.sin(ta)))

        # ── center ARC-reactor core ───────────────────────────────────

        def _center_core(self, p, cx, cy):
            # Radial ambient glow
            g = QRadialGradient(cx, cy, 75)
            g.setColorAt(0.0, QColor(103, 199, 235, 55))
            g.setColorAt(0.5, QColor( 62, 154, 184, 20))
            g.setColorAt(1.0, QColor(  0,   0,   0,  0))
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(g))
            p.drawEllipse(QPointF(cx, cy), 75, 75)
            p.setBrush(Qt.BrushStyle.NoBrush)

            # Concentric pulse rings
            for r, base in [(46,16),(34,28),(22,45),(13,70)]:
                pulse = abs(math.sin(self._t*2.5 + r*0.08))
                c = QColor(C1); c.setAlpha(int(base*(0.5+0.5*pulse)))
                pen = QPen(c); pen.setWidth(1); p.setPen(pen)
                p.drawEllipse(QPointF(cx,cy), r, r)

            # Rotating 6-spoke inner wheel
            for i in range(6):
                a = math.radians(self._t*80 + i*60)
                r1, r2 = 14, 30
                c = QColor(C1); c.setAlpha(180)
                pen = QPen(c); pen.setWidth(2); p.setPen(pen)
                p.drawLine(QPointF(cx+r1*math.cos(a), cy+r1*math.sin(a)),
                           QPointF(cx+r2*math.cos(a), cy+r2*math.sin(a)))

            # Bright core dot
            pr = 7 + abs(math.sin(self._t*5)) * 5
            ig = QRadialGradient(cx, cy, pr)
            ig.setColorAt(0.0, QColor(220, 245, 255, 255))
            ig.setColorAt(0.7, QColor(103, 199, 235, 160))
            ig.setColorAt(1.0, QColor(  0,   0,   0,   0))
            p.setBrush(QBrush(ig)); p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(cx,cy), pr, pr)
            p.setBrush(Qt.BrushStyle.NoBrush)

        # ── waveform ─────────────────────────────────────────────────

        def _waveform(self, p, cx, cy, h, R):
            if len(self._wave) < 2: return
            ww   = min(cx - 40, R + 60)
            x0   = cx - ww
            y0   = h - 72
            amp  = 42
            step = ww*2 / (len(self._wave)-1)

            # baseline
            c3 = QColor(C3); c3.setAlpha(80)
            pen = QPen(c3); pen.setWidth(1); p.setPen(pen)
            p.drawLine(QPointF(x0, y0), QPointF(x0+ww*2, y0))

            # main waveform
            c1 = QColor(C1); c1.setAlpha(190)
            pen = QPen(c1); pen.setWidth(2); p.setPen(pen)
            pts = [QPointF(x0+i*step, y0-self._wave[i]*amp) for i in range(len(self._wave))]
            for i in range(len(pts)-1): p.drawLine(pts[i], pts[i+1])

            # reflection
            c2 = QColor(C2); c2.setAlpha(40)
            pen2 = QPen(c2); pen2.setWidth(1); p.setPen(pen2)
            pts2 = [QPointF(x0+i*step, y0+self._wave[i]*amp*0.28) for i in range(len(self._wave))]
            for i in range(len(pts2)-1): p.drawLine(pts2[i], pts2[i+1])

        # ── side information panels ───────────────────────────────────

        def _side_panel(self, p, w, h, cx, cy, R, left=True):
            # Panel sits beside the main circle, not overlapping
            margin  = 24
            p_w     = max(180, cx - R - margin * 2)
            x0      = margin if left else w - margin - p_w
            y0      = cy - 180
            p_h     = 360

            # Glass panel background
            bg = QColor(10, 28, 48, 140)
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(bg))
            p.drawRoundedRect(int(x0), int(y0), int(p_w), int(p_h), 4, 4)
            p.setBrush(Qt.BrushStyle.NoBrush)

            # Panel border
            bc = QColor(C2); bc.setAlpha(70)
            pen = QPen(bc); pen.setWidth(1); p.setPen(pen)
            p.drawRoundedRect(int(x0), int(y0), int(p_w), int(p_h), 4, 4)

            # Top accent line
            ac = QColor(C1); ac.setAlpha(160)
            pen2 = QPen(ac); pen2.setWidth(2); p.setPen(pen2)
            p.drawLine(int(x0+6), int(y0), int(x0+p_w-6), int(y0))

            # Section title
            p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
            tc = QColor(CA); tc.setAlpha(180)
            p.setPen(QPen(tc))
            title = "SYSTEMS" if left else "SENSORS"
            p.drawText(QRectF(x0, y0+10, p_w, 18), Qt.AlignmentFlag.AlignCenter, title)

            # Divider
            dc = QColor(C3); dc.setAlpha(80)
            pen3 = QPen(dc); pen3.setWidth(1); p.setPen(pen3)
            p.drawLine(int(x0+10), int(y0+30), int(x0+p_w-10), int(y0+30))

            # Metric bars
            iy = y0 + 44
            bar_w = p_w - 28
            bx = x0 + 14
            metrics = (list(self._metrics.items()) if left
                       else [("MIC LVL",  self._metrics["AUDIO"]),
                             ("LATENCY",  0.88 - abs(math.sin(self._t*0.6))*0.3),
                             ("UPTIME",   min(1.0, self._t/120)),
                             ("ACCURACY", 0.92 + math.sin(self._t*0.4)*0.06)])

            for label, val in metrics:
                val = max(0.0, min(1.0, val))
                # label + value
                p.setFont(QFont("Consolas", 8))
                lc = QColor(C2); lc.setAlpha(160)
                p.setPen(QPen(lc))
                p.drawText(int(bx), int(iy), label)
                vc = QColor(CW); vc.setAlpha(200)
                p.setPen(QPen(vc))
                p.drawText(QRectF(bx, iy-14, bar_w, 16),
                           Qt.AlignmentFlag.AlignRight, f"{int(val*100)}%")
                # track
                p.setPen(Qt.PenStyle.NoPen)
                bg2 = QColor(C3); bg2.setAlpha(50)
                p.setBrush(QBrush(bg2))
                p.drawRoundedRect(int(bx), int(iy+3), int(bar_w), 4, 2, 2)
                # fill
                fill = QColor(C1 if val > 0.5 else CAM); fill.setAlpha(200)
                p.setBrush(QBrush(fill))
                p.drawRoundedRect(int(bx), int(iy+3), int(bar_w*val), 4, 2, 2)
                p.setBrush(Qt.BrushStyle.NoBrush)
                iy += 58

            # Hex data stream at bottom of panel
            p.setFont(QFont("Consolas", 7))
            for ri in range(4):
                row_y = y0 + p_h - 80 + ri*18
                a = int(15 + abs(math.sin(self._t*0.5+ri*0.4))*45)
                dc2 = QColor(C2); dc2.setAlpha(a)
                p.setPen(QPen(dc2))
                row = self._hex_data[ri % len(self._hex_data)]
                line = "  ".join(row[:3])
                p.drawText(QRectF(x0+8, row_y, p_w-16, 16),
                           Qt.AlignmentFlag.AlignCenter, line)

        # ── top bar ───────────────────────────────────────────────────

        def _top_bar(self, p, w, cx):
            # Title
            pulse = 155 + int(abs(math.sin(self._t*0.9))*100)
            tc = QColor(C1); tc.setAlpha(pulse)
            p.setFont(QFont("Consolas", 20, QFont.Weight.Bold))
            p.setPen(QPen(tc))
            p.drawText(QRectF(cx-280, 12, 560, 38),
                       Qt.AlignmentFlag.AlignCenter, "J · A · R · V · I · S")

            # Subtitle
            sc = QColor(C2); sc.setAlpha(90)
            p.setFont(QFont("Consolas", 8))
            p.setPen(QPen(sc))
            p.drawText(QRectF(cx-280, 50, 560, 16),
                       Qt.AlignmentFlag.AlignCenter,
                       "JUST  A  RATHER  VERY  INTELLIGENT  SYSTEM")

            # Clock (top-right)
            p.setFont(QFont("Consolas", 13, QFont.Weight.Bold))
            cc = QColor(CA); cc.setAlpha(190)
            p.setPen(QPen(cc))
            p.drawText(w-185, 30, datetime.now().strftime("%H:%M:%S"))
            p.setFont(QFont("Consolas", 8))
            dc = QColor(C2); dc.setAlpha(100)
            p.setPen(QPen(dc))
            p.drawText(w-185, 48, datetime.now().strftime("%Y . %m . %d"))

            # Build tag (top-left)
            p.setFont(QFont("Consolas", 8))
            bc = QColor(C2); bc.setAlpha(110)
            p.setPen(QPen(bc))
            for i, txt in enumerate(["MARK  VII", "BUILD  ACTIVE",
                                      f"SYS  {int(self._t*0.5)%9999:04d}"]):
                p.drawText(22, 28 + i*17, txt)

        # ── bottom bar ────────────────────────────────────────────────

        def _bottom_bar(self, p, w, h, cx):
            sc = QColor(self._status_col)
            pulse = abs(math.sin(self._t*2.8))
            # glow behind text
            gc = QColor(self._status_col); gc.setAlpha(int(15+pulse*22))
            p.setFont(QFont("Consolas", 14, QFont.Weight.Bold))
            p.setPen(QPen(gc))
            p.drawText(QRectF(cx-300, h-44, 600, 28),
                       Qt.AlignmentFlag.AlignCenter, f"◈  {self._status}  ◈")
            sc.setAlpha(int(140+pulse*115))
            p.setPen(QPen(sc))
            p.drawText(QRectF(cx-300, h-44, 600, 28),
                       Qt.AlignmentFlag.AlignCenter, f"◈  {self._status}  ◈")

            # thin bottom line
            lc = QColor(C3); lc.setAlpha(80)
            pen = QPen(lc); pen.setWidth(1); p.setPen(pen)
            p.drawLine(int(cx-200), h-14, int(cx+200), h-14)

        # ── corner brackets ───────────────────────────────────────────

        def _corners(self, p, w, h):
            L, m = 42, 18
            c1 = QColor(C1); c1.setAlpha(200)
            pen = QPen(c1); pen.setWidth(2); p.setPen(pen)
            for (x,y,dx,dy) in [(m,m,1,1),(w-m,m,-1,1),(m,h-m,1,-1),(w-m,h-m,-1,-1)]:
                p.drawLine(x, y, x+dx*L, y)
                p.drawLine(x, y, x, y+dy*L)
            ext = int(5 + abs(math.sin(self._t*1.6))*14)
            c2 = QColor(C2); c2.setAlpha(90)
            pen2 = QPen(c2); pen2.setWidth(1); p.setPen(pen2)
            for (x,y,dx,dy) in [(m,m,1,1),(w-m,m,-1,1),(m,h-m,1,-1),(w-m,h-m,-1,-1)]:
                p.drawLine(x+dx*L, y, x+dx*(L+ext), y)
                p.drawLine(x, y+dy*L, x, y+dy*(L+ext))

        def keyPressEvent(self, e: QKeyEvent):
            if e.key() in (Qt.Key.Key_Escape, Qt.Key.Key_Q):
                self.close()

        def closeEvent(self, e):
            self._anim.stop(); self._poll.stop()
            e.accept()

    app = QApplication(sys.argv)
    win = HUDCanvas()
    win.showMaximized()
    sys.exit(app.exec())


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    mp.freeze_support()
    ui_q  = mp.Queue()
    cmd_q = mp.Queue()
    backend = mp.Process(target=backend_main, args=(ui_q, cmd_q), daemon=True)
    backend.start()
    start_ui(ui_q)
    backend.terminate()

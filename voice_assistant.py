"""
JARVIS - 음성 인식 AI 비서 (Full-Screen HUD)

STT : faster-whisper  (로컬)
LLM : Ollama          (스트리밍)
TTS : edge-tts        (문장별 병렬)
UI  : rich            (풀스크린)
"""

import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

_ollama_dir = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Ollama")
if _ollama_dir not in os.environ.get("PATH", ""):
    os.environ["PATH"] = _ollama_dir + os.pathsep + os.environ.get("PATH", "")

import asyncio
import re
import tempfile
import threading
import time
from collections import deque
from datetime import datetime

import edge_tts
import numpy as np
import ollama
import sounddevice as sd
from faster_whisper import WhisperModel
from rich import box
from rich.align import Align
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.text import Text

# ── 설정 ──────────────────────────────────────────────────────────────────────

SAMPLE_RATE         = 16000
CHANNELS            = 1
CHUNK_DURATION      = 0.1
SILENCE_THRESHOLD   = 0.005
SILENCE_DURATION    = 0.7
MIN_SPEECH_DURATION = 0.3
MAX_RECORDING       = 30.0

MIC_NAME      = "UM-200"
WHISPER_MODEL = "small"
TTS_VOICE     = "ko-KR-HyunsuNeural"
TTS_RATE      = "+25%"          # 재생 속도
OLLAMA_MODEL  = "gemma3:4b"

SYSTEM_PROMPT = """너는 자비스야. 나한테 완전 친한 친구처럼 말해.
반말 쓰고, 편하게 툭툭 던지듯이 얘기해줘.
이모지, 이모티콘, 특수문자 절대 쓰지 마.
2문장 이내로 짧고 빠릿하게 답해. 영어 쓰지 마."""

EXIT_KEYWORDS = {"종료", "꺼줘", "시스템 종료", "exit", "quit", "bye"}
SENT_RE = re.compile(r'(?<=[.?!~。？！\n])\s*')

_EMOJI_RE = re.compile(
    "["
    "\U0001F600-\U0001FFFF"
    "\U00002700-\U000027BF"
    "\U0000200D"
    "\U00002500-\U00002BEF"
    "☀-⛿"
    "]+",
    flags=re.UNICODE,
)

def clean_for_tts(text: str) -> str:
    text = _EMOJI_RE.sub("", text)
    text = re.sub(r"[*_~`#>|\\]", "", text)   # 마크다운 기호
    text = re.sub(r"\s{2,}", " ", text)
    return text.strip()

console = Console()

# ── 유틸 ──────────────────────────────────────────────────────────────────────

def rms(chunk: np.ndarray) -> float:
    return float(np.sqrt(np.mean(chunk ** 2)))

def is_silent(chunk: np.ndarray) -> bool:
    return rms(chunk) < SILENCE_THRESHOLD

# ── TTS 플레이어 (문장 단위 병렬 재생) ────────────────────────────────────────

class TTSPlayer:
    def __init__(self):
        self._q: deque[str] = deque()
        self._lock = threading.Lock()
        self._busy = False
        import pygame
        pygame.mixer.pre_init(44100, -16, 2, 4096)
        pygame.mixer.init()
        self._pygame = pygame
        threading.Thread(target=self._worker, daemon=True).start()

    def enqueue(self, text: str):
        with self._lock:
            self._q.append(text)

    def is_done(self) -> bool:
        with self._lock:
            return not self._q and not self._busy

    def wait_done(self):
        while not self.is_done():
            time.sleep(0.05)

    def _worker(self):
        while True:
            text = None
            with self._lock:
                if self._q:
                    text = self._q.popleft()
                    self._busy = True
            if text:
                try:
                    asyncio.run(self._speak(text))  # type: ignore[arg-type]
                except Exception:
                    pass
                with self._lock:
                    self._busy = False
            else:
                time.sleep(0.02)

    async def _speak(self, text: str):
        text = clean_for_tts(text)
        if not text:
            return
        communicate = edge_tts.Communicate(text, TTS_VOICE, rate=TTS_RATE)
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
            tmp = f.name
        await communicate.save(tmp)
        self._pygame.mixer.music.load(tmp)
        self._pygame.mixer.music.play()
        while self._pygame.mixer.music.get_busy():
            await asyncio.sleep(0.05)
        self._pygame.mixer.music.unload()
        os.unlink(tmp)

# ── UI 상태 ───────────────────────────────────────────────────────────────────

class State:
    def __init__(self):
        self.phase   = "INIT"       # INIT | WAIT | REC | STT | LLM | TTS
        self.mic_lvl = 0.0
        self.chat: list[tuple[str, str, str]] = []
        self.stream  = ""
        self.boot_t  = datetime.now()

state = State()

PHASE_LABEL = {
    "INIT": ("○  초기화 중",        "dim cyan"),
    "WAIT": ("●  대기 중 — 말씀하세요", "green"),
    "REC":  ("◉  녹음 중",          "bold red"),
    "STT":  ("◎  음성 인식 중",      "yellow"),
    "LLM":  ("◌  응답 생성 중",      "bold cyan"),
    "TTS":  ("◈  음성 출력 중",      "cyan"),
}

ASCII_LOGO = (
    "      ██╗ █████╗ ██████╗ ██╗   ██╗██╗███████╗\n"
    "      ██║██╔══██╗██╔══██╗██║   ██║██║██╔════╝\n"
    "      ██║███████║██████╔╝██║   ██║██║███████╗\n"
    " ██   ██║██╔══██║██╔══██╗╚██╗ ██╔╝██║╚════██║\n"
    " ╚█████╔╝██║  ██║██║  ██║ ╚████╔╝ ██║███████║\n"
    "  ╚════╝ ╚═╝  ╚═╝╚═╝  ╚═╝  ╚═══╝  ╚═╝╚══════╝"
)

# ── UI 렌더러 ─────────────────────────────────────────────────────────────────

def _header() -> Panel:
    now    = datetime.now().strftime("%H : %M : %S")
    uptime = str(datetime.now() - state.boot_t).split(".")[0]
    t = Text(justify="center")
    t.append(ASCII_LOGO + "\n", style="bold cyan")
    t.append(f"\n  ◈  {now}  │  UPTIME {uptime}  │  {OLLAMA_MODEL}  │  {WHISPER_MODEL}", style="dim cyan")
    return Panel(t, border_style="cyan", box=box.HEAVY, padding=(0, 2))

def _sidebar() -> Panel:
    t = Text()
    t.append("  ◈  SYSTEMS\n\n", style="bold cyan")

    for label, val in [("STT", f"whisper-{WHISPER_MODEL}"),
                        ("LLM", OLLAMA_MODEL),
                        ("TTS", "edge-tts neural")]:
        t.append(f"  {label:<5}", style="dim cyan")
        t.append(f"{val}\n", style="cyan")

    t.append("\n  ◈  MIC\n\n  ", style="bold cyan")
    lvl = min(int(state.mic_lvl * 500), 22)
    t.append("█" * lvl,        style="bold cyan")
    t.append("░" * (22 - lvl), style="dim cyan")
    t.append(f"\n  {state.mic_lvl:.4f} rms\n\n", style="dim cyan")

    t.append("  ◈  SESSION\n\n", style="bold cyan")
    t.append(f"  TURNS   {len(state.chat)}\n", style="cyan")

    return Panel(t, border_style="cyan dim", box=box.MINIMAL, padding=(0, 0))

def _chat() -> Panel:
    t = Text()
    for ts, who, txt in state.chat[-18:]:
        t.append(f"\n  {ts}  ", style="dim cyan")
        if who == "나":
            t.append("나      ▶  ", style="bold white")
            t.append(txt,          style="white")
        else:
            t.append("JARVIS  ◀  ", style="bold cyan")
            t.append(txt,          style="cyan")

    if state.stream:
        t.append("\n\n  ···   JARVIS  ◀  ", style="bold cyan")
        t.append(state.stream,              style="cyan")
        t.append("▌",                       style="bold cyan")

    return Panel(
        t,
        title="[bold cyan]◈  CONVERSATION LOG[/]",
        border_style="cyan",
        box=box.MINIMAL,
        padding=(0, 1),
    )

def _footer() -> Panel:
    label, style = PHASE_LABEL.get(state.phase, ("●", "cyan"))
    t = Text()
    t.append(f"  {label}", style=style)
    return Panel(t, border_style="cyan dim", box=box.MINIMAL)

def _make_layout() -> Layout:
    lo = Layout()
    lo.split_column(
        Layout(name="header", size=10),
        Layout(name="body"),
        Layout(name="footer", size=3),
    )
    lo["body"].split_row(
        Layout(name="sidebar", ratio=1),
        Layout(name="chat",    ratio=3),
    )
    return lo

def refresh(live: Live, lo: Layout):
    lo["header"].update(_header())
    lo["sidebar"].update(_sidebar())
    lo["chat"].update(_chat())
    lo["footer"].update(_footer())
    live.update(lo)

# ── 자비스 ────────────────────────────────────────────────────────────────────

class Jarvis:
    def __init__(self, live: Live, lo: Layout):
        self.live = live
        self.lo   = lo
        self.tts  = TTSPlayer()

        state.phase = "INIT"
        refresh(live, lo)

        self.whisper = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8")
        self.client  = ollama.Client()
        self.history: list[dict] = []

        self._chunk = int(SAMPLE_RATE * CHUNK_DURATION)
        self._mic   = self._find_mic()

        state.phase = "WAIT"
        refresh(live, lo)

    # ── 마이크 탐색 ───────────────────────────────────────────────────────────

    def _find_mic(self) -> int | None:
        devices = sd.query_devices()
        candidates = [(i, d) for i, d in enumerate(devices) if d["max_input_channels"] > 0]

        for i, d in candidates:
            if MIC_NAME.lower() in d["name"].lower():
                console.print(f"[cyan]마이크 발견: [{i}] {d['name']}[/]")
                return i

        console.print(f"[yellow]⚠  마이크 '{MIC_NAME}' 를 찾지 못했습니다.[/]")
        console.print("[dim]사용 가능한 입력 장치:[/]")
        for i, d in candidates:
            console.print(f"  [{i}] {d['name']}")

        default = sd.default.device[0]  # 기본 입력 장치
        if default >= 0:
            console.print(f"[cyan]→ 기본 입력 장치 [{default}] {devices[default]['name']} 사용[/]\n")
            return default

        if candidates:
            console.print(f"[cyan]→ 첫 번째 입력 장치 [{candidates[0][0]}] {candidates[0][1]['name']} 사용[/]\n")
            return candidates[0][0]

        console.print("[red]입력 장치가 없습니다.[/]\n")
        return None

    def _tick(self):
        refresh(self.live, self.lo)

    # ── 녹음 ─────────────────────────────────────────────────────────────────

    def record(self) -> np.ndarray | None:
        state.phase = "WAIT"
        self._tick()

        buf: list[np.ndarray] = []
        silence_since: float | None = None
        speaking = False
        start_time: float | None = None

        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype="float32",
            blocksize=self._chunk,
            device=self._mic,
        ) as stream:
            while not speaking:
                chunk, _ = stream.read(self._chunk)
                chunk = chunk[:, 0]
                state.mic_lvl = rms(chunk)
                self._tick()
                if not is_silent(chunk):
                    speaking   = True
                    start_time = time.time()
                    buf.append(chunk.copy())
                    state.phase = "REC"
                    self._tick()

            while True:
                chunk, _ = stream.read(self._chunk)
                chunk = chunk[:, 0]
                state.mic_lvl = rms(chunk)
                buf.append(chunk.copy())
                self._tick()

                if is_silent(chunk):
                    if silence_since is None:
                        silence_since = time.time()
                    elif time.time() - silence_since >= SILENCE_DURATION:
                        break
                else:
                    silence_since = None

                if time.time() - start_time > MAX_RECORDING:
                    break

        state.mic_lvl = 0.0
        audio = np.concatenate(buf)
        return audio if len(audio) / SAMPLE_RATE >= MIN_SPEECH_DURATION else None

    # ── STT ──────────────────────────────────────────────────────────────────

    def transcribe(self, audio: np.ndarray) -> str:
        state.phase = "STT"
        self._tick()
        segs, _ = self.whisper.transcribe(
            audio, language="ko", beam_size=5, vad_filter=True,
            initial_prompt="안녕하세요. 한국어로 말하고 있습니다.",
        )
        return " ".join(s.text for s in segs).strip()

    # ── LLM 스트리밍 + TTS ────────────────────────────────────────────────────

    def chat_stream(self, user_text: str):
        self.history.append({"role": "user", "content": user_text})
        messages = [{"role": "system", "content": SYSTEM_PROMPT}] + self.history

        state.phase  = "LLM"
        state.stream = ""
        self._tick()

        full   = ""
        buf    = ""

        for chunk in self.client.chat(model=OLLAMA_MODEL, messages=messages, stream=True):
            tok   = chunk.message.content
            full += tok
            buf  += tok
            state.stream = full
            self._tick()

            parts = SENT_RE.split(buf)
            if len(parts) > 1:
                for sent in parts[:-1]:
                    if sent.strip():
                        self.tts.enqueue(sent.strip())
                        if state.phase == "LLM":
                            state.phase = "TTS"
                            self._tick()
                buf = parts[-1]

        if buf.strip():
            self.tts.enqueue(buf.strip())

        self.history.append({"role": "assistant", "content": full})
        state.stream = ""
        state.chat.append((datetime.now().strftime("%H:%M:%S"), "JARVIS", full))
        self.tts.wait_done()
        state.phase = "WAIT"
        self._tick()

    # ── 메인 루프 ─────────────────────────────────────────────────────────────

    def run(self):
        greeting = "어 나왔어. 뭐 필요해?"
        state.chat.append((datetime.now().strftime("%H:%M:%S"), "JARVIS", greeting))
        self._tick()
        self.tts.enqueue(greeting)
        self.tts.wait_done()

        while True:
            try:
                audio = self.record()
                if audio is None:
                    continue

                text = self.transcribe(audio)
                if not text:
                    state.phase = "WAIT"
                    self._tick()
                    continue

                state.chat.append((datetime.now().strftime("%H:%M:%S"), "나", text))
                self._tick()

                if any(kw in text.lower() for kw in EXIT_KEYWORDS):
                    farewell = "응 알겠어. 나중에 또 불러."
                    state.chat.append((datetime.now().strftime("%H:%M:%S"), "JARVIS", farewell))
                    self._tick()
                    self.tts.enqueue(farewell)
                    self.tts.wait_done()
                    break

                self.chat_stream(text)

            except KeyboardInterrupt:
                break
            except Exception as e:
                state.phase = f"ERR"
                state.stream = str(e)
                self._tick()
                time.sleep(1)
                state.phase  = "WAIT"
                state.stream = ""
                self._tick()

# ── 진입점 ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    lo = _make_layout()
    with Live(lo, console=console, screen=True, refresh_per_second=20) as live:
        Jarvis(live, lo).run()

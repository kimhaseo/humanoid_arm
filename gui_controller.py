"""
gui_controller.py — 수치 직접 입력으로 목표 위치/자세를 조정하고
MeshCat 창에서 IK 결과를 즉시 확인합니다.
시나리오 기능: 현재 위치를 포인트로 기록하고 순서대로 실행
"""

import threading
import tkinter as tk
from tkinter import font as tkfont
import numpy as np
from ik_solver_meshcat import IKSolver
from motor_controller import MotorController
from config.motor_cmd import AngleCommand

# ── 다크 테마 팔레트 ─────────────────────────────────────────────
BG          = "#161616"   # 창 배경 (거의 검정)
CARD        = "#242424"   # 카드 배경 (짙은 회색)
BORDER      = "#3A3A3A"   # 기본 테두리
HEADING     = "#F5F5F5"   # 타이틀
LBL_TEXT    = "#E0E0E0"   # 레이블
LBL_MUTED   = "#707070"   # 설명·보조
LBL_UNIT    = "#555555"   # 단위
ENTRY_BG    = "#2E2E2E"   # 입력창 배경
ENTRY_FG    = "#FFFFFF"   # 입력창 텍스트 (순백)
ENTRY_BD    = "#555555"   # 입력창 기본 테두리
ENTRY_FOCUS = "#818CF8"   # 포커스 테두리 (인디고)
BTN_PRIMARY       = "#5B5BD6"
BTN_PRIMARY_HOVER = "#4747C2"
BTN_GREEN         = "#1A7F5A"
BTN_GREEN_HOVER   = "#15694A"
BTN_RED           = "#C0392B"
BTN_RED_HOVER     = "#A93226"
BTN_GRAY          = "#3A3A3A"
BTN_GRAY_HOVER    = "#4A4A4A"
C_SUCCESS   = "#2ECC71"
C_FAIL      = "#E74C3C"
C_PENDING   = "#F39C12"
C_WARN      = "#F39C12"


class IKControllerGUI:
    # (레이블, 기본값, 단위, 설명)
    POS_FIELDS = [
        ("X",     0.100,  "m", "좌우 거리"),
        ("Y",     0.200,  "m", "전후 거리"),
        ("Z",    -0.300,  "m", "상하 높이"),
    ]
    ROT_FIELDS = [
        ("Roll",  0.0, "°", "X축 회전"),
        ("Pitch", 0.0, "°", "Y축 회전"),
        ("Yaw",   0.0, "°", "Z축 회전"),
    ]

    MOTOR_SPEED_MAX = 1080          # 하드웨어 최대 속도 [deg/s]
    JOINT_SIGN  = {
        "joint_1": -1, "joint_2": +1, "joint_3": +1, "joint_4": +1,
        "joint_5": +1, "joint_6": -1, "joint_7": -1,
    }
    _debug_step = 0

    # ── 속도 프로퍼티 (슬라이더 % 기반) ───────────────────────────
    # 모터 하드웨어 속도는 항상 MAX(1080 deg/s) 고정 → 토크 유지
    # 보간 스텝 간격(dt)만 바꿔서 전체 이동 시간을 조절
    #   50 % = 기본 (dt=0.005 → 2.5 초)
    #  100 % = 최고속 (dt=0.0025 → 1.25 초)
    #   10 % = 최저속 (dt=0.025  → 12.5 초)
    @property
    def move_dt(self) -> float:
        pct = self._speed_var.get()
        return 0.25 / pct            # 50→0.005  100→0.0025  10→0.025

    def __init__(self):
        self.solver = IKSolver(meshcat=True, n_restarts=3, n_steps=20)
        self.motor  = MotorController()
        self.joint_map = {f"joint_{i}": f"left_joint{i}" for i in range(1, 8)}

        self._solve_lock = threading.Lock()
        self.waypoints:  list[tuple] = []

        self.root = tk.Tk()
        self.root.title("IK Controller")
        self.root.configure(bg=BG)
        self.root.geometry("480x820")
        self.root.resizable(False, True)

        self.vars:       dict[str, tk.StringVar] = {}
        self.entries:    dict[str, tk.Entry]     = {}
        self._speed_var  = tk.IntVar(value=50)   # 이동 속도 % (10~100)

        self._init_fonts()
        self._build_scrollable_root()
        self._build_gui()
        self._run_ik()

    # ── 폰트 ──────────────────────────────────────────────────────
    def _init_fonts(self):
        self.F = {
            "title":   tkfont.Font(family="Segoe UI", size=16, weight="bold"),
            "section": tkfont.Font(family="Segoe UI", size=9,  weight="bold"),
            "label":   tkfont.Font(family="Segoe UI", size=12, weight="bold"),
            "desc":    tkfont.Font(family="Segoe UI", size=9),
            "entry":   tkfont.Font(family="Consolas", size=14, weight="bold"),
            "unit":    tkfont.Font(family="Segoe UI", size=11),
            "btn":     tkfont.Font(family="Segoe UI", size=12, weight="bold"),
            "btn_sm":  tkfont.Font(family="Segoe UI", size=11, weight="bold"),
            "status":  tkfont.Font(family="Consolas", size=11),
            "mono":    tkfont.Font(family="Consolas", size=10),
        }

    # ── 스크롤 래퍼 ───────────────────────────────────────────────
    def _build_scrollable_root(self):
        self._canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        sb = tk.Scrollbar(self.root, orient="vertical", command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)

        self.frame = tk.Frame(self._canvas, bg=BG)
        self._win  = self._canvas.create_window((0, 0), window=self.frame, anchor="nw")
        self.frame.bind("<Configure>", lambda _: self._canvas.configure(
            scrollregion=self._canvas.bbox("all")))
        self._canvas.bind("<Configure>", lambda e: self._canvas.itemconfig(
            self._win, width=e.width))
        self.root.bind_all("<MouseWheel>",
            lambda e: self._canvas.yview_scroll(int(-1*(e.delta/120)), "units"))
        self.root.bind_all("<Button-4>",
            lambda e: self._canvas.yview_scroll(-1, "units"))
        self.root.bind_all("<Button-5>",
            lambda e: self._canvas.yview_scroll( 1, "units"))

    # ── 카드 ──────────────────────────────────────────────────────
    def _make_card(self, title: str):
        outer = tk.Frame(self.frame, bg=BORDER)
        outer.pack(padx=18, pady=(0, 12), fill='x')

        card = tk.Frame(outer, bg=CARD)
        card.pack(padx=1, pady=1, fill='x')

        tk.Label(card, text=title.upper(), bg=CARD, fg=LBL_MUTED,
                 font=self.F["section"]).pack(anchor='w', padx=18, pady=(12, 0))
        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', padx=18, pady=(5, 10))
        return card

    # ── 입력 필드 ─────────────────────────────────────────────────
    def _make_field(self, parent, label: str, init, unit: str, desc: str = ""):
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill='x', padx=18, pady=6)

        # ── 레이블 열 (grid 없이 pack, 자연스럽게 높이 결정) ──
        lf = tk.Frame(row, bg=CARD)
        lf.pack(side='left')

        # 메인 레이블: width를 문자 단위로 고정 (칸 너비 통일)
        tk.Label(lf, text=label, bg=CARD, fg=LBL_TEXT,
                 font=self.F["label"], anchor='w', width=6).pack(anchor='w')
        if desc:
            tk.Label(lf, text=desc, bg=CARD, fg=LBL_MUTED,
                     font=self.F["desc"], anchor='w', width=8).pack(anchor='w')

        # ── 입력창: highlightthickness로 테두리 처리 ──────────
        var = tk.StringVar(value=str(init))
        ent = tk.Entry(row, textvariable=var, width=10,
                       font=self.F["entry"], justify='right',
                       bg=ENTRY_BG, fg=ENTRY_FG,
                       insertbackground=ENTRY_FOCUS,
                       relief='flat', bd=4,
                       highlightthickness=2,
                       highlightbackground=ENTRY_BD,
                       highlightcolor=ENTRY_FOCUS)
        ent.pack(side='left', padx=(10, 0))

        ent.bind("<Return>",   self._on_enter)
        ent.bind("<FocusOut>", self._on_focus_out)

        # ── 단위 ──────────────────────────────────────────────
        tk.Label(row, text=unit, bg=CARD, fg=LBL_UNIT,
                 font=self.F["unit"]).pack(side='left', padx=(8, 0))

        self.vars[label]    = var
        self.entries[label] = ent

    # ── 버튼 헬퍼 ─────────────────────────────────────────────────
    # tk.Button은 Windows/macOS 시스템 테마가 bg를 덮어쓰는 경우가 있어서
    # tk.Label + 클릭 바인딩으로 구현 — bg가 항상 정확히 렌더링됨
    def _btn(self, parent, text, cmd, bg, hover, fg="#fff",
             font_key="btn", pady=10, **kw):
        b = tk.Label(parent, text=text, bg=bg, fg=fg,
                     font=self.F[font_key], cursor='hand2',
                     anchor='center', pady=pady, padx=8,
                     relief='flat', **kw)

        disabled = [False]  # mutable flag

        def _click(e):
            if not disabled[0]:
                cmd()

        def _enter(e):
            if not disabled[0]:
                tk.Label.configure(b, bg=hover)

        def _leave(e):
            tk.Label.configure(b, bg=b._cur_bg)

        b.bind("<Button-1>", _click)
        b.bind("<Enter>",    _enter)
        b.bind("<Leave>",    _leave)
        b._cur_bg = bg

        # configure 를 오버라이드해서 state/bg 를 Label 에서도 처리
        def _configure(**kw2):
            st = kw2.pop('state', None)
            if st == 'disabled':
                disabled[0] = True
                tk.Label.configure(b, cursor='arrow', fg="#555555")
            elif st == 'normal':
                disabled[0] = False
                tk.Label.configure(b, cursor='hand2', fg=fg)
            if 'bg' in kw2:
                b._cur_bg = kw2['bg']
            if kw2:
                tk.Label.configure(b, **kw2)

        b.configure = _configure
        b.config    = _configure
        return b

    # ── GUI 구성 ──────────────────────────────────────────────────
    def _build_gui(self):
        # 타이틀 + 모드 뱃지
        title_row = tk.Frame(self.frame, bg=BG)
        title_row.pack(fill='x', padx=20, pady=(22, 4))

        tk.Label(title_row, text="IK Controller", bg=BG, fg=HEADING,
                 font=self.F["title"]).pack(side='left')

        motor_type = type(self.motor.can_handler).__name__
        if "Dummy" in motor_type:
            badge_txt, badge_bg, badge_fg = "SIM",  "#334155", "#94A3B8"
        else:
            badge_txt, badge_bg, badge_fg = "LIVE", "#064E3B", "#6EE7B7"
        tk.Label(title_row, text=f"  {badge_txt}  ", bg=badge_bg, fg=badge_fg,
                 font=self.F["desc"], relief='flat').pack(
            side='left', padx=(10, 0), pady=6)

        tk.Label(self.frame, text="End-effector 목표 위치 및 자세 입력",
                 bg=BG, fg=LBL_MUTED, font=self.F["desc"]).pack(
            anchor='w', padx=20, pady=(0, 14))

        # 위치 카드
        pc = self._make_card("목표 위치  (End-effector)")
        for lbl, init, unit, desc in self.POS_FIELDS:
            self._make_field(pc, lbl, init, unit, desc)
        tk.Frame(pc, bg=CARD, height=8).pack()

        # 자세 카드
        rc = self._make_card("목표 자세  (오일러각)")
        for lbl, init, unit, desc in self.ROT_FIELDS:
            self._make_field(rc, lbl, init, unit, desc)
        tk.Frame(rc, bg=CARD, height=8).pack()

        # 속도 카드
        self._build_speed_card()

        # IK 실행 버튼
        self.btn = self._btn(self.frame, "IK 실행", self._run_ik,
                             BTN_PRIMARY, BTN_PRIMARY_HOVER, pady=12)
        self.btn.pack(fill='x', padx=18, pady=(0, 12))

        # 상태 카드
        sc = self._make_card("상태")
        status_row = tk.Frame(sc, bg=CARD)
        status_row.pack(fill='x', padx=18, pady=(0, 14))

        self._dot = tk.Label(status_row, text="●", bg=CARD,
                             fg=LBL_MUTED, font=self.F["unit"])
        self._dot.pack(side='left')
        self.status_var = tk.StringVar(value="대기 중")
        self.status_lbl = tk.Label(status_row, textvariable=self.status_var,
                                   bg=CARD, fg=LBL_MUTED,
                                   font=self.F["status"], anchor='w')
        self.status_lbl.pack(side='left', padx=(6, 0))

        self._build_scenario_card()

    # ── 시나리오 카드 ──────────────────────────────────────────────
    def _build_scenario_card(self):
        sc = self._make_card("시나리오")

        tk.Label(sc, text="포인트를 기록한 뒤 순서대로 실행합니다.",
                 bg=CARD, fg=LBL_MUTED, font=self.F["desc"]).pack(
            anchor='w', padx=18, pady=(0, 8))

        btn_row = tk.Frame(sc, bg=CARD)
        btn_row.pack(fill='x', padx=18, pady=(0, 8))

        self.btn_record = self._btn(
            btn_row, "+ 현재 위치 기록", self._record_point,
            BTN_GREEN, BTN_GREEN_HOVER, pady=8, font_key="btn_sm")
        self.btn_record.pack(side='left', fill='x', expand=True, padx=(0, 5))

        self.btn_delete = self._btn(
            btn_row, "✕ 선택 삭제", self._delete_point,
            BTN_RED, BTN_RED_HOVER, pady=8, font_key="btn_sm")
        self.btn_delete.pack(side='left', fill='x', expand=True)

        lb_outer = tk.Frame(sc, bg=BORDER)
        lb_outer.pack(fill='x', padx=18, pady=(0, 8))

        self.waypoint_list = tk.Listbox(
            lb_outer, bg=CARD, fg=LBL_TEXT,
            font=self.F["mono"],
            selectbackground=BTN_PRIMARY, selectforeground="#fff",
            relief='flat', bd=5, height=5, activestyle='none')
        self.waypoint_list.pack(padx=1, pady=1, fill='x')

        self.btn_scenario = self._btn(
            sc, "▶  시나리오 실행", self._run_scenario,
            BTN_PRIMARY, BTN_PRIMARY_HOVER, pady=11)
        self.btn_scenario.pack(fill='x', padx=18, pady=(0, 6))

        self.btn_clear = self._btn(
            sc, "전체 초기화", self._clear_scenario,
            BTN_GRAY, BTN_GRAY_HOVER, fg=LBL_TEXT, pady=7, font_key="btn_sm")
        self.btn_clear.pack(fill='x', padx=18, pady=(0, 14))

    # ── 속도 카드 ──────────────────────────────────────────────────
    def _build_speed_card(self):
        sc = self._make_card("이동 속도")

        # 슬라이더 + 값 표시 행
        row = tk.Frame(sc, bg=CARD)
        row.pack(fill='x', padx=18, pady=(0, 4))

        tk.Label(row, text="속도", bg=CARD, fg=LBL_TEXT,
                 font=self.F["label"], anchor='w', width=4).pack(side='left')

        # 퍼센트 라벨 (우측 고정)
        pct_lbl = tk.Label(row, bg=CARD, fg=ENTRY_FG,
                           font=self.F["entry"], width=5, anchor='e')
        pct_lbl.pack(side='right')

        def _update_label(*_):
            pct = self._speed_var.get()
            pct_lbl.configure(text=f"{pct} %")
            # 예상 시간 계산 (n_interp=500 기준)
            secs = 500 * self.move_dt
            time_lbl.configure(text=f"예상 이동 시간  {secs:.1f} 초")

        scale = tk.Scale(
            row, from_=10, to=100, orient='horizontal',
            variable=self._speed_var,
            command=_update_label,
            resolution=5,
            showvalue=0,           # 직접 라벨로 표시
            bg=CARD, fg=LBL_TEXT,
            troughcolor=BORDER,
            activebackground=BTN_PRIMARY,
            highlightthickness=0,
            bd=0, sliderlength=18, sliderrelief='flat',
            length=200)
        scale.pack(side='left', padx=(8, 8), fill='x', expand=True)

        # 예상 이동 시간 안내
        time_lbl = tk.Label(sc, bg=CARD, fg=LBL_MUTED, font=self.F["desc"],
                            anchor='w')
        time_lbl.pack(anchor='w', padx=18, pady=(0, 12))

        # 초기 라벨 설정
        _update_label()

    # ── 상태 표시 ─────────────────────────────────────────────────
    def _set_status(self, text: str, color: str):
        self.status_var.set(text)
        self.status_lbl.configure(fg=color)
        self._dot.configure(fg=color)

    # ── 입력값 파싱 ────────────────────────────────────────────────
    def _parse_fields(self, keys: list[str]) -> list[float] | None:
        values, ok = [], True
        for key in keys:
            try:
                values.append(float(self.vars[key].get()))
                self.entries[key].configure(highlightbackground=ENTRY_BD)
            except ValueError:
                self.entries[key].configure(highlightbackground=C_FAIL)
                ok = False
        return values if ok else None

    # ── 버튼 활성/비활성 ──────────────────────────────────────────
    def _set_buttons_state(self, state: str):
        on = (state == 'normal')
        self.btn.configure(         state=state, bg=BTN_PRIMARY if on else BTN_GRAY)
        self.btn_scenario.configure(state=state, bg=BTN_PRIMARY if on else BTN_GRAY)
        self.btn_record.configure(  state=state, bg=BTN_GREEN   if on else BTN_GRAY)
        self.btn_delete.configure(  state=state, bg=BTN_RED     if on else BTN_GRAY)
        self.btn_clear.configure(   state=state)

    # ── 관절각 → 모터 전달 ────────────────────────────────────────
    def _send_joint_angles(self, q_deg_dict: dict):
        import time as _time
        IKControllerGUI._debug_step += 1

        # 50스텝마다 전체 로그
        if IKControllerGUI._debug_step % 50 == 1:
            print(f"[step {IKControllerGUI._debug_step}] " +
                  "  ".join(f"{k}={v:+.3f}" for k, v in q_deg_dict.items()))

        # joint_6 전용 로그 (매 스텝) — 실제 전송 여부 확인
        j6_angle = q_deg_dict.get("joint_6")
        if j6_angle is not None and IKControllerGUI._debug_step % 10 == 1:
            sign6 = self.JOINT_SIGN.get("joint_6", +1)
            print(f"  [joint_6 CMD] raw={j6_angle:+.3f}° → send={j6_angle*sign6:+.3f}°")

        for ik_joint, angle in q_deg_dict.items():
            if ik_joint in self.joint_map:
                try:
                    sign = self.JOINT_SIGN.get(ik_joint, +1)
                    cmd  = AngleCommand(self.joint_map[ik_joint],
                                        angle * sign, self.MOTOR_SPEED_MAX)
                    self.motor.move_motor_to_angle(cmd)
                    _time.sleep(0.001)   # 1 ms 간격 — SLCAN 버퍼 여유 확보
                except Exception as e:
                    print(f"[{ik_joint}] motor error: {e}")

    # ── IK 콜백 ───────────────────────────────────────────────────
    def _on_enter(self, event):
        event.widget.tk_focusNext().focus()
        self._run_ik()

    def _on_focus_out(self, _=None):
        self._run_ik()

    def _run_ik(self, _=None):
        pos_vals = self._parse_fields(["X", "Y", "Z"])
        rot_vals = self._parse_fields(["Roll", "Pitch", "Yaw"])
        if pos_vals is None or rot_vals is None:
            self._set_status("잘못된 입력값", C_WARN)
            return
        if not self._solve_lock.acquire(blocking=False):
            self._set_status("이동 중 (입력 무시됨)", C_PENDING)
            return

        target_pos = np.array(pos_vals)
        target_rot = IKSolver.euler_to_rotation(*rot_vals)

        self._set_status("보간 이동 중...", C_PENDING)
        self._set_buttons_state('disabled')
        self.root.update_idletasks()

        dt_snap = self.move_dt   # 실행 시점 속도 스냅샷

        def _worker():
            try:
                q_dict, pos_err, rot_err = self.solver.move_to(
                    target_pos, target_rot, n_interp=250, dt=dt_snap,
                    step_callback=self._send_joint_angles, viz_every=10)
            finally:
                self._solve_lock.release()
            self.root.after(0, self._update_status, q_dict, pos_err, rot_err)

        threading.Thread(target=_worker, daemon=True).start()

    # ── 시나리오 ─────────────────────────────────────────────────
    def _record_point(self):
        pos_vals = self._parse_fields(["X", "Y", "Z"])
        rot_vals = self._parse_fields(["Roll", "Pitch", "Yaw"])
        if pos_vals is None or rot_vals is None:
            self._set_status("잘못된 입력값 — 기록 실패", C_WARN)
            return
        wp = (*pos_vals, *rot_vals)
        self.waypoints.append(wp)
        idx = len(self.waypoints)
        x, y, z, roll, pitch, yaw = wp
        self.waypoint_list.insert(
            tk.END,
            f"P{idx:02d}  {x:+.3f} {y:+.3f} {z:+.3f}  "
            f"{roll:+.1f}° {pitch:+.1f}° {yaw:+.1f}°")
        self.waypoint_list.see(tk.END)
        self._set_status(f"P{idx:02d} 기록 완료 — 총 {idx}개", C_SUCCESS)

    def _delete_point(self):
        sel = self.waypoint_list.curselection()
        if not sel:
            return
        idx = sel[0]
        self.waypoints.pop(idx)
        self._refresh_listbox()
        self._set_status(f"P{idx+1:02d} 삭제 — 총 {len(self.waypoints)}개", C_WARN)

    def _clear_scenario(self):
        self.waypoints.clear()
        self.waypoint_list.delete(0, tk.END)
        self._set_status("시나리오 초기화", LBL_MUTED)

    def _refresh_listbox(self):
        self.waypoint_list.delete(0, tk.END)
        for i, (x, y, z, roll, pitch, yaw) in enumerate(self.waypoints, 1):
            self.waypoint_list.insert(
                tk.END,
                f"P{i:02d}  {x:+.3f} {y:+.3f} {z:+.3f}  "
                f"{roll:+.1f}° {pitch:+.1f}° {yaw:+.1f}°")

    def _run_scenario(self):
        if not self.waypoints:
            self._set_status("기록된 포인트 없음", C_WARN)
            return
        if not self._solve_lock.acquire(blocking=False):
            self._set_status("이동 중 (시나리오 대기)", C_PENDING)
            return

        self._set_buttons_state('disabled')
        total   = len(self.waypoints)
        wps     = list(self.waypoints)
        dt_snap = self.move_dt       # 시나리오 시작 시점 속도 스냅샷

        def _worker():
            failed = False
            try:
                for i, (x, y, z, roll, pitch, yaw) in enumerate(wps, 1):
                    def _hl(i=i):
                        self._set_status(f"P{i:02d} / {total}  이동 중", C_PENDING)
                        self.waypoint_list.selection_clear(0, tk.END)
                        self.waypoint_list.selection_set(i - 1)
                        self.waypoint_list.see(i - 1)
                    self.root.after(0, _hl)

                    target_pos = np.array([x, y, z])
                    target_rot = IKSolver.euler_to_rotation(roll, pitch, yaw)
                    q_dict, pos_err, rot_err = self.solver.move_to(
                        target_pos, target_rot, n_interp=250, dt=dt_snap,
                        step_callback=self._send_joint_angles)

                    if q_dict is None:
                        def _fail(i=i, pe=pos_err, re=rot_err):
                            self._set_status(
                                f"FAIL P{i:02d}  {pe*1000:.1f} mm  "
                                f"{np.degrees(re):.1f}°", C_FAIL)
                        self.root.after(0, _fail)
                        failed = True
                        return
            finally:
                self._solve_lock.release()
                self.root.after(0, self._on_scenario_done, total, failed)

        threading.Thread(target=_worker, daemon=True).start()

    def _on_scenario_done(self, total: int, failed: bool):
        self._set_buttons_state('normal')
        self.waypoint_list.selection_clear(0, tk.END)
        if not failed:
            self._set_status(f"완료 — {total}개 포인트", C_SUCCESS)

    def _update_status(self, q_dict, pos_err, rot_err):
        self._set_buttons_state('normal')
        if q_dict is not None:
            self._set_status(
                f"OK   pos {pos_err*1000:.2f} mm   rot {np.degrees(rot_err):.2f}°",
                C_SUCCESS)
        else:
            self._set_status(
                f"FAIL   {pos_err*1000:.2f} mm   {np.degrees(rot_err):.2f}°",
                C_FAIL)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = IKControllerGUI()
    app.run()

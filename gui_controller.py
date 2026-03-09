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
# ── 고대비 라이트 테마 ───────────────────────────────────────────
BG         = "#f0f2f8"
CARD       = "#ffffff"
BORDER     = "#b0b8d0"
LBL_HEAD   = "#1e1b6e"   # 진한 인디고
LBL_TEXT   = "#000000"   # 완전 검정
LBL_UNIT   = "#374151"   # 진한 회색
ENTRY_BG   = "#f8f9ff"
ENTRY_FG   = "#000000"
ENTRY_ERR  = "#fde8e8"
CURSOR_CLR = "#4f46e5"
BTN_BG     = "#3730a3"   # 진한 인디고
BTN_FG     = "#ffffff"
BTN_HOVER  = "#4f46e5"
BTN_GREEN  = "#065f46"   # 진한 초록
BTN_RED    = "#991b1b"   # 진한 빨강
C_SUCCESS  = "#065f46"
C_FAIL     = "#991b1b"
C_PENDING  = "#92400e"   # 진한 주황
C_WARN     = "#7c2d12"


class IKControllerGUI:
    POS_FIELDS = [("X", 0.100, "m"), ("Y", 0.200, "m"), ("Z", -0.300, "m")]
    ROT_FIELDS = [("Roll", 0.0, "deg"), ("Pitch", 0.0, "deg"), ("Yaw", 0.0, "deg")]

    def __init__(self):
        self.solver = IKSolver(meshcat=True, n_restarts=3, n_steps=20)
        self.motor = MotorController()
        self.joint_map = {
            "joint_1": "left_joint1",
            "joint_2": "left_joint2",
            "joint_3": "left_joint3",
            "joint_4": "left_joint4",
            "joint_5": "left_joint5",
            "joint_6": "left_joint6",
            "joint_7": "left_joint7",
        }

        self._solve_lock = threading.Lock()
        self.waypoints: list[tuple] = []

        self.root = tk.Tk()
        self.root.title("IK Controller")
        self.root.configure(bg=BG)
        self.root.geometry("500x820")
        self.root.resizable(False, True)

        self.vars:    dict[str, tk.StringVar] = {}
        self.entries: dict[str, tk.Entry]     = {}

        self._build_scrollable_root()
        self._build_gui()
        self._run_ik()

    # ── 스크롤 캔버스 래퍼 ────────────────────────────────────────
    def _build_scrollable_root(self):
        self._canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        self._scrollbar = tk.Scrollbar(
            self.root, orient="vertical", command=self._canvas.yview
        )
        self._canvas.configure(yscrollcommand=self._scrollbar.set)
        self._scrollbar.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)

        self.frame = tk.Frame(self._canvas, bg=BG)
        self._canvas_window = self._canvas.create_window(
            (0, 0), window=self.frame, anchor="nw"
        )
        self.frame.bind("<Configure>", self._on_frame_configure)
        self._canvas.bind("<Configure>", self._on_canvas_configure)
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)
        self.root.bind_all("<Button-4>",   self._on_mousewheel_linux)
        self.root.bind_all("<Button-5>",   self._on_mousewheel_linux)

    def _on_frame_configure(self, _):
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self._canvas.itemconfig(self._canvas_window, width=event.width)

    def _on_mousewheel(self, event):
        self._canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _on_mousewheel_linux(self, event):
        self._canvas.yview_scroll(-1 if event.num == 4 else 1, "units")

    # ── 폰트 (크기 전반적으로 키움) ──────────────────────────────
    def _fonts(self):
        return {
            "head":   tkfont.Font(family="Arial", size=13, weight="bold"),
            "label":  tkfont.Font(family="Arial", size=15, weight="bold"),  # ↑
            "entry":  tkfont.Font(family="Courier New", size=17, weight="bold"),  # ↑
            "unit":   tkfont.Font(family="Arial", size=13, weight="bold"),  # ↑
            "btn":    tkfont.Font(family="Arial", size=15, weight="bold"),  # ↑
            "btn_sm": tkfont.Font(family="Arial", size=13, weight="bold"),  # ↑
            "status": tkfont.Font(family="Courier New", size=14, weight="bold"),  # ↑
            "title":  tkfont.Font(family="Arial", size=20, weight="bold"),  # ↑
            "mono":   tkfont.Font(family="Courier New", size=12),
        }

    # ── 카드 프레임 ────────────────────────────────────────────────
    def _make_card(self, title, f):
        outer = tk.Frame(self.frame, bg=BORDER, bd=0)
        outer.pack(padx=24, pady=(0, 16), fill='x')

        inner = tk.Frame(outer, bg=CARD, bd=0)
        inner.pack(padx=2, pady=2, fill='x')   # ← 테두리 두껍게

        tk.Label(inner, text=title, bg=CARD, fg=LBL_HEAD,
                 font=f["head"], anchor='w').pack(
            padx=18, pady=(14, 4), fill='x')

        tk.Frame(inner, bg=BORDER, height=2).pack(fill='x', padx=18, pady=(0, 10))  # ← 구분선 두껍게
        return inner

    # ── 입력 필드 한 행 ────────────────────────────────────────────
    def _make_field(self, parent, label, init, unit, f):
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill='x', padx=18, pady=7)   # ← 행 간격 넓힘

        tk.Label(row, text=label, width=6, anchor='e',
                 bg=CARD, fg=LBL_TEXT, font=f["label"]).pack(side='left')

        var = tk.StringVar(value=str(init))
        ent = tk.Entry(
            row, textvariable=var, width=12,
            font=f["entry"], justify='right',
            bg=ENTRY_BG, fg=ENTRY_FG,
            insertbackground=CURSOR_CLR,
            relief='flat', bd=0,
            highlightthickness=2,
            highlightbackground=BORDER,
            highlightcolor=BTN_HOVER,
        )
        ent.pack(side='left', padx=(14, 10), ipady=7)  # ← 입력창 높이 증가

        tk.Label(row, text=unit, anchor='w',
                 bg=CARD, fg=LBL_UNIT, font=f["unit"]).pack(side='left')

        ent.bind("<Return>",   self._on_enter)
        ent.bind("<FocusOut>", self._on_focus_out)

        self.vars[label]    = var
        self.entries[label] = ent

    # ── 전체 GUI 구성 ──────────────────────────────────────────────
    def _build_gui(self):
        f = self._fonts()

        tk.Label(self.frame, text="IK Controller",
                 bg=BG, fg="#000000", font=f["title"]).pack(pady=(24, 18))

        pc = self._make_card("목표 위치", f)
        for lbl, init, unit in self.POS_FIELDS:
            self._make_field(pc, lbl, init, unit, f)
        tk.Frame(pc, bg=CARD, height=8).pack()

        rc = self._make_card("목표 자세", f)
        for lbl, init, unit in self.ROT_FIELDS:
            self._make_field(rc, lbl, init, unit, f)
        tk.Frame(rc, bg=CARD, height=8).pack()

        self.btn = tk.Button(
            self.frame, text="IK  실행",
            command=self._run_ik,
            bg=BTN_BG, fg=BTN_FG,
            font=f["btn"],
            relief='flat', cursor='hand2',
            pady=14, bd=0,
            activebackground=BTN_HOVER,
            activeforeground=BTN_FG,
        )
        self.btn.pack(padx=24, pady=(0, 16), fill='x')

        sc = self._make_card("IK 상태", f)
        self.status_var = tk.StringVar(value="대기 중...")
        self.status_lbl = tk.Label(
            sc, textvariable=self.status_var,
            anchor='w', bg=CARD,
            fg=LBL_TEXT, font=f["status"],
        )
        self.status_lbl.pack(padx=18, pady=(4, 16), fill='x')

        self._build_scenario_card(f)

    # ── 시나리오 카드 ──────────────────────────────────────────────
    def _build_scenario_card(self, f):
        sc = self._make_card("시나리오", f)

        btn_row = tk.Frame(sc, bg=CARD)
        btn_row.pack(fill='x', padx=18, pady=(0, 10))

        self.btn_record = tk.Button(
            btn_row, text="＋  현재 위치 기록",
            command=self._record_point,
            bg=BTN_GREEN, fg=BTN_FG,
            font=f["btn_sm"],
            relief='flat', cursor='hand2',
            pady=10, bd=0,
            activebackground="#047857",
        )
        self.btn_record.pack(side='left', fill='x', expand=True, padx=(0, 6))

        self.btn_delete = tk.Button(
            btn_row, text="✕  선택 삭제",
            command=self._delete_point,
            bg=BTN_RED, fg=BTN_FG,
            font=f["btn_sm"],
            relief='flat', cursor='hand2',
            pady=10, bd=0,
            activebackground="#7f1d1d",
        )
        self.btn_delete.pack(side='left', fill='x', expand=True)

        list_outer = tk.Frame(sc, bg=BORDER)
        list_outer.pack(fill='x', padx=18, pady=(0, 10))

        self.waypoint_list = tk.Listbox(
            list_outer,
            bg=CARD, fg="#000000",              # ← 완전 검정 글씨
            font=f["mono"],
            selectbackground=BTN_BG,
            selectforeground="#ffffff",
            relief='flat', bd=0,
            height=5,
            activestyle='none',
        )
        self.waypoint_list.pack(padx=2, pady=2, fill='x')

        self.btn_scenario = tk.Button(
            sc, text="▶  시나리오 실행",
            command=self._run_scenario,
            bg=BTN_BG, fg=BTN_FG,
            font=f["btn"],
            relief='flat', cursor='hand2',
            pady=12, bd=0,
            activebackground=BTN_HOVER,
        )
        self.btn_scenario.pack(fill='x', padx=18, pady=(0, 8))

        self.btn_clear = tk.Button(
            sc, text="전체 초기화",
            command=self._clear_scenario,
            bg=BORDER, fg="#000000",            # ← 검정 글씨
            font=f["btn_sm"],
            relief='flat', cursor='hand2',
            pady=8, bd=0,
        )
        self.btn_clear.pack(fill='x', padx=18, pady=(0, 14))

    # ── 입력값 파싱 ────────────────────────────────────────────────
    def _parse_fields(self, keys: list[str]) -> list[float] | None:
        values, ok = [], True
        for key in keys:
            try:
                v = float(self.vars[key].get())
                self.entries[key].configure(bg=ENTRY_BG, highlightbackground=BORDER)
                values.append(v)
            except ValueError:
                self.entries[key].configure(bg=ENTRY_ERR, highlightbackground=C_FAIL)
                ok = False
        return values if ok else None

    # ── 버튼 일괄 활성/비활성 ──────────────────────────────────────
    def _set_buttons_state(self, state: str):
        is_normal = (state == 'normal')
        self.btn.configure(state=state, bg=BTN_BG if is_normal else BORDER)
        self.btn_scenario.configure(state=state, bg=BTN_BG if is_normal else BORDER)
        self.btn_record.configure(state=state, bg=BTN_GREEN if is_normal else BORDER)
        self.btn_delete.configure(state=state, bg=BTN_RED if is_normal else BORDER)
        self.btn_clear.configure(state=state)

    # ── 모터 속도 제한 [deg/s] ────────────────────────────────────
    MOTOR_SPEED = 1080

    # ── 보간 스텝 간격 [초] — 클수록 전체 이동 시간 길어짐 ────────
    # 총 이동 시간 ≈ n_interp(500) × MOVE_DT
    #   0.000 → 최대속도 (~0.5 s)
    #   0.005 → 약 2.5 s  ← 시작값
    #   0.010 → 약 5.0 s
    MOVE_DT = 0.005

    # ── 관절 부호 설정 (+1 정방향 / -1 반전) ─────────────────────
    JOINT_SIGN = {
        "joint_1": -1,
        "joint_2": +1,
        "joint_3": +1,
        "joint_4": +1,
        "joint_5": +1,
        "joint_6": +1,
        "joint_7": -1,
    }

    # ── 매 IK 스텝 → 모터 전달 ────────────────────────────────────
    _debug_step = 0  # 진단용 카운터

    def _send_joint_angles(self, q_deg_dict: dict):
        """move_to() 각 보간 스텝마다 호출 — 관절각을 실시간으로 모터에 전달."""
        # ── 진단: 50스텝마다 모든 관절값 출력 (joint_6 확인용) ──
        IKControllerGUI._debug_step += 1
        if IKControllerGUI._debug_step % 50 == 1:
            print(f"[step {IKControllerGUI._debug_step}] " +
                  "  ".join(f"{k}={v:+.3f}" for k, v in q_deg_dict.items()))
        # ─────────────────────────────────────────────────────────
        for ik_joint, angle in q_deg_dict.items():
            if ik_joint in self.joint_map:
                try:
                    motor_name = self.joint_map[ik_joint]
                    sign = self.JOINT_SIGN.get(ik_joint, +1)
                    cmd = AngleCommand(motor_name, angle * sign, self.MOTOR_SPEED)
                    self.motor.move_motor_to_angle(cmd)
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
            self.status_var.set("⚠  잘못된 입력값이 있습니다")
            self.status_lbl.configure(fg=C_WARN)
            return

        if not self._solve_lock.acquire(blocking=False):
            self.status_var.set("...  이동 중 (입력 무시됨)")
            self.status_lbl.configure(fg=C_PENDING)
            return

        target_pos = np.array(pos_vals)
        target_rot = IKSolver.euler_to_rotation(*rot_vals)

        self.status_var.set("...  보간 이동 중")
        self.status_lbl.configure(fg=C_PENDING)
        self._set_buttons_state('disabled')
        self.root.update_idletasks()

        def _worker():
            try:
                q_dict, pos_err, rot_err = self.solver.move_to(
                    target_pos, target_rot, n_interp=500, dt=self.MOVE_DT,
                    step_callback=self._send_joint_angles,
                    viz_every=10,
                )
            finally:
                self._solve_lock.release()
            self.root.after(0, self._update_status, q_dict, pos_err, rot_err)

        threading.Thread(target=_worker, daemon=True).start()

    # ── 시나리오: 포인트 기록 ──────────────────────────────────────
    def _record_point(self):
        pos_vals = self._parse_fields(["X", "Y", "Z"])
        rot_vals = self._parse_fields(["Roll", "Pitch", "Yaw"])
        if pos_vals is None or rot_vals is None:
            self.status_var.set("⚠  잘못된 입력값 — 기록 실패")
            self.status_lbl.configure(fg=C_WARN)
            return

        wp = (*pos_vals, *rot_vals)
        self.waypoints.append(wp)
        idx = len(self.waypoints)
        x, y, z, roll, pitch, yaw = wp
        self.waypoint_list.insert(
            tk.END,
            f"P{idx:02d}  X{x:+.3f} Y{y:+.3f} Z{z:+.3f}"
            f"   R{roll:+.1f} P{pitch:+.1f} Y{yaw:+.1f}"
        )
        self.waypoint_list.see(tk.END)
        self.status_var.set(f"P{idx:02d} 기록 완료 — 총 {idx}개")
        self.status_lbl.configure(fg=C_SUCCESS)

    # ── 시나리오: 선택 삭제 ────────────────────────────────────────
    def _delete_point(self):
        sel = self.waypoint_list.curselection()
        if not sel:
            return
        idx = sel[0]
        self.waypoints.pop(idx)
        self._refresh_listbox()
        self.status_var.set(f"P{idx+1:02d} 삭제 — 총 {len(self.waypoints)}개")
        self.status_lbl.configure(fg=C_WARN)

    # ── 시나리오: 전체 초기화 ──────────────────────────────────────
    def _clear_scenario(self):
        self.waypoints.clear()
        self.waypoint_list.delete(0, tk.END)
        self.status_var.set("시나리오 초기화 완료")
        self.status_lbl.configure(fg=LBL_TEXT)

    # ── 리스트박스 재정렬 ──────────────────────────────────────────
    def _refresh_listbox(self):
        self.waypoint_list.delete(0, tk.END)
        for i, (x, y, z, roll, pitch, yaw) in enumerate(self.waypoints, 1):
            self.waypoint_list.insert(
                tk.END,
                f"P{i:02d}  X{x:+.3f} Y{y:+.3f} Z{z:+.3f}"
                f"   R{roll:+.1f} P{pitch:+.1f} Y{yaw:+.1f}"
            )

    # ── 시나리오: 순서대로 실행 ────────────────────────────────────
    def _run_scenario(self):
        if not self.waypoints:
            self.status_var.set("⚠  기록된 포인트가 없습니다")
            self.status_lbl.configure(fg=C_WARN)
            return

        if not self._solve_lock.acquire(blocking=False):
            self.status_var.set("...  이동 중 (시나리오 대기)")
            self.status_lbl.configure(fg=C_PENDING)
            return

        self._set_buttons_state('disabled')
        total = len(self.waypoints)
        waypoints_copy = list(self.waypoints)

        def _worker():
            failed = False
            try:
                for i, (x, y, z, roll, pitch, yaw) in enumerate(waypoints_copy, 1):
                    def _highlight(i=i):
                        self.status_var.set(f"▶  P{i:02d} / {total}  이동 중")
                        self.status_lbl.configure(fg=C_PENDING)
                        self.waypoint_list.selection_clear(0, tk.END)
                        self.waypoint_list.selection_set(i - 1)
                        self.waypoint_list.see(i - 1)
                    self.root.after(0, _highlight)

                    target_pos = np.array([x, y, z])
                    target_rot = IKSolver.euler_to_rotation(roll, pitch, yaw)
                    q_dict, pos_err, rot_err = self.solver.move_to(
                        target_pos, target_rot, n_interp=500, dt=self.MOVE_DT,
                        step_callback=self._send_joint_angles,
                    )

                    if q_dict is None:
                        def _fail(i=i, pe=pos_err, re=rot_err):
                            self.status_var.set(
                                f"FAIL  P{i:02d}  "
                                f"pos {pe*1000:.2f}mm  rot {np.degrees(re):.2f}°"
                            )
                            self.status_lbl.configure(fg=C_FAIL)
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
            self.status_var.set(f"✓  시나리오 완료 — {total}개 포인트")
            self.status_lbl.configure(fg=C_SUCCESS)

    # ── IK 단일 완료 후 상태 업데이트 ─────────────────────────────
    def _update_status(self, q_dict, pos_err, rot_err):
        self._set_buttons_state('normal')

        if q_dict is not None:

            self.status_var.set(
                f"OK   pos {pos_err * 1000:6.2f} mm"
                f"   rot {np.degrees(rot_err):6.2f} deg"
            )
            self.status_lbl.configure(fg=C_SUCCESS)

        else:
            self.status_var.set(
                f"FAIL   pos {pos_err * 1000:6.2f} mm"
                f"   rot {np.degrees(rot_err):6.2f} deg"
            )
            self.status_lbl.configure(fg=C_FAIL)

    # ── 실행 ──────────────────────────────────────────────────────
    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = IKControllerGUI()
    app.run()
"""
gui_controller.py — 수치 직접 입력으로 목표 위치/자세를 조정하고
MeshCat 창에서 IK 결과를 즉시 확인합니다.

트리거:  Enter 키  /  Tab(포커스 이동)  /  [IK 실행] 버튼
실행:    python gui_controller.py
"""

import threading
import tkinter as tk
from tkinter import font as tkfont
import numpy as np
from ik_solver_meshcat import IKSolver

# ── 다크 테마 색상 ───────────────────────────────────────────────
BG         = "#1a1b2e"
CARD       = "#252640"
BORDER     = "#3a3b5c"
LBL_HEAD   = "#818cf8"
LBL_TEXT   = "#c8d0e7"
LBL_UNIT   = "#6b7280"
ENTRY_BG   = "#1e1f3a"
ENTRY_FG   = "#ffffff"
ENTRY_ERR  = "#3b1a1a"
CURSOR_CLR = "#818cf8"
BTN_BG     = "#4f46e5"
BTN_FG     = "#ffffff"
BTN_HOVER  = "#6366f1"
C_SUCCESS  = "#34d399"
C_FAIL     = "#f87171"
C_PENDING  = "#fbbf24"
C_WARN     = "#f97316"


class IKControllerGUI:
    POS_FIELDS = [("X", 0.300, "m"), ("Y", 0.000, "m"), ("Z", 0.200, "m")]
    ROT_FIELDS = [("Roll", 0.0, "deg"), ("Pitch", 0.0, "deg"), ("Yaw", 0.0, "deg")]

    def __init__(self):
        self.solver = IKSolver(meshcat=True, n_restarts=3, n_steps=200)
        self._solve_lock = threading.Lock()

        self.root = tk.Tk()
        self.root.title("IK Controller")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.vars:    dict[str, tk.StringVar] = {}
        self.entries: dict[str, tk.Entry]     = {}

        self._build_gui()
        self._run_ik()

    # ── 폰트 ──────────────────────────────────────────────────────
    def _fonts(self):
        return {
            "head":   tkfont.Font(family="Helvetica Neue", size=13, weight="bold"),
            "label":  tkfont.Font(family="Helvetica Neue", size=14),
            "entry":  tkfont.Font(family="Menlo",          size=16),
            "unit":   tkfont.Font(family="Helvetica Neue", size=12),
            "btn":    tkfont.Font(family="Helvetica Neue", size=14, weight="bold"),
            "status": tkfont.Font(family="Menlo",          size=13),
            "title":  tkfont.Font(family="Helvetica Neue", size=18, weight="bold"),
        }

    # ── 카드 프레임 ────────────────────────────────────────────────
    def _make_card(self, title, f):
        outer = tk.Frame(self.root, bg=BORDER, bd=0)
        outer.pack(padx=24, pady=(0, 14), fill='x')

        inner = tk.Frame(outer, bg=CARD, bd=0)
        inner.pack(padx=1, pady=1, fill='x')

        tk.Label(inner, text=title, bg=CARD, fg=LBL_HEAD,
                 font=f["head"], anchor='w').pack(
            padx=18, pady=(12, 4), fill='x')

        tk.Frame(inner, bg=BORDER, height=1).pack(fill='x', padx=18, pady=(0, 8))
        return inner

    # ── 입력 필드 한 행 ────────────────────────────────────────────
    def _make_field(self, parent, label, init, unit, f):
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill='x', padx=18, pady=5)

        tk.Label(row, text=label, width=6, anchor='e',
                 bg=CARD, fg=LBL_TEXT, font=f["label"]).pack(side='left')

        var = tk.StringVar(value=str(init))
        ent = tk.Entry(
            row, textvariable=var, width=13,
            font=f["entry"], justify='right',
            bg=ENTRY_BG, fg=ENTRY_FG,
            insertbackground=CURSOR_CLR,
            relief='flat', bd=0,
            highlightthickness=2,
            highlightbackground=BORDER,
            highlightcolor=BTN_HOVER,
        )
        ent.pack(side='left', padx=(12, 8), ipady=6)

        tk.Label(row, text=unit, anchor='w',
                 bg=CARD, fg=LBL_UNIT, font=f["unit"]).pack(side='left')

        ent.bind("<Return>",   self._on_enter)
        ent.bind("<FocusOut>", self._on_focus_out)

        self.vars[label]    = var
        self.entries[label] = ent

    # ── 전체 GUI 구성 ──────────────────────────────────────────────
    def _build_gui(self):
        f = self._fonts()

        tk.Label(self.root, text="IK Controller",
                 bg=BG, fg="#ffffff", font=f["title"]).pack(pady=(24, 16))

        pc = self._make_card("목표 위치", f)
        for lbl, init, unit in self.POS_FIELDS:
            self._make_field(pc, lbl, init, unit, f)
        tk.Frame(pc, bg=CARD, height=8).pack()

        rc = self._make_card("목표 자세", f)
        for lbl, init, unit in self.ROT_FIELDS:
            self._make_field(rc, lbl, init, unit, f)
        tk.Frame(rc, bg=CARD, height=8).pack()

        btn = tk.Button(
            self.root, text="IK  실행",
            command=self._run_ik,
            bg=BTN_BG, fg=BTN_FG,
            font=f["btn"],
            relief='flat', cursor='hand2',
            pady=12, bd=0,
            activebackground=BTN_HOVER,
            activeforeground=BTN_FG,
        )
        btn.pack(padx=24, pady=(0, 14), fill='x')

        sc = self._make_card("IK 상태", f)
        self.status_var = tk.StringVar(value="대기 중...")
        self.status_lbl = tk.Label(
            sc, textvariable=self.status_var,
            anchor='w', bg=CARD,
            fg=LBL_TEXT, font=f["status"],
        )
        self.status_lbl.pack(padx=18, pady=(4, 14), fill='x')

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

    # ── 콜백 ──────────────────────────────────────────────────────
    def _on_enter(self, event):
        event.widget.tk_focusNext().focus()
        self._run_ik()

    def _on_focus_out(self, _=None):
        self._run_ik()

    def _run_ik(self, _=None):
        pos_vals = self._parse_fields(["X", "Y", "Z"])
        rot_vals = self._parse_fields(["Roll", "Pitch", "Yaw"])
        if pos_vals is None or rot_vals is None:
            self.status_var.set("⚠   잘못된 입력값이 있습니다")
            self.status_lbl.configure(fg=C_WARN)
            return

        target_pos = np.array(pos_vals)
        target_rot = IKSolver.euler_to_rotation(*rot_vals)

        self.status_var.set("...  IK 계산 중")
        self.status_lbl.configure(fg=C_PENDING)
        self.root.update_idletasks()

        if not self._solve_lock.acquire(blocking=False):
            return
        try:
            q_dict, pos_err, rot_err = self.solver.solve(target_pos, target_rot)
        finally:
            self._solve_lock.release()

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

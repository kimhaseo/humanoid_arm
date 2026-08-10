"""
Teach & Playback GUI
- Freedrive 모드에서 손으로 관절 이동
- Record 버튼으로 웨이포인트 저장
- 시나리오 저장/불러오기 (JSON)
- 저장된 경로 순서대로 재생
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import json
import math
import os

from control.joint_controller import JointController
from paths import SCENARIOS_DIR

try:
    from viz.robot_visualizer import RobotVisualizer
    _VIS_AVAILABLE = True
except ImportError:
    _VIS_AVAILABLE = False

FREEDRIVE_HZ  = 20

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]


class TeachApp:
    def __init__(self, force_dummy: bool = False):
        self.root = tk.Tk()
        self.root.title("Robot Teach & Playback")
        self.root.resizable(False, False)

        self.force_dummy = force_dummy
        self.ctrl: JointController | None = None
        self.vis:  "RobotVisualizer | None" = None
        self.freedrive_active = False
        self._freedrive_stop  = threading.Event()
        self._play_stop       = threading.Event()
        self.waypoints: list[dict] = []  # [{'angles': {...}, 'duration_sec': float}, ...]

        os.makedirs(SCENARIOS_DIR, exist_ok=True)
        self._build_ui()
        self._connect()
        self._init_visualizer()
        self._init_pose_fields()
        self._schedule_state_update()

    # ── 연결 ──────────────────────────────────────────────────────────────────

    def _connect(self):
        real_error = None
        if not self.force_dummy:
            try:
                self.ctrl = JointController()
                self.ctrl.enable_all()
                self._conn_label, self._conn_color = "연결됨", "green"
                self._set_status(self._conn_label, self._conn_color)
                return
            except Exception as e:
                real_error = e

        # force_dummy=True 이거나 실제 로봇 연결/응답 실패 → 더미 버스로 시뮬레이션 모드 진입
        try:
            from canbus.dummy_bus import DummyBus
            self.ctrl = JointController(bus=DummyBus())
            self.ctrl.enable_all()
            if self.force_dummy:
                self._conn_label = "시뮬레이션 모드 (--dummy 지정됨)"
            else:
                self._conn_label = f"시뮬레이션 모드 (로봇 미연결: {real_error})"
            self._conn_color = "orange"
        except Exception as e2:
            self.ctrl = None
            self._conn_label, self._conn_color = f"연결 실패: {e2}", "red"
        self._set_status(self._conn_label, self._conn_color)

    def _init_pose_fields(self):
        """엔드이펙터 포즈 입력란을 현재(또는 중립) 자세의 FK 값으로 채운다."""
        if self.ctrl is None:
            return

        def _run():
            try:
                position, rpy_deg = self.ctrl.get_ee_pose()
            except Exception:
                return

            def _apply():
                self._pose_vars["X"].set(round(position[0], 1))
                self._pose_vars["Y"].set(round(position[1], 1))
                self._pose_vars["Z"].set(round(position[2], 1))
                self._pose_vars["Roll"].set(round(rpy_deg[0], 1))
                self._pose_vars["Pitch"].set(round(rpy_deg[1], 1))
                self._pose_vars["Yaw"].set(round(rpy_deg[2], 1))

            self.root.after(0, _apply)

        threading.Thread(target=_run, daemon=True).start()

    def _init_visualizer(self):
        if not _VIS_AVAILABLE:
            return
        try:
            self.vis = RobotVisualizer(open_browser=True)
            self.vis.set_background(top=(0.6, 0.6, 0.6), bottom=(0.4, 0.4, 0.4))
            label = getattr(self, "_conn_label", "연결됨")
            color = getattr(self, "_conn_color", "green")
            self._set_status(f"{label}  |  3D 뷰: {self.vis.url}", color)
        except Exception as e:
            self._set_status(f"시각화 초기화 실패: {e}", "orange")

    # ── UI 빌드 ───────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.root.configure(bg="#1e1e1e")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".",             background="#1e1e1e", foreground="#e0e0e0",
                                         font=("Consolas", 10))
        style.configure("TFrame",        background="#1e1e1e")
        style.configure("TLabel",        background="#1e1e1e", foreground="#e0e0e0")
        style.configure("TButton",       background="#333", foreground="#e0e0e0",
                                         relief="flat", padding=6)
        style.map("TButton",             background=[("active", "#555")])
        style.configure("Record.TButton",  background="#1a6b3a", foreground="white", padding=8)
        style.map("Record.TButton",        background=[("active", "#239455")])
        style.configure("Freedrive.TButton", background="#1a3a6b", foreground="white", padding=8)
        style.map("Freedrive.TButton",     background=[("active", "#2355a0")])
        style.configure("FreedriveOn.TButton", background="#6b1a1a", foreground="white", padding=8)
        style.map("FreedriveOn.TButton",   background=[("active", "#a02323")])
        style.configure("Play.TButton",    background="#5a3a00", foreground="white", padding=8)
        style.map("Play.TButton",          background=[("active", "#8a5a00")])
        style.configure("Stop.TButton",    background="#5a1a1a", foreground="white", padding=8)
        style.configure("Treeview",        background="#2a2a2a", foreground="#e0e0e0",
                                           fieldbackground="#2a2a2a", rowheight=22)
        style.configure("Treeview.Heading", background="#333", foreground="#aaa")
        style.map("Treeview",              background=[("selected", "#3a5a8a")])

        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        title = ttk.Label(main, text="Robot Teach & Playback",
                          font=("Consolas", 14, "bold"), foreground="#7ec8e3")
        title.grid(row=0, column=0, columnspan=3, pady=(0, 10))

        self._build_state_panel(main)
        self._build_control_panel(main)
        self._build_waypoint_panel(main)
        self._build_direct_control_panel(main)
        self._build_status_bar(main)

    def _build_state_panel(self, parent):
        frame = ttk.LabelFrame(parent, text=" Joint States ", padding=10)
        frame.grid(row=1, column=0, sticky="ns", padx=(0, 10))

        headers = ["Joint", "Angle (rad)", "Temp (°C)"]
        for c, h in enumerate(headers):
            ttk.Label(frame, text=h, foreground="#aaa").grid(row=0, column=c, padx=8, pady=2)

        self._angle_vars: dict[str, tk.StringVar] = {}
        self._temp_vars:  dict[str, tk.StringVar] = {}

        for r, name in enumerate(JOINT_NAMES, start=1):
            ttk.Label(frame, text=name).grid(row=r, column=0, sticky="w", padx=8)
            av = tk.StringVar(value="---")
            tv = tk.StringVar(value="---")
            ttk.Label(frame, textvariable=av, width=10, anchor="e").grid(row=r, column=1, padx=8)
            ttk.Label(frame, textvariable=tv, width=8,  anchor="e").grid(row=r, column=2, padx=8)
            self._angle_vars[name] = av
            self._temp_vars[name]  = tv

    def _build_control_panel(self, parent):
        frame = ttk.LabelFrame(parent, text=" Control ", padding=10)
        frame.grid(row=1, column=1, sticky="ns", padx=(0, 10))

        self.btn_freedrive = ttk.Button(
            frame, text="Freedrive ON",
            style="Freedrive.TButton",
            command=self._toggle_freedrive, width=18,
        )
        self.btn_freedrive.grid(row=0, column=0, pady=(0, 8))

        ttk.Label(frame, text="손으로 관절 이동 가능", foreground="#888",
                  font=("Consolas", 9)).grid(row=1, column=0)

        ttk.Label(frame, text="구간 길이 (초, 박자)").grid(row=2, column=0)
        dur_frame = ttk.Frame(frame)
        dur_frame.grid(row=3, column=0, sticky="ew")
        self.duration_var = tk.DoubleVar(value=1.0)
        ttk.Spinbox(dur_frame, from_=0.1, to=30.0, increment=0.1,
                    textvariable=self.duration_var, width=6,
                    justify="right").pack(side="left")
        ttk.Label(dur_frame, text="초 → 다음 Record 시 이 시간에 도착").pack(side="left", padx=(6, 0))

        self.btn_record = ttk.Button(
            frame, text="● Record Point",
            style="Record.TButton",
            command=self._record_point, width=18,
        )
        self.btn_record.grid(row=4, column=0, pady=(8, 4))

        ttk.Label(frame, text="현재 위치를 (구간 길이와 함께) 웨이포인트로 저장",
                  foreground="#888", font=("Consolas", 9)).grid(row=5, column=0)

        ttk.Button(frame, text="선택 웨이포인트에 구간 길이 적용",
                   command=self._apply_duration, width=28).grid(row=6, column=0, pady=(4, 0))

        ttk.Separator(frame, orient="horizontal").grid(row=7, column=0, sticky="ew", pady=10)

        ttk.Label(frame, text="재생 배속").grid(row=8, column=0)
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_frame = ttk.Frame(frame)
        speed_frame.grid(row=9, column=0, sticky="ew")
        ttk.Scale(speed_frame, from_=0.25, to=3.0, variable=self.speed_var,
                  orient="horizontal", length=140,
                  command=lambda v: self.speed_label.config(text=f"{float(v):.2f}x")).pack(side="left")
        self.speed_label = ttk.Label(speed_frame, text="1.00x", width=5)
        self.speed_label.pack(side="left")

        btn_row = ttk.Frame(frame)
        btn_row.grid(row=10, column=0, pady=(8, 0))
        self.btn_play = ttk.Button(btn_row, text="▶ 재생", style="Play.TButton",
                                   command=self._play_scenario, width=9)
        self.btn_play.pack(side="left", padx=(0, 4))
        ttk.Button(btn_row, text="■ 정지", style="Stop.TButton",
                   command=self._stop_scenario, width=9).pack(side="left")

        ttk.Separator(frame, orient="horizontal").grid(row=11, column=0, sticky="ew", pady=10)

        ttk.Label(frame, text="시나리오 이름").grid(row=12, column=0)
        self.scenario_name_var = tk.StringVar(value="scenario_1")
        ttk.Entry(frame, textvariable=self.scenario_name_var, width=20).grid(row=13, column=0, pady=4)

        save_row = ttk.Frame(frame)
        save_row.grid(row=14, column=0)
        ttk.Button(save_row, text="저장",    command=self._save_scenario,  width=9).pack(side="left", padx=(0, 4))
        ttk.Button(save_row, text="불러오기", command=self._load_scenario, width=9).pack(side="left")

    def _build_waypoint_panel(self, parent):
        frame = ttk.LabelFrame(parent, text=" Waypoints ", padding=10)
        frame.grid(row=1, column=2, sticky="nsew")

        cols = ["#", "Dur(s)"] + JOINT_NAMES
        self.tree = ttk.Treeview(frame, columns=cols, show="headings", height=15)
        self.tree.heading("#", text="#")
        self.tree.column("#", width=30, anchor="center")
        self.tree.heading("Dur(s)", text="Dur(s)")
        self.tree.column("Dur(s)", width=55, anchor="e")
        for name in JOINT_NAMES:
            self.tree.heading(name, text=name)
            self.tree.column(name, width=70, anchor="e")
        self.tree.grid(row=0, column=0, columnspan=3)

        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        scrollbar.grid(row=0, column=3, sticky="ns")
        self.tree.configure(yscrollcommand=scrollbar.set)

        btn_row = ttk.Frame(frame)
        btn_row.grid(row=1, column=0, columnspan=3, pady=(6, 0))
        ttk.Button(btn_row, text="↑",     command=self._move_up,      width=5).pack(side="left", padx=2)
        ttk.Button(btn_row, text="↓",     command=self._move_down,    width=5).pack(side="left", padx=2)
        ttk.Button(btn_row, text="삭제",   command=self._delete_point, width=7).pack(side="left", padx=2)
        ttk.Button(btn_row, text="전체삭제", command=self._clear_points, width=9).pack(side="left", padx=2)

    def _build_direct_control_panel(self, parent):
        frame = ttk.Frame(parent)
        frame.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))

        # ── 엔드이펙터 6D 포즈 제어 ──────────────────────────────────────────
        pose_frame = ttk.LabelFrame(frame, text=" End-Effector Pose (mm / deg) ", padding=10)
        pose_frame.grid(row=0, column=0, sticky="ns", padx=(0, 10))

        self._pose_vars: dict[str, tk.DoubleVar] = {}
        for i, label in enumerate(["X", "Y", "Z", "Roll", "Pitch", "Yaw"]):
            ttk.Label(pose_frame, text=label, width=6).grid(row=i, column=0, sticky="w", padx=4, pady=2)
            var = tk.DoubleVar(value=0.0)
            ttk.Entry(pose_frame, textvariable=var, width=10, justify="right").grid(row=i, column=1, padx=4, pady=2)
            self._pose_vars[label] = var

        ttk.Label(pose_frame, text="속도 (rad/s)").grid(row=6, column=0, sticky="w", padx=4, pady=(8, 2))
        self.pose_speed_var = tk.DoubleVar(value=1.0)
        ttk.Spinbox(pose_frame, from_=0.1, to=3.0, increment=0.1, textvariable=self.pose_speed_var,
                    width=8, justify="right").grid(row=6, column=1, padx=4, pady=(8, 2))

        self.btn_move_pose = ttk.Button(pose_frame, text="Move to Pose",
                                        command=self._move_to_pose, width=20)
        self.btn_move_pose.grid(row=7, column=0, columnspan=2, pady=(8, 0))

        # ── 조인트별 직접 각도 제어 ──────────────────────────────────────────
        joint_frame = ttk.LabelFrame(frame, text=" Joint Angles (rad) ", padding=10)
        joint_frame.grid(row=0, column=1, sticky="ns")

        self._joint_target_vars: dict[str, tk.DoubleVar] = {}
        for r, name in enumerate(JOINT_NAMES):
            ttk.Label(joint_frame, text=name, width=8).grid(row=r, column=0, sticky="w", padx=4, pady=2)
            var = tk.DoubleVar(value=0.0)
            ttk.Entry(joint_frame, textvariable=var, width=10, justify="right").grid(row=r, column=1, padx=4, pady=2)
            ttk.Button(joint_frame, text="이동", width=6,
                       command=lambda n=name: self._move_single_joint(n)).grid(row=r, column=2, padx=(4, 0), pady=2)
            self._joint_target_vars[name] = var

        ttk.Label(joint_frame, text="속도 (rad/s)").grid(
            row=len(JOINT_NAMES), column=0, sticky="w", padx=4, pady=(8, 2))
        self.joint_speed_var = tk.DoubleVar(value=1.0)
        ttk.Spinbox(joint_frame, from_=0.1, to=3.0, increment=0.1, textvariable=self.joint_speed_var,
                    width=8, justify="right").grid(row=len(JOINT_NAMES), column=1, padx=(4, 0), pady=(8, 2))

        self.btn_move_all_joints = ttk.Button(joint_frame, text="Move All Joints",
                                              command=self._move_all_joints, width=20)
        self.btn_move_all_joints.grid(row=len(JOINT_NAMES) + 1, column=0, columnspan=3, pady=(8, 0))

    def _build_status_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.status_label = tk.Label(bar, text="초기화 중...",
                                     bg="#1e1e1e", fg="#888",
                                     font=("Consolas", 9), anchor="w")
        self.status_label.pack(fill="x")

    # ── 상태 업데이트 ─────────────────────────────────────────────────────────

    def _schedule_state_update(self):
        self._update_state_display()
        self.root.after(100, self._schedule_state_update)

    def _update_state_display(self):
        if self.ctrl is None:
            return
        angles = {}
        for name, motor in self.ctrl.motors.items():
            if name in self._angle_vars:
                self._angle_vars[name].set(f"{motor.state.angle:+.4f}")
                self._temp_vars[name].set(f"{motor.state.temp:.1f}")
            angles[name] = motor.state.angle
        if self.vis is not None:
            try:
                self.vis.display(angles)
            except Exception:
                pass

    def _set_status(self, msg: str, color: str = "#888"):
        self.status_label.config(text=msg, fg=color)

    # ── Freedrive ─────────────────────────────────────────────────────────────

    def _toggle_freedrive(self):
        if not self.freedrive_active:
            self.freedrive_active = True
            self._freedrive_stop.clear()
            # 모터 전원 OFF — 토크 없음
            if self.ctrl is not None:
                self.ctrl.disable_all()
            threading.Thread(target=self._freedrive_loop, daemon=True).start()
            self.btn_freedrive.config(text="Freedrive OFF", style="FreedriveOn.TButton")
            self._set_status("Freedrive 활성화 — 모터 OFF, 손으로 관절을 움직이세요", "#7ec8e3")
        else:
            self.freedrive_active = False
            self._freedrive_stop.set()
            # 모터 다시 활성화
            if self.ctrl is not None:
                self.ctrl.enable_all()
            self.btn_freedrive.config(text="Freedrive ON", style="Freedrive.TButton")
            self._set_status("Freedrive 해제 — 모터 ON", "green")

    def _freedrive_loop(self):
        """모터 OFF 상태에서 주기적으로 상태만 폴링."""
        interval = 1.0 / FREEDRIVE_HZ
        while not self._freedrive_stop.is_set():
            if self.ctrl is not None:
                for motor in self.ctrl.motors.values():
                    motor.request_state()
            time.sleep(interval)

    # ── 웨이포인트 ────────────────────────────────────────────────────────────

    def _record_point(self):
        if self.ctrl is None:
            self._set_status("연결되지 않음", "red")
            return
        angles = {name: motor.state.angle for name, motor in self.ctrl.motors.items()}
        self.waypoints.append({"angles": angles, "duration_sec": self.duration_var.get()})
        self._refresh_tree()
        self._set_status(f"웨이포인트 {len(self.waypoints)}번 저장됨", "green")

    def _apply_duration(self):
        idx = self._selected_index()
        if idx is None:
            self._set_status("웨이포인트를 먼저 선택하세요", "orange")
            return
        self.waypoints[idx]["duration_sec"] = self.duration_var.get()
        self._refresh_tree()
        self.tree.selection_set(str(idx + 1))
        self._set_status(f"웨이포인트 {idx + 1}번 구간 길이 적용됨", "green")

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        for i, wp in enumerate(self.waypoints, start=1):
            angles = wp["angles"]
            row = [str(i), f"{wp['duration_sec']:.2f}"] + [f"{angles.get(name, 0.0):+.3f}" for name in JOINT_NAMES]
            self.tree.insert("", "end", iid=str(i), values=row)

    def _selected_index(self) -> int | None:
        sel = self.tree.selection()
        if not sel:
            return None
        return int(sel[0]) - 1

    def _move_up(self):
        idx = self._selected_index()
        if idx is None or idx == 0:
            return
        self.waypoints[idx - 1], self.waypoints[idx] = self.waypoints[idx], self.waypoints[idx - 1]
        self._refresh_tree()
        self.tree.selection_set(str(idx))

    def _move_down(self):
        idx = self._selected_index()
        if idx is None or idx >= len(self.waypoints) - 1:
            return
        self.waypoints[idx], self.waypoints[idx + 1] = self.waypoints[idx + 1], self.waypoints[idx]
        self._refresh_tree()
        self.tree.selection_set(str(idx + 2))

    def _delete_point(self):
        idx = self._selected_index()
        if idx is None:
            return
        self.waypoints.pop(idx)
        self._refresh_tree()
        self._set_status(f"웨이포인트 {idx + 1}번 삭제됨", "#888")

    def _clear_points(self):
        if not self.waypoints:
            return
        if messagebox.askyesno("확인", "웨이포인트를 전체 삭제할까요?"):
            self.waypoints.clear()
            self._refresh_tree()
            self._set_status("전체 삭제됨", "#888")

    # ── 시나리오 저장/불러오기 ────────────────────────────────────────────────

    def _save_scenario(self):
        if not self.waypoints:
            self._set_status("저장할 웨이포인트가 없습니다", "orange")
            return
        name = self.scenario_name_var.get().strip() or "scenario"
        path = os.path.join(SCENARIOS_DIR, f"{name}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.waypoints, f, indent=2, ensure_ascii=False)
        self._set_status(f"저장됨: {path}", "green")

    def _load_scenario(self):
        path = filedialog.askopenfilename(
            initialdir=SCENARIOS_DIR,
            filetypes=[("JSON", "*.json"), ("All", "*.*")],
            title="시나리오 불러오기",
        )
        if not path:
            return
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        # 구버전 시나리오(관절각만 있는 flat dict) 호환
        self.waypoints = [
            wp if "angles" in wp else {"angles": wp, "duration_sec": 1.0}
            for wp in raw
        ]
        self._refresh_tree()
        name = os.path.splitext(os.path.basename(path))[0]
        self.scenario_name_var.set(name)
        self._set_status(f"불러옴: {name}  ({len(self.waypoints)}개 웨이포인트)", "green")

    # ── 재생 ─────────────────────────────────────────────────────────────────

    def _play_scenario(self):
        if self.ctrl is None:
            self._set_status("연결되지 않음", "red")
            return
        if not self.waypoints:
            self._set_status("재생할 웨이포인트가 없습니다", "orange")
            return
        if self.freedrive_active:
            self._toggle_freedrive()

        self._play_stop.clear()
        self.btn_play.config(state="disabled")
        threading.Thread(target=self._play_thread, daemon=True).start()

    def _stop_scenario(self):
        self._play_stop.set()

    def _play_thread(self):
        tempo = self.speed_var.get()  # 배속 — 1.0 이면 기록된 duration_sec 그대로
        total = len(self.waypoints)
        for i, wp in enumerate(self.waypoints, start=1):
            if self._play_stop.is_set():
                self.root.after(0, lambda: self._set_status("재생 중단됨", "orange"))
                break
            self.root.after(0, lambda i=i: self._set_status(
                f"재생 중... {i}/{total}", "#7ec8e3"))
            self.ctrl.move_joints_timed(
                angles       = wp["angles"],
                duration_sec = wp["duration_sec"] / tempo,
            )
        else:
            self.root.after(0, lambda: self._set_status("재생 완료", "green"))
        self.root.after(0, lambda: self.btn_play.config(state="normal"))

    # ── 직접 제어 (포즈 / 조인트) ─────────────────────────────────────────────

    def _guard_move(self) -> bool:
        if self.ctrl is None:
            self._set_status("연결되지 않음", "red")
            return False
        if self.freedrive_active:
            self._toggle_freedrive()
        return True

    def _move_to_pose(self):
        if not self._guard_move():
            return
        try:
            position    = [self._pose_vars[k].get() for k in ("X", "Y", "Z")]
            orientation = [math.radians(self._pose_vars[k].get()) for k in ("Roll", "Pitch", "Yaw")]
            speed       = self.pose_speed_var.get()
        except tk.TclError:
            self._set_status("포즈 값이 올바르지 않습니다", "orange")
            return

        self.btn_move_pose.config(state="disabled")
        self._set_status("포즈로 이동 중...", "#7ec8e3")

        def _run():
            try:
                self.ctrl.move_pose(position=position, orientation=orientation,
                                     max_vel=speed, max_acc=speed * 2)
                self.root.after(0, lambda: self._set_status("포즈 이동 완료", "green"))
            except Exception as e:
                msg = str(e)
                self.root.after(0, lambda: self._set_status(f"포즈 이동 실패: {msg}", "red"))
            finally:
                self.root.after(0, lambda: self.btn_move_pose.config(state="normal"))

        threading.Thread(target=_run, daemon=True).start()

    def _move_all_joints(self):
        if not self._guard_move():
            return
        try:
            angles = {name: var.get() for name, var in self._joint_target_vars.items()}
            speed  = self.joint_speed_var.get()
        except tk.TclError:
            self._set_status("조인트 값이 올바르지 않습니다", "orange")
            return

        self.btn_move_all_joints.config(state="disabled")
        self._set_status("조인트 이동 중...", "#7ec8e3")

        def _run():
            try:
                self.ctrl.move_joints_traj(angles=angles, max_vel=speed, max_acc=speed * 2)
                self.root.after(0, lambda: self._set_status("조인트 이동 완료", "green"))
            except Exception as e:
                msg = str(e)
                self.root.after(0, lambda: self._set_status(f"조인트 이동 실패: {msg}", "red"))
            finally:
                self.root.after(0, lambda: self.btn_move_all_joints.config(state="normal"))

        threading.Thread(target=_run, daemon=True).start()

    def _move_single_joint(self, joint_name: str):
        if not self._guard_move():
            return
        try:
            angle = self._joint_target_vars[joint_name].get()
            speed = self.joint_speed_var.get()
        except tk.TclError:
            self._set_status("조인트 값이 올바르지 않습니다", "orange")
            return

        self._set_status(f"{joint_name} 이동 중...", "#7ec8e3")

        def _run():
            try:
                self.ctrl.move_joint(joint_name, angle, max_vel=speed, max_acc=speed * 2)
                self.root.after(0, lambda: self._set_status(f"{joint_name} 이동 완료", "green"))
            except Exception as e:
                msg = str(e)
                self.root.after(0, lambda: self._set_status(f"{joint_name} 이동 실패: {msg}", "red"))

        threading.Thread(target=_run, daemon=True).start()

    # ── 종료 ─────────────────────────────────────────────────────────────────

    def _on_close(self):
        self._play_stop.set()
        self._freedrive_stop.set()
        if self.ctrl is not None:
            try:
                self.ctrl.disable_all(clear_error=True)
                self.ctrl.shutdown()
            except Exception:
                pass
        if self.vis is not None:
            try:
                self.vis._vis.viewer.close()
            except Exception:
                pass
        self.root.destroy()

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Robot Teach & Playback GUI")
    parser.add_argument("--dummy", action="store_true",
                         help="실제 CAN 버스 대신 DummyBus로 시뮬레이션 모드로 시작")
    args = parser.parse_args()

    TeachApp(force_dummy=args.dummy).run()

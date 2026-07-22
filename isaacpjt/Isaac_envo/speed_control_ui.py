#!/usr/bin/env python3
"""도킹·리프트 미션 속도 실시간 조절 UI.

dock_lift_handoff_mission 노드의 ROS2 파라미터(ingress_speed, carry_speed)를
슬라이더로 조절해 `ros2 param set`으로 바꾼다. 드래그 중엔 화면 숫자만 갱신되고,
놓는 순간에만 실제로 적용(호출 과다 방지). 미션(dock_lift_handoff_mission.sh)이
먼저 떠 있어야 값이 반영된다 — 안 떠 있으면 슬라이더를 놔도 "실패" 상태만 뜬다.

실행: ./speed_control_ui.sh
"""
import subprocess
import tkinter as tk

NODE = "/dock_lift_handoff_mission"
MIN_SPEED = 0.10
DEFAULTS = {"ingress_speed": 0.45, "carry_speed": 0.60}
LABELS = {"ingress_speed": "진입 속도 (ingress)", "carry_speed": "운반 속도 (carry)"}
# 진입(차 밑 정밀 구간)과 운반(개활지)의 안전 상한이 다르다 — mission.py의
# INGRESS_SPEED_MAX/CARRY_SPEED_MAX와 반드시 일치해야 함(실측: ingress를 3.0까지
# 올렸다가 축 정렬을 놓쳐 진입 실패한 적 있음).
MAX_SPEED = {"ingress_speed": 0.70, "carry_speed": 3.00}
STEP = {"ingress_speed": 0.05, "carry_speed": 0.10}


def set_param(name, value):
    try:
        subprocess.run(
            ["ros2", "param", "set", NODE, name, f"{value:.2f}"],
            capture_output=True, text=True, timeout=3.0, check=True,
        )
        return True, ""
    except subprocess.CalledProcessError as e:
        return False, (e.stderr or e.stdout or str(e)).strip()
    except subprocess.TimeoutExpired:
        return False, "타임아웃 — 미션 노드가 떠 있는지 확인"
    except FileNotFoundError:
        return False, "ros2 커맨드를 찾을 수 없음 — 이 터미널에서 humble source 필요"


def get_param(name):
    try:
        r = subprocess.run(
            ["ros2", "param", "get", NODE, name],
            capture_output=True, text=True, timeout=3.0, check=True,
        )
        return float(r.stdout.strip().rsplit(" ", 1)[-1])
    except Exception:
        return None


class SpeedRow:
    def __init__(self, parent, row, param_name, status_var):
        self.param_name = param_name
        self.value = DEFAULTS[param_name]
        self.status_var = status_var
        tk.Label(parent, text=LABELS[param_name], width=16, anchor="w").grid(
            row=row, column=0, padx=6, pady=6)
        self.value_var = tk.StringVar(value=f"{self.value:.2f}")
        self.scale = tk.Scale(
            parent, from_=MIN_SPEED, to=MAX_SPEED[param_name], resolution=STEP[param_name],
            orient=tk.HORIZONTAL, length=220, showvalue=False,
            command=self._on_drag,
        )
        self.scale.set(self.value)
        self.scale.grid(row=row, column=1, padx=6)
        # 드래그 중엔 값 표시만 갱신, 놓는 순간에만 ros2 param set(과도한 호출 방지).
        self.scale.bind("<ButtonRelease-1>", self._on_release)
        tk.Label(parent, textvariable=self.value_var, width=5).grid(row=row, column=2)

    def _on_drag(self, value_str):
        self.value = float(value_str)
        self.value_var.set(f"{self.value:.2f}")

    def _on_release(self, _event):
        ok, err = set_param(self.param_name, self.value)
        self.status_var.set("적용됨" if ok else f"실패: {err}")

    def sync(self):
        v = get_param(self.param_name)
        if v is None:
            self.status_var.set("동기화 실패 — 미션 노드 확인")
            return
        self.value = v
        self.value_var.set(f"{self.value:.2f}")
        self.scale.set(self.value)


def main():
    root = tk.Tk()
    root.title("Dock/Lift 속도 조절")
    frame = tk.Frame(root, padx=12, pady=12)
    frame.pack()

    status_var = tk.StringVar(value="대기 중")
    rows = [SpeedRow(frame, i, name, status_var) for i, name in enumerate(DEFAULTS)]

    def sync_all():
        for row in rows:
            row.sync()
        status_var.set("동기화 완료")

    tk.Button(frame, text="↻ 현재 값 불러오기", command=sync_all).grid(
        row=len(rows), column=0, columnspan=4, pady=(6, 0), sticky="we")
    tk.Label(frame, textvariable=status_var, fg="gray").grid(
        row=len(rows) + 1, column=0, columnspan=4, pady=(8, 0))
    range_text = " / ".join(f"{LABELS[n].split(' ')[0]} {MIN_SPEED:.2f}~{MAX_SPEED[n]:.2f}"
                            for n in DEFAULTS)
    tk.Label(frame, text=range_text, fg="gray").grid(
        row=len(rows) + 2, column=0, columnspan=4)

    # 창 매니저에 따라 X버튼이 WM_DELETE_WINDOW를 안 보내는 경우가 있어 명시적으로 바인딩.
    root.protocol("WM_DELETE_WINDOW", root.destroy)

    sync_all()
    root.mainloop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""도킹·리프트 미션 속도 실시간 조절 UI.

dock_lift_handoff_mission 노드의 ROS2 파라미터(ingress_speed, carry_speed)를
`ros2 param set`으로 바꾼다. 미션(dock_lift_handoff_mission.sh)이 먼저 떠 있어야
값이 반영된다 — 안 떠 있으면 버튼을 눌러도 "실패" 상태만 뜬다.

실행: ./speed_control_ui.sh
"""
import subprocess
import tkinter as tk

NODE = "/dock_lift_handoff_mission"
STEP = 0.05
MIN_SPEED, MAX_SPEED = 0.10, 1.00
DEFAULTS = {"ingress_speed": 0.40, "carry_speed": 0.40}
LABELS = {"ingress_speed": "진입 속도 (ingress)", "carry_speed": "운반 속도 (carry)"}


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
        tk.Button(parent, text="-", width=3, command=self.dec).grid(row=row, column=1)
        self.value_var = tk.StringVar(value=f"{self.value:.2f}")
        tk.Label(parent, textvariable=self.value_var, width=6).grid(row=row, column=2)
        tk.Button(parent, text="+", width=3, command=self.inc).grid(row=row, column=3)

    def inc(self):
        self._apply(min(MAX_SPEED, round(self.value + STEP, 2)))

    def dec(self):
        self._apply(max(MIN_SPEED, round(self.value - STEP, 2)))

    def _apply(self, new_value):
        self.value = new_value
        self.value_var.set(f"{self.value:.2f}")
        ok, err = set_param(self.param_name, self.value)
        self.status_var.set("적용됨" if ok else f"실패: {err}")

    def sync(self):
        v = get_param(self.param_name)
        if v is None:
            self.status_var.set("동기화 실패 — 미션 노드 확인")
            return
        self.value = v
        self.value_var.set(f"{self.value:.2f}")


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
    tk.Label(frame, text=f"범위 {MIN_SPEED:.2f}~{MAX_SPEED:.2f}, 스텝 {STEP:.2f}", fg="gray").grid(
        row=len(rows) + 2, column=0, columnspan=4)

    sync_all()
    root.mainloop()


if __name__ == "__main__":
    main()

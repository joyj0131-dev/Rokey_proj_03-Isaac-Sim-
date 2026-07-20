#!/usr/bin/env python3
"""관제 상태 대시보드 (개발·디버깅용 로컬 웹 UI).

DB(슬롯/로봇/작업/존락)를 2초마다 읽어 주차장 도면으로 보여준다.
D 팀원의 정식 웹 UI와는 별개인 A의 개발 확인용 도구다.

실행:
    cd ~/cobot3_ws && source install/setup.bash   # 입고 요청 버튼을 쓰려면
    python3 src/parking_control/scripts/dashboard.py
    → 브라우저에서 http://localhost:8080

표준 라이브러리 + mysql-connector만 사용 (Flask 불필요).
"""

import json
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT))

from parking_control.core.db import ParkingDB  # noqa: E402
from parking_control.core.graph import ParkingMap  # noqa: E402

PORT = 8080
MAP = ParkingMap.load(PKG_ROOT / "config" / "parking_map.yaml")
DB = ParkingDB()


def collect_state():
    slots = DB._query(
        "SELECT slot_id, status, is_accessible FROM parking_slots")
    robots = DB._query(
        "SELECT robot_id, status, x, y, battery_percent FROM robots")
    tasks = DB._query(
        "SELECT LEFT(task_id,8) AS task, request_type, state, vehicle_id,"
        " robot_id, slot_id, DATE_FORMAT(created_at,'%H:%i:%s') AS at_time"
        " FROM tasks ORDER BY created_at DESC LIMIT 8")
    locks = DB._query("SELECT zone_id, robot_id FROM zone_locks")

    nodes = {n: dict(MAP.graph.nodes[n]) for n in MAP.graph.nodes}
    edges = [dict(u=u, v=v, zone=d.get("zone"))
             for u, v, d in MAP.graph.edges(data=True)]
    return dict(slots=slots, robots=robots, tasks=tasks, locks=locks,
                nodes=nodes, edges=edges,
                params=MAP.meta["params"])


def send_dispatch(vehicle_id):
    """입고 요청 버튼 → ros2 service call (환경에 ros2가 있어야 동작)."""
    cmd = (
        "source /opt/ros/humble/setup.bash 2>/dev/null;"
        f"source {PKG_ROOT.parent.parent}/install/setup.bash 2>/dev/null;"
        "ros2 service call /dispatch_parking_task"
        " parking_robot_interfaces/srv/RequestParkingTask"
        f" '{{request_type: ENTRY, vehicle_id: {vehicle_id}}}'"
    )
    try:
        out = subprocess.run(["bash", "-c", cmd], capture_output=True,
                             text=True, timeout=12)
        ok = "accepted=True" in out.stdout
        return dict(ok=ok, detail=out.stdout.strip().splitlines()[-1]
                    if out.stdout.strip() else out.stderr.strip()[-200:])
    except subprocess.TimeoutExpired:
        return dict(ok=False, detail="시간 초과 — dispatcher 노드가 떠 있나요?")


PAGE = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<title>주차로봇 관제 대시보드</title>
<style>
:root { color-scheme: light dark;
  --surface:#fcfcfb; --panel:#f1f1ee; --ink:#0b0b0b; --ink2:#52514e;
  --line:#d8d7d2; --good:#0ca30c; --warn:#fab219; --serious:#ec835a;
  --crit:#d03b3b; --accent:#2a78d6; }
@media (prefers-color-scheme: dark) { :root {
  --surface:#1a1a19; --panel:#242422; --ink:#ffffff; --ink2:#c3c2b7;
  --line:#3a3a37; --accent:#3987e5; } }
* { box-sizing:border-box; margin:0; }
body { background:var(--surface); color:var(--ink);
  font:14px/1.5 system-ui, sans-serif; padding:16px; }
h1 { font-size:18px; margin-bottom:2px; }
.sub { color:var(--ink2); font-size:12px; margin-bottom:12px; }
.wrap { display:flex; flex-wrap:wrap; gap:16px; }
.map { flex:1 1 560px; background:var(--panel); border:1px solid var(--line);
  border-radius:8px; padding:8px; }
svg { width:100%; height:auto; display:block; }
.side { flex:1 1 300px; display:flex; flex-direction:column; gap:12px; }
.card { background:var(--panel); border:1px solid var(--line);
  border-radius:8px; padding:10px 12px; }
.card h2 { font-size:13px; color:var(--ink2); font-weight:600;
  margin-bottom:6px; }
table { width:100%; border-collapse:collapse; font-size:12px; }
th { text-align:left; color:var(--ink2); font-weight:500;
  border-bottom:1px solid var(--line); padding:2px 4px; }
td { padding:3px 4px; border-bottom:1px solid var(--line); }
.tiles { display:flex; gap:12px; }
.tile { flex:1; text-align:center; padding:8px 4px; }
.tile b { font-size:24px; display:block; }
.tile span { font-size:11px; color:var(--ink2); }
.dot { display:inline-block; width:8px; height:8px; border-radius:50%;
  margin-right:4px; vertical-align:middle; }
button { background:var(--accent); border:0; color:#fff; padding:6px 14px;
  border-radius:6px; font-size:13px; cursor:pointer; }
button:disabled { opacity:.5; }
#msg { font-size:12px; color:var(--ink2); margin-top:6px; word-break:break-all; }
text { font-family:system-ui, sans-serif; }
</style></head><body>
<h1>주차로봇 관제 대시보드</h1>
<div class="sub">2초마다 자동 갱신 · DB를 직접 읽는 개발용 뷰 (정식 UI는 D 담당)</div>
<div class="wrap">
  <div class="map"><svg id="lot" viewBox="0 0 780 460"></svg></div>
  <div class="side">
    <div class="card tiles" id="tiles"></div>
    <div class="card"><h2>로봇</h2><table id="robots"></table></div>
    <div class="card"><h2>존 락</h2><div id="locks" style="font-size:12px">—</div></div>
    <div class="card"><h2>최근 작업 (tasks)</h2><table id="tasks"></table></div>
    <div class="card"><h2>입고 요청 보내기</h2>
      <button id="go">ENTRY 요청 전송</button>
      <div id="msg">dispatcher 노드가 떠 있어야 동작합니다</div></div>
  </div>
</div>
<script>
const SLOT_FILL = { EMPTY:'none', RESERVED:'var(--warn)', OCCUPIED:'var(--ink2)' };
const SLOT_MARK = { EMPTY:'', RESERVED:'예약', OCCUPIED:'점유' };
const ROBOT_COL = { IDLE:'var(--good)', BUSY:'var(--warn)', CHARGING:'var(--accent)',
                    ERROR:'var(--crit)', OFFLINE:'var(--line)' };
// ros(x,y) → svg: 균일 20px/m, x −19.5..19.5 → 0..780, y +11.5..−11.5 → 0..460 (B행이 위)
const sx = x => (x + 19.5) * 20, sy = y => (11.5 - y) * 20;

function render(s) {
  const svg = [];
  const W = s.params.space_width, L = s.params.space_length;
  // 통로(중앙)와 엣지
  for (const e of s.edges) {
    const a = s.nodes[e.u], b = s.nodes[e.v];
    const locked = s.locks.find(l => l.zone_id === e.zone);
    svg.push(`<line x1="${sx(a.x)}" y1="${sy(a.y)}" x2="${sx(b.x)}" y2="${sy(b.y)}"
      stroke="${locked ? 'var(--serious)' : 'var(--line)'}"
      stroke-width="${e.zone ? (locked ? 6 : 3) : 1}" stroke-dasharray="${e.zone ? '' : '4 4'}"/>`);
    if (locked) {
      const mx = (sx(a.x)+sx(b.x))/2, my = (sy(a.y)+sy(b.y))/2;
      svg.push(`<text x="${mx}" y="${my-8}" text-anchor="middle" font-size="11"
        fill="var(--serious)">🔒${e.zone}:${locked.robot_id}</text>`);
    }
  }
  // 슬롯
  for (const slot of s.slots) {
    const n = s.nodes[slot.slot_id]; if (!n) continue;
    const x = sx(n.x) - W*20/2, y = sy(n.y) - L*20/2;
    const fill = SLOT_FILL[slot.status] || 'none';
    svg.push(`<rect x="${x}" y="${y}" width="${W*20}" height="${L*20}" rx="4"
      fill="${fill}" fill-opacity="${slot.status==='EMPTY'?0:0.55}"
      stroke="${slot.is_accessible ? 'var(--accent)' : 'var(--ink2)'}"
      stroke-width="${slot.is_accessible ? 2.5 : 1.2}"/>`);
    svg.push(`<text x="${sx(n.x)}" y="${sy(n.y)-4}" text-anchor="middle"
      font-size="15" font-weight="600" fill="var(--ink)">${slot.slot_id}${slot.is_accessible?'♿':''}</text>`);
    if (SLOT_MARK[slot.status]) svg.push(`<text x="${sx(n.x)}" y="${sy(n.y)+14}"
      text-anchor="middle" font-size="11" fill="var(--ink)">${SLOT_MARK[slot.status]}</text>`);
  }
  // 도크·입구
  for (const [id, n] of Object.entries(s.nodes)) {
    if (n.kind === 'dock') {
      svg.push(`<rect x="${sx(n.x)-30}" y="${sy(n.y)-28}" width="60" height="56" rx="6"
        fill="none" stroke="${n.role==='charging'?'var(--accent)':'var(--warn)'}"
        stroke-width="2" stroke-dasharray="6 3"/>
        <text x="${sx(n.x)}" y="${sy(n.y)+4}" text-anchor="middle" font-size="10"
        fill="var(--ink2)">${n.role==='charging'?'충전':'대기'}</text>`);
    }
    if (n.kind === 'entrance')
      svg.push(`<text x="${sx(n.x)-4}" y="${sy(n.y)+4}" text-anchor="end"
        font-size="12" fill="var(--ink2)">입구 ▶</text>`);
  }
  // 로봇
  for (const r of s.robots) {
    if (r.x === null || r.status === 'OFFLINE') continue;
    svg.push(`<circle cx="${sx(+r.x)}" cy="${sy(+r.y)}" r="11"
      fill="${ROBOT_COL[r.status]||'var(--ink2)'}" stroke="var(--surface)" stroke-width="2"/>
      <text x="${sx(+r.x)}" y="${sy(+r.y)-15}" text-anchor="middle" font-size="11"
      font-weight="600" fill="var(--ink)">${r.robot_id} (${r.status})</text>`);
  }
  document.getElementById('lot').innerHTML = svg.join('');

  const empty = s.slots.filter(x => x.status==='EMPTY').length;
  const active = s.tasks.filter(t => ['WAITING','PROCESSING'].includes(t.state)).length;
  document.getElementById('tiles').innerHTML =
    `<div class="tile"><b>${empty}</b><span>빈 슬롯 / ${s.slots.length}</span></div>
     <div class="tile"><b>${active}</b><span>진행 중 작업</span></div>
     <div class="tile"><b>${s.locks.length}</b><span>잠긴 존</span></div>`;

  document.getElementById('robots').innerHTML =
    '<tr><th>ID</th><th>상태</th><th>위치</th><th>배터리</th></tr>' +
    s.robots.map(r => `<tr><td>${r.robot_id}</td>
      <td><span class="dot" style="background:${ROBOT_COL[r.status]||'var(--ink2)'}"></span>${r.status}</td>
      <td>${r.x===null?'—':`(${(+r.x).toFixed(1)}, ${(+r.y).toFixed(1)})`}</td>
      <td>${r.battery_percent===null?'—':(+r.battery_percent).toFixed(0)+'%'}</td></tr>`).join('');

  document.getElementById('locks').innerHTML = s.locks.length
    ? s.locks.map(l => `🔒 ${l.zone_id} ← ${l.robot_id}`).join('<br>') : '없음';

  document.getElementById('tasks').innerHTML =
    '<tr><th>시각</th><th>task</th><th>상태</th><th>차량</th><th>슬롯</th></tr>' +
    s.tasks.map(t => `<tr><td>${t.at_time}</td><td>${t.task}</td>
      <td>${t.state}</td><td>${t.vehicle_id}</td><td>${t.slot_id||'—'}</td></tr>`).join('');
}

async function tick() {
  try { render(await (await fetch('/api/state')).json()); }
  catch (e) { document.getElementById('msg').textContent = '서버 연결 끊김: ' + e; }
}
tick(); setInterval(tick, 2000);

document.getElementById('go').onclick = async () => {
  const btn = document.getElementById('go'); btn.disabled = true;
  document.getElementById('msg').textContent = '전송 중...';
  const vid = 'CAR_UI_' + Math.floor(Math.random()*9000+1000);
  try {
    const r = await (await fetch('/api/dispatch', {method:'POST',
      body: JSON.stringify({vehicle_id: vid})})).json();
    document.getElementById('msg').textContent =
      (r.ok ? '✅ 접수됨 ' : '❌ 실패 ') + (r.detail || '');
  } catch (e) { document.getElementById('msg').textContent = '오류: ' + e; }
  btn.disabled = false;
};
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):

    def log_message(self, *args):
        pass  # 콘솔 소음 억제

    def _send(self, body, content_type):
        data = body.encode()
        self.send_response(200)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self._send(PAGE, "text/html")
        elif self.path == "/api/state":
            self._send(json.dumps(collect_state(), default=str),
                       "application/json")
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/api/dispatch":
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or b"{}")
            vehicle_id = str(payload.get("vehicle_id", "CAR_UI"))[:24]
            self._send(json.dumps(send_dispatch(vehicle_id)),
                       "application/json")
        else:
            self.send_error(404)


if __name__ == "__main__":
    print(f"대시보드: http://localhost:{PORT}  (Ctrl+C로 종료)")
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()

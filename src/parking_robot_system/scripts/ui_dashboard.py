#!/usr/bin/env python3
"""parking_robot_system 용 관제 UI (웹 대시보드).

팀원 parking_robot_system(토픽+슬롯지정 모델)에 직접 배선한다. dongsoo의
parking_control/scripts/dashboard.py(MySQL 기반)와는 별개 — 그건 그대로 둔다.

배선:
  구독  /parking_slots (std_msgs/String JSON: slot_id,occupied,is_accessible,x,y,yaw_deg)
        /robot_rear/odom, /robot_front/odom (nav_msgs/Odometry)  ← 지도 y = -odom.z
        /vehicle/pose (geometry_msgs/PoseStamped)                ← 운반 차량 위치
        task_state (parking_robot_interfaces/msg/TaskState)      ← 실시간 로그
  서비스 /park_in_slot, /exit_slot (parking_robot_interfaces/srv/ParkInSlot, slot_id)
        ← UI에서 슬롯(A2 등) 지정 + 입차/출차 버튼

실행: bash src/parking_robot_system/scripts/ui_dashboard.sh
      → http://localhost:8081  (러너 + parking_robot_system 런치가 떠 있어야 실동작)
"""
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import TaskState
from parking_robot_interfaces.srv import ParkInSlot

PORT = 8081
ROBOTS = ("robot_rear", "robot_front")


def _yaw(q):
    import math
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class UiNode(Node):
    def __init__(self):
        super().__init__("ui_dashboard")
        self.slots = []                       # /parking_slots 파싱 결과
        self.robots = {}                      # rid -> dict(x, y, yaw)  (지도 좌표계 y=-z)
        self.vehicle = None                   # dict(x, y)
        self.log = deque(maxlen=200)
        self._lock = threading.Lock()
        grp = ReentrantCallbackGroup()
        self.create_subscription(String, "/parking_slots", self._on_slots, 10, callback_group=grp)
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, rid=r: self._on_odom(rid, m), 10, callback_group=grp)
        self.create_subscription(PoseStamped, "/vehicle/pose", self._on_veh, 10, callback_group=grp)
        self.create_subscription(TaskState, "task_state", self._on_task, 20, callback_group=grp)
        self.park_cli = self.create_client(ParkInSlot, "/park_in_slot", callback_group=grp)
        self.exit_cli = self.create_client(ParkInSlot, "/exit_slot", callback_group=grp)

    def _on_slots(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self.slots = data

    def _on_odom(self, rid, m):
        p = m.pose.pose.position
        with self._lock:
            # odom(x,z) → 지도(x, y=-z)  (러너 /parking_slots 와 동일 규약)
            self.robots[rid] = dict(x=round(p.x, 2), y=round(-p.z, 2),
                                    yaw=round(_yaw(m.pose.pose.orientation), 1))

    def _on_veh(self, m):
        p = m.pose.position
        with self._lock:
            self.vehicle = dict(x=round(p.x, 2), y=round(-p.z, 2))

    def _on_task(self, m):
        with self._lock:
            self.log.append(dict(time=time.strftime("%H:%M:%S"), robot_id=m.robot_id,
                                 task_id=m.task_id[:8], state=m.state, current_step=m.current_step))

    def snapshot(self):
        with self._lock:
            return dict(slots=list(self.slots),
                        robots=dict(self.robots),
                        vehicle=self.vehicle,
                        log=list(self.log)[-40:][::-1])

    def request(self, action, slot_id):
        cli = self.park_cli if action == "park" else self.exit_cli
        name = "/park_in_slot" if action == "park" else "/exit_slot"
        if not cli.wait_for_service(timeout_sec=2.0):
            return dict(ok=False, detail=f"서비스 없음: {name} (gateway 노드가 떠 있나요?)")
        fut = cli.call_async(ParkInSlot.Request(slot_id=slot_id))
        end = time.time() + 12.0
        while time.time() < end and not fut.done():
            time.sleep(0.05)   # 응답은 executor 백그라운드 스레드가 채움
        if not fut.done() or fut.result() is None:
            return dict(ok=False, detail="응답 시간 초과")
        r = fut.result()
        tail = f" (task_id={r.task_id[:8]})" if r.task_id else ""
        return dict(ok=bool(r.accepted), detail=(r.message or "") + tail)


PAGE = """<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">
<title>parking_robot_system 관제 UI</title><style>
:root{color-scheme:light dark;--bg:#fcfcfb;--panel:#f1f1ee;--ink:#0b0b0b;--ink2:#52514e;
--line:#d8d7d2;--ok:#0ca30c;--busy:#fab219;--acc:#2a78d6;--occ:#52514e;--crit:#d03b3b;}
@media(prefers-color-scheme:dark){:root{--bg:#1a1a19;--panel:#242422;--ink:#fff;--ink2:#c3c2b7;--line:#3a3a37;--acc:#3987e5;}}
*{box-sizing:border-box;margin:0}body{background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,sans-serif;padding:16px}
h1{font-size:18px}.sub{color:var(--ink2);font-size:12px;margin-bottom:12px}
.wrap{display:flex;flex-wrap:wrap;gap:16px}.map{flex:1 1 560px;background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:8px}
.side{flex:1 1 300px;display:flex;flex-direction:column;gap:12px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 12px}
.card h2{font-size:13px;color:var(--ink2);font-weight:600;margin-bottom:6px}
svg{width:100%;height:auto;display:block}table{width:100%;border-collapse:collapse;font-size:12px}
th{text-align:left;color:var(--ink2);font-weight:500;border-bottom:1px solid var(--line);padding:2px 4px}
td{padding:3px 4px;border-bottom:1px solid var(--line)}
input,select{background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:5px 8px;font-size:13px}
button{background:var(--acc);border:0;color:#fff;padding:6px 12px;border-radius:6px;font-size:13px;cursor:pointer;margin-left:4px}
button.alt{background:var(--busy)}button:disabled{opacity:.5}
#msg{font-size:12px;color:var(--ink2);margin-top:8px;word-break:break-all}
#log{font-size:12px;max-height:240px;overflow-y:auto;font-family:ui-monospace,Menlo,Consolas,monospace}
#log div{padding:2px 0;border-bottom:1px dotted var(--line)}#log .t{color:var(--ink2)}
</style></head><body>
<h1>parking_robot_system 관제 UI</h1>
<div class="sub">1초 갱신 · 토픽(/parking_slots, odom, /vehicle/pose, task_state) 구독 · 서비스(/park_in_slot, /exit_slot)</div>
<div class="wrap">
 <div class="map"><svg id="lot" viewBox="0 0 820 360"></svg></div>
 <div class="side">
  <div class="card"><h2>슬롯 지정 요청</h2>
   슬롯 <select id="slot"></select>
   <button id="park">입차(ENTRY)</button>
   <button id="exit" class="alt">출차(EXIT)</button>
   <div id="msg">러너 + parking_robot_system 런치가 떠 있어야 동작합니다</div></div>
  <div class="card"><h2>로봇</h2><table id="robots"></table></div>
  <div class="card"><h2>실시간 작업 로그 (task_state)</h2><div id="log"></div></div>
 </div>
</div>
<script>
// 월드(x, y=-z) → svg. x[-42,20], y[-13,13] 창(인계장 x=-29.6 포함)
const sx=x=>(x+42)/62*820, sy=y=>(13-y)/26*360;
const OCC={true:'var(--occ)',false:'none'};
function render(s){
 const g=[];
 // 인계장 영역 표시
 g.push(`<rect x="${sx(-41.1)}" y="${sy(12)}" width="${(sx(-18.1)-sx(-41.1))}" height="${(sy(-12)-sy(12))}" fill="none" stroke="var(--line)" stroke-dasharray="5 4"/>`);
 g.push(`<text x="${sx(-29.6)}" y="${sy(12)-4}" text-anchor="middle" font-size="11" fill="var(--ink2)">인계장</text>`);
 // 슬롯
 for(const sl of s.slots){
  const w=3.4,l=6.6, x=sx(sl.x)-(sx(sl.x+w)-sx(sl.x))/2, y=sy(sl.y)-(sy(sl.y)-sy(sl.y+l))/2;
  g.push(`<rect x="${x}" y="${y}" width="${sx(sl.x+w)-sx(sl.x)}" height="${sy(sl.y)-sy(sl.y+l)}" rx="3"
    fill="${OCC[sl.occupied]}" fill-opacity="0.5" stroke="${sl.is_accessible?'var(--acc)':'var(--ink2)'}" stroke-width="${sl.is_accessible?2.2:1.1}"/>`);
  g.push(`<text x="${sx(sl.x)}" y="${sy(sl.y)+4}" text-anchor="middle" font-size="13" font-weight="600" fill="var(--ink)">${sl.slot_id}${sl.is_accessible?'♿':''}</text>`);
  if(sl.occupied) g.push(`<text x="${sx(sl.x)}" y="${sy(sl.y)+17}" text-anchor="middle" font-size="10" fill="var(--ink2)">점유</text>`);
 }
 // 운반 차량
 if(s.vehicle) g.push(`<rect x="${sx(s.vehicle.x)-14}" y="${sy(s.vehicle.y)-9}" width="28" height="18" rx="3" fill="var(--crit)" fill-opacity="0.65"/><text x="${sx(s.vehicle.x)}" y="${sy(s.vehicle.y)-12}" text-anchor="middle" font-size="10" fill="var(--ink)">차량</text>`);
 // 로봇
 for(const [rid,r] of Object.entries(s.robots)){
  g.push(`<circle cx="${sx(r.x)}" cy="${sy(r.y)}" r="9" fill="var(--busy)" stroke="var(--bg)" stroke-width="2"/><text x="${sx(r.x)}" y="${sy(r.y)-12}" text-anchor="middle" font-size="10" font-weight="600" fill="var(--ink)">${rid.replace('robot_','')}</text>`);
 }
 document.getElementById('lot').innerHTML=g.join('');
 // 슬롯 셀렉트(최초 1회 채움)
 const sel=document.getElementById('slot');
 if(!sel.options.length && s.slots.length){
  sel.innerHTML=s.slots.map(sl=>`<option value="${sl.slot_id}">${sl.slot_id}${sl.occupied?' (점유)':''}</option>`).join('');
 }
 document.getElementById('robots').innerHTML='<tr><th>ID</th><th>위치(x,y)</th><th>yaw°</th></tr>'+
  Object.entries(s.robots).map(([rid,r])=>`<tr><td>${rid}</td><td>(${r.x}, ${r.y})</td><td>${r.yaw}</td></tr>`).join('');
 document.getElementById('log').innerHTML=s.log.length?
  s.log.map(e=>`<div><span class="t">[${e.time}]</span> <b>${e.robot_id||'-'}</b> · ${e.state}: ${e.current_step}</div>`).join('')
  :'<span style="color:var(--ink2)">아직 없음 (task_state 대기 중)</span>';
}
async function tick(){try{render(await(await fetch('/api/state')).json());}catch(e){document.getElementById('msg').textContent='서버 연결 끊김: '+e;}}
tick();setInterval(tick,1000);
async function send(action){
 const btns=document.querySelectorAll('button');btns.forEach(b=>b.disabled=true);
 document.getElementById('msg').textContent=(action==='park'?'입차':'출차')+' 요청 중...';
 const slot=document.getElementById('slot').value;
 try{const r=await(await fetch('/api/request',{method:'POST',body:JSON.stringify({action,slot_id:slot})})).json();
  document.getElementById('msg').textContent=(r.ok?'✅ 접수 ':'❌ 거절/실패 ')+(r.detail||'');
 }catch(e){document.getElementById('msg').textContent='오류: '+e;}
 btns.forEach(b=>b.disabled=false);
}
document.getElementById('park').onclick=()=>send('park');
document.getElementById('exit').onclick=()=>send('exit');
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    node = None

    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        data = body.encode()
        self.send_response(200)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self._send(PAGE, "text/html")
        elif self.path == "/api/state":
            self._send(json.dumps(Handler.node.snapshot(), default=str), "application/json")
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/api/request":
            n = int(self.headers.get("Content-Length", 0))
            p = json.loads(self.rfile.read(n) or b"{}")
            action = "park" if p.get("action") == "park" else "exit"
            slot_id = str(p.get("slot_id", ""))[:16]
            self._send(json.dumps(Handler.node.request(action, slot_id)), "application/json")
        else:
            self.send_error(404)


def main():
    rclpy.init()
    node = UiNode()
    Handler.node = node
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f"UI 대시보드: http://localhost:{PORT}  (Ctrl+C 종료)")
    try:
        ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

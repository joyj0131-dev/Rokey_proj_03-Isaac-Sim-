const API_BASE = "/api";

const statusLabels = {
  IDLE: "대기",
  BUSY: "작업 중",
  CHARGING: "충전 중",
  ERROR: "오류",
  OFFLINE: "연결 끊김",
  EMPTY: "빈 공간",
  RESERVED: "예약",
  OCCUPIED: "주차 중",
  WAITING: "요청 대기",
  ROBOT_ASSIGNED: "로봇 할당",
  APPROACHING: "차량 접근",
  LIFTING: "차량 리프트",
  MOVING_TO_SLOT: "주차 위치 이동",
  RETURNING: "대기 구역 복귀",
  COMPLETED: "완료",
  CANCELLED: "취소",
};

const alertLabels = {
  WARNING: "주의",
  ERROR: "오류",
  OBSTACLE: "장애물 감지",
  ROBOT_ERROR: "로봇 오류",
  EMERGENCY_STOP: "비상정지",
  SENSOR: "센서 연결",
  SYSTEM: "시스템",
};

const requestTypeLabels = {
  PARK_IN: "입차",
  PARK_OUT: "출차",
};

const workspaceTabs = ["live", "requests", "tasks"];
const HIDDEN_SENSOR_ALERTS_STORAGE_KEY = "parking-ui-hidden-sensor-alerts";
let latestDashboard = null;
let selectedMapItem = null;
let pendingFocusRequestId = null;
let showLidarMarkers = false;
let lastDashboardReceivedAt = null;
let messageHideTimer = null;
let recentWorkflowEvents = [];
let selectedTaskEventRequestId = null;
let selectedEventScope = "all";
let selectedEventCategory = "all";
const lastRequestStates = new Map();
const robotSpeechBubbles = new Map();
const seenSpeechAlertIds = new Set();
const hiddenSensorAlertIds = new Set();

try {
  const storedSensorAlertIds = JSON.parse(
    window.sessionStorage.getItem(HIDDEN_SENSOR_ALERTS_STORAGE_KEY) || "[]"
  );
  if (Array.isArray(storedSensorAlertIds)) {
    storedSensorAlertIds.forEach((sensorId) =>
      hiddenSensorAlertIds.add(String(sensorId))
    );
  }
} catch (_error) {
  // 저장소가 차단된 브라우저에서도 알림 자체는 정상 동작하게 한다.
}

// 폴링 간격(0.5~2초)보다 로봇 이동이 빨리 끝나면 순간이동처럼 보이므로,
// 서버가 보낸 좌표는 "목표"로만 쓰고 화면 표시 좌표는 매 프레임 보간한다.
const ROBOT_MOVE_ANIMATION_MS = 300;
const robotDisplayPositions = new Map(); // robot_id -> {x, y} 현재 화면 좌표
const robotAnimations = new Map(); // robot_id -> {fromX, fromY, toX, toY, startTime}
let robotAnimationFrame = null;

function shortRobotName(robotId) {
  const v4Labels = {
    entry_lead: "입차 L",
    entry_follow: "입차 F",
    exit_lead: "출차 L",
    exit_follow: "출차 F",
  };
  if (v4Labels[robotId]) return v4Labels[robotId];
  const match = String(robotId).match(/(\d+)$/);
  return match ? `R${Number(match[1])}` : robotId;
}

const ROS_EXPECTED_ROBOTS = [
  { id: "entry_lead", x: -3.2, y: -2.2 },
  { id: "entry_follow", x: -1.2, y: -2.2 },
  { id: "exit_lead", x: -3.2, y: 2.2 },
  { id: "exit_follow", x: -1.2, y: 2.2 },
];

function normalizeRosDashboardRobots(data) {
  if (data.system?.mode !== "ros2") return data;

  const expectedIds = new Set(ROS_EXPECTED_ROBOTS.map((robot) => robot.id));
  const legacyIdleDocks = [
    { x: -3.2, y: -2.2 },
    { x: -1.2, y: -2.2 },
  ];
  const receivedRobots = [...(data.robots || [])];
  const hasV4Robot = receivedRobots.some((robot) => expectedIds.has(robot.id));
  // 이전 테스트 스택(robot_1 등)만 실행 중이면 해당 로봇을 그대로 보여준다.
  // v4 로봇이 하나라도 수신되면 오래된 DB 행은 숨기고 4대 편성만 표시한다.
  let robots = hasV4Robot
    ? receivedRobots.filter((robot) => expectedIds.has(robot.id))
    : receivedRobots;
  if (!hasV4Robot) {
    robots = robots.map((robot, index) => {
      const idleDock = legacyIdleDocks[index];
      if (!idleDock || robot.status !== "IDLE" || robot.current_task_id != null) {
        return robot;
      }
      return { ...robot, ...idleDock };
    });
  }
  const receivedNames = new Set(robots.map((robot) => shortRobotName(robot.id)));

  if (hasV4Robot || robots.length === 0) {
    for (const expected of ROS_EXPECTED_ROBOTS) {
      if (receivedNames.has(shortRobotName(expected.id))) continue;
      robots.push({
        ...expected,
        status: "OFFLINE",
        battery: 0,
        current_task_id: null,
        error_message: "ROS2 상태 데이터 미수신",
      });
    }
  }

  data.robots = robots.sort((left, right) =>
    shortRobotName(left.id).localeCompare(shortRobotName(right.id), "ko", {
      numeric: true,
    })
  );
  if (
    data.system?.health !== "ERROR" &&
    data.robots.some((robot) => robot.status === "OFFLINE")
  ) {
    data.system.health = "WARNING";
  }
  return data;
}

function formatDateTime(value) {
  if (!value) return "-";
  return new Date(value).toLocaleString("ko-KR", {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
}

function formatCompactDateTime(value) {
  if (!value) return "-";
  const date = new Date(value);
  const now = new Date();
  const time = [
    String(date.getHours()).padStart(2, "0"),
    String(date.getMinutes()).padStart(2, "0"),
    String(date.getSeconds()).padStart(2, "0"),
  ].join(":");
  const isToday = date.getFullYear() === now.getFullYear()
    && date.getMonth() === now.getMonth()
    && date.getDate() === now.getDate();
  if (isToday) return `오늘 ${time}`;
  return `${String(date.getMonth() + 1).padStart(2, "0")}.${
    String(date.getDate()).padStart(2, "0")
  } ${time}`;
}

function formatEventTime(value) {
  if (!value) return "-";
  const date = new Date(value);
  return [
    String(date.getHours()).padStart(2, "0"),
    String(date.getMinutes()).padStart(2, "0"),
    String(date.getSeconds()).padStart(2, "0"),
  ].join(":");
}

function assignedRobotIds(request) {
  if (request.robot_ids?.length) return request.robot_ids;
  return request.robot_id ? [request.robot_id] : [];
}

function assignedRobotLabel(request) {
  const ids = assignedRobotIds(request);
  return ids.length ? ids.map(shortRobotName).join(" + ") : "대기";
}

function assignedRobotTableLabel(request) {
  const ids = assignedRobotIds(request);
  if (!ids.length) return "대기";
  const idSet = new Set(ids);
  if (idSet.has("entry_lead") && idSet.has("entry_follow")) {
    return "입차 L · F";
  }
  if (idSet.has("exit_lead") && idSet.has("exit_follow")) {
    return "출차 L · F";
  }
  const names = ids.map(shortRobotName).join(" + ");
  return ids.length > 1 ? names.replaceAll(" + ", " · ") : names;
}

function robotTeamRole(robotId) {
  const normalized = String(robotId || "").toLowerCase();
  if (normalized.startsWith("entry")) return "entry";
  if (normalized.startsWith("exit")) return "exit";
  return null;
}

function obstacleAlertScope(alert) {
  const zoneId = String(alert?.zone_id || "").toUpperCase();
  const prefixes = [
    ...`${zoneId} ${String(alert?.message || "").toUpperCase()}`
      .matchAll(/\b(ZIN|ZOUT)[A-Z0-9_]*\b/g),
  ].map((match) => match[1]);
  const uniquePrefixes = new Set(prefixes);
  if (uniquePrefixes.size === 1) {
    return uniquePrefixes.has("ZIN") ? "entry" : "exit";
  }
  if (uniquePrefixes.size > 1) return null;

  const y = Number(alert?.location_y);
  if (alert?.location_y != null && Number.isFinite(y)) {
    if (y <= -1) return "entry";
    if (y >= 1) return "exit";
  }
  return null;
}

function robotIsMoving(robot) {
  return robot.status === "BUSY" || robot.current_task_id != null;
}

function obstacleAffectsRobot(alert, robot) {
  if (alert?.category !== "OBSTACLE" || !robotIsMoving(robot)) return false;
  const scope = obstacleAlertScope(alert);
  const robotRole = robotTeamRole(robot.id);
  if (scope && robotRole) return scope === robotRole;

  if (alert.robot_id) {
    const alertRole = robotTeamRole(alert.robot_id);
    return alertRole && robotRole
      ? alertRole === robotRole
      : alert.robot_id === robot.id;
  }
  // 구역을 판정할 수 없는 장애물은 안전 우선으로 이동 중인 모든 로봇에 적용.
  return true;
}

function obstacleAffectsRole(alert, role) {
  if (alert?.category !== "OBSTACLE") return false;
  const scope = obstacleAlertScope(alert);
  if (scope) return scope === role;
  if (alert.robot_id) {
    const alertRole = robotTeamRole(alert.robot_id);
    if (alertRole) return alertRole === role;
  }
  return true;
}

function affectedRobotsForObstacle(alert, robots) {
  return robots.filter((robot) => obstacleAffectsRobot(alert, robot));
}

function requestStatusLabel(request) {
  if (
    request.status === "ROBOT_ASSIGNED" &&
    assignedRobotIds(request).length === 2
  ) {
    return "로봇 2대 할당";
  }
  return statusLabels[request.status];
}

function obstacleForRequest(request, alerts = []) {
  const role = request.request_type === "PARK_IN" ? "entry" : "exit";
  return alerts.find((alert) => obstacleAffectsRole(alert, role)) || null;
}

function requestOperationalStatusLabel(request, alerts = []) {
  return obstacleForRequest(request, alerts)
    ? "장애물 대기"
    : requestStatusLabel(request);
}

function activateWorkspaceTab(tabName, focus = false) {
  const selectedTab = workspaceTabs.includes(tabName) ? tabName : "live";

  document.querySelectorAll(".workspace-tab").forEach((button) => {
    const isActive = button.dataset.tab === selectedTab;
    button.classList.toggle("active", isActive);
    button.setAttribute("aria-selected", String(isActive));
    button.tabIndex = isActive ? 0 : -1;
    if (isActive && focus) button.focus();
  });

  document.querySelectorAll(".tab-panel").forEach((panel) => {
    const isActive = panel.dataset.panel === selectedTab;
    panel.classList.toggle("active", isActive);
    panel.hidden = !isActive;
  });
}

function setupWorkspaceTabs() {
  const buttons = [...document.querySelectorAll(".workspace-tab")];

  buttons.forEach((button, index) => {
    button.addEventListener("click", () => {
      activateWorkspaceTab(button.dataset.tab);
    });

    button.addEventListener("keydown", (event) => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault();

      let nextIndex = index;
      if (event.key === "ArrowLeft") nextIndex = (index - 1 + buttons.length) % buttons.length;
      if (event.key === "ArrowRight") nextIndex = (index + 1) % buttons.length;
      if (event.key === "Home") nextIndex = 0;
      if (event.key === "End") nextIndex = buttons.length - 1;
      activateWorkspaceTab(buttons[nextIndex].dataset.tab, true);
    });
  });

  document.querySelectorAll("[data-open-tab]").forEach((button) => {
    button.addEventListener("click", () => {
      activateWorkspaceTab(button.dataset.openTab, true);
    });
  });

  activateWorkspaceTab("live");
}

async function apiRequest(path, options = {}) {
  const { headers = {}, ...requestOptions } = options;
  const response = await fetch(`${API_BASE}${path}`, {
    cache: "no-store",
    ...requestOptions,
    headers: {
      "Content-Type": "application/json",
      ...headers,
    },
  });

  const data = await response.json();

  if (!response.ok) {
    throw new Error(formatApiError(data));
  }

  return data;
}

function formatApiError(data) {
  const detail = data?.detail;
  if (typeof detail === "string" && detail.trim()) return detail;

  if (Array.isArray(detail)) {
    const fieldLabels = {
      operator_id: "관제 담당자 ID",
      inspection_note: "점검 결과",
      approval_note: "운영 복귀 사유",
      area_clear: "작업 구역 안전 확인",
      robots_stopped: "전체 로봇 정지 확인",
      load_secured: "차량·리프트 지지 상태 확인",
      sensors_checked: "센서·통신 상태 확인",
    };
    const messages = detail.map((error) => {
      const field = Array.isArray(error?.loc)
        ? error.loc[error.loc.length - 1]
        : null;
      const label = fieldLabels[field] || "입력 내용";
      const errorType = String(error?.type || "");
      const minimum = error?.ctx?.min_length ?? error?.ctx?.limit_value;
      if (
        errorType.includes("string_too_short")
        || errorType.includes("min_length")
      ) {
        return `${label}는 최소 ${minimum || 1}자 이상 입력해주세요.`;
      }
      if (errorType.includes("missing")) {
        return `${label}을 입력해주세요.`;
      }
      return `${label}을 확인해주세요.`;
    });
    return [...new Set(messages)].join("\n");
  }

  if (detail && typeof detail === "object") {
    return detail.message || "입력 내용을 확인한 후 다시 요청해주세요.";
  }
  return data?.message || "요청 처리 중 오류가 발생했습니다.";
}

function renderSummary(summary, robots, sensors, system, alerts = [], requests = []) {
  const robotHealthy = robots.filter((robot) => !["ERROR", "OFFLINE"].includes(robot.status)).length;
  const obstacleAlerts = alerts.filter(
    (alert) => alert.category === "OBSTACLE"
  );
  const pausedRobots = robots.filter(
    (robot) => obstacleAlerts.some((alert) => obstacleAffectsRobot(alert, robot))
  );
  const isSensorHealthy = (sensor) => system.mode === "mock"
    ? ["MOCK", "ONLINE"].includes(sensor.status)
    : sensor.status === "ONLINE";
  const lidarHealthy = sensors.filter(isSensorHealthy).length;
  const totalSlots = summary.total_slots;
  const reservedSlots = Math.max(
    0,
    totalSlots - summary.empty_slots - summary.occupied_slots
  );
  const activeRequest = requests.find(
    (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
  );
  const unavailableRobots = robots.filter(
    (robot) => ["ERROR", "OFFLINE"].includes(robot.status)
  );
  const unavailableSensors = sensors.filter(
    (sensor) => !isSensorHealthy(sensor)
  );
  const hasCriticalAlert = alerts.some((alert) => alert.level === "ERROR");
  const items = [
    {
      icon: "P",
      label: "주차 현황",
      value: `${summary.empty_slots} / ${totalSlots}`,
      badge: "사용 가능",
      detail: `점유 ${summary.occupied_slots} · 예약 ${reservedSlots}`,
      progress: totalSlots ? (summary.empty_slots / totalSlots) * 100 : 0,
      tone: "success",
    },
    {
      icon: "▤",
      label: "작업 현황",
      value: summary.active_requests,
      unit: "건",
      badge: summary.active_requests > 0 ? "진행 중" : "대기",
      detail: activeRequest
        ? "현재 작업은 도면에서 확인"
        : "새 요청 대기 중",
      tone: hasCriticalAlert ? "danger" : summary.active_requests > 0 ? "primary" : "neutral",
    },
    {
      icon: "🤖",
      label: "로봇 팀",
      value: `${robotHealthy} / ${robots.length}`,
      badge: unavailableRobots.length
        ? "확인 필요"
        : pausedRobots.length ? "일시 정지" : "정상",
      detail: unavailableRobots.length
        ? `${unavailableRobots.map((robot) => shortRobotName(robot.id)).join(" · ")} 확인`
        : pausedRobots.length
          ? `${pausedRobots.map((robot) => shortRobotName(robot.id)).join(" · ")} 장애물 대기`
        : robots.length > 1
          ? "입차·출차 로봇 팀 연결"
          : robots[0] ? `${shortRobotName(robots[0].id)} 연결` : "로봇 데이터 없음",
      tone: robotHealthy === robots.length && !pausedRobots.length
        ? "success"
        : "warning",
    },
    {
      icon: "◉",
      label: "센서",
      value: `${lidarHealthy} / ${sensors.length}`,
      badge: unavailableSensors.length
        ? "연결 필요"
        : system.mode === "mock" ? "MOCK" : "정상",
      detail: unavailableSensors.length
        ? `${unavailableSensors.map((sensor) => sensor.id).join(" · ")} 미수신 · 안전 감지 제한`
        : system.mode === "mock"
          ? "LiDAR 테스트 데이터"
          : sensors.length ? `LiDAR ${sensors.length}대 정상` : "센서 정보 없음",
      tone: unavailableSensors.length
        ? "warning"
        : system.mode === "mock" ? "primary" : "success",
    },
  ];

  const taskTabCount = document.getElementById("taskTabCount");
  taskTabCount.textContent = summary.active_requests;
  taskTabCount.title = `현재 진행 중인 작업 ${summary.active_requests}건`;
  document.getElementById("summaryCards").innerHTML = `
    <div class="status-overview-items">
      ${items
    .map(
      (item) => `
        <div class="status-overview-item ${item.tone}">
          <div class="status-overview-heading">
            <span class="status-overview-icon" aria-hidden="true">${item.icon}</span>
            <span class="status-overview-label">${item.label}</span>
          </div>
          <div class="status-overview-value">
            <strong>${item.value}</strong>
            ${item.unit ? `<span class="status-overview-unit">${item.unit}</span>` : ""}
            ${item.badge ? `<span class="status-overview-badge">${item.badge}</span>` : ""}
          </div>
          <span class="status-overview-detail">${item.detail || ""}</span>
          ${item.progress == null ? "" : `
            <span class="status-overview-progress" aria-label="사용 가능 ${Math.round(item.progress)}%">
              <i style="width:${Math.max(0, Math.min(100, item.progress))}%"></i>
            </span>
          `}
        </div>
      `
    )
    .join("")}
    </div>
  `;
}

// 실제 parking_map.yaml v4의 3면(A1~A3) 배치를 한 화면에 표시하는 좌표계.
// HTML의 viewBox와 항상 같은 값을 유지한다.
const LOT_MAP_WIDTH = 1174;
const LOT_MAP_HEIGHT = 520;
const LOT_SLOT_WIDTH = 108;
const LOT_SLOT_HEIGHT = 172;
const LOT_DOCK_WIDTH = 204;
const LOT_DOCK_HEIGHT = 120;
const LOT_VEHICLE_ZONE_WIDTH = 142;
const LOT_VEHICLE_ZONE_HEIGHT = 78;
const LOT_ROBOT_CARD_WIDTH = 92;
const LOT_ROBOT_CARD_HEIGHT = 88;
// 실제 map 좌표는 유지하되, 차량 대기 구역(entry/exit_outer)부터
// 로봇 인계 지점(entry/exit_wait)까지의 긴 외곽 통로만 화면에서 압축한다.
// 압축 뒤 전체 운용 클러스터를 왼쪽으로 옮겨 좌우 여백을 비슷하게 맞춘다.
const LOT_EXTERNAL_LANE_COMPRESSION = 0.22;
const LOT_CLUSTER_X_SHIFT = -150;
const ROBOT_VISUAL_X_OFFSET = {
  entry_lead: -24,
  entry_follow: 24,
  exit_lead: -24,
  exit_follow: 24,
};

const dockRoleLabels = {
  entry: "입차 로봇 대기",
  exit: "출차 로봇 대기",
};

function computeLotTransform(points) {
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const marginLeft = 82;
  const marginRight = 70;
  const marginY = 54;
  const spanX = maxX - minX || 1;
  const spanY = maxY - minY || 1;
  const drawableWidth = LOT_MAP_WIDTH - marginLeft - marginRight;
  const drawableHeight = LOT_MAP_HEIGHT - marginY * 2;

  return {
    sx: (x) => marginLeft + ((x - minX) / spanX) * drawableWidth,
    // y는 위로 갈수록 커지도록 뒤집는다 (화면 좌표는 아래로 갈수록 커짐).
    sy: (y) => LOT_MAP_HEIGHT - marginY - ((y - minY) / spanY) * drawableHeight,
  };
}

function robotMapSubtitle(robot, requests, isPaused = false) {
  if (isPaused) return "장애물 대기";
  const request = robot.current_task_id == null
    ? null
    : requests.find((item) => item.id === robot.current_task_id);
  // 대기·충전·오류 여부는 상단 상태 배지로 충분히 전달된다. 도면에서는
  // 실제 작업이 있을 때만 한 줄 설명을 추가하고 나머지는 상세 패널에 둔다.
  if (!request) return "";

  const stageLabels = {
    ROBOT_ASSIGNED: "작업 준비",
    APPROACHING: "차량 접근",
    LIFTING: "차량 인양",
    MOVING_TO_SLOT: `${request.slot_id || "주차면"} 이동`,
    RETURNING: "대기 구역 복귀",
  };
  return stageLabels[request.status] || `${request.slot_id || "주차면"} 작업`;
}

function renderLotMap(slots, robots, mapInfo, sensorStatus = [], requests = [], alerts = []) {
  const svg = document.getElementById("lotMap");
  const emptyMessage = document.getElementById("lotMapEmpty");

  const nodes = (mapInfo && mapInfo.nodes) || [];
  const docks = (mapInfo && mapInfo.docks) || [];
  const vehicleZones = (mapInfo && mapInfo.vehicle_zones) || [];
  const sensors = ((mapInfo && mapInfo.sensors) || []).map((sensor) => ({
    ...sensor,
    ...(sensorStatus.find((status) => status.id === sensor.id) || {}),
  }));
  const entrance = mapInfo && mapInfo.entrance;

  const placedSlots = slots.filter((s) => s.x != null && s.y != null);
  const placedRobots = robots.filter((r) => r.x != null && r.y != null);

  if (!placedSlots.length && !placedRobots.length) {
    svg.innerHTML = "";
    emptyMessage.classList.remove("hidden");
    return;
  }
  emptyMessage.classList.add("hidden");

  // 이동하는 로봇 좌표를 축척 기준에 포함하면 주차면 진입 시 로봇의
  // 편대 간격만큼 경계가 늘어나 도면 전체가 흔들린다. 고정 시설물만으로
  // 좌표계를 만들고, 로봇은 고정된 좌표계 위에서만 이동시킨다.
  const fixedPoints = [
    ...nodes,
    ...placedSlots,
    ...docks,
    ...sensors.filter((sensor) => sensor.x != null && sensor.y != null),
  ];
  if (entrance) fixedPoints.push(entrance);
  const layoutPoints = fixedPoints.length ? fixedPoints : placedRobots;
  const baseTransform = computeLotTransform(layoutPoints);
  const externalLaneAnchor = nodes.find(
    (node) => node.id === "entry_wait"
  ) || nodes.find((node) => node.id === "exit_wait");
  const externalLaneAnchorX = externalLaneAnchor?.x;
  const externalLaneAnchorScreenX = externalLaneAnchor
    ? baseTransform.sx(externalLaneAnchor.x)
    : null;
  const sx = (x) => {
    const screenX = baseTransform.sx(x);
    const compressedX = (
      externalLaneAnchorScreenX != null
      && externalLaneAnchorX != null
      && x < externalLaneAnchorX
    )
      ? externalLaneAnchorScreenX
        - (externalLaneAnchorScreenX - screenX) * LOT_EXTERNAL_LANE_COMPRESSION
      : screenX;
    return compressedX + LOT_CLUSTER_X_SHIFT;
  };
  const sy = baseTransform.sy;
  const parts = [`
    <defs>
      <marker id="arrow-entry" viewBox="0 0 10 10" refX="8" refY="5"
        markerWidth="3.2" markerHeight="3.2" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z"></path>
      </marker>
      <marker id="arrow-exit" viewBox="0 0 10 10" refX="8" refY="5"
        markerWidth="3.2" markerHeight="3.2" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z"></path>
      </marker>
    </defs>
    <rect class="lot-floor" x="4" y="4" width="${LOT_MAP_WIDTH - 8}"
      height="${LOT_MAP_HEIGHT - 8}" rx="18"></rect>
  `];

  const nodeById = new Map(nodes.map((node) => [node.id, node]));
  const pointForNode = (nodeId) => {
    const node = nodeById.get(nodeId);
    return node ? `${sx(node.x)},${sy(node.y)}` : null;
  };
  const pointsForRoute = (nodeIds) => {
    const points = nodeIds.map(pointForNode);
    return points.every(Boolean) ? points.join(" ") : null;
  };
  const pathForRoute = (nodeIds, finalPoint = null, initialPoint = null) => {
    const points = nodeIds
      .map((nodeId) => nodeById.get(nodeId))
      .filter(Boolean)
      .map((node) => [sx(node.x), sy(node.y)]);
    if (initialPoint) points.unshift(initialPoint);
    if (finalPoint) points.push(finalPoint);
    if (points.length < 2) return null;
    return points.map(([x, y], index) => `${index ? "L" : "M"} ${x} ${y}`).join(" ");
  };
  const activeMapRequest = requests.find(
    (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
  );
  const activeRouteRole = activeMapRequest
    ? activeMapRequest.request_type === "PARK_IN" ? "entry" : "exit"
    : null;
  const routeDangerForRole = (role) => alerts.some(
    (alert) => obstacleAffectsRole(alert, role)
  );
  const routeState = (role) => activeRouteRole == null
    ? "standby"
    : activeRouteRole === role
      ? `planned ${routeDangerForRole(role) ? "danger" : ""}`
      : "dimmed";

  // v4는 위쪽이 출차, 아래쪽이 입차 전용 통로다. 각 슬롯은 양쪽
  // 통로와 연결되어 입차는 아래→슬롯, 출차는 슬롯→위 순서로 이동한다.
  const entryLane = pointsForRoute([
    "entry_outer", "entry_gate", "entry_wait", "crossing_entry", "entry_a3",
  ]);
  const exitLane = pointsForRoute([
    "exit_a3", "crossing_exit", "exit_wait", "exit_gate", "exit_outer",
  ]);
  if (entryLane && exitLane) {
    parts.push(`
      <polyline class="lot-route-halo" points="${entryLane}"></polyline>
      <polyline class="lot-route entry ${routeState("entry")}" points="${entryLane}"></polyline>
      <polyline class="lot-route-halo" points="${exitLane}"></polyline>
      <polyline class="lot-route exit ${routeState("exit")}" points="${exitLane}"></polyline>
    `);

    for (const slot of placedSlots) {
      const entryPoint = pointForNode(`entry_${slot.id.toLowerCase()}`);
      const exitPoint = pointForNode(`exit_${slot.id.toLowerCase()}`);
      const slotTopPoint = `${sx(slot.x)},${sy(slot.y) - LOT_SLOT_HEIGHT / 2}`;
      const slotBottomPoint = `${sx(slot.x)},${sy(slot.y) + LOT_SLOT_HEIGHT / 2}`;
      const isActiveTarget = activeMapRequest?.slot_id === slot.id;
      if (
        entryPoint
        && (
          activeRouteRole == null
          || (activeRouteRole === "entry" && isActiveTarget)
        )
      ) {
        const branchState = activeRouteRole === "entry" && isActiveTarget
          ? `planned ${routeDangerForRole("entry") ? "danger" : ""}`
          : "standby";
        parts.push(`
          <polyline class="lot-route-branch entry ${branchState}"
            points="${entryPoint} ${slotBottomPoint}"></polyline>
        `);
      }
      if (
        exitPoint
        && (
          activeRouteRole == null
          || (activeRouteRole === "exit" && isActiveTarget)
        )
      ) {
        const branchState = activeRouteRole === "exit" && isActiveTarget
          ? `planned ${routeDangerForRole("exit") ? "danger" : ""}`
          : "standby";
        parts.push(`
          <polyline class="lot-route-branch exit ${branchState}"
            points="${slotTopPoint} ${exitPoint}"></polyline>
        `);
      }
    }

    if (activeMapRequest?.slot_id) {
      const targetSlot = placedSlots.find(
        (slot) => slot.id === activeMapRequest.slot_id
      );
      if (targetSlot) {
        const targetNodeId = `${activeRouteRole}_${targetSlot.id.toLowerCase()}`;
        let activePath = null;
        if (
          activeRouteRole === "entry"
          && ["ROBOT_ASSIGNED", "APPROACHING"].includes(activeMapRequest.status)
        ) {
          activePath = pathForRoute(["entry_outer", "entry_gate", "entry_wait"]);
        } else if (
          activeRouteRole === "entry"
          && activeMapRequest.status === "MOVING_TO_SLOT"
        ) {
          activePath = pathForRoute(
            ["entry_wait", "crossing_entry", targetNodeId],
            [sx(targetSlot.x), sy(targetSlot.y) + LOT_SLOT_HEIGHT / 2]
          );
        } else if (
          activeRouteRole === "exit"
          && ["ROBOT_ASSIGNED", "APPROACHING", "LIFTING"].includes(activeMapRequest.status)
        ) {
          activePath = pathForRoute(
            [targetNodeId],
            null,
            [sx(targetSlot.x), sy(targetSlot.y) - LOT_SLOT_HEIGHT / 2]
          );
        } else if (
          activeRouteRole === "exit"
          && activeMapRequest.status === "MOVING_TO_SLOT"
        ) {
          activePath = pathForRoute(
            [targetNodeId, "crossing_exit", "exit_wait", "exit_gate", "exit_outer"],
            null,
            [sx(targetSlot.x), sy(targetSlot.y) - LOT_SLOT_HEIGHT / 2]
          );
        }
        if (activePath) {
          parts.push(`
            <path class="lot-route-current ${activeRouteRole} ${
              routeDangerForRole(activeRouteRole) ? "danger" : ""
            }" d="${activePath}" marker-end="url(#arrow-${activeRouteRole})"></path>
            <circle class="lot-route-runner ${activeRouteRole} ${
              routeDangerForRole(activeRouteRole) ? "danger" : ""
            }" r="4.5">
              <animateMotion dur="4s" repeatCount="indefinite" path="${activePath}"></animateMotion>
            </circle>
          `);
        }
      }
    }

    const entryWait = nodeById.get("entry_wait");
    const exitWait = nodeById.get("exit_wait");
    if (entryWait && exitWait) {
      parts.push(`
        <text class="lot-route-label entry" x="${sx(entryWait.x)}" y="${sy(entryWait.y) - 13}">
          입차 동선
        </text>
        <text class="lot-route-label exit" x="${sx(exitWait.x)}" y="${sy(exitWait.y) + 22}">
          출차 동선
        </text>
      `);
    }
  }

  for (const zone of vehicleZones) {
    const cx = sx(zone.x);
    const cy = sy(zone.y);
    const directionLabel = zone.role === "entry" ? "입차 대기" : "출차 대기";
    const zoneRequest = requests.find((request) => {
      if (["COMPLETED", "CANCELLED"].includes(request.status)) return false;
      const requestRole = request.request_type === "PARK_IN" ? "entry" : "exit";
      if (requestRole !== zone.role) return false;
      if (zone.role === "entry") {
        return ["WAITING", "ROBOT_ASSIGNED", "APPROACHING", "LIFTING"].includes(request.status);
      }
      return ["MOVING_TO_SLOT", "RETURNING"].includes(request.status);
    });
    parts.push(`
      <g class="lot-operation-zone ${zoneRequest ? "active" : ""}">
        <rect class="lot-vehicle-zone-rect ${zone.role}"
          x="${cx - LOT_VEHICLE_ZONE_WIDTH / 2}" y="${cy - LOT_VEHICLE_ZONE_HEIGHT / 2}"
          width="${LOT_VEHICLE_ZONE_WIDTH}" height="${LOT_VEHICLE_ZONE_HEIGHT}"
          rx="14"></rect>
        <text class="lot-vehicle-zone-label" x="${cx}" y="${zoneRequest ? cy - 19 : cy + 5}">
          ${directionLabel}
        </text>
        ${zoneRequest ? `
          <g class="lot-waiting-vehicle ${zone.role}" aria-label="${zoneRequest.vehicle_number}">
            <rect x="${cx - 22}" y="${cy - 5}" width="44" height="20" rx="7"></rect>
            <path d="M ${cx - 13} ${cy - 5} L ${cx - 7} ${cy - 13}
              H ${cx + 8} L ${cx + 15} ${cy - 5} Z"></path>
            <circle cx="${cx - 13}" cy="${cy + 15}" r="4"></circle>
            <circle cx="${cx + 13}" cy="${cy + 15}" r="4"></circle>
            <text x="${cx}" y="${cy + 33}">${zoneRequest.vehicle_number}</text>
          </g>
        ` : ""}
      </g>
    `);
  }

  for (const role of ["exit", "entry"]) {
    const roleDocks = docks.filter((dock) => dock.role === role);
    if (!roleDocks.length) continue;
    const deployedRequest = requests.find((request) => {
      if (["COMPLETED", "CANCELLED"].includes(request.status)) return false;
      const requestRole = request.request_type === "PARK_IN" ? "entry" : "exit";
      return requestRole === role && assignedRobotIds(request).length > 0;
    });
    const cx = roleDocks.reduce((sum, dock) => sum + sx(dock.x), 0) / roleDocks.length;
    const cy = roleDocks.reduce((sum, dock) => sum + sy(dock.y), 0) / roleDocks.length;
    const dockLabelY = role === "exit"
      ? cy - LOT_DOCK_HEIGHT / 2 - 8
      : cy + LOT_DOCK_HEIGHT / 2 + 18;
    parts.push(`
      <g class="lot-operation-zone">
        <rect class="lot-dock-rect ${role} ${deployedRequest ? "deployed" : ""}"
          x="${cx - LOT_DOCK_WIDTH / 2}" y="${cy - LOT_DOCK_HEIGHT / 2}"
          width="${LOT_DOCK_WIDTH}" height="${LOT_DOCK_HEIGHT}" rx="14"></rect>
        <text class="lot-dock-label" x="${cx}" y="${dockLabelY}">
          ${deployedRequest
            ? `${role === "entry" ? "입차" : "출차"}팀 운용 중`
            : dockRoleLabels[role]}
        </text>
      </g>
    `);
  }

  if (showLidarMarkers) {
    const obstacleActive = alerts.some(
      (alert) => alert.category === "OBSTACLE"
    );
    for (const sensor of sensors) {
      const cx = sx(sensor.x);
      // 센서의 실제 x 좌표는 유지하고, y=0인 주행 구역 중앙선에 표시한다.
      const cy = sy(sensor.y);
      const isSelected = selectedMapItem?.type === "sensor" && selectedMapItem.id === sensor.id;
      parts.push(`
        <g class="lot-selectable" role="button" tabindex="0"
          data-entity-type="sensor" data-entity-id="${sensor.id}" aria-label="LiDAR ${sensor.id} ${sensor.status}">
          <circle class="lot-sensor-coverage ${sensor.status} ${obstacleActive ? "alert" : ""}"
            cx="${cx}" cy="${cy}" r="112"></circle>
          <circle class="lot-sensor-ring ${sensor.status} ${isSelected ? "selected" : ""}"
            cx="${cx}" cy="${cy}" r="16"></circle>
          <circle class="lot-sensor-dot ${sensor.status}" cx="${cx}" cy="${cy}" r="5"></circle>
          <text class="lot-sensor-label" x="${cx}" y="${cy - 23}">
            ${sensor.id} · ${sensor.status === "ONLINE" ? `${sensor.rate_hz ?? "-"} Hz` : sensor.status === "MOCK" ? "테스트" : "수신 대기"}
          </text>
          ${obstacleActive ? `
            <g class="lot-sensor-alert">
              <circle cx="${cx + 62}" cy="${cy - 50}" r="13"></circle>
              <text x="${cx + 62}" y="${cy - 46}">!</text>
            </g>
          ` : ""}
        </g>
      `);
    }
  }

  for (const slot of placedSlots) {
    const cx = sx(slot.x);
    const cy = sy(slot.y);
    const isSelected = selectedMapItem?.type === "slot" && selectedMapItem.id === slot.id;
    const slotRequest = requests.find(
      (request) => request.slot_id === slot.id
        && !["COMPLETED", "CANCELLED"].includes(request.status)
    );
    const hasVehicle = slot.status === "OCCUPIED" || (
      slotRequest?.request_type === "PARK_IN"
      && ["MOVING_TO_SLOT", "RETURNING"].includes(slotRequest.status)
    );
    const movementLabel = slotRequest
      ? slotRequest.request_type === "PARK_IN" ? "IN" : "OUT"
      : null;
    parts.push(`
      <g class="lot-selectable" role="button" tabindex="0"
        data-entity-type="slot" data-entity-id="${slot.id}" aria-label="${slot.id} ${statusLabels[slot.status]}">
      <rect
        class="lot-slot-rect ${slot.status} ${slotRequest ? "active-target" : ""}"
        x="${cx - LOT_SLOT_WIDTH / 2}" y="${cy - LOT_SLOT_HEIGHT / 2}"
        width="${LOT_SLOT_WIDTH}" height="${LOT_SLOT_HEIGHT}"
        rx="8"
      ></rect>
      ${isSelected ? `
        <rect class="lot-slot-selection-outline"
          x="${cx - LOT_SLOT_WIDTH / 2 - 4}" y="${cy - LOT_SLOT_HEIGHT / 2 - 4}"
          width="${LOT_SLOT_WIDTH + 8}" height="${LOT_SLOT_HEIGHT + 8}" rx="11"></rect>
      ` : ""}
      ${hasVehicle ? `
        <text class="lot-slot-vehicle" x="${cx}" y="${cy - 35}" aria-hidden="true">🚗</text>
      ` : ""}
      ${movementLabel ? `
        <rect class="lot-slot-movement ${slotRequest.request_type}"
          x="${cx + 18}" y="${cy - 70}" width="30" height="17" rx="8.5"></rect>
        <text class="lot-slot-movement-label" x="${cx + 33}" y="${cy - 58}">
          ${movementLabel}
        </text>
      ` : ""}
      <text class="lot-slot-label" x="${cx}" y="${hasVehicle ? cy + 5 : cy - 5}">
        ${slot.id}${slot.is_accessible ? " ♿" : ""}
      </text>
      <text class="lot-slot-sub" x="${cx}" y="${hasVehicle ? cy + 26 : cy + 14}">
        ${slotRequest ? `${statusLabels[slot.status]} · 작업 중` : statusLabels[slot.status]}
      </text>
      </g>
    `);
  }

  const locatedObstacles = alerts.filter(
    (alert) => alert.category === "OBSTACLE"
      && alert.location_x != null
      && alert.location_y != null
      && Number.isFinite(Number(alert.location_x))
      && Number.isFinite(Number(alert.location_y))
  );
  for (const alert of locatedObstacles) {
    const cx = sx(Number(alert.location_x));
    const cy = sy(Number(alert.location_y));
    const isSelected = selectedMapItem?.type === "obstacle"
      && String(selectedMapItem.id) === String(alert.id);
    const markerLabel = alert.sensor_id
      ? `장애물 · ${alert.sensor_id}`
      : "감지 장애물";
    parts.push(`
      <g class="lot-selectable lot-obstacle-marker ${isSelected ? "selected" : ""}"
        role="button" tabindex="0" data-entity-type="obstacle"
        data-entity-id="${alert.id}" aria-label="${markerLabel}">
        <circle class="lot-obstacle-radius" cx="${cx}" cy="${cy}" r="44"></circle>
        <circle class="lot-obstacle-pulse" cx="${cx}" cy="${cy}" r="25"></circle>
        <circle class="lot-obstacle-core" cx="${cx}" cy="${cy}" r="15"></circle>
        <text class="lot-obstacle-symbol" x="${cx}" y="${cy + 5}">!</text>
        <rect class="lot-obstacle-label-bg" x="${cx - 43}" y="${cy - 55}"
          width="86" height="22" rx="11"></rect>
        <text class="lot-obstacle-label" x="${cx}" y="${cy - 40}">
          ${markerLabel}
        </text>
      </g>
    `);
  }

  // 작업에 두 로봇이 배정되면 실제 좌표를 연결해 편대 상태를 표시한다.
  // 별도 거리 센서값이 없으므로 두 ROS map 좌표 사이의 거리를 사용한다.
  for (const request of requests) {
    if (["COMPLETED", "CANCELLED"].includes(request.status)) continue;
    const members = assignedRobotIds(request)
      .map((robotId) => placedRobots.find((robot) => robot.id === robotId))
      .filter(Boolean)
      .slice(0, 2);
    if (members.length !== 2) continue;

    const [leader, follower] = members;
    const x1 = sx(leader.x) + (ROBOT_VISUAL_X_OFFSET[leader.id] || 0);
    const y1 = sy(leader.y);
    const x2 = sx(follower.x) + (ROBOT_VISUAL_X_OFFSET[follower.id] || 0);
    const y2 = sy(follower.y);
    const centerX = (x1 + x2) / 2;
    const centerY = (y1 + y2) / 2;
    const displayDistance = Math.hypot(x2 - x1, y2 - y1) || 1;
    const labelOffset = 48;
    const labelX = centerX + (-(y2 - y1) / displayDistance) * labelOffset;
    const labelY = centerY + ((x2 - x1) / displayDistance) * labelOffset;
    const gap = pointDistance(leader, follower);
    const obstacleActive = alerts.some(
      (alert) => members.some((robot) => obstacleAffectsRobot(alert, robot))
    );
    const formationTone = obstacleActive ? "danger" : gap > 3 ? "warning" : "normal";
    const teamLabel = request.request_type === "PARK_IN" ? "입차팀" : "출차팀";
    const formationLabel = obstacleActive
      ? `${request.request_type === "PARK_IN" ? "입차" : "출차"}팀 대기`
      : gap > 3
        ? `${teamLabel} · 간격 조정`
        : `${teamLabel} · ${requestOperationalStatusLabel(request, alerts)}`;
    const labelWidth = Math.max(88, Math.min(132, formationLabel.length * 8 + 18));
    parts.push(`
      <g class="lot-formation ${formationTone}" aria-label="${formationLabel}">
        <line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"></line>
        <line class="label-guide" x1="${centerX}" y1="${centerY}"
          x2="${labelX}" y2="${labelY}"></line>
        <rect x="${labelX - labelWidth / 2}" y="${labelY - 12}"
          width="${labelWidth}" height="24" rx="12"></rect>
        <text x="${labelX}" y="${labelY + 4}">${formationLabel}</text>
      </g>
    `);
  }

  for (const robot of placedRobots) {
    const cx = sx(robot.x) + (ROBOT_VISUAL_X_OFFSET[robot.id] || 0);
    const cy = sy(robot.y);
    const isSelected = selectedMapItem?.type === "robot" && selectedMapItem.id === robot.id;
    const currentRequest = robot.current_task_id == null
      ? null
      : requests.find((request) => request.id === robot.current_task_id);
    const pairedRobotIds = currentRequest ? assignedRobotIds(currentRequest) : [robot.id];
    const cooperationIndex = currentRequest
      ? pairedRobotIds.indexOf(robot.id)
      : -1;
    const cooperationRole = cooperationIndex === 0
      ? "리더"
      : cooperationIndex === 1 ? "팔로워" : null;
    const obstacleAlert = alerts.find(
      (alert) => obstacleAffectsRobot(alert, robot)
    );
    const visualStatus = obstacleAlert ? "PAUSED" : robot.status;
    const statusText = obstacleAlert ? "일시 정지" : statusLabels[robot.status] || robot.status;
    const badgeWidth = visualStatus === "OFFLINE" ? 50 : statusText.length >= 4 ? 44 : 38;
    const badgeX = cx + LOT_ROBOT_CARD_WIDTH / 2 - badgeWidth - 7;
    const speech = robotSpeechBubbles.get(robot.id);
    if (speech && speech.expiresAt <= Date.now()) {
      robotSpeechBubbles.delete(robot.id);
    }
    const activeSpeech = robotSpeechBubbles.get(robot.id);
    const robotSubtitle = robotMapSubtitle(robot, requests, Boolean(obstacleAlert));
    const isWorkingMarker = Boolean(
      currentRequest
      && !["COMPLETED", "CANCELLED"].includes(currentRequest.status)
    );
    const robotIconY = robotSubtitle ? cy + 2 : cy + 10;
    const speechWidth = activeSpeech
      ? Math.max(54, Math.min(96, activeSpeech.text.length * 9 + 20))
      : 0;
    if (isWorkingMarker) {
      const roleLetter = cooperationRole === "리더"
        ? "L"
        : cooperationRole === "팔로워" ? "F" : "R";
      parts.push(`
        <g class="lot-selectable lot-robot-marker lot-robot-compact ${visualStatus}"
          role="button" tabindex="0" data-entity-type="robot" data-entity-id="${robot.id}"
          aria-label="${shortRobotName(robot.id)} ${statusText}">
          <circle class="lot-robot-compact-bg ${visualStatus} ${isSelected ? "selected" : ""}"
            cx="${cx}" cy="${cy}" r="25"></circle>
          <text class="lot-robot-compact-icon" x="${cx}" y="${cy + 2}" aria-hidden="true">🤖</text>
          <circle class="lot-robot-compact-role-bg" cx="${cx + 19}" cy="${cy - 19}" r="10"></circle>
          <text class="lot-robot-compact-role" x="${cx + 19}" y="${cy - 15}">${roleLetter}</text>
        </g>
      `);
      continue;
    }
    parts.push(`
      <g class="lot-selectable lot-robot-marker ${visualStatus}" role="button" tabindex="0"
        data-entity-type="robot" data-entity-id="${robot.id}" aria-label="${shortRobotName(robot.id)} ${statusText}">
        ${activeSpeech ? `
          <g class="lot-robot-speech" aria-hidden="true">
            <rect x="${cx - speechWidth / 2}" y="${cy - 75}"
              width="${speechWidth}" height="23" rx="11"></rect>
            <path d="M ${cx - 5} ${cy - 52} L ${cx} ${cy - 45} L ${cx + 5} ${cy - 52} Z"></path>
            <text x="${cx}" y="${cy - 59}">${activeSpeech.text}</text>
          </g>
        ` : ""}
        <rect
          class="lot-robot-card ${visualStatus} ${isSelected ? "selected" : ""}"
          x="${cx - LOT_ROBOT_CARD_WIDTH / 2}" y="${cy - LOT_ROBOT_CARD_HEIGHT / 2}"
          width="${LOT_ROBOT_CARD_WIDTH}" height="${LOT_ROBOT_CARD_HEIGHT}" rx="12"
        ></rect>
        <text class="lot-robot-name" x="${cx - LOT_ROBOT_CARD_WIDTH / 2 + 9}" y="${cy - 30}">
          ${shortRobotName(robot.id)}${cooperationRole ? ` · ${cooperationRole === "리더" ? "L" : "F"}` : ""}
        </text>
        <rect class="lot-robot-badge ${visualStatus}"
          x="${badgeX}" y="${cy - 40}" width="${badgeWidth}" height="18" rx="9"></rect>
        <text class="lot-robot-badge-label ${visualStatus}"
          x="${badgeX + badgeWidth / 2}" y="${cy - 27}">${statusText}</text>
        <text class="lot-robot-icon" x="${cx}" y="${robotIconY}" aria-hidden="true">🤖</text>
        ${robotSubtitle ? `
          <text class="lot-robot-subtitle" x="${cx}" y="${cy + 33}">
            ${robotSubtitle}
          </text>
        ` : ""}
      </g>
    `);
  }

  svg.innerHTML = parts.join("");
  svg.querySelectorAll(".lot-selectable").forEach((element) => {
    const select = () => selectMapItem(element.dataset.entityType, element.dataset.entityId);
    element.addEventListener("click", select);
    element.addEventListener("keydown", (event) => {
      if (!["Enter", " "].includes(event.key)) return;
      event.preventDefault();
      select();
    });
  });
}

function easeOutQuad(t) {
  return 1 - (1 - t) * (1 - t);
}

// 최신 로봇 배열의 x/y를, 진행 중인 애니메이션이 있으면 보간된 좌표로 덮어써서 반환한다.
function applyRobotDisplayPositions(robots) {
  return robots.map((robot) => {
    const display = robotDisplayPositions.get(robot.id);
    if (!display || robot.x == null || robot.y == null) return robot;
    return { ...robot, x: display.x, y: display.y };
  });
}

// 새 좌표가 도착하면 즉시 스냅하지 않고, 현재 보이는 위치 -> 새 좌표로 가는
// 애니메이션 목표만 세팅한다. 처음 보는 로봇은 애니메이션 없이 바로 배치.
function updateRobotAnimationTargets(robots) {
  const now = performance.now();
  const seenIds = new Set();

  for (const robot of robots) {
    if (robot.x == null || robot.y == null) continue;
    seenIds.add(robot.id);
    const current = robotDisplayPositions.get(robot.id);

    if (!current) {
      robotDisplayPositions.set(robot.id, { x: robot.x, y: robot.y });
      continue;
    }
    if (current.x === robot.x && current.y === robot.y) {
      robotAnimations.delete(robot.id);
      continue;
    }
    robotAnimations.set(robot.id, {
      fromX: current.x,
      fromY: current.y,
      toX: robot.x,
      toY: robot.y,
      startTime: now,
    });
  }

  for (const robotId of [...robotDisplayPositions.keys()]) {
    if (!seenIds.has(robotId)) {
      robotDisplayPositions.delete(robotId);
      robotAnimations.delete(robotId);
    }
  }
}

function renderLatestLotMap() {
  if (!latestDashboard) return;
  renderLotMap(
    latestDashboard.slots,
    applyRobotDisplayPositions(latestDashboard.robots),
    latestDashboard.map,
    latestDashboard.sensors || [],
    latestDashboard.requests || [],
    latestDashboard.alerts || []
  );
}

function stepRobotAnimations(timestamp) {
  let stillAnimating = false;

  for (const [robotId, anim] of robotAnimations) {
    const t = Math.min(1, (timestamp - anim.startTime) / ROBOT_MOVE_ANIMATION_MS);
    const eased = easeOutQuad(t);
    robotDisplayPositions.set(robotId, {
      x: anim.fromX + (anim.toX - anim.fromX) * eased,
      y: anim.fromY + (anim.toY - anim.fromY) * eased,
    });
    if (t >= 1) {
      robotAnimations.delete(robotId);
    } else {
      stillAnimating = true;
    }
  }

  renderLatestLotMap();
  robotAnimationFrame = stillAnimating ? requestAnimationFrame(stepRobotAnimations) : null;
}

function ensureRobotAnimationLoop() {
  if (robotAnimationFrame == null) {
    robotAnimationFrame = requestAnimationFrame(stepRobotAnimations);
  }
}

function selectMapItem(type, id) {
  selectedMapItem = { type, id };
  renderLatestLotMap();
  if (!latestDashboard) return;
  renderSelectionDetail(latestDashboard);
}

function focusObstacleAlert(alertId) {
  selectedMapItem = { type: "obstacle", id: String(alertId) };
  showLidarMarkers = true;
  const lidarButton = document.getElementById("lidarVisibilityButton");
  lidarButton.classList.add("active");
  lidarButton.setAttribute("aria-pressed", "true");
  lidarButton.textContent = "센서 영역 숨기기";
  activateWorkspaceTab("live", true);
  renderLatestLotMap();
  if (latestDashboard) renderSelectionDetail(latestDashboard);
  document.querySelector(".live-floor-panel")?.scrollIntoView({
    behavior: "smooth",
    block: "start",
  });
}

function toggleLidarMarkers() {
  showLidarMarkers = !showLidarMarkers;
  const button = document.getElementById("lidarVisibilityButton");
  button.classList.toggle("active", showLidarMarkers);
  button.setAttribute("aria-pressed", String(showLidarMarkers));
  button.textContent = showLidarMarkers ? "센서 영역 숨기기" : "센서 영역 보기";

  if (!showLidarMarkers && selectedMapItem?.type === "sensor") {
    selectedMapItem = null;
  }
  renderLatestLotMap();
  if (!latestDashboard) return;
  renderSelectionDetail(latestDashboard);
}

function pointDistance(left, right) {
  if (left?.x == null || left?.y == null || right?.x == null || right?.y == null) {
    return Number.POSITIVE_INFINITY;
  }
  return Math.hypot(left.x - right.x, left.y - right.y);
}

function describeRobotLocation(robot, mapInfo, slots) {
  if (robot.x == null || robot.y == null) return "위치 정보 없음";

  const nearbyDock = (mapInfo?.docks || [])
    .map((dock) => ({ dock, distance: pointDistance(robot, dock) }))
    .sort((left, right) => left.distance - right.distance)[0];
  if (nearbyDock?.distance <= 1.5) {
    return nearbyDock.dock.role === "entry"
      ? "입차 로봇 대기"
      : nearbyDock.dock.role === "exit"
        ? "출차 로봇 대기"
        : "로봇 대기 구역";
  }

  const nearbyVehicleZone = (mapInfo?.vehicle_zones || [])
    .map((zone) => ({ zone, distance: pointDistance(robot, zone) }))
    .sort((left, right) => left.distance - right.distance)[0];
  if (nearbyVehicleZone?.distance <= 2.2) {
    return nearbyVehicleZone.zone.role === "entry"
      ? "입차 차량 대기 구역"
      : "출차 차량 대기 구역";
  }

  if (Math.abs(robot.y - (-6.875)) <= 1.2) return "입차 전용 통로";
  if (Math.abs(robot.y - 6.875) <= 1.2) return "출차 전용 통로";

  const nearbySlot = slots
    .filter((slot) => slot.x != null && slot.y != null)
    .map((slot) => ({ slot, distance: pointDistance(robot, slot) }))
    .sort((left, right) => left.distance - right.distance)[0];
  if (nearbySlot?.distance <= 3) return `${nearbySlot.slot.id} 앞`;

  return "주차장 내부";
}

function robotOperationLabel(robot, request) {
  if (robot.status === "OFFLINE") return "연결 끊김";
  if (robot.status === "ERROR") return "오류";
  return request ? requestStatusLabel(request) : statusLabels[robot.status] || robot.status;
}

function obstacleZoneLabel(zoneId) {
  if (!zoneId) return "주행 통로";
  const zoneIds = String(zoneId)
    .split(",")
    .map((item) => item.trim())
    .filter(Boolean);
  if (zoneIds.length > 1) {
    return zoneIds.map((item) => obstacleZoneLabel(item)).join(" · ");
  }
  const slotMatch = zoneId.match(/^Z(IN|OUT)_?(A[1-3])/);
  if (slotMatch) {
    return `${slotMatch[2]} ${slotMatch[1] === "IN" ? "입차" : "출차"} 진입로`;
  }
  if (zoneId.startsWith("ZIN")) return "입차 주행 통로";
  if (zoneId.startsWith("ZOUT")) return "출차 주행 통로";
  return zoneId;
}

function renderSelectionDetail(dashboard) {
  const detail = document.getElementById("selectionDetail");
  const slots = dashboard?.slots || [];
  const robots = dashboard?.robots || [];
  const sensors = dashboard?.sensors || [];
  if (!selectedMapItem) {
    detail.innerHTML = `
      <div class="detail-empty-icon" aria-hidden="true">⌖</div>
      <span class="detail-kicker">선택 정보</span>
      <h3>운영 객체를 선택하세요</h3>
      <p>주차면·로봇·센서를 선택하면 현재 상태와 작업 정보를 확인할 수 있습니다.</p>
    `;
    return;
  }

  if (selectedMapItem.type === "task") {
    const request = (dashboard.requests || []).find(
      (item) => String(item.id) === String(selectedMapItem.id)
    );
    if (!request) {
      selectedMapItem = null;
      renderSelectionDetail(dashboard);
      return;
    }
    const requestObstacle = obstacleForRequest(request, dashboard.alerts || []);
    const assignedIds = assignedRobotIds(request);
    const assignedRobots = assignedIds
      .map((robotId) => robots.find((robot) => robot.id === robotId))
      .filter(Boolean);
    const formationGap = assignedRobots.length >= 2
      ? pointDistance(assignedRobots[0], assignedRobots[1])
      : null;
    const unavailableSensors = sensors.filter((sensor) => {
      if (dashboard.system?.mode === "mock") {
        return !["MOCK", "ONLINE"].includes(sensor.status);
      }
      return sensor.status !== "ONLINE";
    });
    const safetyLabel = requestObstacle
      ? `${obstacleZoneLabel(requestObstacle.zone_id)} 장애물 대기`
      : unavailableSensors.length
        ? `${unavailableSensors.map((sensor) => sensor.id).join(" · ")} 미수신 · 안전 감지 제한 운용`
        : "정상";
    const cooperationLabel = assignedRobots.length < 2
      ? "로봇 배정 중"
      : formationGap != null && formationGap > 3
        ? `간격 조정 필요 · ${formationGap.toFixed(1)} m`
        : "정상";
    detail.innerHTML = `
      <span class="detail-kicker">현재 작업 상세</span>
      <div class="detail-title-row">
        <h3>${requestTypeLabels[request.request_type]} #${request.id}</h3>
      </div>
      <div class="detail-status-summary ${requestObstacle ? "PAUSED" : "BUSY"}">
        <span>현재 단계</span>
        <strong>${requestOperationalStatusLabel(request, dashboard.alerts || [])}</strong>
        <small>${requestObstacle ? "장애물 해소 후 자동 재개" : `경과 ${formatElapsed(request.created_at)}`}</small>
      </div>
      <div class="robot-task-progress task-detail-progress">
        ${renderTaskStepper(request, true)}
      </div>
      <dl class="detail-list robot-detail-list">
        <div><dt>차량 번호</dt><dd>${request.vehicle_number}</dd></div>
        <div><dt>목표 주차면</dt><dd>${request.slot_id || "배정 중"}</dd></div>
        <div><dt>담당 로봇</dt><dd>${assignedIds.length ? assignedRobotTableLabel(request) : "배정 중"}</dd></div>
        <div><dt>경과 시간</dt><dd>${formatElapsed(request.created_at)}</dd></div>
        <div><dt>협동 상태</dt><dd><span class="detail-state ${
          formationGap != null && formationGap > 3 ? "warning" : "normal"
        }">● ${cooperationLabel}</span></dd></div>
        <div><dt>안전 상태</dt><dd><span class="detail-state ${
          requestObstacle ? "danger" : unavailableSensors.length ? "warning" : "normal"
        }">● ${safetyLabel}</span></dd></div>
      </dl>
    `;
    return;
  }

  if (selectedMapItem.type === "slot") {
    const slot = slots.find((item) => item.id === selectedMapItem.id);
    if (!slot) return;
    const currentRequest = (dashboard.requests || []).find(
      (request) => request.slot_id === slot.id && !["COMPLETED", "CANCELLED"].includes(request.status)
    );
    const requestObstacle = currentRequest
      ? obstacleForRequest(currentRequest, dashboard.alerts || [])
      : null;
    const assignedIds = currentRequest ? assignedRobotIds(currentRequest) : [];
    detail.innerHTML = `
      <span class="detail-kicker">주차면 상세</span>
      <div class="detail-title-row">
        <h3>${slot.id}${slot.is_accessible ? " ♿" : ""}</h3>
      </div>
      <div class="detail-status-summary ${slot.status}">
        <span>현재 주차면 상태</span>
        <strong>${statusLabels[slot.status]}</strong>
        <small>${slot.vehicle_number || (currentRequest ? `${currentRequest.vehicle_number} 작업 중` : "배정된 차량 없음")}</small>
      </div>
      <dl class="detail-list">
        ${slot.vehicle_number ? `
          <div><dt>차량 번호</dt><dd>${slot.vehicle_number}</dd></div>
        ` : ""}
        ${currentRequest ? `
          <div><dt>현재 작업</dt><dd>${requestTypeLabels[currentRequest.request_type]} 요청 #${currentRequest.id} · ${requestOperationalStatusLabel(currentRequest, dashboard.alerts || [])}</dd></div>
          ${requestObstacle ? `
            <div class="detail-warning-row"><dt>대기 사유</dt><dd>${obstacleZoneLabel(requestObstacle.zone_id)} 장애물 · 해소 후 자동 재개</dd></div>
          ` : ""}
        ` : ""}
        ${assignedIds.length ? `
          <div><dt>할당 로봇</dt><dd>${assignedRobotLabel(currentRequest)}</dd></div>
        ` : ""}
      </dl>
    `;
    return;
  }

  if (selectedMapItem.type === "sensor") {
    const sensor = sensors.find((item) => item.id === selectedMapItem.id);
    if (!sensor) return;
    const sensorLabel = sensor.status === "ONLINE" ? "정상 수신" : sensor.status === "MOCK" ? "테스트 데이터" : "수신 대기";
    detail.innerHTML = `
      <span class="detail-kicker">천장 LiDAR 상세</span>
      <div class="detail-title-row">
        <h3>${sensor.id}</h3>
      </div>
      <div class="detail-status-summary ${sensor.status}">
        <span>센서 연결 상태</span>
        <strong>${sensorLabel}</strong>
        <small>${sensor.last_seen_sec == null ? "수신 기록 없음" : `${sensor.last_seen_sec}초 전 데이터 수신`}</small>
      </div>
      <dl class="detail-list">
        <div><dt>ROS2 토픽</dt><dd class="topic-value">${sensor.topic}</dd></div>
        <div><dt>수신 주기</dt><dd>${sensor.rate_hz == null ? "-" : `${sensor.rate_hz} Hz`}</dd></div>
        <div><dt>마지막 수신</dt><dd>${sensor.last_seen_sec == null ? "수신 기록 없음" : `${sensor.last_seen_sec}초 전`}</dd></div>
        <div><dt>운영 영향</dt><dd>${
          ["ONLINE", "MOCK"].includes(sensor.status)
            ? "안전 감지 정상"
            : "안전 감지 제한 · 관제 확인 필요"
        }</dd></div>
      </dl>
    `;
    return;
  }

  if (selectedMapItem.type === "obstacle") {
    const alert = (dashboard.alerts || []).find(
      (item) => item.category === "OBSTACLE"
        && String(item.id) === String(selectedMapItem.id)
    );
    if (!alert) {
      selectedMapItem = null;
      renderSelectionDetail(dashboard);
      return;
    }
    const affectedRobots = affectedRobotsForObstacle(alert, robots);
    const affectedIds = new Set(affectedRobots.map((robot) => robot.id));
    const relatedRequest = (dashboard.requests || []).find(
      (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
        && assignedRobotIds(request).some((robotId) => affectedIds.has(robotId))
    );
    const coordinateLabel = alert.location_x == null || alert.location_y == null
      ? "위치 정보 없음"
      : `x ${Number(alert.location_x).toFixed(2)} · y ${Number(alert.location_y).toFixed(2)} m`;
    detail.innerHTML = `
      <span class="detail-kicker danger">LiDAR 안전 감지</span>
      <div class="detail-title-row">
        <h3><span class="detail-danger-icon">!</span>감지 장애물</h3>
      </div>
      <div class="detail-status-summary PAUSED">
        <span>현재 안전 조치</span>
        <strong>${
          affectedRobots.length
            ? "영향 로봇 일시 정지"
            : "해당 통로 신규 진입 차단"
        }</strong>
        <small>${alert.message}</small>
      </div>
      <dl class="detail-list">
        <div><dt>감지 센서</dt><dd>${alert.sensor_id || "센서 ID 미수신"}</dd></div>
        <div><dt>감지 구역</dt><dd>${obstacleZoneLabel(alert.zone_id)}</dd></div>
        <div><dt>감지 구역 중심</dt><dd>${coordinateLabel}</dd></div>
        <div><dt>영향 로봇</dt><dd>${
          affectedRobots.length
            ? affectedRobots.map((robot) => shortRobotName(robot.id)).join(" · ")
            : "현재 이동 로봇 없음"
        }</dd></div>
        <div><dt>관련 작업</dt><dd>${
          relatedRequest
            ? `${requestTypeLabels[relatedRequest.request_type]} #${relatedRequest.id}`
            : "관련 진행 작업 없음"
        }</dd></div>
        <div><dt>발생 시각</dt><dd>${formatDateTime(alert.created_at)}</dd></div>
      </dl>
    `;
    return;
  }

  const robot = robots.find((item) => item.id === selectedMapItem.id);
  if (!robot) return;
  const currentRequest = robot.current_task_id == null
    ? null
    : (dashboard.requests || []).find((request) => request.id === robot.current_task_id);
  const robotAlert = (dashboard.alerts || []).find(
    (alert) => alert.category === "ROBOT_ERROR"
      ? alert.robot_id === robot.id
      : obstacleAffectsRobot(alert, robot)
  );
  const communicationLabel = robot.status === "OFFLINE"
    ? "데이터 미수신"
    : dashboard.system?.mode === "mock"
      ? "MOCK 테스트 데이터"
      : "관제 상태 수신됨";
  const taskLabel = currentRequest
    ? `${requestTypeLabels[currentRequest.request_type]} · ${currentRequest.vehicle_number}`
    : "할당 없음";
  const requestLabel = currentRequest
    ? `#${currentRequest.id} · ${formatDateTime(currentRequest.created_at)}`
    : "없음";
  const targetLabel = currentRequest
    ? currentRequest.slot_id || "주차면 배정 중"
    : "없음";
  const safetyLabel = robotAlert?.message || robot.error_message || "이상 없음";
  const isObstaclePaused = robotAlert?.category === "OBSTACLE";
  detail.innerHTML = `
    <span class="detail-kicker">로봇 상세</span>
    <div class="detail-title-row">
      <h3><span class="detail-robot-icon">🤖</span>${shortRobotName(robot.id)}</h3>
    </div>
    <div class="detail-status-summary ${isObstaclePaused ? "PAUSED" : robot.status}">
      <span>현재 운영 상태</span>
      <strong>${isObstaclePaused ? "장애물 대기" : robotOperationLabel(robot, currentRequest)}</strong>
      <small>${describeRobotLocation(robot, dashboard.map, slots)}</small>
    </div>
    ${currentRequest ? `
      <div class="robot-task-progress">
        <span>현재 단계 · ${requestOperationalStatusLabel(currentRequest, dashboard.alerts || [])}</span>
        ${renderTaskStepper(currentRequest, true)}
      </div>
    ` : ""}
    <dl class="detail-list robot-detail-list">
      <div><dt>현재 위치</dt><dd>${describeRobotLocation(robot, dashboard.map, slots)}</dd></div>
      <div><dt>현재 작업</dt><dd>${taskLabel}</dd></div>
      <div><dt>목표 주차면</dt><dd>${targetLabel}</dd></div>
      <div><dt>할당 요청</dt><dd>${requestLabel}</dd></div>
      <div><dt>통신</dt><dd><span class="detail-state ${robot.status === "OFFLINE" ? "warning" : "normal"}">● ${communicationLabel}</span></dd></div>
      <div><dt>안전</dt><dd><span class="detail-state ${robotAlert || robot.error_message ? "danger" : "normal"}">● ${safetyLabel}</span></dd></div>
    </dl>
  `;
}

function workflowEventForStatus(request) {
  const assigned = assignedRobotLabel(request);
  const definitions = {
    WAITING: ["요청 대기", `${request.vehicle_number} · 로봇 배정 대기`],
    ROBOT_ASSIGNED: ["협업 로봇 배정", `${request.vehicle_number} · ${assigned}`],
    APPROACHING: ["차량 접근 시작", `${assigned} · ${request.vehicle_number}`],
    LIFTING: ["차량 인양 중", `${assigned} · ${request.vehicle_number}`],
    MOVING_TO_SLOT: ["주차면 이동 중", `${request.slot_id || "주차면"} · ${request.vehicle_number}`],
    RETURNING: ["로봇 복귀 중", `${assigned} · 대기 구역 이동`],
    COMPLETED: [
      request.request_type === "PARK_IN" ? "주차 완료" : "출차 완료",
      `${request.slot_id || "주차면"} · ${request.vehicle_number}`,
    ],
    CANCELLED: ["요청 취소", `${request.vehicle_number} · 요청 #${request.id}`],
  };
  const [label, description] = definitions[request.status] || [
    requestStatusLabel(request),
    request.vehicle_number,
  ];
  const motionStatuses = new Set([
    "ROBOT_ASSIGNED",
    "APPROACHING",
    "LIFTING",
    "MOVING_TO_SLOT",
    "RETURNING",
  ]);
  return {
    time: new Date().toISOString(),
    tone: request.status === "CANCELLED"
      ? "danger"
      : request.status === "COMPLETED"
        ? "success"
        : motionStatuses.has(request.status) ? "motion" : "primary",
    category: motionStatuses.has(request.status) ? "robot" : "task",
    request_id: request.id,
    label,
    description,
  };
}

function showRobotSpeech(robotIds, text, durationMs = 2000) {
  const expiresAt = Date.now() + durationMs;
  for (const robotId of robotIds) {
    robotSpeechBubbles.set(robotId, { text, expiresAt });
  }
}

function showRequestStatusSpeech(request, system) {
  const robotIds = assignedRobotIds(request);
  if (!robotIds.length) return;
  const isDemo = system?.mode === "mock";
  const messages = isDemo
    ? {
        RETURNING: "복귀합니다",
        COMPLETED: "도착!",
      }
    : {
        RETURNING: "대기 구역 복귀",
        COMPLETED: "복귀 완료",
      };
  const message = messages[request.status];
  if (message) showRobotSpeech(robotIds, message);
}

function captureAlertSpeech(alerts, robots, system) {
  for (const alert of alerts) {
    if (seenSpeechAlertIds.has(alert.id)) continue;
    seenSpeechAlertIds.add(alert.id);
    if (alert.category !== "OBSTACLE") continue;
    const targetIds = affectedRobotsForObstacle(alert, robots)
      .map((robot) => robot.id);
    showRobotSpeech(
      targetIds,
      system?.mode === "mock" ? "장애물 발견!" : "장애물 감지",
      2500
    );
  }
}

function captureRequestEvents(requests, system) {
  if (!requests.length) {
    lastRequestStates.clear();
    recentWorkflowEvents = [];
    selectedTaskEventRequestId = null;
    selectedEventScope = "all";
    robotSpeechBubbles.clear();
    return;
  }
  if (
    selectedTaskEventRequestId != null
    && !requests.some((request) => request.id === selectedTaskEventRequestId)
  ) {
    selectedTaskEventRequestId = null;
    selectedEventScope = "all";
  }

  for (const request of [...requests].reverse()) {
    const previousStatus = lastRequestStates.get(request.id);
    if (previousStatus == null) {
      recentWorkflowEvents.push({
        time: request.created_at,
        tone: "primary",
        category: "task",
        request_id: request.id,
        label: `${requestTypeLabels[request.request_type]} 요청 #${request.id} 등록`,
        description: request.vehicle_number,
      });
      if (request.status !== "WAITING") {
        recentWorkflowEvents.push(workflowEventForStatus(request));
      }
      if (!["COMPLETED", "CANCELLED"].includes(request.status)) {
        showRequestStatusSpeech(request, system);
      }
    } else if (previousStatus !== request.status) {
      recentWorkflowEvents.push(workflowEventForStatus(request));
      showRequestStatusSpeech(request, system);
    }
    lastRequestStates.set(request.id, request.status);
  }

  recentWorkflowEvents = recentWorkflowEvents.slice(-20);
}

function eventCategoryForAlert(alert) {
  if (alert.category === "SENSOR") return "sensor";
  if (["EMERGENCY_STOP", "OBSTACLE"].includes(alert.category)) return "safety";
  if (alert.category === "ROBOT_ERROR") return "robot";
  return "safety";
}

function renderRecentEvents(alerts) {
  let events = [
    ...alerts.map((alert) => ({
      time: alert.created_at,
      tone: alert.level === "ERROR" ? "danger" : "warning",
      category: eventCategoryForAlert(alert),
      request_id: null,
      label: alertLabels[alert.category] || "시스템 이벤트",
      description: alert.message,
    })),
    ...recentWorkflowEvents,
  ]
    .sort((a, b) => new Date(b.time) - new Date(a.time));

  if (selectedEventScope === "selected" && selectedTaskEventRequestId != null) {
    events = events.filter(
      (event) => event.request_id === selectedTaskEventRequestId
    );
  }
  if (selectedEventCategory !== "all") {
    events = events.filter(
      (event) => event.category === selectedEventCategory
    );
  }
  events = events.slice(0, 12);

  const container = document.getElementById("recentEventList");
  const selectedRequest = latestDashboard?.requests?.find(
    (request) => request.id === selectedTaskEventRequestId
  );
  const selectedLabel = document.getElementById("selectedEventTaskLabel");
  selectedLabel.textContent = (
    selectedEventScope === "selected" && selectedRequest
  )
    ? `#${selectedRequest.id} ${requestTypeLabels[selectedRequest.request_type]} · ${selectedRequest.vehicle_number}`
    : "전체 시스템 이벤트";

  document.getElementById("allEventsButton").classList.toggle(
    "active", selectedEventScope === "all"
  );
  const selectedButton = document.getElementById("selectedTaskEventsButton");
  selectedButton.disabled = selectedTaskEventRequestId == null;
  selectedButton.classList.toggle(
    "active", selectedEventScope === "selected"
  );

  if (!events.length) {
    container.innerHTML = `
      <p class="recent-events-empty">
        ${selectedEventScope === "selected"
          ? "선택한 작업에 해당하는 이벤트가 없습니다."
          : "선택한 조건의 최근 이벤트가 없습니다."}
      </p>
    `;
    return;
  }

  const eventIcon = (event) => {
    if (event.tone === "danger" || event.tone === "warning") return "!";
    if (event.tone === "success") return "✓";
    if (event.category === "robot") return "↻";
    return "•";
  };

  container.innerHTML = events.map((event) => `
    <article class="recent-event-item ${event.tone} ${event.category}">
      <time>${formatEventTime(event.time)}</time>
      <i aria-hidden="true">${eventIcon(event)}</i>
      <div>
        <strong>${event.label}</strong>
        <span>${event.description}</span>
      </div>
    </article>
  `).join("");
}

const taskProgressSteps = [
  { status: "WAITING", label: "요청 접수" },
  { status: "ROBOT_ASSIGNED", label: "로봇 할당" },
  { status: "APPROACHING", label: "차량 접근" },
  { status: "LIFTING", label: "리프트" },
  { status: "MOVING_TO_SLOT", label: "주차 이동" },
  { status: "COMPLETED", label: "완료" },
];

function requestProgressIndex(status) {
  if (status === "RETURNING") return 4;
  const index = taskProgressSteps.findIndex((step) => step.status === status);
  return index < 0 ? 0 : index;
}

function formatElapsed(createdAt) {
  const elapsedSec = Math.max(
    0,
    Math.floor((Date.now() - new Date(createdAt).getTime()) / 1000)
  );
  const minutes = String(Math.floor(elapsedSec / 60)).padStart(2, "0");
  const seconds = String(elapsedSec % 60).padStart(2, "0");
  return `${minutes}:${seconds}`;
}

function renderTaskActiveFilters(searchValue, statusFilter, typeFilter) {
  const container = document.getElementById("taskActiveFilters");
  const filters = [];
  if (searchValue) {
    filters.push({
      label: `검색: ${searchValue}`,
      controlId: "taskSearchInput",
      resetValue: "",
    });
  }
  if (statusFilter !== "all") {
    filters.push({
      label: `상태: ${document.getElementById("taskStatusFilter").selectedOptions[0].textContent}`,
      controlId: "taskStatusFilter",
      resetValue: "all",
    });
  }
  if (typeFilter !== "all") {
    filters.push({
      label: `유형: ${document.getElementById("taskTypeFilter").selectedOptions[0].textContent}`,
      controlId: "taskTypeFilter",
      resetValue: "all",
    });
  }

  container.replaceChildren();
  container.hidden = filters.length === 0;
  if (!filters.length) return;

  const summary = document.createElement("strong");
  summary.textContent = `적용 필터 ${filters.length}개`;
  container.appendChild(summary);

  filters.forEach((filter) => {
    const chip = document.createElement("button");
    chip.type = "button";
    chip.className = "task-filter-chip";
    chip.textContent = `${filter.label} ×`;
    chip.title = `${filter.label} 필터 해제`;
    chip.addEventListener("click", () => {
      document.getElementById(filter.controlId).value = filter.resetValue;
      if (latestDashboard) {
        renderRequests(latestDashboard.requests || [], latestDashboard.system);
      }
    });
    container.appendChild(chip);
  });

  const clearButton = document.createElement("button");
  clearButton.type = "button";
  clearButton.className = "task-filter-clear";
  clearButton.textContent = "모두 해제";
  clearButton.addEventListener("click", () => {
    document.getElementById("taskSearchInput").value = "";
    document.getElementById("taskStatusFilter").value = "all";
    document.getElementById("taskTypeFilter").value = "all";
    if (latestDashboard) {
      renderRequests(latestDashboard.requests || [], latestDashboard.system);
    }
  });
  container.appendChild(clearButton);
}

function renderTaskStepper(request, compact = false) {
  const currentIndex = requestProgressIndex(request.status);
  const completed = request.status === "COMPLETED";
  return `
    <ol class="task-stepper ${compact ? "compact" : ""}" aria-label="작업 진행 단계">
      ${taskProgressSteps.map((step, index) => {
        const state = completed || index < currentIndex
          ? "done"
          : index === currentIndex ? "current" : "pending";
        return `
          <li class="${state}">
            <i>${state === "done" ? "✓" : index + 1}</i>
            <span>${step.label}</span>
          </li>
        `;
      }).join("")}
    </ol>
  `;
}

function renderActiveTaskBanner(requests, alerts = []) {
  const banner = document.getElementById("activeTaskBanner");
  const request = requests.find(
    (item) => !["COMPLETED", "CANCELLED"].includes(item.status)
  );
  if (!request) {
    banner.classList.add("hidden");
    banner.innerHTML = "";
    banner.removeAttribute("role");
    banner.removeAttribute("tabindex");
    banner.onclick = null;
    banner.onkeydown = null;
    return;
  }

  const obstacleAlert = obstacleForRequest(request, alerts);
  const obstacleActive = obstacleAlert != null;
  const step = requestProgressIndex(request.status) + 1;
  banner.classList.remove("hidden");
  banner.classList.remove("danger");
  banner.classList.toggle("obstacle-wait", obstacleActive);
  banner.setAttribute("role", "button");
  banner.setAttribute("tabindex", "0");
  banner.setAttribute("aria-label", `${requestTypeLabels[request.request_type]} 작업 #${request.id} 상세 보기`);
  const selectTask = () => selectMapItem("task", String(request.id));
  banner.onclick = selectTask;
  banner.onkeydown = (event) => {
    if (!["Enter", " "].includes(event.key)) return;
    event.preventDefault();
    selectTask();
  };
  banner.innerHTML = `
    <div class="active-task-banner-main">
      <span class="active-task-banner-kicker">
        ${requestTypeLabels[request.request_type]} 작업 #${request.id} · ${obstacleActive ? "일시정지" : "진행 중"}
      </span>
      <strong>${request.vehicle_number} → ${request.slot_id || "슬롯 배정 중"}</strong>
    </div>
    <div class="active-task-banner-status">
      <span>${obstacleActive ? "장애물 대기" : requestOperationalStatusLabel(request, alerts)}</span>
      <small>${
        obstacleActive
          ? `${obstacleAlert.sensor_id || "센서"} · ${obstacleZoneLabel(obstacleAlert.zone_id)}`
          : `${assignedRobotTableLabel(request)} · 경과 ${formatElapsed(request.created_at)}`
      }</small>
    </div>
    <ol class="active-task-mini-steps" aria-label="${step} / 6 단계">
      ${taskProgressSteps.map((item, index) => {
        const state = index < step - 1
          ? "done"
          : index === step - 1 ? "current" : "pending";
        return `
          <li class="${state}">
            <i>${index + 1}</i>
            <span>${item.label}</span>
          </li>
        `;
      }).join("")}
    </ol>
  `;
}

function renderRequests(requests, system) {
  const container = document.getElementById("requestTable");
  const showManualAdvance = (!system || system.mock_controls) && !system?.mock_auto_advance;
  const searchValue = document
    .getElementById("taskSearchInput")
    .value.trim()
    .toLowerCase();
  const statusFilter = document.getElementById("taskStatusFilter").value;
  const typeFilter = document.getElementById("taskTypeFilter").value;
  const isTerminal = (request) =>
    ["COMPLETED", "CANCELLED"].includes(request.status);
  const isError = (request) => request.status === "CANCELLED";

  const activeCount = requests.filter((request) => !isTerminal(request)).length;
  const completedCount = requests.filter(
    (request) => request.status === "COMPLETED"
  ).length;
  const errorCount = requests.filter(isError).length;
  document.getElementById("taskStats").innerHTML = `
    <span>진행 <strong>${activeCount}</strong></span>
    <span>완료 <strong>${completedCount}</strong></span>
    <span>오류 <strong>${errorCount}</strong></span>
  `;

  const filteredRequests = requests.filter((request) => {
    const searchMatches = !searchValue
      || request.vehicle_number.toLowerCase().includes(searchValue)
      || String(request.id).includes(searchValue)
      || String(request.slot_id || "").toLowerCase().includes(searchValue);
    const statusMatches = statusFilter === "all"
      || (statusFilter === "active" && !isTerminal(request))
      || (statusFilter === "completed" && request.status === "COMPLETED")
      || (statusFilter === "error" && isError(request));
    const typeMatches = typeFilter === "all"
      || request.request_type === typeFilter;
    return searchMatches && statusMatches && typeMatches;
  });
  document.getElementById("taskFilterResult").textContent = (
    searchValue || statusFilter !== "all" || typeFilter !== "all"
  )
    ? `${filteredRequests.length}건 표시 · 전체 ${requests.length}건`
    : `전체 ${requests.length}건`;
  renderTaskActiveFilters(searchValue, statusFilter, typeFilter);

  if (!requests.length) {
    container.innerHTML = `
      <div class="task-section-heading">
        <h4>현재 진행 작업</h4>
        <span>0건</span>
      </div>
      <div class="tasks-active-empty">
        <strong>등록된 작업 요청이 없습니다.</strong>
        <span>입차 또는 출차 요청이 접수되면 진행 단계가 표시됩니다.</span>
      </div>
      <div class="task-section-heading history">
        <h4>완료·취소 기록</h4>
        <span>0건</span>
      </div>
      <div class="filtered-empty">아직 완료된 작업 기록이 없습니다.</div>
    `;
    return;
  }

  const activeRequests = filteredRequests.filter(
    (request) => !isTerminal(request)
  );
  const historyRequests = filteredRequests.filter(isTerminal);
  const showActiveSection = ["all", "active"].includes(statusFilter);
  const showHistorySection = ["all", "completed", "error"].includes(statusFilter);

  container.innerHTML = `
    ${showActiveSection ? `
      <div class="task-section-heading">
        <h4>현재 진행 작업</h4>
        <span>${activeRequests.length}건</span>
      </div>
      ${activeRequests.length ? `
      <div class="active-task-list">
        ${activeRequests.map((request) => `
          <article
            class="active-task-card selectable-task ${request.request_type} ${
              selectedTaskEventRequestId === request.id ? "selected" : ""
            }"
            role="button"
            tabindex="0"
            data-task-request-id="${request.id}"
          >
            <div class="active-task-heading">
              <div>
                <span class="request-type-label ${request.request_type}">
                  ${requestTypeLabels[request.request_type]}
                </span>
                <small>작업 #${request.id}</small>
                <h3>${request.vehicle_number} → ${request.slot_id || "슬롯 배정 중"}</h3>
              </div>
              <div class="active-task-meta">
                <span class="badge ${
                  obstacleForRequest(request, latestDashboard?.alerts || [])
                    ? "PAUSED"
                    : request.status
                }">${requestOperationalStatusLabel(request, latestDashboard?.alerts || [])}</span>
                <small>경과 ${formatElapsed(request.created_at)}</small>
              </div>
            </div>
            ${renderTaskStepper(request)}
            <div class="active-task-team">
              <span>협동 로봇</span>
              <strong title="L: 리더 로봇 · F: 팔로워 로봇">
                ${assignedRobotTableLabel(request)}
              </strong>
              ${showManualAdvance ? `
                <button
                  class="advance-button"
                  type="button"
                  onclick="event.stopPropagation(); advanceRequest(${request.id})"
                >
                  다음 단계
                </button>
              ` : ""}
            </div>
          </article>
        `).join("")}
      </div>
      ` : `
        <div class="tasks-active-empty">
          <strong>현재 진행 중인 작업이 없습니다.</strong>
          <span>새 입·출차 요청이 접수되면 이 영역에 우선 표시됩니다.</span>
        </div>
      `}
    ` : ""}
    ${showHistorySection ? `
      <div class="task-section-heading history">
        <h4>완료·취소 기록</h4>
        <span>${historyRequests.length}건</span>
      </div>
      ${historyRequests.length ? `
      <div class="table-wrap">
      <table>
        <colgroup>
          <col class="task-col-id" />
          <col class="task-col-type" />
          <col class="task-col-vehicle" />
          <col class="task-col-slot" />
          <col class="task-col-robots" />
          <col class="task-col-status" />
          <col class="task-col-time" />
        </colgroup>
        <thead>
          <tr>
            <th>ID</th>
            <th>유형</th>
            <th>차량 번호</th>
            <th>주차면</th>
            <th>협동 로봇</th>
            <th>진행 상태</th>
            <th>등록 시간</th>
          </tr>
        </thead>
        <tbody>
          ${historyRequests
            .map(
              (request) => `
                <tr
                  class="selectable-task ${
                    selectedTaskEventRequestId === request.id ? "selected" : ""
                  }"
                  role="button"
                  tabindex="0"
                  data-task-request-id="${request.id}"
                >
                  <td>#${request.id}</td>
                  <td>
                    <span class="request-type-badge ${request.request_type}">
                      ${requestTypeLabels[request.request_type]}
                    </span>
                  </td>
                  <td>${request.vehicle_number}</td>
                  <td>${request.slot_id || "-"}</td>
                  <td>
                    <span
                      class="robot-pair-label"
                      title="L: 리더 로봇 · F: 팔로워 로봇"
                    >${assignedRobotTableLabel(request)}</span>
                  </td>
                  <td>
                    <span class="badge ${request.status}">
                      ${requestStatusLabel(request)}
                    </span>
                  </td>
                  <td>${formatCompactDateTime(request.created_at)}</td>
                </tr>
              `
            )
            .join("")}
        </tbody>
      </table>
    </div>
      ` : `
        <div class="filtered-empty">
          선택한 조건에 해당하는 완료·취소 기록이 없습니다.
        </div>
      `}
    ` : ""}
  `;

  container.querySelectorAll("[data-task-request-id]").forEach((element) => {
    const select = () => selectTaskEvents(
      Number(element.dataset.taskRequestId)
    );
    element.addEventListener("click", select);
    element.addEventListener("keydown", (event) => {
      if (!["Enter", " "].includes(event.key)) return;
      event.preventDefault();
      select();
    });
  });
}

function selectTaskEvents(requestId) {
  selectedTaskEventRequestId = requestId;
  selectedEventScope = "selected";
  if (latestDashboard) {
    renderRequests(latestDashboard.requests || [], latestDashboard.system);
    renderRecentEvents(latestDashboard.alerts || []);
  }
}

function sensorConnectionAlerts(sensors, system) {
  const offlineSensors = sensors.filter((sensor) => {
      if (system?.mode === "mock") {
        return !["MOCK", "ONLINE"].includes(sensor.status);
      }
      return sensor.status !== "ONLINE";
    });
  const offlineSensorIds = new Set(
    offlineSensors.map((sensor) => String(sensor.id))
  );
  let hiddenStateChanged = false;

  // 숨김은 현재 연결 끊김에만 적용한다. 다시 연결된 센서는 숨김을
  // 자동 해제하여 다음에 새로 끊겼을 때 경고가 다시 나타나게 한다.
  hiddenSensorAlertIds.forEach((sensorId) => {
    if (!offlineSensorIds.has(sensorId)) {
      hiddenSensorAlertIds.delete(sensorId);
      hiddenStateChanged = true;
    }
  });
  if (hiddenStateChanged) persistHiddenSensorAlerts();

  return offlineSensors
    .filter((sensor) => !hiddenSensorAlertIds.has(String(sensor.id)))
    .map((sensor) => {
      const topic = String(sensor.topic || "").toLowerCase();
      const sensorType = topic.includes("camera")
        ? "카메라"
        : topic.includes("lidar") || topic.includes("point")
          ? "LiDAR"
          : "센서";
      return {
        id: `sensor-${sensor.id}`,
        sensor_id: String(sensor.id),
        level: "WARNING",
        category: "SENSOR",
        message: `${sensor.id} ${sensorType} 데이터가 수신되지 않습니다. 센서 연결을 확인해주세요.`,
        created_at: null,
        dismissible: true,
      };
    });
}

function persistHiddenSensorAlerts() {
  try {
    window.sessionStorage.setItem(
      HIDDEN_SENSOR_ALERTS_STORAGE_KEY,
      JSON.stringify([...hiddenSensorAlertIds])
    );
  } catch (_error) {
    // 저장 실패 시에도 현재 페이지에서는 Set 값으로 숨김을 유지한다.
  }
}

function hideSensorAlert(sensorId) {
  hiddenSensorAlertIds.add(String(sensorId));
  persistHiddenSensorAlerts();
  renderAlerts(
    latestDashboard?.alerts || [],
    latestDashboard?.sensors || [],
    latestDashboard?.system || null
  );
}

function restoreSensorAlerts() {
  hiddenSensorAlertIds.clear();
  persistHiddenSensorAlerts();
  renderAlerts(
    latestDashboard?.alerts || [],
    latestDashboard?.sensors || [],
    latestDashboard?.system || null
  );
}

function renderAlerts(alerts, sensors = [], system = null) {
  const panel = document.getElementById("alertPanel");
  const list = document.getElementById("alertList");
  const sensorAlerts = sensorConnectionAlerts(sensors, system);
  const hiddenOfflineSensorCount = sensors.filter((sensor) => {
    const offline = system?.mode === "mock"
      ? !["MOCK", "ONLINE"].includes(sensor.status)
      : sensor.status !== "ONLINE";
    return offline && hiddenSensorAlertIds.has(String(sensor.id));
  }).length;
  const restoreButton = document.getElementById("restoreSensorAlertsButton");
  restoreButton.classList.toggle("hidden", hiddenOfflineSensorCount === 0);
  restoreButton.textContent = hiddenOfflineSensorCount
    ? `🔔 숨긴 알림 ${hiddenOfflineSensorCount}`
    : "숨긴 알림";
  restoreButton.title = hiddenOfflineSensorCount
    ? `숨긴 센서 연결 알림 ${hiddenOfflineSensorCount}개를 다시 표시합니다.`
    : "숨긴 센서 연결 알림을 다시 표시합니다.";

  const safetyConnectionAlerts = system?.safety?.state === "UNKNOWN"
    ? [{
        id: "safety-supervisor-unavailable",
        level: "ERROR",
        category: "SYSTEM",
        message: "중앙 안전 관리자 상태를 수신할 수 없습니다. 새 작업과 모션이 차단됩니다.",
        created_at: null,
        dismissible: false,
      }]
    : [];
  const visibleAlerts = [
    ...alerts.map((alert) => ({
      ...alert,
      dismissible: alert.category !== "EMERGENCY_STOP",
    })),
    ...safetyConnectionAlerts,
    ...sensorAlerts,
  ];

  if (!visibleAlerts.length) {
    panel.classList.add("hidden");
    list.innerHTML = "";
    return;
  }

  panel.classList.remove("hidden");
  list.innerHTML = visibleAlerts
    .map(
      (alert) => `
        <div class="alert-card ${alert.level}">
          <div class="alert-main">
            <div class="alert-badges">
              <span class="badge ${alert.level}">
                ${alertLabels[alert.level] || alert.level}
              </span>
              <span class="alert-category">
                ${alertLabels[alert.category] || alert.category}
              </span>
            </div>
            <p class="alert-message">${alert.message}</p>
            <span class="alert-time">${alert.created_at ? alert.created_at.replace("T", " ") : "현재 상태"}</span>
          </div>
          ${alert.category === "EMERGENCY_STOP" &&
            system?.safety?.state !== "UNKNOWN" ? `
            <button
              type="button"
              class="secondary-button small safety-recovery-button"
              data-open-safety-recovery
            >
              ${system?.safety?.state === "READY_FOR_OPERATION"
                ? "운영 복귀 승인"
                : "안전 복구 절차"}
            </button>
          ` : alert.category === "SENSOR" && alert.sensor_id ? `
            <button
              type="button"
              class="secondary-button small"
              data-hide-sensor-alert="${alert.sensor_id}"
              title="현재 센서 연결 끊김 알림을 숨깁니다."
            >
              숨기기
            </button>
          ` : alert.category === "OBSTACLE"
            && alert.location_x != null
            && alert.location_y != null ? `
            <div class="alert-actions">
              <button
                type="button"
                class="secondary-button small"
                data-focus-obstacle-alert="${alert.id}"
              >
                위치 보기
              </button>
              <span class="alert-latched">해소 시 자동 해제</span>
            </div>
          ` : alert.dismissible ? `
            <button
              class="secondary-button small"
              onclick="resolveAlert(${alert.id})"
            >
              해제
            </button>
          ` : `
            <span class="alert-latched">
              ${alert.category === "EMERGENCY_STOP" ? "관제 확인 필요" : "자동 복구 대기"}
            </span>
          `}
        </div>
      `
    )
    .join("");

  list.querySelectorAll("[data-hide-sensor-alert]").forEach((button) => {
    button.addEventListener("click", () => {
      hideSensorAlert(button.dataset.hideSensorAlert);
    });
  });
  list.querySelectorAll("[data-focus-obstacle-alert]").forEach((button) => {
    button.addEventListener("click", () => {
      focusObstacleAlert(button.dataset.focusObstacleAlert);
    });
  });
  list.querySelectorAll("[data-open-safety-recovery]").forEach((button) => {
    button.addEventListener("click", openSafetyRecoveryDialog);
  });
}

function renderSystem(system) {
  if (!system) return;

  const modeBadge = document.getElementById("modeBadge");
  modeBadge.textContent = system.mode === "mock" ? "Mock Mode" : "ROS2 Mode";

  const statusBox = document.getElementById("systemStatus");
  const statusText = document.getElementById("systemStatusText");
  const activeAlerts = latestDashboard?.alerts || [];
  const offlineSensors = (latestDashboard?.sensors || []).filter(
    (sensor) => system.mode !== "mock" && sensor.status !== "ONLINE"
  );
  const unavailableRobots = (latestDashboard?.robots || []).filter(
    (robot) => ["ERROR", "OFFLINE"].includes(robot.status)
  );
  const safety = system.safety || {
    state: system.emergency_stop ? "STOPPED_LATCHED" : "NORMAL",
    motion_allowed: !system.emergency_stop,
  };
  const warningReason = safety.state === "READY_FOR_OPERATION"
    ? "운영 복귀 승인 대기"
    : safety.state === "UNKNOWN"
      ? "안전 관리자 확인 필요"
      : system.emergency_stop
        ? "비상정지 작동"
    : activeAlerts.find(
    (alert) => alert.category === "OBSTACLE"
  )
    ? "장애물 감지"
    : activeAlerts.find((alert) => alert.level === "ERROR")
      ? "로봇·시스템 오류"
      : offlineSensors.length
        ? `${offlineSensors.map((sensor) => sensor.id).join("·")} 연결 필요`
        : unavailableRobots.length
          ? `${unavailableRobots.map((robot) => shortRobotName(robot.id)).join("·")} 확인 필요`
          : "주의 필요";

  statusBox.classList.remove("warn", "danger");

  if (system.health === "ERROR") {
    statusBox.classList.add("danger");
    statusText.textContent = warningReason === "주의 필요"
      ? "시스템 오류"
      : warningReason;
  } else if (system.health === "WARNING") {
    statusBox.classList.add("warn");
    statusText.textContent = warningReason;
  } else {
    statusText.textContent = "시스템 정상";
  }

  const emergencyButton = document.getElementById("emergencyStopButton");
  emergencyButton.disabled = Boolean(system.emergency_stop);
  emergencyButton.classList.toggle("active", Boolean(system.emergency_stop));
  emergencyButton.innerHTML = system.emergency_stop
    ? `<span aria-hidden="true">■</span> ${
        safety.state === "READY_FOR_OPERATION"
          ? "안전 점검 완료"
          : "비상정지 작동 중"
      }`
    : `<span aria-hidden="true">■</span> 비상정지`;

  document.querySelectorAll("#requestForm input, #requestForm select, #requestForm button")
    .forEach((control) => {
      control.disabled = Boolean(system.emergency_stop);
    });

  // 백업은 모드와 무관하게 항상 노출 (서버 호출 없이 현재 화면 데이터만 내려받음).
  document
    .getElementById("resetButton")
    .classList.toggle("hidden", system.mode !== "mock");
  document.getElementById("resetButton").disabled = Boolean(system.emergency_stop);
  document
    .getElementById("dbResetButton")
    .classList.toggle("hidden", system.mode !== "ros2");
  document.getElementById("dbResetButton").disabled = Boolean(system.emergency_stop);
  document
    .getElementById("mockVehicleGuide")
    .classList.toggle("hidden", !system.mock_controls);

  renderSafetyRecoveryDialog(system);
}

function setSafetyRecoveryMessage(message, isError = false) {
  const box = document.getElementById("safetyRecoveryMessage");
  box.textContent = message;
  box.classList.remove("hidden", "error");
  if (isError) box.classList.add("error");
}

function renderSafetyRecoveryDialog(system) {
  const dialog = document.getElementById("safetyRecoveryDialog");
  if (!dialog || !dialog.open) return;

  const safety = system?.safety || {};
  const resetForm = document.getElementById("safetyResetForm");
  const approvalForm = document.getElementById("operationApprovalForm");
  const stateBox = document.getElementById("safetyRecoveryState");
  const labels = {
    STOPPED_LATCHED: "비상정지 · 현장 점검 필요",
    READY_FOR_OPERATION: "점검 완료 · 운영 복귀 승인 대기",
    NORMAL: "정상 운영",
    UNKNOWN: "중앙 안전 상태 확인 중",
  };
  stateBox.innerHTML = `
    <strong>${labels[safety.state] || safety.state || "상태 확인 중"}</strong>
    <span>정지 세대 #${safety.stop_epoch || 0}</span>
    ${safety.reason ? `<p>${safety.reason}</p>` : ""}
    ${(safety.blockers || []).length
      ? `<ul>${safety.blockers.map((blocker) => `<li>${blocker}</li>`).join("")}</ul>`
      : ""}
  `;
  resetForm.classList.toggle("hidden", safety.state !== "STOPPED_LATCHED");
  approvalForm.classList.toggle(
    "hidden", safety.state !== "READY_FOR_OPERATION"
  );
  if (
    safety.state === "READY_FOR_OPERATION" &&
    !document.getElementById("operationApprovalOperator").value
  ) {
    document.getElementById("operationApprovalOperator").value =
      safety.operator_id || "";
  }
}

function openSafetyRecoveryDialog() {
  const dialog = document.getElementById("safetyRecoveryDialog");
  document.getElementById("safetyRecoveryMessage").classList.add("hidden");
  renderSafetyRecoveryDialog(latestDashboard?.system);
  if (!dialog.open) dialog.showModal();
  renderSafetyRecoveryDialog(latestDashboard?.system);
}

function showMessage(message, isError = false) {
  const messageBox = document.getElementById("messageBox");

  window.clearTimeout(messageHideTimer);
  messageBox.textContent = message;
  messageBox.classList.remove("hidden", "error");

  if (isError) {
    messageBox.classList.add("error");
  }

  messageHideTimer = window.setTimeout(() => {
    messageBox.classList.add("hidden");
  }, isError ? 5000 : 4000);
}

function updateLiveStatus(isOnline, system) {
  const status = document.getElementById("liveUpdateStatus");
  if (!status) return;

  status.classList.remove("hidden");
  status.classList.remove("pending", "offline", "connected");
  if (!isOnline) {
    status.classList.add("offline");
    status.querySelector("span").textContent = "서버 연결 끊김";
    return;
  }

  if (system?.mode === "mock") {
    // 전역 헤더의 Mock Mode 배지와 중복되므로 도면 내부 표시는 숨긴다.
    status.classList.add("hidden");
    return;
  }

  status.classList.add("connected");
  status.querySelector("span").textContent = "관제 데이터 수신 · 방금";
}

async function refreshDashboard() {
  try {
    const data = normalizeRosDashboardRobots(await apiRequest("/dashboard"));
    latestDashboard = data;
    lastDashboardReceivedAt = new Date();

    if (pendingFocusRequestId != null) {
      const focusedRequest = data.requests.find(
        (request) => request.id === pendingFocusRequestId
      );
      if (focusedRequest) {
        selectedMapItem = { type: "task", id: String(focusedRequest.id) };
        pendingFocusRequestId = null;
      }
    }

    // 첫 화면에서도 상세 패널이 비어 보이지 않도록 주차된 주차면을
    // 우선 선택하고, 없으면 첫 번째 주차면을 기본값으로 사용한다.
    if (!selectedMapItem) {
      const activeRequest = data.requests.find(
        (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
      );
      if (activeRequest) {
        selectedMapItem = { type: "task", id: String(activeRequest.id) };
      } else {
        const defaultSlot = data.slots.find((slot) => slot.status === "OCCUPIED") || data.slots[0];
        if (defaultSlot) selectedMapItem = { type: "slot", id: defaultSlot.id };
      }
    }

    renderSummary(
      data.summary,
      data.robots,
      data.sensors || [],
      data.system,
      data.alerts || [],
      data.requests || []
    );
    renderActiveTaskBanner(data.requests || [], data.alerts || []);
    captureRequestEvents(data.requests, data.system);
    captureAlertSpeech(data.alerts || [], data.robots, data.system);
    updateRobotAnimationTargets(data.robots);
    renderLotMap(
      data.slots,
      applyRobotDisplayPositions(data.robots),
      data.map,
      data.sensors || [],
      data.requests,
      data.alerts || []
    );
    ensureRobotAnimationLoop();
    renderSelectionDetail(data);
    renderRequests(data.requests, data.system);
    renderAlerts(data.alerts || [], data.sensors || [], data.system);
    renderRecentEvents(data.alerts || []);
    renderSystem(data.system);
    updateRequestAvailability();
    updateLiveStatus(true, data.system);
  } catch (error) {
    updateLiveStatus(false);
    showMessage(error.message, true);
  }
}

async function advanceRequest(requestId) {
  try {
    await apiRequest(`/requests/${requestId}/advance`, {
      method: "POST",
    });

    await refreshDashboard();
  } catch (error) {
    showMessage(error.message, true);
  }
}

function updateRequestFlow() {
  const isParkIn = document.getElementById("requestType").value === "PARK_IN";
  document.getElementById("requestFlowTitle").textContent = isParkIn
    ? "입차 요청 처리"
    : "출차 요청 처리";
  document.getElementById("requestFlowStart").textContent = isParkIn
    ? "차량"
    : "주차장";
  document.getElementById("requestFlowEnd").textContent = isParkIn
    ? "주차장"
    : "차량";
  document.getElementById("requestFlowDirection").textContent = "→";
  updateRequestAvailability();
}

function updateRequestAvailability() {
  const requestType = document.getElementById("requestType").value;
  const role = requestType === "PARK_IN" ? "entry" : "exit";
  const roleLabel = requestType === "PARK_IN" ? "입차" : "출차";
  const alternateLabel = requestType === "PARK_IN" ? "출차" : "입차";
  const button = document.getElementById("requestSubmitButton");
  const hint = document.getElementById("requestSafetyHint");
  const systemStopped = Boolean(latestDashboard?.system?.emergency_stop);
  const obstacle = (latestDashboard?.alerts || []).find(
    (alert) => obstacleAffectsRole(alert, role)
  );

  button.classList.toggle("safety-blocked", systemStopped || Boolean(obstacle));
  button.disabled = systemStopped || Boolean(obstacle);
  hint.classList.remove("warning", "danger");

  if (systemStopped) {
    button.textContent = "비상정지 해제 후 등록 가능";
    hint.classList.add("danger");
    hint.textContent = "전체 비상정지 상태입니다. 안전 복구와 운영 승인을 완료한 뒤 새 요청을 등록해주세요.";
    hint.classList.remove("hidden");
    return;
  }

  if (obstacle) {
    button.textContent = `${roleLabel} 통로 장애물 해소 대기`;
    hint.classList.add("warning");
    hint.textContent = (
      `${obstacleZoneLabel(obstacle.zone_id)}에 장애물이 감지되어 ${roleLabel} 요청을 차단했습니다. `
      + `${alternateLabel} 요청은 계속 등록할 수 있습니다.`
    );
    hint.classList.remove("hidden");
    return;
  }

  button.textContent = "요청 등록";
  hint.textContent = "";
  hint.classList.add("hidden");
}

document
  .getElementById("requestType")
  .addEventListener("change", updateRequestFlow);

document
  .getElementById("requestForm")
  .addEventListener("submit", async (event) => {
    event.preventDefault();

    const requestType = document.getElementById("requestType").value;
    const vehicleNumber = document
      .getElementById("vehicleNumber")
      .value.trim();

    if (!vehicleNumber) {
      showMessage("차량 번호를 입력해주세요.", true);
      document.getElementById("vehicleNumber").focus();
      return;
    }

    try {
      const result = await apiRequest("/requests", {
        method: "POST",
        body: JSON.stringify({
          request_type: requestType,
          vehicle_number: vehicleNumber,
        }),
      });

      showMessage(
        `${requestTypeLabels[result.request_type]} 요청 #${result.id}이 등록되었습니다. 작업·이벤트 탭에서 진행 상태를 확인하세요.`
      );

      pendingFocusRequestId = result.id;
      selectedMapItem = { type: "task", id: String(result.id) };
      pendingFocusRequestId = null;
      event.target.reset();
      updateRequestFlow();
      await refreshDashboard();
      activateWorkspaceTab("live");
    } catch (error) {
      showMessage(error.message, true);
    }
  });

document
  .getElementById("taskSearchInput")
  .addEventListener("input", () => {
    if (latestDashboard) {
      renderRequests(latestDashboard.requests || [], latestDashboard.system);
    }
  });
["taskStatusFilter", "taskTypeFilter"].forEach((elementId) => {
  document.getElementById(elementId).addEventListener("change", () => {
    if (latestDashboard) {
      renderRequests(latestDashboard.requests || [], latestDashboard.system);
    }
  });
});
document.getElementById("allEventsButton").addEventListener("click", () => {
  selectedEventScope = "all";
  renderRecentEvents(latestDashboard?.alerts || []);
});
document
  .getElementById("selectedTaskEventsButton")
  .addEventListener("click", () => {
    if (selectedTaskEventRequestId == null) return;
    selectedEventScope = "selected";
    renderRecentEvents(latestDashboard?.alerts || []);
  });
document.querySelectorAll("[data-event-category]").forEach((button) => {
  button.addEventListener("click", () => {
    selectedEventCategory = button.dataset.eventCategory;
    document.querySelectorAll("[data-event-category]").forEach((item) => {
      item.classList.toggle("active", item === button);
    });
    renderRecentEvents(latestDashboard?.alerts || []);
  });
});

document
  .getElementById("resetButton")
  .addEventListener("click", async () => {
    if (!window.confirm("Mock 요청과 상태를 모두 초기화할까요?")) return;
    try {
      await apiRequest("/reset", {
        method: "POST",
      });

      showMessage("Mock 데이터가 초기화되었습니다.");
      await refreshDashboard();
    } catch (error) {
      showMessage(error.message, true);
    }
  });

document
  .getElementById("dbResetButton")
  .addEventListener("click", async () => {
    if (
      !window.confirm(
        "테스트 DB의 주차면·로봇 상태와 작업 이력을 모두 삭제합니다.\n이 작업은 되돌릴 수 없습니다. 계속할까요?"
      )
    )
      return;
    const confirmation = window.prompt(
      "초기화를 실행하려면 아래에 'DB 초기화'를 정확히 입력해주세요."
    );
    if (confirmation !== "DB 초기화") {
      showMessage("DB 초기화가 취소되었습니다.", true);
      return;
    }
    try {
      await apiRequest("/ros2/db-reset", {
        method: "POST",
      });

      showMessage("테스트 DB가 초기화되었습니다.");
      await refreshDashboard();
    } catch (error) {
      showMessage(error.message, true);
    }
  });

function downloadMockBackup() {
  const button = document.getElementById("backupButton");

  if (!latestDashboard) {
    const originalText = button.textContent;
    button.textContent = "데이터 준비 중";
    window.setTimeout(() => {
      button.textContent = originalText;
    }, 1600);
    return;
  }

  const exportedAt = new Date();
  const backup = {
    backup_version: 1,
    exported_at: exportedAt.toISOString(),
    mode: latestDashboard.system?.mode || "mock",
    dashboard: latestDashboard,
  };
  const blob = new Blob([JSON.stringify(backup, null, 2)], {
    type: "application/json;charset=utf-8",
  });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  const timestamp = exportedAt.toISOString().replace(/[:.]/g, "-");

  link.href = url;
  link.download = `parking-control-backup-${timestamp}.json`;
  document.body.appendChild(link);
  link.click();
  link.remove();
  window.setTimeout(() => URL.revokeObjectURL(url), 0);

  const originalText = button.textContent;
  button.textContent = "백업 완료";
  window.setTimeout(() => {
    button.textContent = originalText;
  }, 1600);
}

document
  .getElementById("backupButton")
  .addEventListener("click", downloadMockBackup);

document.querySelectorAll("[data-vehicle-number]").forEach((button) => {
  button.addEventListener("click", () => {
    document.getElementById("requestType").value = "PARK_OUT";
    updateRequestFlow();
    const input = document.getElementById("vehicleNumber");
    input.value = button.dataset.vehicleNumber;
    input.focus();
  });
});

async function resolveAlert(alertId) {
  try {
    await apiRequest(`/alerts/${alertId}/resolve`, {
      method: "POST",
    });

    await refreshDashboard();
  } catch (error) {
    showMessage(error.message, true);
  }
}

async function activateEmergencyStop() {
  if (latestDashboard?.system?.emergency_stop) return;
  const activeRequests = (latestDashboard?.requests || []).filter(
    (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
  );
  const assignedTargets = new Set(
    activeRequests.flatMap((request) => assignedRobotIds(request))
  );
  const targetRobotCount = assignedTargets.size || (
    latestDashboard?.robots || []
  ).filter((robot) => robot.status !== "OFFLINE").length;
  if (
    !window.confirm(
      "모든 로봇에 비상정지를 요청합니다.\n\n"
      + `현재 진행 작업: ${activeRequests.length}건\n`
      + `정지 대상 로봇: ${targetRobotCount}대\n\n`
      + "진행 중인 작업은 중단되며 현장 점검과 별도의 운영 복귀 승인이 필요합니다."
    )
  )
    return;

  const button = document.getElementById("emergencyStopButton");
  button.disabled = true;
  try {
    const result = await apiRequest("/emergency-stop", { method: "POST" });
    showMessage(result.message);
    await refreshDashboard();
  } catch (error) {
    button.disabled = false;
    showMessage(error.message, true);
  }
}

document
  .getElementById("lidarVisibilityButton")
  .addEventListener("click", toggleLidarMarkers);
document
  .getElementById("emergencyStopButton")
  .addEventListener("click", activateEmergencyStop);
document
  .getElementById("restoreSensorAlertsButton")
  .addEventListener("click", restoreSensorAlerts);

document
  .getElementById("closeSafetyRecoveryDialog")
  .addEventListener("click", () => {
    document.getElementById("safetyRecoveryDialog").close();
  });

document
  .getElementById("safetyResetForm")
  .addEventListener("submit", async (event) => {
    event.preventDefault();
    const safetyChecks = [
      ["checkAreaClear", "작업 구역에 사람이 없고 안전함"],
      ["checkRobotsStopped", "전체 로봇 정지 상태"],
      ["checkLoadSecured", "차량·리프트 지지 상태"],
      ["checkSensors", "센서·통신 상태 또는 계획된 비활성 상태"],
    ];
    const uncheckedChecks = safetyChecks.filter(
      ([elementId]) => !document.getElementById(elementId).checked
    );
    if (uncheckedChecks.length) {
      setSafetyRecoveryMessage(
        `해제 요청 전 모든 안전 점검 항목을 확인해주세요.\n${
          uncheckedChecks
            .map(([, label]) => `• ${label}`)
            .join("\n")
        }`,
        true
      );
      document.getElementById(uncheckedChecks[0][0]).focus();
      return;
    }

    const button = event.submitter;
    button.disabled = true;
    try {
      const result = await apiRequest("/safety/reset-request", {
        method: "POST",
        body: JSON.stringify({
          operator_id: document.getElementById("safetyResetOperator").value.trim(),
          inspection_note: document.getElementById("safetyInspectionNote").value.trim(),
          area_clear: document.getElementById("checkAreaClear").checked,
          robots_stopped: document.getElementById("checkRobotsStopped").checked,
          load_secured: document.getElementById("checkLoadSecured").checked,
          sensors_checked: document.getElementById("checkSensors").checked,
        }),
      });
      setSafetyRecoveryMessage(result.message);
      await refreshDashboard();
    } catch (error) {
      setSafetyRecoveryMessage(error.message, true);
    } finally {
      button.disabled = false;
    }
  });

document
  .getElementById("operationApprovalForm")
  .addEventListener("submit", async (event) => {
    event.preventDefault();
    if (
      !window.confirm(
        "운영 복귀를 승인할까요?\n기존 작업은 재개되지 않으며 새 작업 접수만 허용됩니다."
      )
    )
      return;
    const button = event.submitter;
    button.disabled = true;
    try {
      const result = await apiRequest("/safety/approve-operation", {
        method: "POST",
        body: JSON.stringify({
          operator_id: document
            .getElementById("operationApprovalOperator")
            .value.trim(),
          approval_note: document
            .getElementById("operationApprovalNote")
            .value.trim(),
        }),
      });
      setSafetyRecoveryMessage(result.message);
      await refreshDashboard();
      window.setTimeout(() => {
        const dialog = document.getElementById("safetyRecoveryDialog");
        if (dialog.open && latestDashboard?.system?.safety?.state === "NORMAL") {
          dialog.close();
        }
      }, 1200);
    } catch (error) {
      setSafetyRecoveryMessage(error.message, true);
    } finally {
      button.disabled = false;
    }
  });

window.advanceRequest = advanceRequest;
window.resolveAlert = resolveAlert;

setupWorkspaceTabs();
updateRequestFlow();

async function runDashboardRefreshLoop() {
  await refreshDashboard();
  const hasActiveTask = (latestDashboard?.summary?.active_requests || 0) > 0;
  window.setTimeout(runDashboardRefreshLoop, hasActiveTask ? 250 : 2000);
}

runDashboardRefreshLoop();

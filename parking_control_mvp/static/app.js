const API_BASE = "/api";

const statusLabels = {
  IDLE: "대기",
  BUSY: "작업 중",
  CHARGING: "충전 중",
  ERROR: "오류",
  OFFLINE: "연결 끊김",
  SAFETY_STOPPED: "안전 정지",
  RECOVERY_REQUIRED: "복구 대기",
  RECOVERING: "복귀 중",
  EMPTY: "빈 공간",
  RESERVED: "예약",
  OCCUPIED: "주차 중",
  WAITING: "요청 대기",
  ROBOT_ASSIGNED: "로봇 할당",
  APPROACHING: "차량 접근",
  LIFTING: "차량 리프트",
  MOVING_TO_SLOT: "주차 이동",
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

const requestFlowDefinitions = {
  PARK_IN: {
    title: "입차 요청 처리",
    start: "vehicle",
    end: "parking",
    steps: [
      ["요청 접수", "차량번호 확인"],
      ["슬롯 배정", "가까운 빈 주차면 선택"],
      ["로봇 할당", "입차 L·F 편성"],
      ["차량 접근", "차량 하부로 이동"],
      ["주차면 이동", "리프트 후 배정면으로 이동"],
      ["완료·복귀", "차량 배치 후 도크 복귀"],
    ],
  },
  PARK_OUT: {
    title: "출차 요청 처리",
    start: "parking",
    end: "vehicle",
    steps: [
      ["요청 접수", "차량번호 확인"],
      ["차량 위치 확인", "현재 주차면 조회"],
      ["로봇 할당", "출차 L·F 편성"],
      ["차량 접근", "주차 차량 하부로 이동"],
      ["출차 대기구역 이동", "리프트 후 출차 구역으로 이동"],
      ["완료·복귀", "차량 인계 후 도크 복귀"],
    ],
  },
};

const requestRobotTeams = {
  PARK_IN: ["entry_lead", "entry_follow"],
  PARK_OUT: ["exit_lead", "exit_follow"],
};

const workspaceTabs = ["live", "requests", "tasks"];
const HIDDEN_SENSOR_ALERTS_STORAGE_KEY = "parking-ui-hidden-sensor-alerts";
let latestDashboard = null;
let selectedMapItem = null;
let pendingFocusRequestId = null;
let latestLidarVisualization = null;
let lidarViewMode = "full";
let lidarDetailRefreshTimer = null;
let lidarDetailLoading = false;
let lastDashboardReceivedAt = null;
let messageHideTimer = null;
let requestValidationTimer = null;
let latestRequestResult = null;
let requestSubmitting = false;
let vehicleFieldTouched = false;
let recentWorkflowEvents = [];
let selectedTaskEventRequestId = null;
let selectedEventScope = "all";
let selectedEventCategory = "all";
let activeInspectorTab = new URLSearchParams(window.location.search).get(
  "inspector"
) || "task";
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

function requestMapStageLabel(request) {
  const labels = {
    WAITING: "요청 대기",
    ROBOT_ASSIGNED: "출발 준비",
    APPROACHING: "차량 접근 중",
    LIFTING: "차량 인양 중",
    MOVING_TO_SLOT: `${request.slot_id || "주차면"} 이동 중`,
    RETURNING: "대기 구역 복귀 중",
  };
  return labels[request.status] || requestStatusLabel(request);
}

function slotTaskMapLabel(request) {
  if (["WAITING", "ROBOT_ASSIGNED", "APPROACHING", "LIFTING"].includes(request.status)) {
    return `${requestTypeLabels[request.request_type]} 대상`;
  }
  if (request.status === "MOVING_TO_SLOT") {
    return `${requestTypeLabels[request.request_type]} 이동 중`;
  }
  if (request.status === "RETURNING") {
    return request.request_type === "PARK_IN" ? "주차 완료" : "출차 완료";
  }
  return statusLabels[request.status] || "작업 대상";
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
      recovery_note: "복귀 판단 근거",
      path_clear: "복귀 경로 안전 확인",
      load_cleared: "차량·적재물 안전 확인",
      arms_retracted: "리프트 암 회수 확인",
      sensors_ready: "복귀용 센서·통신 확인",
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
  const recoveryRobots = robots.filter((robot) =>
    ["SAFETY_STOPPED", "RECOVERY_REQUIRED", "RECOVERING"].includes(robot.status)
  );
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
      label: "가용 주차면",
      value: `${summary.empty_slots} / ${totalSlots}`,
      badge: summary.empty_slots > 0 ? "사용 가능" : "만차",
      detail: `점유 ${summary.occupied_slots} · 예약 ${reservedSlots}`,
      progress: totalSlots ? (summary.empty_slots / totalSlots) * 100 : 0,
      tone: summary.empty_slots > 0 ? "available" : "warning",
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
      icon: parkingRobotHtmlIcon("status-robot-svg"),
      label: "로봇 팀",
      value: `${robotHealthy} / ${robots.length}`,
      badge: unavailableRobots.length
        ? "확인 필요"
        : recoveryRobots.length
          ? recoveryRobots.some((robot) => robot.status === "RECOVERING")
            ? "복귀 중"
            : "복구 필요"
        : pausedRobots.length ? "일시 정지" : "정상",
      detail: unavailableRobots.length
        ? `${unavailableRobots.map((robot) => shortRobotName(robot.id)).join(" · ")} 확인`
        : recoveryRobots.length
          ? `${recoveryRobots.map((robot) => shortRobotName(robot.id)).join(" · ")} ${
              recoveryRobots.some((robot) => robot.status === "RECOVERING")
                ? "도크 복귀 중"
                : "현재 위치 안전 정지"
            }`
        : pausedRobots.length
          ? `${pausedRobots.map((robot) => shortRobotName(robot.id)).join(" · ")} 장애물 대기`
        : robots.length > 1
          ? "입차·출차 로봇 팀 연결"
          : robots[0] ? `${shortRobotName(robots[0].id)} 연결` : "로봇 데이터 없음",
      tone: robotHealthy === robots.length
        && !pausedRobots.length
        && !recoveryRobots.length
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

function parkingRobotIconShapes() {
  return `
    <rect class="parking-robot-wheel" x="-22" y="-12" width="8" height="24" rx="4"></rect>
    <rect class="parking-robot-wheel" x="14" y="-12" width="8" height="24" rx="4"></rect>
    <rect class="parking-robot-body" x="-18" y="-16" width="36" height="32" rx="9"></rect>
    <rect class="parking-robot-face" x="-13" y="-10" width="26" height="19" rx="5"></rect>
    <circle class="parking-robot-sensor" cx="-6" cy="-2" r="3.2"></circle>
    <circle class="parking-robot-sensor" cx="6" cy="-2" r="3.2"></circle>
    <path class="parking-robot-grille" d="M -7 5 Q 0 9 7 5"></path>
    <path class="parking-robot-direction" d="M -5 -16 L 0 -23 L 5 -16 Z"></path>
  `;
}

function parkingRobotMapIcon(cx, cy, scale, className) {
  return `
    <g transform="translate(${cx} ${cy}) scale(${scale})" aria-hidden="true">
      <g class="${className}">
        ${parkingRobotIconShapes()}
      </g>
    </g>
  `;
}

function parkingRobotHtmlIcon(className, label = "") {
  return `
    <svg class="parking-robot-svg ${className}" viewBox="-25 -25 50 50"
      ${label ? `role="img" aria-label="${label}"` : 'aria-hidden="true"'}>
      ${parkingRobotIconShapes()}
    </svg>
  `;
}

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

function renderLotMap(
  slots,
  robots,
  mapInfo,
  sensorStatus = [],
  requests = [],
  alerts = [],
  safetyIncidents = [],
  cooperativeLoads = []
) {
  const svg = document.getElementById("lotMap");
  const emptyMessage = document.getElementById("lotMapEmpty");

  const nodes = (mapInfo && mapInfo.nodes) || [];
  const docks = (mapInfo && mapInfo.docks) || [];
  const vehicleZones = (mapInfo && mapInfo.vehicle_zones) || [];
  const sensors = ((mapInfo && mapInfo.sensors) || []).map((sensor) => ({
    ...sensor,
    ...(sensorStatus.find((status) => status.id === sensor.id) || {}),
  }));
  const activeObstacleAlerts = alerts.filter(
    (alert) => alert.category === "OBSTACLE" && alert.active !== false
  );
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
        let completedPath = null;
        if (
          activeRouteRole === "entry"
          && activeMapRequest.status === "APPROACHING"
        ) {
          activePath = pathForRoute(["entry_outer", "entry_gate", "entry_wait"]);
        } else if (
          activeRouteRole === "entry"
          && activeMapRequest.status === "MOVING_TO_SLOT"
        ) {
          completedPath = pathForRoute(["entry_outer", "entry_gate", "entry_wait"]);
          activePath = pathForRoute(
            ["entry_wait", "crossing_entry", targetNodeId],
            [sx(targetSlot.x), sy(targetSlot.y) + LOT_SLOT_HEIGHT / 2]
          );
        } else if (
          activeRouteRole === "entry"
          && ["LIFTING", "RETURNING"].includes(activeMapRequest.status)
        ) {
          completedPath = pathForRoute(["entry_outer", "entry_gate", "entry_wait"]);
        } else if (
          activeRouteRole === "exit"
          && ["APPROACHING", "LIFTING"].includes(activeMapRequest.status)
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
        } else if (
          activeRouteRole === "exit"
          && activeMapRequest.status === "RETURNING"
        ) {
          completedPath = pathForRoute(
            [targetNodeId, "crossing_exit", "exit_wait", "exit_gate", "exit_outer"],
            null,
            [sx(targetSlot.x), sy(targetSlot.y) - LOT_SLOT_HEIGHT / 2]
          );
        }
        if (completedPath) {
          parts.push(`
            <path class="lot-route-completed" d="${completedPath}"></path>
          `);
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
      <text class="lot-slot-label" x="${cx}" y="${hasVehicle ? cy + 5 : cy - 5}">
        ${slot.id}${slot.is_accessible ? " ♿" : ""}
      </text>
      <text class="lot-slot-sub" x="${cx}" y="${hasVehicle ? cy + 26 : cy + 14}">
        ${slotRequest ? slotTaskMapLabel(slotRequest) : statusLabels[slot.status]}
      </text>
      ${slotRequest ? `
        <text class="lot-slot-task-meta" x="${cx}" y="${hasVehicle ? cy + 42 : cy + 31}">
          차량 ${slotRequest.vehicle_number}
        </text>
      ` : ""}
      </g>
    `);
  }

  const locatedObstacles = activeObstacleAlerts.filter(
    (alert) =>
      alert.location_x != null
      && alert.location_y != null
      && Number.isFinite(Number(alert.location_x))
      && Number.isFinite(Number(alert.location_y))
  );

  // L1과 감지 좌표를 실제 map 좌표 거리로 연결한다. PointCloud 메시지는
  // 객체 종류를 제공하지 않으므로 사람으로 단정하지 않고 "장애물"로 표시한다.
  for (const alert of locatedObstacles) {
    const sensor = sensors.find((item) => item.id === alert.sensor_id)
      || sensors[0];
    if (
      !sensor
      || sensor.x == null
      || sensor.y == null
    ) continue;
    const sensorX = sx(Number(sensor.x));
    const sensorY = sy(Number(sensor.y));
    const obstacleX = sx(Number(alert.location_x));
    const obstacleY = sy(Number(alert.location_y));
    const distanceM = Math.hypot(
      Number(alert.location_x) - Number(sensor.x),
      Number(alert.location_y) - Number(sensor.y)
    );
    const labelX = (sensorX + obstacleX) / 2;
    const labelY = (sensorY + obstacleY) / 2 - 30;
    parts.push(`
      <g class="lot-sensor-detection-link" aria-hidden="true">
        <line x1="${sensorX}" y1="${sensorY}"
          x2="${obstacleX}" y2="${obstacleY}"></line>
        <rect x="${labelX - 38}" y="${labelY - 10}"
          width="76" height="20" rx="10"></rect>
        <text x="${labelX}" y="${labelY + 4}">
          감지 거리 ${distanceM.toFixed(2)}m
        </text>
      </g>
    `);
  }

  // 평상시에는 센서 원점과 연결 상태만 표시한다. 이론 감지 범위는 메인
  // 관제 화면에 그리지 않고 PointCloud·TF 검증은 상세 화면/RViz2에 맡긴다.
  for (const sensor of sensors) {
    if (sensor.x == null || sensor.y == null) continue;
    const cx = sx(Number(sensor.x));
    const cy = sy(Number(sensor.y));
    const isSelected = selectedMapItem?.type === "sensor"
      && selectedMapItem.id === sensor.id;
    const stateLabel = sensor.status === "ONLINE"
      ? `정상${sensor.rate_hz == null ? "" : ` · ${sensor.rate_hz}Hz`}`
      : sensor.status === "MOCK"
        ? "Mock 데이터"
        : "연결 필요";
    const calloutX = Math.min(LOT_MAP_WIDTH - 72, cx + 94);
    const calloutY = Math.max(48, cy - 124);
    parts.push(`
      <g class="lot-selectable lot-sensor-marker ${sensor.status}"
        role="button" tabindex="0" data-entity-type="sensor"
        data-entity-id="${sensor.id}"
        aria-label="천장 LiDAR ${sensor.id} ${sensor.status}">
        <circle class="lot-sensor-hit-target" cx="${cx}" cy="${cy}" r="25"></circle>
        <circle class="lot-sensor-ring ${sensor.status} ${isSelected ? "selected" : ""}"
          cx="${cx}" cy="${cy}" r="16"></circle>
        <circle class="lot-sensor-dot ${sensor.status}" cx="${cx}" cy="${cy}" r="5"></circle>
        <g class="lot-sensor-callout">
          <line x1="${cx + 10}" y1="${cy - 10}"
            x2="${calloutX - 47}" y2="${calloutY + 11}"></line>
          <rect x="${calloutX - 50}" y="${calloutY - 18}"
            width="100" height="39" rx="10"></rect>
          <text class="lot-sensor-label" x="${calloutX}" y="${calloutY - 2}">
            ${sensor.id} · 천장 LiDAR
          </text>
          <text class="lot-sensor-meta" x="${calloutX}" y="${calloutY + 13}">
            ${stateLabel}
          </text>
        </g>
      </g>
    `);
  }

  for (const alert of locatedObstacles) {
    const cx = sx(Number(alert.location_x));
    const cy = sy(Number(alert.location_y));
    const isSelected = selectedMapItem?.type === "obstacle"
      && String(selectedMapItem.id) === String(alert.id);
    const incident = safetyIncidents.find(
      (item) => String(item.alert_id) === String(alert.id)
    );
    const markerLabel = incident
      ? `${formatSafetyIncidentId(incident.id)} · ${alert.sensor_id || "장애물"}`
      : alert.sensor_id ? `장애물 · ${alert.sensor_id}` : "감지 장애물";
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
        <text class="lot-obstacle-radius-label" x="${cx}" y="${cy + 52}">
          안전 영향 구역
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
    const load = cooperativeLoads.find((item) => item.request_id === request.id);
    const loadDanger = Boolean(
      load?.slip_suspected || load?.load_anomaly_suspected
    );
    const syncWarning = load?.synchronized === false;
    const formationTone = obstacleActive || loadDanger
      ? "danger"
      : gap > 3 || syncWarning ? "warning" : "normal";
    const teamLabel = request.request_type === "PARK_IN" ? "입차팀" : "출차팀";
    const formationLabel = obstacleActive
      ? `${request.request_type === "PARK_IN" ? "입차" : "출차"}팀 대기`
      : load?.slip_suspected
        ? `${teamLabel} · 미끄러짐 의심`
        : load?.load_anomaly_suspected
          ? `${teamLabel} · 하중 이상 의심`
          : load?.stable === true
            ? "적재 안정"
            : load?.synchronized === true
              ? "동기화 정상"
              : load?.synchronized === false
                ? `${teamLabel} · 동기화 조정`
      : gap > 3
        ? "간격 조정"
        : requestMapStageLabel(request);
    const labelWidth = Math.max(76, Math.min(122, formationLabel.length * 8 + 16));
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
            cx="${cx}" cy="${cy}" r="23"></circle>
          ${parkingRobotMapIcon(cx, cy + 1, 0.78, "lot-robot-compact-icon")}
          <circle class="lot-robot-compact-role-bg" cx="${cx + 17}" cy="${cy - 17}" r="7"></circle>
          <text class="lot-robot-compact-role" x="${cx + 17}" y="${cy - 14.5}">${roleLetter}</text>
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
        ${parkingRobotMapIcon(cx, robotIconY, 0.92, "lot-robot-icon")}
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
    latestDashboard.alerts || [],
    latestDashboard.safety_incidents || [],
    latestDashboard.cooperative_loads || []
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
  activateWorkspaceTab("live", true);
  renderLatestLotMap();
  if (latestDashboard) renderSelectionDetail(latestDashboard);
  document.querySelector(".live-floor-panel")?.scrollIntoView({
    behavior: "smooth",
    block: "start",
  });
}

function lidarStatusLabel(status) {
  if (status === "ONLINE") return "정상";
  if (status === "MOCK") return "LiDAR";
  return "연결 끊김";
}

function renderLidarVisualization(data) {
  latestLidarVisualization = data;
  const canvas = document.getElementById("lidarDetailCanvas");
  const shell = canvas.parentElement;
  const empty = document.getElementById("lidarDetailEmpty");
  const statusBox = document.querySelector(".lidar-live-state");
  const status = String(data.sensor_status || "OFFLINE").toUpperCase();
  const ignored = data.points?.ignored || [];
  const used = data.points?.used || [];
  const slotUsed = data.points?.slot_used || [];
  const hasMeasurement = ["ONLINE", "MOCK"].includes(status);
  const isMockSample = status === "MOCK" || data.source === "MOCK_SAMPLE";
  const measurementStatus = String(
    data.measurement_status || (hasMeasurement ? "OK" : "NO_DATA")
  ).toUpperCase();
  const lastSeenLabel = data.last_seen_sec == null
    ? "—"
    : Number(data.last_seen_sec) <= 1
      ? "방금"
      : `${Number(data.last_seen_sec).toFixed(1)}초 전`;
  const metric = (value) =>
    value == null ? "—" : Number(value).toLocaleString();

  statusBox.classList.remove("online", "mock", "offline");
  statusBox.classList.add(status.toLowerCase());
  document
    .getElementById("lidarMockNotice")
    .classList.toggle("hidden", !isMockSample);
  document
    .getElementById("lidarSampleBadge")
    .classList.toggle("hidden", !isMockSample);
  document.getElementById("lidarDetailStatus").textContent =
    measurementStatus === "TF_ERROR"
      ? `${data.sensor_id || "L1"} TF 오류`
      : measurementStatus === "NO_VALID_POINTS"
        ? `${data.sensor_id || "L1"} 유효 포인트 없음`
        : `${data.sensor_id || "L1"} ${lidarStatusLabel(status)}`;
  document.getElementById("lidarDetailSource").textContent = [
    data.topic,
    data.status_message,
  ].filter(Boolean).join(" · ");
  document.getElementById("lidarLastSeen").textContent = lastSeenLabel;
  document.getElementById("lidarRate").textContent =
    data.rate_hz == null ? "—" : `${Number(data.rate_hz).toFixed(1)} Hz`;
  document.getElementById("lidarFrame").textContent = data.frame_id || "—";
  const coordinateStatus = document.getElementById("lidarTfStatus");
  coordinateStatus.classList.remove("ok", "check");
  if (data.coordinate_status === "OK") {
    coordinateStatus.classList.add("ok");
    coordinateStatus.textContent = `${data.frame_id || "map"} 좌표 수신`;
  } else if (data.coordinate_status === "ERROR") {
    coordinateStatus.classList.add("check");
    coordinateStatus.textContent = "map TF 변환 실패";
  } else if (data.coordinate_status === "CHECK") {
    coordinateStatus.classList.add("check");
    coordinateStatus.textContent = `${data.frame_id || "unknown"} 좌표계 확인 필요`;
  } else {
    coordinateStatus.textContent = "좌표 변환 확인 대기";
  }
  const showBlockingState = !hasMeasurement
    || ["TF_ERROR", "RESULT_WAITING"].includes(measurementStatus);
  const emptyTitle = document.getElementById("lidarDetailEmptyTitle");
  emptyTitle.textContent = !hasMeasurement
    ? "LiDAR 데이터 미수신"
    : measurementStatus === "TF_ERROR"
      ? "map TF 변환 실패"
      : "슬롯 판정 결과 대기";
  document.getElementById("lidarDetailEmptyTopic").textContent =
    data.status_message
    || `${data.topic || "/parking/lidar/points_world"} 토픽과 ROS2 연결 상태를 확인하세요.`;
  document.getElementById("lidarHeightThreshold").textContent =
    `${Number(data.height_threshold_m || 0).toFixed(2)}m`;
  document.getElementById("lidarPointThreshold").textContent =
    `${data.point_threshold || 0}개`;
  document.getElementById("lidarReceivedPointCount").textContent =
    metric(data.point_total);
  document.getElementById("lidarValidPointCount").textContent =
    metric(data.valid_point_count);
  document.getElementById("lidarDisplayPointCount").textContent =
    metric(data.display_point_count);
  const waitingCount = (data.slots || []).filter(
    (slot) => ["UNAVAILABLE", "WAITING"].includes(slot.status)
  ).length;
  const emptyCount = Number(data.empty_count || 0);
  const uncertainCount = Number(data.uncertain_count || 0);
  document.querySelector("#lidarOccupancySummary strong").textContent =
    `점유 ${data.occupied_count || 0}`
    + ` · 공석 ${emptyCount}`
    + ` · 불확실 ${uncertainCount}`
    + (waitingCount ? ` · 대기 ${waitingCount}` : "");
  const mismatchedSlots = (data.slots || []).filter(
    (slot) => slot.status_match === false
  );
  const consistencyWarning = document.getElementById(
    "lidarConsistencyWarning"
  );
  consistencyWarning.classList.toggle("hidden", mismatchedSlots.length === 0);
  consistencyWarning.querySelector("span").textContent = mismatchedSlots.length
    ? mismatchedSlots.map((slot) =>
      `${slot.id} — LiDAR: ${
        slot.status === "OCCUPIED" ? "점유" : "공석"
      } · 관리 상태: ${
        slot.control_status === "OCCUPIED" ? "점유" : "공석"
      }`
    ).join(" / ")
    : "LiDAR 판정과 메인 관제 슬롯 상태가 일치합니다.";
  empty.classList.toggle("hidden", !showBlockingState);

  const width = Math.max(760, Math.round(shell.clientWidth));
  const height = Math.max(500, Math.round(canvas.clientHeight));
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  const context = canvas.getContext("2d");
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#1a1b19";
  context.fillRect(0, 0, width, height);

  const fullBounds = data.bounds || {
    min_x: -24,
    max_x: 14,
    min_y: -12,
    max_y: 12,
  };
  const margin = { left: 66, right: 26, top: 66, bottom: 52 };
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const positionedSlots = (data.slots || []).filter(
    (slot) =>
      Number.isFinite(Number(slot.x))
      && Number.isFinite(Number(slot.y))
      && Number.isFinite(Number(slot.width))
      && Number.isFinite(Number(slot.length))
  );
  const slotBounds = positionedSlots.length
    ? {
      min_x: Math.min(...positionedSlots.map(
        (slot) => Number(slot.x) - Number(slot.width) / 2
      )) - 1.6,
      max_x: Math.max(...positionedSlots.map(
        (slot) => Number(slot.x) + Number(slot.width) / 2
      )) + 1.6,
      min_y: Math.min(...positionedSlots.map(
        (slot) => Number(slot.y) - Number(slot.length) / 2
      )) - 1.2,
      max_y: Math.max(...positionedSlots.map(
        (slot) => Number(slot.y) + Number(slot.length) / 2
      )) + 1.2,
    }
    : fullBounds;
  const requestedBounds = {
    ...(lidarViewMode === "full" ? fullBounds : slotBounds),
  };
  const requestedWidth = requestedBounds.max_x - requestedBounds.min_x;
  const requestedHeight = requestedBounds.max_y - requestedBounds.min_y;
  const plotAspect = plotWidth / plotHeight;
  const dataAspect = requestedWidth / requestedHeight;
  const bounds = { ...requestedBounds };
  // 사람·장애물 좌표의 거리와 방향이 왜곡되지 않도록 x/y 축의 화면
  // 배율을 동일하게 유지하고, 남는 방향에 여백을 더한다.
  if (dataAspect < plotAspect) {
    const padding = (requestedHeight * plotAspect - requestedWidth) / 2;
    bounds.min_x -= padding;
    bounds.max_x += padding;
  } else if (dataAspect > plotAspect) {
    const padding = (requestedWidth / plotAspect - requestedHeight) / 2;
    bounds.min_y -= padding;
    bounds.max_y += padding;
  }
  const viewModeButton = document.getElementById("lidarViewModeButton");
  viewModeButton.textContent = lidarViewMode === "full"
    ? "주차면 중심 보기"
    : "전체 좌표 진단 보기";
  viewModeButton.setAttribute(
    "aria-pressed",
    String(lidarViewMode === "full")
  );
  const sx = (x) =>
    margin.left
    + ((x - bounds.min_x) / (bounds.max_x - bounds.min_x)) * plotWidth;
  const sy = (y) =>
    margin.top
    + ((bounds.max_y - y) / (bounds.max_y - bounds.min_y)) * plotHeight;

  context.fillStyle = "#f2f2ee";
  context.font = "600 17px sans-serif";
  context.textAlign = "center";
  context.fillText(
    `전체 감지 영역 · 주차면 ${data.occupied_count || 0}/${
      data.total_slots || 0
    } 점유`,
    width / 2,
    31
  );

  context.strokeStyle = "#454842";
  context.lineWidth = 1;
  context.font = "11px sans-serif";
  context.fillStyle = "#c5c6c0";
  const firstXTick = Math.ceil(bounds.min_x / 5) * 5;
  for (let x = firstXTick; x <= bounds.max_x; x += 5) {
    const px = sx(x);
    context.beginPath();
    context.moveTo(px, margin.top);
    context.lineTo(px, margin.top + plotHeight);
    context.stroke();
    context.textAlign = "center";
    context.fillText(String(x), px, height - 27);
  }
  const firstYTick = Math.ceil(bounds.min_y / 5) * 5;
  for (let y = firstYTick; y <= bounds.max_y; y += 5) {
    const py = sy(y);
    context.beginPath();
    context.moveTo(margin.left, py);
    context.lineTo(margin.left + plotWidth, py);
    context.stroke();
    context.textAlign = "right";
    context.fillText(String(y), margin.left - 10, py + 4);
  }

  context.strokeStyle = "#70736c";
  context.strokeRect(margin.left, margin.top, plotWidth, plotHeight);
  context.fillStyle = "#d1d2cc";
  context.textAlign = "center";
  context.font = "12px sans-serif";
  context.fillText("x (m)", margin.left + plotWidth / 2, height - 7);
  context.save();
  context.translate(17, margin.top + plotHeight / 2);
  context.rotate(-Math.PI / 2);
  context.fillText("y (m)", 0, 0);
  context.restore();

  function drawPoints(points, color, radius) {
    context.fillStyle = color;
    context.beginPath();
    for (const point of points) {
      const px = sx(Number(point[0]));
      const py = sy(Number(point[1]));
      context.moveTo(px + radius, py);
      context.arc(px, py, radius, 0, Math.PI * 2);
    }
    context.fill();
  }

  drawPoints(
    ignored,
    lidarViewMode === "full"
      ? "rgba(106, 109, 102, 0.26)"
      : "rgba(106, 109, 102, 0.12)",
    1.1
  );
  drawPoints(
    used,
    lidarViewMode === "full"
      ? "rgba(156, 200, 232, 0.54)"
      : "rgba(156, 200, 232, 0.38)",
    1.65
  );
  drawPoints(slotUsed, "rgba(255, 185, 159, 0.98)", 2.15);

  const sensor = data.sensor_position;
  if (
    sensor
    && sensor.x >= bounds.min_x
    && sensor.x <= bounds.max_x
    && sensor.y >= bounds.min_y
    && sensor.y <= bounds.max_y
  ) {
    const sensorX = sx(sensor.x);
    const sensorY = sy(sensor.y);
    context.strokeStyle = "rgba(96, 165, 250, 0.5)";
    context.lineWidth = 1.5;
    for (const radius of [9, 17]) {
      context.beginPath();
      context.arc(sensorX, sensorY, radius, 0, Math.PI * 2);
      context.stroke();
    }
    context.fillStyle = "#60a5fa";
    context.beginPath();
    context.arc(sensorX, sensorY, 4.5, 0, Math.PI * 2);
    context.fill();
    context.font = "700 10px sans-serif";
    context.textAlign = "left";
    context.fillText(data.sensor_id || "L1", sensorX + 10, sensorY - 9);
  }

  for (const slot of data.slots || []) {
    const x0 = sx(slot.x - slot.width / 2);
    const x1 = sx(slot.x + slot.width / 2);
    const y0 = sy(slot.y + slot.length / 2);
    const y1 = sy(slot.y - slot.length / 2);
    const occupied = slot.status === "OCCUPIED";
    const uncertain = slot.status === "UNCERTAIN";
    const unavailable = ["UNAVAILABLE", "WAITING"].includes(slot.status);
    const mismatched = slot.status_match === false;
    context.fillStyle = unavailable
      ? "rgba(64, 76, 82, 0.34)"
      : uncertain
        ? "rgba(180, 112, 18, 0.25)"
      : occupied
        ? "rgba(205, 67, 64, 0.26)"
        : "rgba(8, 128, 35, 0.27)";
    context.strokeStyle = mismatched
      ? "#f59e0b"
      : unavailable
      ? "#81909a"
      : uncertain
        ? "#f59e0b"
      : occupied
        ? "#f05a54"
        : "#16a43a";
    context.lineWidth = mismatched ? 3 : 2;
    context.fillRect(x0, y0, x1 - x0, y1 - y0);
    context.strokeRect(x0, y0, x1 - x0, y1 - y0);

    const centerX = (x0 + x1) / 2;
    const centerY = (y0 + y1) / 2;
    context.textAlign = "center";
    context.fillStyle = "#f5f5f1";
    context.font = "700 17px sans-serif";
    context.fillText(slot.id, centerX, centerY - 18);
    context.font = "700 12px sans-serif";
    context.fillStyle = unavailable
      ? "#cbd5db"
      : uncertain
        ? "#fde68a"
      : occupied
        ? "#ffd5cd"
        : "#c7f5d2";
    context.fillText(
      unavailable
        ? "판정 대기"
        : uncertain
          ? "불확실"
          : occupied
            ? "점유"
            : "공석",
      centerX,
      centerY + 2
    );
    context.fillStyle = "#f0f1eb";
    context.font = "10px sans-serif";
    context.fillText(
      unavailable
        ? "센서 데이터 대기"
        : uncertain
          ? "추가 데이터 확인 중"
          : occupied
            ? "차량 감지"
            : "차량 없음",
      centerX,
      centerY + 20
    );
    if (mismatched) {
      context.fillStyle = "#f59e0b";
      context.beginPath();
      context.arc(x1 - 12, y0 + 12, 8, 0, Math.PI * 2);
      context.fill();
      context.fillStyle = "#201b12";
      context.font = "800 11px sans-serif";
      context.fillText("!", x1 - 12, y0 + 16);
    }
  }
}

async function refreshLidarDetail() {
  if (lidarDetailLoading) return;
  const dialog = document.getElementById("lidarDetailDialog");
  if (!dialog?.open) return;
  lidarDetailLoading = true;
  try {
    const data = await apiRequest("/lidar/visualization");
    renderLidarVisualization(data);
  } catch (error) {
    document.querySelector(".lidar-live-state").classList.add("offline");
    document.getElementById("lidarDetailStatus").textContent =
      "상세 데이터 조회 실패";
    document.getElementById("lidarDetailSource").textContent = error.message;
  } finally {
    lidarDetailLoading = false;
  }
}

function openLidarDetailDialog() {
  const dialog = document.getElementById("lidarDetailDialog");
  lidarViewMode = "full";
  if (!dialog.open) dialog.showModal();
  refreshLidarDetail();
  window.clearInterval(lidarDetailRefreshTimer);
  lidarDetailRefreshTimer = window.setInterval(refreshLidarDetail, 1000);
}

function closeLidarDetailDialog() {
  const dialog = document.getElementById("lidarDetailDialog");
  if (dialog.open) dialog.close();
  window.clearInterval(lidarDetailRefreshTimer);
  lidarDetailRefreshTimer = null;
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

function formatSafetyIncidentId(incidentId) {
  return `SI-${String(incidentId).padStart(4, "0")}`;
}

function safetyIncidentForAlert(alertId, dashboard = latestDashboard) {
  return (dashboard?.safety_incidents || []).find(
    (incident) => String(incident.alert_id) === String(alertId)
  ) || null;
}

function incidentEventStageLabel(stage) {
  const labels = {
    DETECTED: "장애물 감지",
    ROBOTS_STOPPED: "영향 로봇 정지",
    TASK_PAUSED: "관련 작업 안전정지",
    OBSTACLE_CLEARED: "장애물 해소",
    OPERATION_RESUMED: "작업 자동 재개",
  };
  return labels[stage] || stage;
}

function incidentEventStageOrder(stage) {
  return [
    "DETECTED",
    "ROBOTS_STOPPED",
    "TASK_PAUSED",
    "OBSTACLE_CLEARED",
    "OPERATION_RESUMED",
  ].indexOf(stage);
}

function inspectorRequest(dashboard) {
  const requests = dashboard?.requests || [];
  if (selectedMapItem?.type === "task") {
    const selected = requests.find(
      (request) => String(request.id) === String(selectedMapItem.id)
    );
    if (selected) return selected;
  }
  if (selectedMapItem?.type === "robot") {
    const robot = (dashboard?.robots || []).find(
      (item) => item.id === selectedMapItem.id
    );
    const selected = requests.find(
      (request) => request.id === robot?.current_task_id
    );
    if (selected) return selected;
  }
  if (selectedMapItem?.type === "slot") {
    const selected = requests.find(
      (request) => request.slot_id === selectedMapItem.id
        && !["COMPLETED", "CANCELLED"].includes(request.status)
    );
    if (selected) return selected;
  }
  return requests.find(
    (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
  ) || null;
}

function metricText(value, unit = "") {
  if (value == null || Number.isNaN(Number(value))) return "미수신";
  return `${Number(value).toFixed(1)}${unit}`;
}

function loadMetricText(value, unit = "", digits = 1) {
  if (value == null || Number.isNaN(Number(value))) return "—";
  return `${Number(value).toFixed(digits)}${unit}`;
}

function alignmentTone(value) {
  if (value == null) return "unknown";
  if (Math.abs(value) <= 20) return "normal";
  if (Math.abs(value) <= 50) return "warning";
  return "danger";
}

function cooperativeLoadPhase(status) {
  if (["WAITING", "ROBOT_ASSIGNED"].includes(status)) {
    return {
      key: "preparing",
      title: "협동 적재 준비",
      description: "로봇 배정과 차량 접근을 준비하고 있습니다.",
    };
  }
  if (status === "APPROACHING") {
    return {
      key: "approaching",
      title: "협동 적재 준비",
      description: "차축 중심을 확인하며 적재 위치를 조정하고 있습니다.",
    };
  }
  if (status === "LIFTING") {
    return {
      key: "lifting",
      title: "협동 적재 진행",
      description: "네 지지점의 암 전개와 동기화를 확인합니다.",
    };
  }
  if (status === "MOVING_TO_SLOT") {
    return {
      key: "transporting",
      title: "적재 운반 상태",
      description: "차량 기울기와 지지 안정성을 감시하고 있습니다.",
    };
  }
  if (status === "RETURNING") {
    return {
      key: "returning",
      title: "적재 작업 마무리",
      description: "차량 배치를 마치고 로봇이 대기 구역으로 복귀 중입니다.",
    };
  }
  if (status === "COMPLETED") {
    return {
      key: "completed",
      title: "적재 작업 완료",
      description: "차량 배치와 로봇 작업이 모두 종료되었습니다.",
    };
  }
  return {
    key: "cancelled",
    title: "적재 작업 취소",
    description: "작업이 취소되어 적재 상태 감시를 종료했습니다.",
  };
}

function assignedTeamState(request, dashboard) {
  const assignedIds = assignedRobotIds(request);
  const expectedIds = assignedIds.length
    ? assignedIds
    : request.request_type === "PARK_IN"
      ? ["entry_lead", "entry_follow"]
      : ["exit_lead", "exit_follow"];
  const robots = (dashboard?.robots || []).filter(
    (robot) => expectedIds.includes(robot.id)
  );
  if (!robots.length) return "상태 미확인";
  if (robots.every((robot) => robot.status === "IDLE")) return "대기";
  if (robots.some((robot) => ["ERROR", "OFFLINE"].includes(robot.status))) {
    return "확인 필요";
  }
  if (robots.some((robot) => robot.status === "BUSY")) return "작업 중";
  if (robots.every((robot) => robot.status === "CHARGING")) return "충전 중";
  return "상태 확인";
}

function terminalLoadSummary(phase, request, dashboard) {
  const teamState = assignedTeamState(request, dashboard);
  const duration = formatTaskDuration(request);
  const robotLabel = assignedRobotTableLabel(request);
  if (phase.key === "cancelled") {
    return `
      <div><span>작업 결과</span><strong>취소</strong></div>
      <div><span>작업 시간</span><strong>${duration}</strong></div>
      <div><span>대상 주차면</span><strong>${request.slot_id || "미배정"}</strong></div>
      <div><span>사용 로봇</span><strong>${robotLabel}</strong></div>
      <div><span>로봇 상태</span><strong>${teamState}</strong></div>
      <div><span>적재 감시</span><strong>종료</strong></div>
    `;
  }
  return `
    <div><span>작업 결과</span><strong>완료</strong></div>
    <div><span>작업 시간</span><strong>${duration}</strong></div>
    <div><span>최종 주차면</span><strong>${request.slot_id || "—"}</strong></div>
    <div><span>사용 로봇</span><strong>${robotLabel}</strong></div>
    <div><span>로봇 상태</span><strong>${teamState}</strong></div>
    <div><span>운반 감시</span><strong>종료</strong></div>
  `;
}

function returningLoadSummary(request, dashboard) {
  return `
    <div><span>차량 배치</span><strong>완료</strong></div>
    <div><span>운반 감시</span><strong>종료</strong></div>
    <div><span>현재 단계</span><strong>복귀</strong></div>
    <div><span>로봇 상태</span><strong>${assignedTeamState(request, dashboard)}</strong></div>
  `;
}

function loadTelemetryFresh(load) {
  if (!load) return false;
  if (load.source === "MOCK") return true;
  return load.telemetry_age_sec != null && load.telemetry_age_sec <= 2;
}

function phaseHasExpectedData(phase, load) {
  if (!load) return false;
  if (phase.key === "approaching") {
    return load.front_alignment_error_mm != null
      || load.rear_alignment_error_mm != null;
  }
  if (phase.key === "lifting") {
    return (load.support_points || []).some(
      (point) => point.actual_percent != null
    );
  }
  if (phase.key === "transporting") {
    return [
      load.pitch_deg,
      load.roll_deg,
      load.stable,
      load.synchronized,
      load.tire_support_count,
      load.vehicle_rise_mm,
    ].some((value) => value != null);
  }
  return true;
}

function loadOverallState(phase, load) {
  const fresh = loadTelemetryFresh(load);
  const hasExpectedData = phaseHasExpectedData(phase, load);
  if (phase.key === "preparing") {
    return { tone: "neutral", label: "측정 전", detail: "차량 접근 대기" };
  }
  if (phase.key === "returning") {
    return { tone: "normal", label: "적재 완료", detail: "로봇 복귀 중" };
  }
  if (phase.key === "completed") {
    return { tone: "normal", label: "작업 완료", detail: "로봇 작업 종료" };
  }
  if (phase.key === "cancelled") {
    return { tone: "warning", label: "작업 취소", detail: "적재 감시 종료" };
  }
  if (!fresh || !hasExpectedData) {
    return {
      tone: "warning",
      label: phase.key === "approaching" ? "수신 대기" : "적재 확인 불가",
      detail: "제한 정보 표시 중",
    };
  }
  if (load.slip_suspected === true || load.load_anomaly_suspected === true) {
    return { tone: "danger", label: "적재 상태 위험", detail: "즉시 확인 필요" };
  }
  if (
    load.synchronized === false
    || (phase.key === "transporting" && load.stable === false)
  ) {
    return { tone: "warning", label: "적재 상태 주의", detail: "편차 조정 필요" };
  }
  if (phase.key === "approaching") {
    const values = [
      load.front_alignment_error_mm,
      load.rear_alignment_error_mm,
    ].filter((value) => value != null);
    const aligned = values.length === 2
      && values.every((value) => Math.abs(value) <= 20);
    return aligned
      ? { tone: "normal", label: "정렬 정상", detail: "허용 범위 이내" }
      : { tone: "active", label: "정렬 조정 중", detail: "차축 위치 보정" };
  }
  if (phase.key === "lifting") {
    return load.tire_support_count === 4 && load.synchronized === true
      ? { tone: "normal", label: "지지 정상", detail: "4개 지지점 확인" }
      : { tone: "active", label: "적재 진행 중", detail: "암 전개·지지 확인" };
  }
  return load.stable === true
    ? { tone: "normal", label: "적재 안정 추정", detail: "운반 상태 정상" }
    : { tone: "active", label: "운반 감시 중", detail: "안정 상태 추정" };
}

function renderAlignmentMetric(label, value) {
  const tone = alignmentTone(value);
  const width = value == null
    ? 0
    : Math.min(100, Math.abs(Number(value)) / 50 * 100);
  return `
    <div class="load-alignment-card ${tone}">
      <div class="load-alignment-heading">
        <span>${label}</span>
        <strong>${loadMetricText(value, "mm")}</strong>
      </div>
      <div class="load-limit-line">
        <span>허용 ≤ 20mm</span>
        <i>${value == null ? "미측정" : tone === "normal" ? "정상" : tone === "warning" ? "주의" : "위험"}</i>
      </div>
      <div class="load-limit-bar" aria-hidden="true"><i style="width:${width}%"></i></div>
    </div>
  `;
}

function loadBooleanLabel(value, positive, negative) {
  if (value == null) return { label: "—", tone: "unknown" };
  return value
    ? { label: positive, tone: "normal" }
    : { label: negative, tone: "warning" };
}

function renderLoadEmptyNotice(phase, load) {
  const fresh = loadTelemetryFresh(load);
  const hasExpectedData = phaseHasExpectedData(phase, load);
  if (phase.key === "preparing") {
    return `
      <div class="load-stage-notice neutral">
        <strong>적재 데이터 대기 중</strong>
        <span>현재 로봇 배정 단계입니다. 정렬 데이터는 차량 접근부터 표시됩니다.</span>
      </div>
    `;
  }
  if (phase.key === "approaching" && (!fresh || !hasExpectedData)) {
    return `
      <div class="load-stage-notice neutral">
        <strong>정렬 데이터 수신 대기</strong>
        <span>현재 차량 접근 단계입니다. 암·지지·기울기 데이터는 리프트 단계부터 표시됩니다.</span>
      </div>
    `;
  }
  if (
    ["lifting", "transporting"].includes(phase.key)
    && (!fresh || !hasExpectedData)
  ) {
    const dataStateLabel = load?.telemetry_age_sec == null
      ? "센서 데이터 미수신"
      : "센서 데이터 수신 지연";
    return `
      <div class="load-stage-notice warning">
        <strong>${dataStateLabel}</strong>
        <span>현재는 위치·단계 기반 제한 정보만 표시합니다. 수신 시 상세 지표가 자동으로 나타납니다.</span>
      </div>
    `;
  }
  return "";
}

function renderLoadSummary(phase, load, request, dashboard) {
  const points = load?.support_points || [];
  const actualCount = points.filter(
    (point) => point.actual_percent != null
  ).length;
  if (phase.key === "preparing") {
    return `
      <div><span>정렬</span><strong>미측정</strong></div>
      <div><span>암 전개</span><strong>미측정</strong></div>
      <div><span>타이어 지지</span><strong>미측정</strong></div>
      <div><span>동기화</span><strong>대기</strong></div>
    `;
  }
  if (phase.key === "approaching") {
    const alignmentValues = [
      load?.front_alignment_error_mm,
      load?.rear_alignment_error_mm,
    ].filter((value) => value != null);
    const aligned = alignmentValues.length === 2
      && alignmentValues.every((value) => Math.abs(value) <= 20);
    return `
      <div><span>정렬 측정</span><strong>${alignmentValues.length} / 2</strong></div>
      <div><span>접근 상태</span><strong>${aligned ? "정상" : alignmentValues.length ? "조정 중" : "수신 대기"}</strong></div>
      <div><span>암 전개</span><strong>측정 전</strong></div>
      <div><span>동기화</span><strong>대기</strong></div>
    `;
  }
  if (phase.key === "lifting") {
    return `
      <div><span>암 실제값</span><strong>${actualCount} / 4</strong></div>
      <div><span>타이어 지지 추정</span><strong>${load?.tire_support_count == null ? "—" : `${load.tire_support_count} / 4`}</strong></div>
      <div><span>차량 상승량</span><strong>${loadMetricText(load?.vehicle_rise_mm, "mm")}</strong></div>
      <div><span>동기화</span><strong>${load?.synchronized == null ? "—" : load.synchronized ? "정상" : "조정 필요"}</strong></div>
    `;
  }
  if (phase.key === "transporting") {
    return `
      <div><span>Pitch</span><strong>${loadMetricText(load?.pitch_deg, "°")}</strong></div>
      <div><span>Roll</span><strong>${loadMetricText(load?.roll_deg, "°")}</strong></div>
      <div><span>적재 안정 추정</span><strong>${load?.stable == null ? "—" : load.stable ? "정상" : "주의"}</strong></div>
      <div><span>동기화</span><strong>${load?.synchronized == null ? "—" : load.synchronized ? "정상" : "조정 필요"}</strong></div>
    `;
  }
  if (phase.key === "returning") {
    return returningLoadSummary(request, dashboard);
  }
  return terminalLoadSummary(phase, request, dashboard);
}

function renderSupportPoint(point) {
  const pointClass = String(point.id || "").replaceAll("_", "-");
  const stateClass = point.supported === true
    ? "supported"
    : point.supported === false ? "pending" : "unknown";
  const supportLabel = point.supported == null
    ? "수신 대기"
    : point.supported ? "지지 추정" : "전개 중";
  return `
    <button type="button"
      class="support-point ${pointClass} ${stateClass}"
      data-load-robot-id="${point.robot_id}"
      title="${point.label} 담당 로봇을 지도에서 강조">
      <i class="support-state-dot" aria-hidden="true"></i>
      <span>${point.label.replace("축 ", " ")}</span>
      <strong>${loadMetricText(point.actual_percent, "%")}</strong>
      <small>명령 ${loadMetricText(point.command_percent, "%")}</small>
      <em>${supportLabel}</em>
    </button>
  `;
}

function renderSupportDiagram(request, load) {
  const pointById = new Map(
    (load?.support_points || []).map((point) => [point.id, point])
  );
  const points = [
    ["front_left", "앞축 좌", load?.lead_robot_id],
    ["front_right", "앞축 우", load?.lead_robot_id],
    ["rear_left", "뒤축 좌", load?.follow_robot_id],
    ["rear_right", "뒤축 우", load?.follow_robot_id],
  ].map(([id, label, robotId]) => pointById.get(id) || {
    id,
    label,
    robot_id: robotId || "",
    command_percent: null,
    actual_percent: null,
    supported: null,
  });
  return `
    <div class="vehicle-load-diagram" aria-label="차량 주변 네 지지점 상태">
      <div class="support-point-grid">
        ${points.map(renderSupportPoint).join("")}
        <div class="vehicle-load-body">
          <span>차량 ${request.vehicle_number}</span>
          <small>상승 ${loadMetricText(load?.vehicle_rise_mm, "mm")}</small>
        </div>
      </div>
    </div>
  `;
}

function renderLoadPhaseDetail(request, load, phase) {
  if (phase.key === "preparing") return "";
  if (phase.key === "approaching") {
    return `
      <div class="load-phase-section">
        ${renderAlignmentMetric("앞축 정렬 오차", load?.front_alignment_error_mm)}
        ${renderAlignmentMetric("뒤축 정렬 오차", load?.rear_alignment_error_mm)}
        <div class="load-measurement-note">
          <span>리프트 데이터</span><strong>측정 전</strong>
        </div>
      </div>
    `;
  }
  if (phase.key === "lifting") {
    return `
      <div class="load-phase-section">
        ${renderSupportDiagram(request, load)}
        <div class="load-command-legend">
          <span><i class="actual"></i>실제 관절값</span>
          <span><i class="estimated"></i>타이어 지지 추정</span>
        </div>
      </div>
    `;
  }
  if (phase.key === "transporting") {
    const stable = loadBooleanLabel(
      load?.stable, "안정 추정", "불안정 추정"
    );
    const slip = loadBooleanLabel(
      load?.slip_suspected === null || load?.slip_suspected === undefined
        ? null : !load.slip_suspected,
      "의심 없음",
      "미끄러짐 의심"
    );
    const sync = loadBooleanLabel(
      load?.synchronized, "동기화 정상", "동기화 조정 필요"
    );
    return `
      <div class="load-phase-section">
        <dl class="load-metric-grid transport">
          <div><dt>차량 Pitch <small>제한 2.0°</small></dt><dd>${loadMetricText(load?.pitch_deg, "°")}</dd></div>
          <div><dt>차량 Roll <small>제한 2.0°</small></dt><dd>${loadMetricText(load?.roll_deg, "°")}</dd></div>
          <div><dt>차량 상승량</dt><dd>${loadMetricText(load?.vehicle_rise_mm, "mm")}</dd></div>
          <div><dt>타이어 지지 추정</dt><dd>${load?.tire_support_count == null ? "—" : `${load.tire_support_count} / 4`}</dd></div>
        </dl>
        <div class="load-health-list">
          <span class="${stable.tone}"><i></i>${stable.label}</span>
          <span class="${slip.tone}"><i></i>${slip.label}</span>
          <span class="${sync.tone}"><i></i>${sync.label}</span>
          <span class="${load?.load_anomaly_suspected === true ? "danger" : load?.load_anomaly_suspected === false ? "normal" : "unknown"}">
            <i></i>${load?.load_anomaly_suspected == null ? "하중 —" : load.load_anomaly_suspected ? "하중 이상 의심" : "하중 편차 없음"}
          </span>
        </div>
      </div>
    `;
  }
  if (phase.key === "completed") {
    return `
      <div class="load-stage-notice normal">
        <strong>협동 적재 작업을 완료했습니다</strong>
        <span>차량 배치와 로봇 작업이 종료되어 적재 상태 감시를 마쳤습니다.</span>
      </div>
    `;
  }
  if (phase.key === "cancelled") {
    return `
      <div class="load-stage-notice warning">
        <strong>협동 적재 작업이 취소되었습니다</strong>
        <span>작업·이벤트 탭에서 취소 원인과 마지막 진행 단계를 확인해주세요.</span>
      </div>
    `;
  }
  return `
    <div class="load-stage-notice normal">
      <strong>차량 배치를 완료했습니다</strong>
      <span>담당 로봇 팀이 대기 위치로 복귀하고 있습니다.</span>
    </div>
  `;
}

function renderLoadFreshness(load) {
  if (load?.source === "MOCK") {
    return "최근 적재 데이터 · 방금 · 10Hz (Mock)";
  }
  if (load?.telemetry_age_sec == null) return "";
  const age = Number(load.telemetry_age_sec);
  const ageLabel = age < 0.1 ? "방금" : `${age.toFixed(1)}초 전`;
  const rateLabel = load.telemetry_rate_hz == null
    ? ""
    : ` · ${Number(load.telemetry_rate_hz).toFixed(1)}Hz`;
  return `최근 적재 데이터 · ${ageLabel}${rateLabel}`;
}

function renderCooperativeLoadDetail(dashboard) {
  const container = document.getElementById("cooperativeLoadDetail");
  const request = inspectorRequest(dashboard);
  const load = request
    ? (dashboard?.cooperative_loads || []).find(
        (item) => item.request_id === request.id
      )
    : null;
  if (!request) {
    container.innerHTML = `
      <div class="detail-empty-icon" aria-hidden="true">↔</div>
      <span class="detail-kicker">협동 적재 상태</span>
      <h3>진행 작업이 없습니다</h3>
      <p>입·출차 작업이 시작되면 차축 정렬과 네 타이어 지지 상태를 표시합니다.</p>
    `;
    return;
  }

  const phase = cooperativeLoadPhase(request.status);
  const overall = loadOverallState(phase, load);
  const showPhaseMeasurements = (
    loadTelemetryFresh(load) && phaseHasExpectedData(phase, load)
  );
  const terminalPhase = ["completed", "cancelled"].includes(phase.key);
  const showSummary = (
    phase.key === "returning" || terminalPhase || showPhaseMeasurements
  );
  const showPhaseDetail = (
    phase.key === "returning" || terminalPhase || showPhaseMeasurements
  );
  const telemetryFresh = loadTelemetryFresh(load);
  const sourceLabel = load?.source === "MOCK"
    ? "Mock 시뮬레이션"
    : load?.source === "MEASURED_ESTIMATED"
      ? telemetryFresh ? "관절 측정 · 접촉 추정" : "적재 데이터 수신 지연"
      : phase.key === "preparing" ? "측정 전" : "수신 대기";
  const sourceClass = load?.source === "MEASURED_ESTIMATED" && !telemetryFresh
    ? "stale"
    : String(load?.source || "unavailable").toLowerCase();
  const freshnessText = terminalPhase ? "" : renderLoadFreshness(load);
  container.innerHTML = `
    <div class="inspector-title-line">
      <div>
        <span class="detail-kicker">적재 상태</span>
        <h3>차량 ${request.vehicle_number}</h3>
        <p class="load-current-stage">현재 단계 · ${requestStatusLabel(request)}</p>
      </div>
      ${terminalPhase
        ? ""
        : `<span class="data-source-badge ${sourceClass}">${sourceLabel}</span>`}
    </div>
    <div class="load-overall-state ${overall.tone}">
      <i aria-hidden="true"></i>
      <div>
        <span class="load-overall-caption">적재 종합 상태</span>
        <strong>${overall.label}</strong>
        <span>${overall.detail}</span>
      </div>
    </div>
    <p class="load-phase-description">${phase.description}</p>
    ${showSummary ? `
      <div class="load-summary-grid">
        ${renderLoadSummary(phase, load, request, dashboard)}
      </div>
    ` : ""}
    ${renderLoadEmptyNotice(phase, load)}
    ${showPhaseDetail
      ? renderLoadPhaseDetail(request, load, phase)
      : ""}
    ${freshnessText ? `<p class="load-freshness">${freshnessText}</p>` : ""}
    ${showPhaseMeasurements
      ? `<p class="data-honesty-note">관절 피드백은 실제값, 접촉·하중·안정 상태는 추정값으로 구분합니다.</p>`
      : ""}
  `;
  container.querySelectorAll("[data-load-robot-id]").forEach((button) => {
    if (!button.dataset.loadRobotId) {
      button.disabled = true;
      return;
    }
    button.addEventListener("click", () => {
      selectMapItem("robot", button.dataset.loadRobotId);
    });
  });
}

function visionForRequest(dashboard, request) {
  const assignedIds = request ? assignedRobotIds(request) : [];
  const states = dashboard?.vision_alignments || [];
  return states.find((state) => assignedIds.includes(state.robot_id))
    || states[0]
    || null;
}

function renderVisionAlignmentDetail(dashboard) {
  const container = document.getElementById("visionAlignmentDetail");
  const request = inspectorRequest(dashboard);
  const vision = visionForRequest(dashboard, request);
  if (!vision) {
    container.innerHTML = `
      <div class="detail-empty-icon vision" aria-hidden="true">⌗</div>
      <span class="detail-kicker">ArUco · Depth 비전</span>
      <h3>인식 결과 미수신</h3>
      <p>진행 작업의 전방 카메라와 marker_localizer 진단 토픽을 기다리고 있습니다.</p>
    `;
    return;
  }

  const width = 280;
  const height = 158;
  const points = (vision.marker_corners || [])
    .map((point) => `${Number(point[0]) * width},${Number(point[1]) * height}`)
    .join(" ");
  const markerCenter = (vision.marker_corners || []).length
    ? (vision.marker_corners || []).reduce(
        (acc, point) => [acc[0] + Number(point[0]), acc[1] + Number(point[1])],
        [0, 0]
      ).map((value) => value / vision.marker_corners.length)
    : null;
  const target = vision.target_center || [0.5, 0.5];
  const statusLabels = {
    NO_DATA: "데이터 미수신",
    SEARCHING: "마커 탐색 중",
    ADJUSTING: "정렬 조정 중",
    ALIGNED: "정렬 완료",
  };
  const stateTone = vision.alignment_state === "ALIGNED"
    ? "normal"
    : vision.marker_detected ? "warning" : "unknown";
  const age = vision.updated_at
    ? Math.max(0, (Date.now() - new Date(vision.updated_at).getTime()) / 1000)
    : null;
  container.innerHTML = `
    <div class="inspector-title-line">
      <div>
        <span class="detail-kicker">ArUco · Depth 비전</span>
        <h3>${shortRobotName(vision.robot_id)} 전방 카메라</h3>
      </div>
      <span class="data-source-badge ${vision.source.toLowerCase()}">${
        vision.source === "MOCK" ? "Mock 인식" : "ArUco 융합"
      }</span>
    </div>
    <div class="vision-mini-screen ${vision.connected ? "" : "offline"}">
      <svg viewBox="0 0 ${width} ${height}" role="img" aria-label="ArUco 검출 오버레이">
        <defs>
          <linearGradient id="visionBg" x1="0" x2="1" y1="0" y2="1">
            <stop offset="0" stop-color="#111827"></stop>
            <stop offset="1" stop-color="#334155"></stop>
          </linearGradient>
        </defs>
        <rect width="${width}" height="${height}" fill="url(#visionBg)"></rect>
        <path d="M0 130 L80 82 L205 82 L280 130" class="vision-lane"></path>
        <line x1="${target[0] * width - 10}" y1="${target[1] * height}" x2="${target[0] * width + 10}" y2="${target[1] * height}" class="vision-target"></line>
        <line x1="${target[0] * width}" y1="${target[1] * height - 10}" x2="${target[0] * width}" y2="${target[1] * height + 10}" class="vision-target"></line>
        ${points ? `<polygon points="${points}" class="vision-marker-box"></polygon>` : ""}
        ${markerCenter ? `
          <line x1="${markerCenter[0] * width}" y1="${markerCenter[1] * height}" x2="${target[0] * width}" y2="${target[1] * height}" class="vision-error-line"></line>
          <circle cx="${markerCenter[0] * width}" cy="${markerCenter[1] * height}" r="4" class="vision-marker-center"></circle>
        ` : ""}
        <text x="12" y="21" class="vision-overlay-label">${
          vision.marker_detected ? `ARUCO ${vision.marker_id}` : "MARKER SEARCH"
        }</text>
      </svg>
      <span class="vision-live-badge ${vision.connected ? "online" : "offline"}">${
        vision.connected ? "LIVE DATA" : "OFFLINE"
      }</span>
    </div>
    <div class="vision-state-row ${stateTone}">
      <i></i><strong>${statusLabels[vision.alignment_state] || vision.alignment_state}</strong>
      <span>${age == null ? "보정 시각 미수신" : `${age.toFixed(1)}초 전 갱신`}</span>
    </div>
    <dl class="vision-metrics">
      <div><dt>마커 ID</dt><dd>${vision.marker_id ?? "미검출"}</dd></div>
      <div><dt>거리</dt><dd>${metricText(vision.distance_m, "m")}</dd></div>
      <div><dt>횡 오차</dt><dd>${metricText(vision.lateral_error_mm, "mm")}</dd></div>
      <div><dt>전후 오차</dt><dd>${metricText(vision.longitudinal_error_mm, "mm")}</dd></div>
      <div><dt>Yaw 오차</dt><dd>${metricText(vision.yaw_error_deg, "°")}</dd></div>
      <div><dt>재투영 오차</dt><dd>${metricText(vision.reprojection_error_px, "px")}</dd></div>
    </dl>
    <div class="vision-pipeline">
      <span>카메라</span><i>→</i><span>ArUco</span><i>→</i><span>Odom 융합</span><i>→</i><span>정렬</span>
    </div>
  `;
}

function activateInspectorTab(tabName) {
  activeInspectorTab = ["task", "load", "vision"].includes(tabName)
    ? tabName
    : "task";
  document.querySelectorAll("[data-inspector-tab]").forEach((button) => {
    const active = button.dataset.inspectorTab === activeInspectorTab;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
  });
  document.querySelectorAll("[data-inspector-panel]").forEach((panel) => {
    const active = panel.dataset.inspectorPanel === activeInspectorTab;
    panel.classList.toggle("active", active);
    panel.hidden = !active;
  });
}

function setupInspectorTabs() {
  document.querySelectorAll("[data-inspector-tab]").forEach((button) => {
    button.addEventListener("click", () => {
      activateInspectorTab(button.dataset.inspectorTab);
    });
  });
  activateInspectorTab(activeInspectorTab);
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
    const requestIncident = (dashboard.safety_incidents || []).find(
      (incident) => incident.status !== "RECOVERED"
        && incident.affected_request_ids.includes(request.id)
    );
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
    const recovery = dashboard.system?.recovery || {};
    const requestNeedsRecovery = (
      recovery.source_request_ids || []
    ).some((requestId) => String(requestId) === String(request.id))
      && ["SAFETY_STOPPED", "REQUIRED", "RECOVERING", "BLOCKED"].includes(
        recovery.status
      );
    const safetyLabel = requestObstacle
      ? `${
          requestIncident ? `${formatSafetyIncidentId(requestIncident.id)} · ` : ""
        }${obstacleZoneLabel(requestObstacle.zone_id)} 장애물 대기`
      : requestNeedsRecovery
        ? recovery.status === "RECOVERING"
          ? "담당 로봇 도크 복귀 중"
          : "담당 로봇 복구 필요 · 현재 위치 안전 정지"
      : unavailableSensors.length
        ? `${unavailableSensors.map((sensor) => sensor.id).join(" · ")} 미수신 · 안전 감지 제한 운용`
        : "정상";
    const isCompleted = request.status === "COMPLETED";
    const isCancelled = request.status === "CANCELLED";
    const isTerminal = isCompleted || isCancelled;
    const cooperationWarning = (
      !isTerminal && formationGap != null && formationGap > 3
    );
    const cooperationLabel = isCompleted
      ? "정상 종료"
      : isCancelled
        ? "작업 종료"
        : assignedRobots.length < 2
          ? "로봇 배정 중"
          : cooperationWarning
            ? `간격 조정 필요 · ${formationGap.toFixed(1)} m`
            : "정상";
    const stageSummary = requestStepSummary(request);
    const detailKicker = isCompleted
      ? "최근 완료 작업"
      : isCancelled ? "취소 작업 상세" : "현재 작업 상세";
    const statusSummaryClass = requestObstacle
      ? "PAUSED"
      : isCompleted ? "ONLINE" : isCancelled ? "ERROR" : "BUSY";
    detail.innerHTML = `
      <span class="detail-kicker">${detailKicker}</span>
      <div class="detail-title-row">
        <h3>${requestTypeLabels[request.request_type]} #${request.id}</h3>
      </div>
      <div class="detail-status-summary ${statusSummaryClass}">
        <span>${isTerminal ? "작업 결과" : "현재 단계"}</span>
        <strong>${
          requestObstacle
            ? "장애물 대기"
            : isCancelled ? "취소" : `${stageSummary.number}. ${stageSummary.current}`
        }</strong>
        <small>${
          requestObstacle
            ? `${stageSummary.number}. ${stageSummary.current}에서 정지 · 해소 후 자동 재개`
            : isCompleted
              ? `최종 주차면 ${request.slot_id || "—"} · 작업 시간 ${formatTaskDuration(request)}`
              : isCancelled
                ? "작업·이벤트 탭에서 취소 원인 확인"
            : stageSummary.next
              ? `다음 단계 · ${stageSummary.next}`
              : "마지막 단계"
        }</small>
      </div>
      <dl class="detail-list robot-detail-list task-detail-grid">
        <div><dt>차량 번호</dt><dd>${request.vehicle_number}</dd></div>
        <div><dt>목표 주차면</dt><dd>${request.slot_id || "배정 중"}</dd></div>
        <div><dt>담당 로봇</dt><dd>${assignedIds.length ? assignedRobotTableLabel(request) : "배정 중"}</dd></div>
        <div><dt>${isTerminal ? "작업 시간" : "경과 시간"}</dt><dd>${
          isTerminal ? formatTaskDuration(request) : formatElapsed(request.created_at)
        }</dd></div>
      </dl>
      <dl class="detail-list task-health-list">
        <div><dt>협동 상태</dt><dd><span class="detail-state ${
          cooperationWarning ? "warning" : "normal"
        }">● ${cooperationLabel}</span></dd></div>
        <div><dt>안전 상태</dt><dd><span class="detail-state ${
          requestObstacle
            ? "danger"
            : requestNeedsRecovery || unavailableSensors.length
              ? "warning"
              : "normal"
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
        <div><dt>설치 위치</dt><dd>map (${Number(sensor.x).toFixed(2)}, ${Number(sensor.y).toFixed(2)}, ${Number(sensor.z ?? 5.12).toFixed(2)}m)</dd></div>
        <div><dt>스캔 방식</dt><dd>${sensor.fov_deg || 360}° · ${sensor.zone || "주차장 전체"}</dd></div>
        <div><dt>좌표 프레임</dt><dd>${sensor.frame_id || "map"}</dd></div>
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
    let alert = (dashboard.alerts || []).find(
      (item) => item.category === "OBSTACLE"
        && String(item.id) === String(selectedMapItem.id)
    );
    const incident = safetyIncidentForAlert(selectedMapItem.id, dashboard);
    if (!alert && !incident) {
      selectedMapItem = null;
      renderSelectionDetail(dashboard);
      return;
    }
    if (!alert) {
      alert = {
        id: incident.alert_id,
        category: "OBSTACLE",
        message: "장애물이 해소되어 정상 운용을 재개했습니다.",
        sensor_id: incident.sensor_id,
        zone_id: incident.zone_id,
        location_x: incident.location_x,
        location_y: incident.location_y,
        created_at: incident.detected_at,
      };
    }
    const affectedRobots = incident
      ? robots.filter((robot) => incident.affected_robot_ids.includes(robot.id))
      : affectedRobotsForObstacle(alert, robots);
    const affectedIds = new Set(affectedRobots.map((robot) => robot.id));
    const relatedRequest = (dashboard.requests || []).find(
      (request) => incident
        ? incident.affected_request_ids.includes(request.id)
        : !["COMPLETED", "CANCELLED"].includes(request.status)
          && assignedRobotIds(request).some((robotId) => affectedIds.has(robotId))
    );
    const coordinateLabel = alert.location_x == null || alert.location_y == null
      ? "위치 정보 없음"
      : `x ${Number(alert.location_x).toFixed(2)} · y ${Number(alert.location_y).toFixed(2)} m`;
    const detectingSensor = sensors.find(
      (sensor) => sensor.id === alert.sensor_id
    ) || sensors[0];
    const detectionDistance = (
      detectingSensor
      && alert.location_x != null
      && alert.location_y != null
      && detectingSensor.x != null
      && detectingSensor.y != null
    )
      ? Math.hypot(
        Number(alert.location_x) - Number(detectingSensor.x),
        Number(alert.location_y) - Number(detectingSensor.y)
      )
      : null;
    detail.innerHTML = `
      <span class="detail-kicker ${
        incident?.status === "RECOVERED" ? "recovered" : "danger"
      }">${
        incident
          ? `안전 사건 ${formatSafetyIncidentId(incident.id)}`
          : "LiDAR 안전 감지"
      }</span>
      <div class="detail-title-row">
        <h3><span class="detail-danger-icon ${
          incident?.status === "RECOVERED" ? "recovered" : ""
        }">${incident?.status === "RECOVERED" ? "✓" : "!"}</span>${
          incident?.status === "RECOVERED" ? "안전 사건 복구 완료" : "장애물 안전정지"
        }</h3>
      </div>
      <div class="detail-status-summary ${incident?.status === "RECOVERED" ? "ONLINE" : "PAUSED"}">
        <span>현재 안전 조치</span>
        <strong>${
          incident?.status === "RECOVERED"
            ? "정상 운용 재개"
            : affectedRobots.length
            ? "영향 로봇 일시 정지"
            : "해당 통로 신규 진입 차단"
        }</strong>
        <small>${
          incident?.status === "RECOVERED"
            ? `${formatDateTime(incident.resolved_at)} 복구`
            : alert.message
        }</small>
      </div>
      ${incident ? `
        <ol class="safety-incident-timeline" aria-label="안전 사건 처리 과정">
          ${incident.events.map((event) => `
            <li class="done ${event.stage}">
              <i>${event.stage === "OPERATION_RESUMED" ? "✓" : "•"}</i>
              <div>
                <strong>${incidentEventStageLabel(event.stage)}</strong>
                <span>${event.message}</span>
                <time>${formatEventTime(event.created_at)}</time>
              </div>
            </li>
          `).join("")}
          ${incident.status !== "RECOVERED" ? `
            <li class="pending">
              <i>4</i>
              <div>
                <strong>해소·자동 재개 대기</strong>
                <span>센서 해소 프레임 수신 후 자동으로 재개합니다.</span>
              </div>
            </li>
          ` : ""}
        </ol>
      ` : ""}
      <dl class="detail-list">
        <div><dt>감지 센서</dt><dd>${alert.sensor_id || "센서 ID 미수신"}</dd></div>
        <div><dt>감지 객체</dt><dd>장애물 · 객체 분류 미지원</dd></div>
        <div><dt>감지 구역</dt><dd>${obstacleZoneLabel(alert.zone_id)}</dd></div>
        <div><dt>감지 좌표</dt><dd>${coordinateLabel}</dd></div>
        <div><dt>센서 거리</dt><dd>${
          detectionDistance == null ? "거리 계산 불가" : `${detectionDistance.toFixed(2)} m`
        }</dd></div>
        <div><dt>안전 영향</dt><dd>관련 통로 진입 차단 · 영향 로봇 일시 정지</dd></div>
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
        ${incident?.resolved_at ? `
          <div><dt>복구 시각</dt><dd>${formatDateTime(incident.resolved_at)}</dd></div>
        ` : ""}
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
  const recoverySafetyLabel = {
    SAFETY_STOPPED: "비상정지 유지 · 현재 위치 고정",
    RECOVERY_REQUIRED: "현장 점검 완료 · 도크 복귀 필요",
    RECOVERING: "안전 경로로 도크 복귀 중",
  }[robot.status];
  const safetyLabel = robotAlert?.message
    || robot.error_message
    || recoverySafetyLabel
    || "이상 없음";
  const isObstaclePaused = robotAlert?.category === "OBSTACLE";
  detail.innerHTML = `
    <span class="detail-kicker">로봇 상세</span>
    <div class="detail-title-row">
      <h3>${parkingRobotHtmlIcon("detail-robot-icon", "정면 센서형 주차로봇")}${shortRobotName(robot.id)}</h3>
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
      <div><dt>안전</dt><dd><span class="detail-state ${
        robotAlert || robot.error_message
          ? "danger"
          : recoverySafetyLabel ? "warning" : "normal"
      }">● ${safetyLabel}</span></dd></div>
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

function renderRecentEvents(
  alerts,
  safetyIncidents = latestDashboard?.safety_incidents || []
) {
  const incidentAlertIds = new Set(
    safetyIncidents.map((incident) => String(incident.alert_id))
  );
  const safetyIncidentEvents = safetyIncidents.flatMap((incident) =>
    incident.events.map((event) => ({
      time: event.created_at,
      tone: ["OBSTACLE_CLEARED", "OPERATION_RESUMED"].includes(event.stage)
        ? "success"
        : "warning",
      category: "safety",
      request_id: incident.affected_request_ids[0] ?? null,
      request_ids: incident.affected_request_ids,
      incident_id: incident.id,
      alert_id: incident.alert_id,
      sequence: incidentEventStageOrder(event.stage),
      label: `${formatSafetyIncidentId(incident.id)} · ${incidentEventStageLabel(event.stage)}`,
      description: event.message,
    }))
  );
  let events = [
    ...alerts
      .filter(
        (alert) => alert.category !== "OBSTACLE"
          || !incidentAlertIds.has(String(alert.id))
      )
      .map((alert) => ({
      time: alert.created_at,
      tone: alert.level === "ERROR" ? "danger" : "warning",
      category: eventCategoryForAlert(alert),
      request_id: null,
      label: alertLabels[alert.category] || "시스템 이벤트",
      description: alert.message,
    })),
    ...safetyIncidentEvents,
    ...recentWorkflowEvents,
  ]
    .sort((a, b) => (
      new Date(b.time) - new Date(a.time)
      || (b.sequence ?? -1) - (a.sequence ?? -1)
    ));

  if (selectedEventScope === "selected" && selectedTaskEventRequestId != null) {
    events = events.filter(
      (event) => event.request_id === selectedTaskEventRequestId
        || event.request_ids?.includes(selectedTaskEventRequestId)
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
    <article class="recent-event-item ${event.tone} ${event.category} ${
      event.incident_id ? "selectable-incident" : ""
    }" ${
      event.incident_id
        ? `role="button" tabindex="0" data-incident-alert-id="${event.alert_id}"`
        : ""
    }>
      <time>${formatEventTime(event.time)}</time>
      <i aria-hidden="true">${eventIcon(event)}</i>
      <div>
        <strong>${event.label}</strong>
        <span>${event.description}</span>
      </div>
    </article>
  `).join("");

  container.querySelectorAll("[data-incident-alert-id]").forEach((element) => {
    const select = () => {
      selectMapItem("obstacle", element.dataset.incidentAlertId);
      activateWorkspaceTab("live");
    };
    element.addEventListener("click", select);
    element.addEventListener("keydown", (event) => {
      if (!["Enter", " "].includes(event.key)) return;
      event.preventDefault();
      select();
    });
  });
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

function requestStepSummary(request) {
  const index = requestProgressIndex(request.status);
  return {
    number: index + 1,
    current: taskProgressSteps[index]?.label || requestStatusLabel(request),
    next: taskProgressSteps[index + 1]?.label || null,
  };
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

function formatTaskDuration(request) {
  if (!request?.created_at) return "—";
  const startedAt = new Date(request.created_at).getTime();
  const endedAt = request.completed_at
    ? new Date(request.completed_at).getTime()
    : Date.now();
  if (!Number.isFinite(startedAt) || !Number.isFinite(endedAt)) return "—";
  const elapsedSec = Math.max(0, Math.floor((endedAt - startedAt) / 1000));
  const hours = Math.floor(elapsedSec / 3600);
  const minutes = String(Math.floor((elapsedSec % 3600) / 60)).padStart(2, "0");
  const seconds = String(elapsedSec % 60).padStart(2, "0");
  return hours > 0
    ? `${String(hours).padStart(2, "0")}:${minutes}:${seconds}`
    : `${minutes}:${seconds}`;
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

function renderActiveTaskBanner(requests, alerts = [], sensors = [], system = null) {
  const banner = document.getElementById("activeTaskBanner");
  const request = requests.find(
    (item) => !["COMPLETED", "CANCELLED"].includes(item.status)
  );
  if (!request) {
    const recentRequest = requests.find(
      (item) => ["COMPLETED", "CANCELLED"].includes(item.status)
    );
    if (!recentRequest) {
      banner.classList.add("hidden");
      banner.classList.remove("idle-recent", "danger", "obstacle-wait");
      banner.innerHTML = "";
      banner.removeAttribute("role");
      banner.removeAttribute("tabindex");
      banner.onclick = null;
      banner.onkeydown = null;
      return;
    }
    const completed = recentRequest.status === "COMPLETED";
    banner.classList.remove("hidden", "danger", "obstacle-wait");
    banner.classList.add("idle-recent");
    banner.setAttribute("role", "button");
    banner.setAttribute("tabindex", "0");
    banner.setAttribute(
      "aria-label",
      `최근 ${completed ? "완료" : "취소"} 작업 #${recentRequest.id} 상세 보기`
    );
    const selectRecentTask = () =>
      selectMapItem("task", String(recentRequest.id));
    banner.onclick = selectRecentTask;
    banner.onkeydown = (event) => {
      if (!["Enter", " "].includes(event.key)) return;
      event.preventDefault();
      selectRecentTask();
    };
    banner.innerHTML = `
      <div class="active-task-idle-icon" aria-hidden="true">${completed ? "✓" : "!"}</div>
      <div class="active-task-idle-copy">
        <span>현재 진행 작업 없음</span>
        <strong>최근 ${completed ? "완료" : "취소"} 작업 · ${requestTypeLabels[recentRequest.request_type]} #${recentRequest.id}</strong>
        <small>차량 ${recentRequest.vehicle_number} → ${recentRequest.slot_id || "주차면 미배정"} · ${
          completed ? `소요 ${formatTaskDuration(recentRequest)}` : "원인 확인 필요"
        }</small>
      </div>
      <span class="active-task-idle-action">최근 작업 보기 ›</span>
    `;
    return;
  }

  const obstacleAlert = obstacleForRequest(request, alerts);
  const obstacleActive = obstacleAlert != null;
  const unavailableSensors = sensors.filter((sensor) => {
    if (system?.mode === "mock") return !["MOCK", "ONLINE"].includes(sensor.status);
    return sensor.status !== "ONLINE";
  });
  const sensorLimited = unavailableSensors.length > 0;
  const step = requestProgressIndex(request.status) + 1;
  banner.classList.remove("hidden", "idle-recent", "danger");
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
      <strong>
        <span>${requestTypeLabels[request.request_type]} #${request.id}</span>
        <i aria-hidden="true">·</i>
        <span class="active-task-route">${request.vehicle_number} → ${request.slot_id || "슬롯 배정 중"}</span>
      </strong>
      <span class="sr-only">${
        obstacleActive
          ? `장애물 대기: ${obstacleAlert.sensor_id || "센서"} · ${obstacleZoneLabel(obstacleAlert.zone_id)}`
          : `${requestOperationalStatusLabel(request, alerts)}`
      }${sensorLimited ? `, ${unavailableSensors.map((sensor) => sensor.id).join(" · ")} 미수신으로 센서 제한 운용 중` : ""}</span>
    </div>
    <ol class="active-task-mini-steps" aria-label="${step} / 6 단계">
      ${taskProgressSteps.map((item, index) => {
        const state = index < step - 1
          ? "done"
          : index === step - 1 ? "current" : "pending";
        return `
          <li class="${state}">
            <i>${state === "done" ? "✓" : index + 1}</i>
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
    ? `🔔 센서 알림 ${hiddenOfflineSensorCount} 다시 보기`
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
              ${alert.category === "OBSTACLE" ? (() => {
                const incident = safetyIncidentForAlert(alert.id);
                if (!incident) return "";
                return `
                  <span class="alert-incident-badge">
                    ${formatSafetyIncidentId(incident.id)} · ${
                      incident.status === "RECOVERED" ? "복구 완료" : "안전정지"
                    }
                  </span>
                `;
              })() : ""}
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
          ` : alert.category === "OBSTACLE" ? `
            <div class="alert-actions">
              ${alert.location_x != null && alert.location_y != null ? `
              <button
                type="button"
                class="secondary-button small"
                data-focus-obstacle-alert="${alert.id}"
              >
                위치 보기
              </button>
              ` : ""}
              ${system?.mode === "mock" ? `
              <button
                type="button"
                class="secondary-button small"
                data-resolve-obstacle-alert="${alert.id}"
              >
                장애물 해제
              </button>
              ` : `
              <span class="alert-latched">해소 시 자동 해제</span>
              `}
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
  list.querySelectorAll("[data-resolve-obstacle-alert]").forEach((button) => {
    button.addEventListener("click", async () => {
      button.disabled = true;
      button.textContent = "해제 중…";
      await resolveAlert(button.dataset.resolveObstacleAlert);
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
  const recovery = system.recovery || { status: "NONE", robot_ids: [] };
  const recoveryPending = [
    "SAFETY_STOPPED",
    "REQUIRED",
    "RECOVERING",
    "BLOCKED",
  ].includes(recovery.status);
  const warningReason = safety.state === "READY_FOR_OPERATION"
    ? recovery.status === "RECOVERING"
      ? "제한 로봇 복귀 중"
      : ["NONE", "COMPLETED"].includes(recovery.status)
        ? "정상 운영 승인 대기"
        : "로봇 안전 복귀 필요"
    : safety.state === "UNKNOWN"
      ? "안전 관리자 확인 필요"
      : system.emergency_stop
        ? "비상정지 작동"
    : recovery.status === "RECOVERING"
      ? "로봇 안전 복귀 중"
      : recoveryPending
        ? "로봇 복구 필요"
    : activeAlerts.find(
    (alert) => alert.category === "OBSTACLE"
  )
    ? "장애물 감지"
    : activeAlerts.find((alert) => alert.level === "ERROR")
      ? "로봇·시스템 오류"
      : offlineSensors.length
        ? "센서 제한 운용"
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
  const hardStopped = safety.state === "STOPPED_LATCHED";
  emergencyButton.disabled = hardStopped;
  emergencyButton.classList.toggle("active", hardStopped);
  emergencyButton.innerHTML = hardStopped
    ? `<span aria-hidden="true">■</span> 비상정지 작동 중`
    : recovery.status === "RECOVERING"
      ? `<span aria-hidden="true">■</span> 복귀 운전 정지`
      : `<span aria-hidden="true">■</span> 비상정지`;

  document.querySelectorAll("#requestForm input, #requestForm select, #requestForm button")
    .forEach((control) => {
      control.disabled = Boolean(system.emergency_stop)
        || recoveryPending
        || requestSubmitting;
    });

  // 백업은 모드와 무관하게 항상 노출 (서버 호출 없이 현재 화면 데이터만 내려받음).
  document
    .getElementById("resetButton")
    .classList.toggle("hidden", system.mode !== "mock");
  document.getElementById("resetButton").disabled =
    Boolean(system.emergency_stop) || recoveryPending;
  document
    .getElementById("dbResetButton")
    .classList.toggle("hidden", system.mode !== "ros2");
  document.getElementById("dbResetButton").disabled =
    Boolean(system.emergency_stop) || recoveryPending;
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
  const recoveryForm = document.getElementById("robotRecoveryForm");
  const stateBox = document.getElementById("safetyRecoveryState");
  const recovery = system?.recovery || {
    status: "NONE",
    robot_ids: [],
    control_available: false,
  };
  const labels = {
    STOPPED_LATCHED: "비상정지 · 현장 점검 필요",
    READY_FOR_OPERATION: "점검 완료 · 다음 복구 단계 확인",
    NORMAL: "정상 운영",
    UNKNOWN: "중앙 안전 상태 확인 중",
  };
  const recoveryStateLabel = safety.state === "READY_FOR_OPERATION"
    ? recovery.status === "RECOVERING"
      ? "제한 안전 복귀 운전 중"
      : recovery.status === "COMPLETED"
        ? "도크 복귀 완료 · 정상 운영 승인 대기"
        : recovery.status === "NONE"
          ? "점검 완료 · 정상 운영 승인 대기"
          : "점검 완료 · 로봇 안전 복귀 필요"
    : null;
  stateBox.innerHTML = `
    <strong>${recoveryStateLabel || labels[safety.state] || safety.state || "상태 확인 중"}</strong>
    <span>정지 세대 #${safety.stop_epoch || 0}</span>
    ${safety.reason ? `<p>${safety.reason}</p>` : ""}
    ${(safety.blockers || []).length
      ? `<ul>${safety.blockers.map((blocker) => `<li>${blocker}</li>`).join("")}</ul>`
      : ""}
  `;
  resetForm.classList.toggle("hidden", safety.state !== "STOPPED_LATCHED");
  const showRecovery = safety.state === "READY_FOR_OPERATION"
    && ["REQUIRED", "RECOVERING", "BLOCKED"].includes(
      recovery.status
    );
  const showApproval = safety.state === "READY_FOR_OPERATION"
    && ["NONE", "COMPLETED"].includes(recovery.status);
  approvalForm.classList.toggle("hidden", !showApproval);
  recoveryForm.classList.toggle("hidden", !showRecovery);
  if (showRecovery) {
    const recoveryLabels = {
      REQUIRED: "복구 대기",
      RECOVERING: "도크 복귀 중",
      BLOCKED: "복귀 차단",
      COMPLETED: "도크 복귀 완료",
    };
    const targetNames = (recovery.robot_ids || []).map(shortRobotName);
    document.getElementById("robotRecoveryTargets").innerHTML = `
      <strong>${recoveryLabels[recovery.status] || recovery.status}</strong>
      <span>대상 로봇 · ${targetNames.join(" · ") || "없음"}</span>
      <p>${recovery.message || "현재 위치와 도크 경로를 확인해주세요."}</p>
    `;
    const unavailable = document.getElementById("robotRecoveryUnavailable");
    const startButton = document.getElementById("startRobotRecoveryButton");
    const cannotStart = recovery.control_available === false
      || recovery.status !== "REQUIRED";
    startButton.disabled = cannotStart;
    startButton.textContent = recovery.status === "RECOVERING"
      ? "로봇 도크 복귀 중"
      : recovery.status === "COMPLETED"
        ? "도크 복귀 완료"
        : recovery.control_available === false
          ? "실제 복귀 제어기 연결 필요"
          : "로봇 안전 복귀 시작";
    unavailable.classList.toggle(
      "hidden", recovery.control_available !== false
    );
    unavailable.textContent = recovery.control_available === false
      ? "현재 ROS2 관제에는 도크 복귀 action이 연결되지 않았습니다. "
        + "로봇은 현재 위치에서 정지를 유지하며, 제어기 연결 전에는 대기로 표시하지 않습니다."
      : "";
  }
  if (showApproval) {
    const targetNames = (recovery.robot_ids || []).map(shortRobotName);
    document.getElementById("operationRecoverySummary").innerHTML = `
      <strong>${
        recovery.status === "COMPLETED"
          ? "2단계 완료 · 도크 복귀 확인"
          : "복귀 대상 로봇 없음"
      }</strong>
      <span>${
        targetNames.length
          ? `확인 로봇 · ${targetNames.join(" · ")}`
          : "현장 점검 결과를 기준으로 정상 운영 승인을 진행합니다."
      }</span>
      <p>${
        recovery.status === "COMPLETED"
          ? recovery.message
          : "정상 운영 승인 후에만 새 작업을 접수합니다."
      }</p>
    `;
  }
  if (
    showApproval &&
    !document.getElementById("operationApprovalOperator").value
  ) {
    document.getElementById("operationApprovalOperator").value =
      safety.operator_id || "";
  }
  if (
    showRecovery
    && !document.getElementById("robotRecoveryOperator").value
  ) {
    document.getElementById("robotRecoveryOperator").value =
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
  latestRequestResult = null;
  messageBox.textContent = message;
  messageBox.classList.remove(
    "hidden",
    "error",
    "request-result",
    "compact"
  );

  if (isError) {
    messageBox.classList.add("error");
  }

  messageHideTimer = window.setTimeout(() => {
    messageBox.classList.add("hidden");
  }, isError ? 5000 : 4000);
}

function showRequestResult(result, compact = false) {
  const messageBox = document.getElementById("messageBox");
  const requestLabel = requestTypeLabels[result.request_type] || "주차";

  latestRequestResult = result;
  window.clearTimeout(messageHideTimer);
  messageBox.replaceChildren();
  messageBox.classList.remove("hidden", "error", "compact");
  messageBox.classList.add("request-result");
  if (compact) messageBox.classList.add("compact");

  const kicker = document.createElement("span");
  kicker.className = "request-result-kicker";
  kicker.textContent = "최근 등록 작업";

  const title = document.createElement("strong");
  title.className = "request-result-title";
  title.textContent = compact
    ? `${requestLabel} #${result.id} · ${result.vehicle_number} → ${
      result.slot_id || "배정 대기"
    }`
    : `${requestLabel} 요청 #${result.id}이 등록되었습니다.`;

  const summary = document.createElement("p");
  summary.className = "request-result-summary";
  [
    `차량 ${result.vehicle_number}`,
    result.slot_id ? `주차면 ${result.slot_id}` : "주차면 배정 대기",
    (result.robot_ids || []).length
      ? `담당 로봇 ${(result.robot_ids || []).map(shortRobotName).join("·")}`
      : "담당 로봇 배정 대기",
  ].forEach((text) => {
    const item = document.createElement("span");
    item.textContent = text;
    summary.append(item);
  });

  const actions = document.createElement("div");
  actions.className = "request-result-actions";
  const liveButton = document.createElement("button");
  liveButton.type = "button";
  liveButton.textContent = compact ? "관제 보기" : "관제 화면에서 보기";
  liveButton.addEventListener("click", () => activateWorkspaceTab("live"));
  const taskButton = document.createElement("button");
  taskButton.type = "button";
  taskButton.textContent = compact ? "작업 보기" : "작업·이벤트에서 보기";
  taskButton.addEventListener("click", () => {
    document.getElementById("taskSearchInput").value = result.vehicle_number;
    renderRequests(
      latestDashboard?.requests || [],
      latestDashboard?.system
    );
    activateWorkspaceTab("tasks");
  });
  actions.append(liveButton, taskButton);
  messageBox.append(kicker, title);
  if (!compact) messageBox.append(summary);
  messageBox.append(actions);

  if (!compact) {
    messageHideTimer = window.setTimeout(() => {
      if (latestRequestResult === result) showRequestResult(result, true);
    }, 10000);
  }
}

function compactLatestRequestResult() {
  const messageBox = document.getElementById("messageBox");
  if (
    latestRequestResult
    && messageBox.classList.contains("request-result")
    && !messageBox.classList.contains("compact")
  ) {
    showRequestResult(latestRequestResult, true);
  }
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

    // 첫 화면에서도 상세 패널이 비어 보이지 않도록 진행 작업을 우선한다.
    // 진행 작업이 없으면 최근 완료/취소 작업을 선택해 "현재 작업 없음"과
    // 우측의 과거 작업 상세가 서로 모순되어 보이지 않게 한다.
    if (!selectedMapItem) {
      const activeRequest = data.requests.find(
        (request) => !["COMPLETED", "CANCELLED"].includes(request.status)
      );
      if (activeRequest) {
        selectedMapItem = { type: "task", id: String(activeRequest.id) };
      } else {
        const recentRequest = data.requests.find(
          (request) => ["COMPLETED", "CANCELLED"].includes(request.status)
        );
        if (recentRequest) {
          selectedMapItem = { type: "task", id: String(recentRequest.id) };
        } else {
          const defaultSlot = data.slots.find(
            (slot) => slot.status === "OCCUPIED"
          ) || data.slots[0];
          if (defaultSlot) selectedMapItem = { type: "slot", id: defaultSlot.id };
        }
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
    renderActiveTaskBanner(
      data.requests || [],
      data.alerts || [],
      data.sensors || [],
      data.system
    );
    captureRequestEvents(data.requests, data.system);
    captureAlertSpeech(data.alerts || [], data.robots, data.system);
    updateRobotAnimationTargets(data.robots);
    renderLotMap(
      data.slots,
      applyRobotDisplayPositions(data.robots),
      data.map,
      data.sensors || [],
      data.requests,
      data.alerts || [],
      data.safety_incidents || [],
      data.cooperative_loads || []
    );
    ensureRobotAnimationLoop();
    renderSelectionDetail(data);
    renderCooperativeLoadDetail(data);
    renderVisionAlignmentDetail(data);
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

function requestFlowPlaceMarkup(kind) {
  if (kind === "vehicle") {
    return `
      <svg viewBox="0 0 24 24" aria-hidden="true">
        <path d="M3 13h18M5 13l2-5h10l2 5v6h-2v-2H7v2H5v-6Z"></path>
        <circle cx="8" cy="14.5" r="1"></circle>
        <circle cx="16" cy="14.5" r="1"></circle>
      </svg>
      <b>차량</b>
    `;
  }
  return `
    <svg viewBox="0 0 24 24" aria-hidden="true">
      <rect x="4" y="3" width="16" height="18" rx="3"></rect>
      <path d="M9 17V7h4a3 3 0 0 1 0 6H9m0-3h4"></path>
    </svg>
    <b>주차장</b>
  `;
}

function setRequestType(requestType) {
  const normalized = requestFlowDefinitions[requestType]
    ? requestType
    : "PARK_IN";
  document.getElementById("requestType").value = normalized;
  document.querySelectorAll("[data-request-type]").forEach((button) => {
    const selected = button.dataset.requestType === normalized;
    button.classList.toggle("active", selected);
    button.setAttribute("aria-pressed", String(selected));
  });
  updateRequestFlow();
}

function updateRequestFlow() {
  const requestType = document.getElementById("requestType").value;
  const flow = requestFlowDefinitions[requestType]
    || requestFlowDefinitions.PARK_IN;
  const specificLabel = requestType === "PARK_IN"
    ? "입차 전용"
    : "출차 전용";
  document.getElementById("requestFlowTitle").textContent = flow.title;
  document.getElementById("requestFlowStart").innerHTML =
    requestFlowPlaceMarkup(flow.start);
  document.getElementById("requestFlowEnd").innerHTML =
    requestFlowPlaceMarkup(flow.end);
  document.getElementById("requestFlowDirection").textContent = "→";
  document.getElementById("requestFlowSteps").innerHTML = flow.steps
    .map(
      ([title, description], index) => `
        <li class="${[1, 4].includes(index) ? "flow-specific" : ""}">
          <i>${index + 1}</i>
          <span>
            <strong>
              ${title}
              ${[1, 4].includes(index) ? `<em>${specificLabel}</em>` : ""}
            </strong>
            <small>${description}</small>
          </span>
        </li>
      `
    )
    .join("");
  updateRequestAvailability();
}

function normalizeVehicleNumber(value) {
  return String(value || "").replace(/\s+/g, "");
}

function hasSupportedVehicleNumberFormat(value) {
  const normalized = normalizeVehicleNumber(value);
  return /^\d{4}$/.test(normalized)
    || /^(?:[가-힣]{2})?\d{2,3}[가-힣]\d{4}$/.test(normalized);
}

function evaluateRequestPreflight() {
  const requestType = document.getElementById("requestType").value;
  const vehicleNumber = normalizeVehicleNumber(
    document.getElementById("vehicleNumber").value
  );
  const dashboard = latestDashboard;
  const roleLabel = requestType === "PARK_IN" ? "입차" : "출차";
  const expectedTeamIds = requestRobotTeams[requestType];
  const robots = dashboard?.robots || [];
  const team = expectedTeamIds
    .map((robotId) => robots.find((robot) => robot.id === robotId))
    .filter(Boolean);
  const teamConnected = team.length === expectedTeamIds.length
    && team.every((robot) => !["ERROR", "OFFLINE"].includes(robot.status));
  const teamReady = teamConnected && team.every(
    (robot) => robot.status === "IDLE" && robot.current_task_id == null
  );
  const teamLabel = teamReady
    ? `${roleLabel} L·F · 정상`
    : teamConnected
      ? `${roleLabel} L·F · 배정 대기`
      : `${roleLabel} L·F · 연결 확인`;

  const base = {
    state: "pending",
    status: "정보 확인 중",
    message: "관제 데이터를 수신하면 요청 가능 여부를 표시합니다.",
    submitBlocked: true,
    vehicleNumber,
    targetLabel: requestType === "PARK_IN" ? "가용 주차면" : "차량 위치",
    target: "확인 중",
    team: dashboard ? teamLabel : "확인 중",
    assignment: requestType === "PARK_IN" ? "자동 배정" : null,
    vehicleState: requestType === "PARK_OUT" ? "입력 대기" : null,
    eligibility: requestType === "PARK_OUT"
      ? "차량 조회 후 확인"
      : null,
  };
  if (!dashboard) return base;

  const slots = dashboard.slots || [];
  const requests = dashboard.requests || [];
  const normalizedVehicle = vehicleNumber.toLocaleLowerCase("ko-KR");
  const activeForVehicle = vehicleNumber
    ? requests.find(
      (request) =>
        !["COMPLETED", "CANCELLED"].includes(request.status)
        && normalizeVehicleNumber(request.vehicle_number)
          .toLocaleLowerCase("ko-KR")
          === normalizedVehicle
    )
    : null;

  if (requestType === "PARK_IN") {
    const emptySlots = slots.filter((slot) => slot.status === "EMPTY");
    const parkedSlot = vehicleNumber
      ? slots.find(
        (slot) =>
          ["RESERVED", "OCCUPIED"].includes(slot.status)
          && normalizeVehicleNumber(slot.vehicle_number)
            .toLocaleLowerCase("ko-KR")
            === normalizedVehicle
      )
      : null;
    base.target = `${emptySlots.length}면`;

    if (!vehicleNumber) {
      return {
        ...base,
        status: "차량 번호를 입력해 주세요.",
        message: `입력 후 중복 여부와 요청 가능 상태를 확인합니다. 현재 가용 주차면은 ${emptySlots.length}면입니다.`,
      };
    }
    if (!hasSupportedVehicleNumberFormat(vehicleNumber)) {
      return {
        ...base,
        state: "blocked",
        status: "차량 번호 형식을 확인해 주세요.",
        message: "예: 12가3456 형식으로 입력해 주세요. 발표용 숫자 차량번호 4자리도 사용할 수 있습니다.",
      };
    }
    if (activeForVehicle) {
      return {
        ...base,
        state: "blocked",
        status: "입차 요청 불가",
        message: `${vehicleNumber} 차량의 작업 #${activeForVehicle.id}이 이미 진행 중입니다.`,
      };
    }
    if (parkedSlot) {
      return {
        ...base,
        state: "blocked",
        status: "입차 요청 불가",
        message: `${vehicleNumber} 차량은 ${parkedSlot.id}에 주차 또는 예약되어 있습니다. 출차 요청을 선택해 주세요.`,
      };
    }
    if (emptySlots.length === 0) {
      return {
        ...base,
        state: "blocked",
        status: "입차 요청 불가",
        message: "현재 가용 주차면이 없습니다.",
      };
    }
    return {
      ...base,
      state: teamReady ? "ready" : "warning",
      status: teamReady ? "입차 요청 가능" : "입차 요청 가능 · 로봇 배정 대기",
      message: teamReady
        ? `가용 주차면 ${emptySlots.length}면 · 입차 로봇팀이 대기 중입니다.`
        : `가용 주차면 ${emptySlots.length}면 · 요청은 등록되며 로봇팀이 준비되면 배정됩니다.`,
      submitBlocked: false,
    };
  }

  base.target = vehicleNumber ? "조회 중" : "차량 번호 입력 후 조회";
  if (!vehicleNumber) {
    return {
      ...base,
      status: "차량 번호를 입력해 주세요.",
      message: "차량 번호를 입력하면 현재 주차면과 출차 가능 여부를 확인합니다.",
    };
  }
  if (!hasSupportedVehicleNumberFormat(vehicleNumber)) {
    return {
      ...base,
      state: "blocked",
      status: "차량 번호 형식을 확인해 주세요.",
      message: "예: 12가3456 형식으로 입력해 주세요. 발표용 숫자 차량번호 4자리도 사용할 수 있습니다.",
      target: "조회하지 않음",
      vehicleState: "형식 오류",
      eligibility: "불가",
    };
  }

  const occupiedSlot = slots.find(
    (slot) =>
      slot.status === "OCCUPIED"
      && normalizeVehicleNumber(slot.vehicle_number)
        .toLocaleLowerCase("ko-KR")
        === normalizedVehicle
  );
  if (activeForVehicle) {
    return {
      ...base,
      state: "blocked",
      status: "출차 요청 불가",
      message: `${vehicleNumber} 차량의 작업 #${activeForVehicle.id}이 이미 진행 중입니다.`,
      target: activeForVehicle.slot_id || "작업에서 확인",
      vehicleState: `${requestTypeLabels[activeForVehicle.request_type]} 요청 진행 중`,
      eligibility: "새 요청 불가",
    };
  }
  if (!occupiedSlot) {
    return {
      ...base,
      state: "blocked",
      status: "출차 요청 불가",
      message: "현재 주차 중인 차량에서 찾을 수 없습니다. 차량 번호를 다시 확인해 주세요.",
      target: "조회 결과 없음",
      vehicleState: "등록 정보 없음",
      eligibility: "불가",
    };
  }
  return {
    ...base,
    state: teamReady ? "ready" : "warning",
    status: teamReady ? "출차 요청 가능" : "출차 요청 가능 · 로봇 배정 대기",
    message: teamReady
      ? `${vehicleNumber} · ${occupiedSlot.id} 주차 중 · 출차 로봇팀이 대기 중입니다.`
      : `${vehicleNumber} · ${occupiedSlot.id} 주차 중 · 요청은 등록되며 로봇팀 준비 후 배정됩니다.`,
    submitBlocked: false,
    target: `${occupiedSlot.id} · 주차 중`,
    vehicleState: "주차 중",
    eligibility: teamReady ? "가능" : "가능 · 로봇 배정 대기",
  };
}

function renderRequestPreflight(preflight) {
  const requestType = document.getElementById("requestType").value;
  const box = document.getElementById("requestPreflight");
  box.classList.remove("pending", "ready", "warning", "blocked");
  box.classList.add(preflight.state);
  document.getElementById("requestPreflightStatus").textContent =
    preflight.status;
  document.getElementById("requestPreflightMessage").textContent =
    preflight.message;
  document.getElementById("requestReviewType").textContent =
    requestTypeLabels[requestType];
  document.getElementById("requestReviewVehicle").textContent =
    preflight.vehicleNumber || "입력 대기";
  document.getElementById("requestReviewTargetLabel").textContent =
    preflight.targetLabel;
  document.getElementById("requestReviewTarget").textContent =
    preflight.target;
  document.getElementById("requestReviewTeam").textContent =
    preflight.team;
  const isParkIn = requestType === "PARK_IN";
  document
    .getElementById("requestReviewAssignmentRow")
    .classList.toggle("hidden", !isParkIn);
  document
    .getElementById("requestReviewVehicleStateRow")
    .classList.toggle("hidden", isParkIn);
  document
    .getElementById("requestReviewEligibilityRow")
    .classList.toggle("hidden", isParkIn);
  document.getElementById("requestReviewAssignment").textContent =
    preflight.assignment || "자동 배정";
  document.getElementById("requestReviewVehicleState").textContent =
    preflight.vehicleState || "입력 대기";
  document.getElementById("requestReviewEligibility").textContent =
    preflight.eligibility || "차량 조회 후 확인";
  renderVehicleFieldValidation(preflight);
}

function renderVehicleFieldValidation(preflight) {
  const input = document.getElementById("vehicleNumber");
  const error = document.getElementById("vehicleNumberError");
  let message = "";

  if (vehicleFieldTouched && !preflight.vehicleNumber) {
    message = "차량 번호를 입력해 주세요.";
  } else if (
    vehicleFieldTouched
    && preflight.vehicleNumber
    && !hasSupportedVehicleNumberFormat(preflight.vehicleNumber)
  ) {
    message = "예: 12가3456 형식으로 입력해 주세요. Mock에서는 숫자 4자리도 사용할 수 있습니다.";
  }

  input.classList.toggle("invalid", Boolean(message));
  input.setAttribute("aria-invalid", String(Boolean(message)));
  error.textContent = message;
  error.classList.toggle("hidden", !message);
}

function setRequestFormSubmitting(active) {
  requestSubmitting = active;
  const recoveryStatus = latestDashboard?.system?.recovery?.status || "NONE";
  const recoveryPending = [
    "SAFETY_STOPPED",
    "REQUIRED",
    "RECOVERING",
    "BLOCKED",
  ].includes(recoveryStatus);
  const controlsLocked = active
    || Boolean(latestDashboard?.system?.emergency_stop)
    || recoveryPending;
  document.getElementById("vehicleNumber").disabled = controlsLocked;
  document.querySelectorAll("[data-request-type]").forEach((button) => {
    button.disabled = controlsLocked;
  });
  document.querySelectorAll("[data-vehicle-number]").forEach((button) => {
    button.disabled = controlsLocked;
  });
  const guide = document.getElementById("mockVehicleGuide");
  if (guide) guide.toggleAttribute("inert", controlsLocked);
}

function setRequestSubmitReason(reason = "") {
  const element = document.getElementById("requestSubmitReason");
  const button = document.getElementById("requestSubmitButton");
  element.textContent = reason;
  element.classList.toggle("hidden", !reason);
  button.title = reason;
}

function updateRequestAvailability() {
  const requestType = document.getElementById("requestType").value;
  const role = requestType === "PARK_IN" ? "entry" : "exit";
  const roleLabel = requestType === "PARK_IN" ? "입차" : "출차";
  const alternateLabel = requestType === "PARK_IN" ? "출차" : "입차";
  const button = document.getElementById("requestSubmitButton");
  const hint = document.getElementById("requestSafetyHint");
  const preflight = evaluateRequestPreflight();
  const defaultButtonLabel = `${roleLabel} 요청 등록`;
  renderRequestPreflight(preflight);
  if (requestSubmitting) {
    button.disabled = true;
    button.textContent = `${roleLabel} 요청 등록 중…`;
    setRequestSubmitReason("요청 처리 결과를 기다리는 중");
    hint.textContent = "";
    hint.classList.add("hidden");
    return;
  }
  const safetyState = latestDashboard?.system?.safety?.state || "UNKNOWN";
  const systemStopped = ["STOPPED_LATCHED", "UNKNOWN"].includes(safetyState);
  const recoveryStatus = latestDashboard?.system?.recovery?.status || "NONE";
  const recoveryPending = [
    "SAFETY_STOPPED",
    "REQUIRED",
    "RECOVERING",
    "BLOCKED",
  ].includes(recoveryStatus);
  const obstacle = (latestDashboard?.alerts || []).find(
    (alert) => obstacleAffectsRole(alert, role)
  );
  const offlineSensors = (latestDashboard?.sensors || []).filter(
    (sensor) => sensor.status !== "ONLINE"
  );

  button.classList.toggle(
    "safety-blocked",
    systemStopped || recoveryPending || Boolean(obstacle)
  );
  button.disabled = systemStopped
    || recoveryPending
    || Boolean(obstacle)
    || preflight.submitBlocked;
  setRequestSubmitReason(
    preflight.submitBlocked ? preflight.status.replace(/[.]$/, "") : ""
  );
  hint.classList.remove("warning", "danger");

  if (systemStopped) {
    button.textContent = "비상정지 해제 후 등록 가능";
    hint.classList.add("danger");
    hint.textContent = "전체 비상정지 상태입니다. 안전 복구와 운영 승인을 완료한 뒤 새 요청을 등록해주세요.";
    hint.classList.remove("hidden");
    setRequestSubmitReason("비상정지 해제와 안전 복구 완료 필요");
    return;
  }

  if (safetyState === "READY_FOR_OPERATION") {
    const waitingForFinalApproval = ["NONE", "COMPLETED"].includes(
      recoveryStatus
    );
    button.textContent = waitingForFinalApproval
      ? "정상 운영 승인 후 등록 가능"
      : recoveryStatus === "RECOVERING"
        ? "로봇 도크 복귀 완료 후 등록 가능"
        : "로봇 안전 복귀 후 등록 가능";
    hint.classList.add("warning");
    hint.textContent = waitingForFinalApproval
      ? "로봇 도크 복귀 확인이 끝났습니다. 최종 정상 운영 승인을 완료해주세요."
      : recoveryStatus === "RECOVERING"
        ? "대상 로봇이 제한 운전으로 각 도크에 복귀 중입니다."
        : "현장 점검이 완료되었습니다. 대상 로봇의 제한 안전 복귀를 먼저 진행해주세요.";
    hint.classList.remove("hidden");
    setRequestSubmitReason(button.textContent);
    return;
  }

  if (recoveryPending) {
    button.textContent = recoveryStatus === "RECOVERING"
      ? "로봇 도크 복귀 완료 후 등록 가능"
      : "로봇 안전 복귀 후 등록 가능";
    hint.classList.add("warning");
    hint.textContent = recoveryStatus === "RECOVERING"
      ? "대상 로봇이 각 도크로 복귀 중입니다. 위치 확인 후 자동으로 대기 상태가 됩니다."
      : "중간 위치에 정지한 로봇이 복구 대기 상태입니다. 안전 복귀 절차를 완료해주세요.";
    hint.classList.remove("hidden");
    setRequestSubmitReason(button.textContent);
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
    setRequestSubmitReason(`${roleLabel} 통로 장애물 해소 필요`);
    return;
  }

  button.textContent = preflight.state === "blocked"
    ? `${roleLabel} 요청 불가`
    : defaultButtonLabel;

  if (offlineSensors.length && !preflight.submitBlocked) {
    hint.classList.add("warning");
    hint.textContent = (
      `센서 제한 운용 중입니다. 요청은 등록할 수 있지만 `
      + `${offlineSensors.map((sensor) => sensor.id).join("·")} 데이터 상태에 따라 작업 시작이 지연될 수 있습니다.`
    );
    hint.classList.remove("hidden");
  } else {
    hint.textContent = "";
    hint.classList.add("hidden");
  }
}

function queueRequestAvailability() {
  vehicleFieldTouched = true;
  compactLatestRequestResult();
  window.clearTimeout(requestValidationTimer);
  const vehicleNumber = normalizeVehicleNumber(
    document.getElementById("vehicleNumber").value
  );
  if (!vehicleNumber) {
    updateRequestAvailability();
    return;
  }

  const requestType = document.getElementById("requestType").value;
  const pending = evaluateRequestPreflight();
  renderRequestPreflight({
    ...pending,
    state: "pending",
    status: "차량 정보 조회 중...",
    message: "현재 주차 상태와 진행 중인 작업을 확인하고 있습니다.",
    submitBlocked: true,
    target: requestType === "PARK_OUT" ? "조회 중" : pending.target,
    vehicleState: requestType === "PARK_OUT" ? "조회 중" : null,
    eligibility: requestType === "PARK_OUT" ? "확인 중" : null,
  });
  const button = document.getElementById("requestSubmitButton");
  button.disabled = true;
  button.textContent = `${requestTypeLabels[requestType]} 요청 등록`;
  setRequestSubmitReason("차량 정보 조회 중");
  requestValidationTimer = window.setTimeout(
    updateRequestAvailability,
    220
  );
}

document.querySelectorAll("[data-request-type]").forEach((button) => {
  button.addEventListener("click", () => {
    setRequestType(button.dataset.requestType);
  });
});

document
  .getElementById("vehicleNumber")
  .addEventListener("input", queueRequestAvailability);
document
  .getElementById("vehicleNumber")
  .addEventListener("blur", () => {
    vehicleFieldTouched = true;
    updateRequestAvailability();
  });

document
  .getElementById("requestForm")
  .addEventListener("submit", async (event) => {
    event.preventDefault();
    if (requestSubmitting) return;

    const requestType = document.getElementById("requestType").value;
    const vehicleNumber = normalizeVehicleNumber(
      document.getElementById("vehicleNumber").value
    );

    vehicleFieldTouched = true;
    updateRequestAvailability();
    const submitButton = document.getElementById("requestSubmitButton");
    if (submitButton.disabled) {
      document.getElementById("vehicleNumber").focus();
      return;
    }

    setRequestFormSubmitting(true);
    updateRequestAvailability();
    try {
      const result = await apiRequest("/requests", {
        method: "POST",
        body: JSON.stringify({
          request_type: requestType,
          vehicle_number: vehicleNumber,
        }),
      });

      pendingFocusRequestId = result.id;
      selectedMapItem = { type: "task", id: String(result.id) };
      event.target.reset();
      vehicleFieldTouched = false;
      setRequestType("PARK_IN");
      await refreshDashboard();
      const refreshedRequest = (latestDashboard?.requests || []).find(
        (request) => String(request.id) === String(result.id)
      );
      showRequestResult(refreshedRequest || result);
    } catch (error) {
      showMessage(error.message, true);
    } finally {
      setRequestFormSubmitting(false);
      updateRequestAvailability();
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
    if (requestSubmitting) return;
    setRequestType("PARK_OUT");
    const input = document.getElementById("vehicleNumber");
    input.value = button.dataset.vehicleNumber;
    vehicleFieldTouched = true;
    compactLatestRequestResult();
    updateRequestAvailability();
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
  if (
    latestDashboard?.system?.safety?.state === "STOPPED_LATCHED"
  ) return;
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
      `${
        latestDashboard?.system?.recovery?.status === "RECOVERING"
          ? "제한 복귀 운전을 즉시 정지합니다."
          : "모든 로봇에 비상정지를 요청합니다."
      }\n\n`
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
  .getElementById("lidarDetailButton")
  .addEventListener("click", openLidarDetailDialog);
document
  .getElementById("emergencyStopButton")
  .addEventListener("click", activateEmergencyStop);
document
  .getElementById("restoreSensorAlertsButton")
  .addEventListener("click", restoreSensorAlerts);

document
  .getElementById("closeLidarDetailDialog")
  .addEventListener("click", closeLidarDetailDialog);
document
  .getElementById("lidarViewModeButton")
  .addEventListener("click", () => {
    lidarViewMode = lidarViewMode === "full" ? "slots" : "full";
    if (latestLidarVisualization) {
      renderLidarVisualization(latestLidarVisualization);
    }
  });
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
        "정상 운영 복귀를 승인할까요?\n"
        + "대상 로봇의 도크 도달 확인이 끝난 상태에서만 승인해야 합니다.\n"
        + "기존 취소 작업은 재개되지 않으며 새 작업만 접수합니다."
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
        const recoveryStatus =
          latestDashboard?.system?.recovery?.status || "NONE";
        if (
          dialog.open
          && latestDashboard?.system?.safety?.state === "NORMAL"
          && ["NONE", "COMPLETED"].includes(recoveryStatus)
        ) {
          dialog.close();
        }
      }, 1200);
    } catch (error) {
      setSafetyRecoveryMessage(error.message, true);
    } finally {
      button.disabled = false;
    }
  });

document
  .getElementById("robotRecoveryForm")
  .addEventListener("submit", async (event) => {
    event.preventDefault();
    const recoveryChecks = [
      ["checkRecoveryPath", "도크까지 복귀 경로 안전"],
      ["checkRecoveryLoad", "차량·적재물 분리 또는 별도 안전 확보"],
      ["checkRecoveryArms", "리프트 암 회수"],
      ["checkRecoverySensors", "복귀용 센서·통신"],
    ];
    const uncheckedChecks = recoveryChecks.filter(
      ([elementId]) => !document.getElementById(elementId).checked
    );
    if (uncheckedChecks.length) {
      setSafetyRecoveryMessage(
        `복귀 시작 전 모든 조건을 확인해주세요.\n${
          uncheckedChecks.map(([, label]) => `• ${label}`).join("\n")
        }`,
        true
      );
      document.getElementById(uncheckedChecks[0][0]).focus();
      return;
    }
    if (
      !window.confirm(
        "대상 로봇만 안전 도크로 복귀시킬까요?\n"
        + "취소된 차량 작업은 재개되지 않으며, 도크 도달 확인 후에만 대기로 전환됩니다."
      )
    )
      return;

    const button = event.submitter;
    button.disabled = true;
    try {
      const result = await apiRequest("/safety/start-recovery", {
        method: "POST",
        body: JSON.stringify({
          operator_id: document
            .getElementById("robotRecoveryOperator")
            .value.trim(),
          recovery_note: document
            .getElementById("robotRecoveryNote")
            .value.trim(),
          path_clear: document.getElementById("checkRecoveryPath").checked,
          load_cleared: document.getElementById("checkRecoveryLoad").checked,
          arms_retracted: document.getElementById("checkRecoveryArms").checked,
          sensors_ready: document.getElementById("checkRecoverySensors").checked,
        }),
      });
      setSafetyRecoveryMessage(result.message);
      await refreshDashboard();
    } catch (error) {
      setSafetyRecoveryMessage(error.message, true);
    } finally {
      button.disabled =
        latestDashboard?.system?.recovery?.status !== "REQUIRED";
    }
  });

window.advanceRequest = advanceRequest;
window.resolveAlert = resolveAlert;

document
  .getElementById("lidarDetailDialog")
  .addEventListener("close", () => {
    window.clearInterval(lidarDetailRefreshTimer);
    lidarDetailRefreshTimer = null;
  });
document
  .getElementById("lidarDetailDialog")
  .addEventListener("click", (event) => {
    if (event.target === event.currentTarget) closeLidarDetailDialog();
  });
window.addEventListener("resize", () => {
  const dialog = document.getElementById("lidarDetailDialog");
  if (dialog?.open && latestLidarVisualization) {
    renderLidarVisualization(latestLidarVisualization);
  }
});

setupWorkspaceTabs();
setupInspectorTabs();
updateRequestFlow();
const initialLidarParams = new URLSearchParams(window.location.search);
if (initialLidarParams.get("lidar") === "1") {
  window.requestAnimationFrame(openLidarDetailDialog);
}

async function runDashboardRefreshLoop() {
  await refreshDashboard();
  const hasActiveTask = (latestDashboard?.summary?.active_requests || 0) > 0;
  const recoveryActive = latestDashboard?.system?.recovery?.status === "RECOVERING";
  window.setTimeout(
    runDashboardRefreshLoop,
    hasActiveTask || recoveryActive ? 250 : 2000
  );
}

runDashboardRefreshLoop();

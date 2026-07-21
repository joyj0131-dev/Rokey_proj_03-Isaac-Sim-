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
  SYSTEM: "시스템",
};

const requestTypeLabels = {
  PARK_IN: "입고",
  PARK_OUT: "출차",
};

const workspaceTabs = ["live", "requests", "tasks"];
let latestDashboard = null;
let selectedMapItem = null;
let pendingFocusRequestId = null;
let showLidarMarkers = false;
let lastDashboardReceivedAt = null;
let messageHideTimer = null;
let recentWorkflowEvents = [];
const lastRequestStates = new Map();
const robotSpeechBubbles = new Map();
const seenSpeechAlertIds = new Set();

function shortRobotName(robotId) {
  const match = String(robotId).match(/(\d+)$/);
  return match ? `R${Number(match[1])}` : robotId;
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
  const names = ids.map(shortRobotName).join(" + ");
  return ids.length > 1 ? `협업: ${names}` : names;
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
    throw new Error(data.detail || "요청 처리 중 오류가 발생했습니다.");
  }

  return data;
}

function renderSummary(summary, robots, sensors, system, alerts = []) {
  const robotHealthy = robots.filter((robot) => !["ERROR", "OFFLINE"].includes(robot.status)).length;
  const lidarOnline = sensors.filter((sensor) => sensor.status === "ONLINE").length;
  const lidarHealthy = system.mode === "mock" ? sensors.length : lidarOnline;
  const totalSlots = summary.total_slots;
  const items = [
    {
      icon: "P",
      label: "빈 주차면",
      value: `${summary.empty_slots} / ${totalSlots}`,
      badge: "가용",
      tone: "success",
    },
    {
      icon: "🚗",
      label: "주차 차량",
      value: summary.occupied_slots,
      unit: "대",
      badge: "정상",
      tone: "occupied",
    },
    {
      icon: "▤",
      label: "진행 요청",
      value: summary.active_requests,
      unit: "건",
      badge: summary.active_requests > 0 ? "진행 중" : "대기",
      tone: "primary",
    },
    {
      icon: "🤖",
      label: "로봇 상태",
      value: `${robotHealthy} / ${robots.length}`,
      badge: robotHealthy === robots.length ? "정상" : "확인 필요",
      tone: robotHealthy === robots.length ? "success" : "warning",
    },
    {
      icon: "◉",
      label: "LiDAR 상태",
      value: `${lidarHealthy} / ${sensors.length}`,
      badge: system.mode === "mock"
        ? "MOCK"
        : lidarOnline === sensors.length ? "정상" : "연결 필요",
      tone: system.mode === "mock" ? "primary" : lidarOnline === sensors.length ? "success" : "warning",
    },
  ];

  const taskAttentionCount = summary.active_requests + alerts.length;
  const taskTabCount = document.getElementById("taskTabCount");
  taskTabCount.textContent = taskAttentionCount;
  taskTabCount.title = `진행 작업 ${summary.active_requests}건 · 미해제 경고 ${alerts.length}건`;
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
        </div>
      `
    )
    .join("")}
    </div>
  `;
}

// 실제 parking_map.yaml의 A/B 8면 배치를 한 화면에 표시하는 좌표계.
// HTML의 viewBox와 항상 같은 값을 유지한다.
const LOT_MAP_WIDTH = 1100;
const LOT_MAP_HEIGHT = 430;
const LOT_SLOT_WIDTH = 82;
const LOT_SLOT_HEIGHT = 126;
const LOT_DOCK_WIDTH = 92;
const LOT_DOCK_HEIGHT = 106;
const LOT_ROBOT_CARD_WIDTH = 84;
const LOT_ROBOT_CARD_HEIGHT = 84;

const dockRoleLabels = {
  waiting: "대기",
  charging: "충전",
};

function computeLotTransform(points) {
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const marginLeft = 96;
  const marginRight = 48;
  const marginY = 78;
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
  if (isPaused) return "장애물 감지";
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

  const docks = (mapInfo && mapInfo.docks) || [];
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
    ...placedSlots,
    ...docks,
    ...sensors.filter((sensor) => sensor.x != null && sensor.y != null),
  ];
  if (entrance) fixedPoints.push(entrance);
  const layoutPoints = fixedPoints.length ? fixedPoints : placedRobots;
  const { sx, sy } = computeLotTransform(layoutPoints);
  const parts = [];

  // 입구는 지도 시작점을 알리는 공간 표지로만 간결하게 표시한다.
  const aisleY = entrance ? entrance.y : docks[0] && docks[0].y;
  if (aisleY != null && entrance) {
    const xs = layoutPoints.map((p) => p.x);
    const laneY = sy(aisleY);
    const laneStart = sx(Math.min(...xs));
    const laneEnd = sx(Math.max(...xs));
    const b4 = placedSlots.find((slot) => slot.id === "B4");
    const b5 = placedSlots.find((slot) => slot.id === "B5");
    const zoneLabelX = b4 && b5
      ? (sx(b4.x) + sx(b5.x)) / 2
      : (laneStart + laneEnd) / 2;

    parts.push(`
      <text class="lot-driving-zone-label" x="${zoneLabelX}" y="${laneY + 5}">
        로봇 주행 구역
      </text>
      <g class="lot-access-tag exit" aria-label="주차장 출구">
        <rect x="${laneStart - 82}" y="${laneY - 34}" width="68" height="25" rx="8"></rect>
        <text x="${laneStart - 48}" y="${laneY - 17}">← 출구</text>
      </g>
      <g class="lot-access-tag entrance" aria-label="주차장 입구">
        <rect x="${laneStart - 82}" y="${laneY + 9}" width="68" height="25" rx="8"></rect>
        <text x="${laneStart - 48}" y="${laneY + 26}">→ 입구</text>
      </g>
    `);
  }

  for (const dock of docks) {
    const cx = sx(dock.x);
    const cy = sy(dock.y);
    parts.push(`
      <rect
        class="lot-dock-rect ${dock.role}"
        x="${cx - LOT_DOCK_WIDTH / 2}" y="${cy - LOT_DOCK_HEIGHT / 2}"
        width="${LOT_DOCK_WIDTH}" height="${LOT_DOCK_HEIGHT}"
        rx="8"
      ></rect>
      <text class="lot-dock-label" x="${cx}" y="${cy - LOT_DOCK_HEIGHT / 2 + 14}">
        ${dockRoleLabels[dock.role] || dock.role} 구역
      </text>
    `);
  }

  if (showLidarMarkers) {
    for (const sensor of sensors) {
      const cx = sx(sensor.x);
      // 센서의 실제 x 좌표는 유지하고, y=0인 주행 구역 중앙선에 표시한다.
      const cy = sy(sensor.y);
      const isSelected = selectedMapItem?.type === "sensor" && selectedMapItem.id === sensor.id;
      parts.push(`
        <g class="lot-selectable" role="button" tabindex="0"
          data-entity-type="sensor" data-entity-id="${sensor.id}" aria-label="LiDAR ${sensor.id} ${sensor.status}">
          <circle class="lot-sensor-ring ${sensor.status} ${isSelected ? "selected" : ""}"
            cx="${cx}" cy="${cy}" r="16"></circle>
          <circle class="lot-sensor-dot ${sensor.status}" cx="${cx}" cy="${cy}" r="5"></circle>
          <text class="lot-sensor-label" x="${cx}" y="${cy - 23}">
            ${sensor.id} · ${sensor.status === "ONLINE" ? `${sensor.rate_hz ?? "-"} Hz` : sensor.status === "MOCK" ? "테스트" : "수신 대기"}
          </text>
        </g>
      `);
    }
  }

  for (const slot of placedSlots) {
    const cx = sx(slot.x);
    const cy = sy(slot.y);
    const isSelected = selectedMapItem?.type === "slot" && selectedMapItem.id === slot.id;
    parts.push(`
      <g class="lot-selectable" role="button" tabindex="0"
        data-entity-type="slot" data-entity-id="${slot.id}" aria-label="${slot.id} ${statusLabels[slot.status]}">
      <rect
        class="lot-slot-rect ${slot.status} ${isSelected ? "selected" : ""}"
        x="${cx - LOT_SLOT_WIDTH / 2}" y="${cy - LOT_SLOT_HEIGHT / 2}"
        width="${LOT_SLOT_WIDTH}" height="${LOT_SLOT_HEIGHT}"
        rx="8"
      ></rect>
      <text class="lot-slot-label" x="${cx}" y="${cy - 5}">
        ${slot.id}${slot.is_accessible ? " ♿" : ""}
      </text>
      <text class="lot-slot-sub" x="${cx}" y="${cy + 14}">
        ${slot.vehicle_number ? slot.vehicle_number : statusLabels[slot.status]}
      </text>
      </g>
    `);
  }

  for (const robot of placedRobots) {
    const cx = sx(robot.x);
    const cy = sy(robot.y);
    const isSelected = selectedMapItem?.type === "robot" && selectedMapItem.id === robot.id;
    const currentRequest = robot.current_task_id == null
      ? null
      : requests.find((request) => request.id === robot.current_task_id);
    const pairedRobotIds = currentRequest ? assignedRobotIds(currentRequest) : [robot.id];
    const obstacleAlert = alerts.find(
      (alert) => alert.category === "OBSTACLE" &&
        (alert.robot_id == null || pairedRobotIds.includes(alert.robot_id))
    );
    const visualStatus = obstacleAlert ? "PAUSED" : robot.status;
    const statusText = obstacleAlert ? "일시 정지" : statusLabels[robot.status] || robot.status;
    const badgeWidth = visualStatus === "OFFLINE" ? 46 : statusText.length >= 4 ? 40 : 34;
    const badgeX = cx + LOT_ROBOT_CARD_WIDTH / 2 - badgeWidth - 7;
    const speech = robotSpeechBubbles.get(robot.id);
    if (speech && speech.expiresAt <= Date.now()) {
      robotSpeechBubbles.delete(robot.id);
    }
    const activeSpeech = robotSpeechBubbles.get(robot.id);
    const robotSubtitle = robotMapSubtitle(robot, requests, Boolean(obstacleAlert));
    const robotIconY = robotSubtitle ? cy + 2 : cy + 10;
    const speechWidth = activeSpeech
      ? Math.max(54, Math.min(96, activeSpeech.text.length * 9 + 20))
      : 0;
    parts.push(`
      <g class="lot-selectable lot-robot-marker ${visualStatus}" role="button" tabindex="0"
        data-entity-type="robot" data-entity-id="${robot.id}" aria-label="${shortRobotName(robot.id)} ${statusText}">
        ${activeSpeech ? `
          <g class="lot-robot-speech" aria-hidden="true">
            <rect x="${cx - speechWidth / 2}" y="${cy - 69}"
              width="${speechWidth}" height="23" rx="11"></rect>
            <path d="M ${cx - 5} ${cy - 46} L ${cx} ${cy - 39} L ${cx + 5} ${cy - 46} Z"></path>
            <text x="${cx}" y="${cy - 53}">${activeSpeech.text}</text>
          </g>
        ` : ""}
        <rect
          class="lot-robot-card ${visualStatus} ${isSelected ? "selected" : ""}"
          x="${cx - LOT_ROBOT_CARD_WIDTH / 2}" y="${cy - LOT_ROBOT_CARD_HEIGHT / 2}"
          width="${LOT_ROBOT_CARD_WIDTH}" height="${LOT_ROBOT_CARD_HEIGHT}" rx="12"
        ></rect>
        <text class="lot-robot-name" x="${cx - LOT_ROBOT_CARD_WIDTH / 2 + 9}" y="${cy - 26}">
          ${shortRobotName(robot.id)}
        </text>
        <rect class="lot-robot-badge ${visualStatus}"
          x="${badgeX}" y="${cy - 36}" width="${badgeWidth}" height="17" rx="8.5"></rect>
        <text class="lot-robot-badge-label ${visualStatus}"
          x="${badgeX + badgeWidth / 2}" y="${cy - 24}">${statusText}</text>
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

function selectMapItem(type, id) {
  selectedMapItem = { type, id };
  if (!latestDashboard) return;
  renderLotMap(
    latestDashboard.slots,
    latestDashboard.robots,
    latestDashboard.map,
    latestDashboard.sensors,
    latestDashboard.requests,
    latestDashboard.alerts
  );
  renderSelectionDetail(latestDashboard);
}

function toggleLidarMarkers() {
  showLidarMarkers = !showLidarMarkers;
  const button = document.getElementById("lidarVisibilityButton");
  button.classList.toggle("active", showLidarMarkers);
  button.setAttribute("aria-pressed", String(showLidarMarkers));
  button.textContent = showLidarMarkers ? "LiDAR 위치 끄기" : "LiDAR 위치 켜기";

  if (!showLidarMarkers && selectedMapItem?.type === "sensor") {
    selectedMapItem = null;
  }
  if (!latestDashboard) return;
  renderLotMap(
    latestDashboard.slots,
    latestDashboard.robots,
    latestDashboard.map,
    latestDashboard.sensors || [],
    latestDashboard.requests || [],
    latestDashboard.alerts || []
  );
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
    return nearbyDock.dock.role === "charging" ? "충전 구역" : "대기 구역";
  }

  if (pointDistance(robot, mapInfo?.entrance) <= 1.8) return "입차 구역";

  const aisleY = mapInfo?.entrance?.y;
  if (aisleY != null && Math.abs(robot.y - aisleY) <= 1.6) return "중앙 통로";

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

function renderSelectionDetail(dashboard) {
  const detail = document.getElementById("selectionDetail");
  const slots = dashboard?.slots || [];
  const robots = dashboard?.robots || [];
  const sensors = dashboard?.sensors || [];
  if (!selectedMapItem) {
    detail.innerHTML = `
      <span class="detail-kicker">선택 정보</span>
      <h3>도면에서 항목을 선택하세요</h3>
      <p>주차면이나 로봇을 누르면 상태를 자세히 확인할 수 있습니다.</p>
    `;
    return;
  }

  if (selectedMapItem.type === "slot") {
    const slot = slots.find((item) => item.id === selectedMapItem.id);
    if (!slot) return;
    const currentRequest = (dashboard.requests || []).find(
      (request) => request.slot_id === slot.id && !["COMPLETED", "CANCELLED"].includes(request.status)
    );
    const assignedIds = currentRequest ? assignedRobotIds(currentRequest) : [];
    detail.innerHTML = `
      <span class="detail-kicker">주차면 상세</span>
      <div class="detail-title-row">
        <h3>${slot.id}${slot.is_accessible ? " ♿" : ""}</h3>
        <span class="badge ${slot.status}">${statusLabels[slot.status]}</span>
      </div>
      <dl class="detail-list">
        <div><dt>상태</dt><dd>${statusLabels[slot.status]}</dd></div>
        ${slot.vehicle_number ? `
          <div><dt>차량 번호</dt><dd>${slot.vehicle_number}</dd></div>
        ` : ""}
        ${currentRequest ? `
          <div><dt>현재 작업</dt><dd>${requestTypeLabels[currentRequest.request_type]} 요청 #${currentRequest.id} · ${requestStatusLabel(currentRequest)}</dd></div>
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
        <span class="sensor-state ${sensor.status}">${sensorLabel}</span>
      </div>
      <dl class="detail-list">
        <div><dt>ROS2 토픽</dt><dd class="topic-value">${sensor.topic}</dd></div>
        <div><dt>수신 주기</dt><dd>${sensor.rate_hz == null ? "-" : `${sensor.rate_hz} Hz`}</dd></div>
        <div><dt>마지막 수신</dt><dd>${sensor.last_seen_sec == null ? "수신 기록 없음" : `${sensor.last_seen_sec}초 전`}</dd></div>
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
    (alert) => alert.robot_id === robot.id ||
      (!alert.robot_id && alert.category === "OBSTACLE") ||
      (
        alert.category === "OBSTACLE" &&
        currentRequest &&
        assignedRobotIds(currentRequest).includes(alert.robot_id)
      )
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
  const updateLabel = lastDashboardReceivedAt
    ? lastDashboardReceivedAt.toLocaleTimeString("ko-KR", {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      })
    : "-";
  detail.innerHTML = `
    <span class="detail-kicker">로봇 상세</span>
    <div class="detail-title-row">
      <h3>🤖 ${shortRobotName(robot.id)}</h3>
      <span class="badge ${isObstaclePaused ? "PAUSED" : robot.status}">${isObstaclePaused ? "장애물 정지" : robotOperationLabel(robot, currentRequest)}</span>
    </div>
    <dl class="detail-list robot-detail-list">
      <div><dt>시스템 ID</dt><dd>${robot.id}</dd></div>
      <div><dt>현재 위치</dt><dd>${describeRobotLocation(robot, dashboard.map, slots)}</dd></div>
      <div><dt>현재 작업</dt><dd>${taskLabel}</dd></div>
      <div><dt>목표 주차면</dt><dd>${targetLabel}</dd></div>
      <div><dt>할당 요청</dt><dd>${requestLabel}</dd></div>
      <div><dt>통신 상태</dt><dd>${communicationLabel}</dd></div>
      <div><dt>안전 상태</dt><dd class="${robotAlert || robot.error_message ? "detail-warning" : ""}">${safetyLabel}</dd></div>
      <div><dt>화면 갱신</dt><dd>${updateLabel}</dd></div>
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
  return {
    time: new Date().toISOString(),
    tone: request.status === "CANCELLED" ? "danger" : "primary",
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
        ROBOT_ASSIGNED: "출발!",
        RETURNING: "복귀합니다",
        COMPLETED: "도착!",
      }
    : {
        ROBOT_ASSIGNED: "이동 시작",
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
    const targetIds = alert.robot_id
      ? [alert.robot_id]
      : robots.filter((robot) => robot.status === "BUSY").map((robot) => robot.id);
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
    robotSpeechBubbles.clear();
    return;
  }

  for (const request of [...requests].reverse()) {
    const previousStatus = lastRequestStates.get(request.id);
    if (previousStatus == null) {
      recentWorkflowEvents.push({
        time: request.created_at,
        tone: "primary",
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

function renderRecentEvents(alerts) {
  const events = [
    ...alerts.map((alert) => ({
      time: alert.created_at,
      tone: alert.level === "ERROR" ? "danger" : "warning",
      label: alertLabels[alert.category] || "시스템 이벤트",
      description: alert.message,
    })),
    ...recentWorkflowEvents,
  ]
    .sort((a, b) => new Date(b.time) - new Date(a.time))
    .slice(0, 12);

  const container = document.getElementById("recentEventList");
  if (!events.length) {
    container.innerHTML = `<p class="recent-events-empty">최근 발생한 작업이나 경고가 없습니다.</p>`;
    return;
  }

  container.innerHTML = events.map((event) => `
    <article class="recent-event-item ${event.tone}">
      <i></i>
      <div><strong>${event.label}</strong><span>${event.description}</span></div>
      <time>${formatDateTime(event.time)}</time>
    </article>
  `).join("");
}

function renderRequests(requests, system) {
  const container = document.getElementById("requestTable");
  const showManualAdvance = (!system || system.mock_controls) && !system?.mock_auto_advance;

  if (!requests.length) {
    container.innerHTML = `
      <div class="empty-state">
        <div class="empty-state-icon">T</div>
        <strong>등록된 작업 요청이 없습니다.</strong>
        <span>
          입고 또는 출차 요청을 등록하면<br />
          작업 진행 상태가 이곳에 표시됩니다.
        </span>
      </div>
    `;
    return;
  }

  container.innerHTML = `
    <div class="table-wrap">
      <table>
        <thead>
          <tr>
            <th>ID</th>
            <th>유형</th>
            <th>차량 번호</th>
            <th>주차면</th>
            <th>할당 로봇</th>
            <th>진행 상태</th>
            <th>등록 시간</th>
            ${showManualAdvance ? "<th>제어</th>" : ""}
          </tr>
        </thead>
        <tbody>
          ${requests
            .map(
              (request) => `
                <tr>
                  <td>#${request.id}</td>
                  <td>${requestTypeLabels[request.request_type]}</td>
                  <td>${request.vehicle_number}</td>
                  <td>${request.slot_id || "-"}</td>
                  <td>${assignedRobotTableLabel(request)}</td>
                  <td>
                    <span class="badge ${request.status}">
                      ${requestStatusLabel(request)}
                    </span>
                  </td>
                  <td>${formatDateTime(request.created_at)}</td>
                  ${showManualAdvance ? `<td>
                    ${
                      request.status !== "COMPLETED" &&
                      request.status !== "CANCELLED"
                        ? `
                          <button
                            class="advance-button"
                            onclick="advanceRequest(${request.id})"
                          >
                            다음 단계
                          </button>
                        `
                        : "-"
                    }
                  </td>` : ""}
                </tr>
              `
            )
            .join("")}
        </tbody>
      </table>
    </div>
  `;
}

function renderAlerts(alerts) {
  const panel = document.getElementById("alertPanel");
  const list = document.getElementById("alertList");

  if (!alerts.length) {
    panel.classList.add("hidden");
    list.innerHTML = "";
    return;
  }

  panel.classList.remove("hidden");
  list.innerHTML = alerts
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
            <span class="alert-time">${alert.created_at.replace("T", " ")}</span>
          </div>
          <button
            class="secondary-button small"
            onclick="resolveAlert(${alert.id})"
          >
            해제
          </button>
        </div>
      `
    )
    .join("");
}

function renderSystem(system) {
  if (!system) return;

  const modeBadge = document.getElementById("modeBadge");
  modeBadge.textContent = system.mode === "mock" ? "Mock Mode" : "ROS2 Mode";

  const statusBox = document.getElementById("systemStatus");
  const statusText = document.getElementById("systemStatusText");

  statusBox.classList.remove("warn", "danger");

  if (system.health === "ERROR") {
    statusBox.classList.add("danger");
    statusText.textContent = "시스템 오류";
  } else if (system.health === "WARNING") {
    statusBox.classList.add("warn");
    statusText.textContent = "주의 필요";
  } else {
    statusText.textContent = "시스템 정상";
  }

  document
    .getElementById("resetButton")
    .classList.toggle("hidden", !system.mock_controls);
  document
    .getElementById("mockVehicleGuide")
    .classList.toggle("hidden", !system.mock_controls);
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
  status.classList.remove("pending", "offline");
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

  const now = new Date().toLocaleTimeString("ko-KR", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
  status.querySelector("span").textContent = `LIVE · ROS2 · ${now}`;
}

async function refreshDashboard() {
  try {
    const data = await apiRequest("/dashboard");
    latestDashboard = data;
    lastDashboardReceivedAt = new Date();

    if (pendingFocusRequestId != null) {
      const focusedRequest = data.requests.find(
        (request) => request.id === pendingFocusRequestId
      );
      if (focusedRequest?.slot_id) {
        selectedMapItem = { type: "slot", id: focusedRequest.slot_id };
        pendingFocusRequestId = null;
      } else if (focusedRequest?.robot_id) {
        selectedMapItem = { type: "robot", id: focusedRequest.robot_id };
      }
    }

    // 첫 화면에서도 상세 패널이 비어 보이지 않도록 주차된 주차면을
    // 우선 선택하고, 없으면 첫 번째 주차면을 기본값으로 사용한다.
    if (!selectedMapItem) {
      const defaultSlot = data.slots.find((slot) => slot.status === "OCCUPIED") || data.slots[0];
      if (defaultSlot) selectedMapItem = { type: "slot", id: defaultSlot.id };
    }

    renderSummary(
      data.summary,
      data.robots,
      data.sensors || [],
      data.system,
      data.alerts || []
    );
    captureRequestEvents(data.requests, data.system);
    captureAlertSpeech(data.alerts || [], data.robots, data.system);
    renderLotMap(
      data.slots,
      data.robots,
      data.map,
      data.sensors || [],
      data.requests,
      data.alerts || []
    );
    renderSelectionDetail(data);
    renderRequests(data.requests, data.system);
    renderAlerts(data.alerts || []);
    renderRecentEvents(data.alerts || []);
    renderSystem(data.system);
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
      if (result.slot_id) {
        selectedMapItem = { type: "slot", id: result.slot_id };
        pendingFocusRequestId = null;
      }
      event.target.reset();
      await refreshDashboard();
      activateWorkspaceTab("live");
    } catch (error) {
      showMessage(error.message, true);
    }
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

function downloadDashboardBackup() {
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
  .addEventListener("click", downloadDashboardBackup);

document.querySelectorAll("[data-vehicle-number]").forEach((button) => {
  button.addEventListener("click", () => {
    document.getElementById("requestType").value = "PARK_OUT";
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

document
  .getElementById("lidarVisibilityButton")
  .addEventListener("click", toggleLidarMarkers);

window.advanceRequest = advanceRequest;
window.resolveAlert = resolveAlert;

setupWorkspaceTabs();

async function runDashboardRefreshLoop() {
  await refreshDashboard();
  const hasActiveTask = (latestDashboard?.summary?.active_requests || 0) > 0;
  window.setTimeout(runDashboardRefreshLoop, hasActiveTask ? 500 : 2000);
}

runDashboardRefreshLoop();

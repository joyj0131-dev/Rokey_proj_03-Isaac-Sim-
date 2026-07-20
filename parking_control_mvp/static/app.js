const API_BASE = "/api";

const statusLabels = {
  IDLE: "대기",
  BUSY: "작업 중",
  CHARGING: "충전 중",
  ERROR: "오류",
  EMPTY: "빈 공간",
  RESERVED: "예약",
  OCCUPIED: "주차 중",
  WAITING: "요청 대기",
  ROBOT_ASSIGNED: "로봇 할당",
  APPROACHING: "차량 접근",
  LIFTING: "차량 리프트",
  MOVING_TO_SLOT: "주차 위치 이동",
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

async function apiRequest(path, options = {}) {
  const response = await fetch(`${API_BASE}${path}`, {
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
    ...options,
  });

  const data = await response.json();

  if (!response.ok) {
    throw new Error(data.detail || "요청 처리 중 오류가 발생했습니다.");
  }

  return data;
}

function renderSummary(summary) {
  const cards = [
    {
      label: "빈 주차면",
      value: summary.empty_slots,
      icon: "E",
      accent: "#d1fae5",
      iconBg: "#ecfdf5",
      iconColor: "#059669",
    },
    {
      label: "사용 중 주차면",
      value: summary.occupied_slots,
      icon: "P",
      accent: "#fee2e2",
      iconBg: "#fef2f2",
      iconColor: "#dc2626",
    },
    {
      label: "진행 중 요청",
      value: summary.active_requests,
      icon: "T",
      accent: "#dbeafe",
      iconBg: "#eff6ff",
      iconColor: "#2563eb",
    },
  ];

  document.getElementById("summaryCards").innerHTML = cards
    .map(
      (card) => `
        <article
          class="summary-card"
          style="
            --card-accent: ${card.accent};
            --icon-bg: ${card.iconBg};
            --icon-color: ${card.iconColor};
          "
        >
          <div class="summary-content">
            <div class="summary-top">
              <span>${card.label}</span>
              <div class="summary-icon">${card.icon}</div>
            </div>
            <strong>${card.value}</strong>
          </div>
        </article>
      `
    )
    .join("");
}

function renderRobots(robots) {
  document.getElementById("robotList").innerHTML = robots
    .map(
      (robot) => `
        <div class="robot-card">
          <div class="robot-card-top">
            <div class="robot-title-group">
              <div class="robot-avatar">BOT</div>
              <span class="robot-name">${robot.id}</span>
            </div>

            <span class="badge ${robot.status}">
              ${statusLabels[robot.status]}
            </span>
          </div>

          <div class="battery-label">
            <span>배터리</span>
            <strong>${robot.battery}%</strong>
          </div>

          <div class="battery-track">
            <div class="battery-fill" style="width: ${robot.battery}%"></div>
          </div>

          <div class="robot-meta">
            <span>현재 작업</span>
            <span>
              ${robot.current_task_id ? `#${robot.current_task_id}` : "할당 없음"}
            </span>
          </div>

          ${
            robot.error_message
              ? `<div class="robot-error">${robot.error_message}</div>`
              : ""
          }
        </div>
      `
    )
    .join("");
}

const LOT_MAP_WIDTH = 780;
const LOT_MAP_HEIGHT = 460;
const LOT_SLOT_SIZE = 56;
const LOT_DOCK_SIZE = 50;
const LOT_ROBOT_RADIUS = 11;

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

  const padX = Math.max((maxX - minX) * 0.2, LOT_SLOT_SIZE / 20);
  const padY = Math.max((maxY - minY) * 0.2, LOT_SLOT_SIZE / 20);
  const x0 = minX - padX;
  const x1 = maxX + padX;
  const y0 = minY - padY;
  const y1 = maxY + padY;
  const spanX = x1 - x0 || 1;
  const spanY = y1 - y0 || 1;

  return {
    sx: (x) => ((x - x0) / spanX) * LOT_MAP_WIDTH,
    // y는 위로 갈수록 커지도록 뒤집는다 (화면 좌표는 아래로 갈수록 커짐).
    sy: (y) => LOT_MAP_HEIGHT - ((y - y0) / spanY) * LOT_MAP_HEIGHT,
  };
}

function renderLotMap(slots, robots, mapInfo) {
  const svg = document.getElementById("lotMap");
  const emptyMessage = document.getElementById("lotMapEmpty");

  const docks = (mapInfo && mapInfo.docks) || [];
  const entrance = mapInfo && mapInfo.entrance;

  const placedSlots = slots.filter((s) => s.x != null && s.y != null);
  const placedRobots = robots.filter((r) => r.x != null && r.y != null);

  if (!placedSlots.length && !placedRobots.length) {
    svg.innerHTML = "";
    emptyMessage.classList.remove("hidden");
    return;
  }
  emptyMessage.classList.add("hidden");

  const allPoints = [...placedSlots, ...placedRobots, ...docks];
  if (entrance) allPoints.push(entrance);
  const { sx, sy } = computeLotTransform(allPoints);
  const parts = [];

  // 통로: 입구와 도크가 지나는 y(보통 0)를 기준으로 가로선을 하나 그린다.
  const aisleY = entrance ? entrance.y : docks[0] && docks[0].y;
  if (aisleY != null) {
    const xs = allPoints.map((p) => p.x);
    const laneY = sy(aisleY);
    parts.push(`
      <line
        class="lot-aisle-line"
        x1="${sx(Math.min(...xs))}" y1="${laneY}"
        x2="${sx(Math.max(...xs))}" y2="${laneY}"
      ></line>
    `);
  }

  if (entrance) {
    const cx = sx(entrance.x);
    const cy = sy(entrance.y);
    parts.push(`
      <text class="lot-entrance-label" x="${cx}" y="${cy - 10}">입구 ▶</text>
    `);
  }

  for (const dock of docks) {
    const cx = sx(dock.x);
    const cy = sy(dock.y);
    const half = LOT_DOCK_SIZE / 2;
    parts.push(`
      <rect
        class="lot-dock-rect ${dock.role}"
        x="${cx - half}" y="${cy - half}"
        width="${LOT_DOCK_SIZE}" height="${LOT_DOCK_SIZE}"
        rx="8"
      ></rect>
      <text class="lot-dock-label" x="${cx}" y="${cy + 4}">
        ${dockRoleLabels[dock.role] || dock.role}
      </text>
    `);
  }

  for (const slot of placedSlots) {
    const cx = sx(slot.x);
    const cy = sy(slot.y);
    const half = LOT_SLOT_SIZE / 2;
    parts.push(`
      <rect
        class="lot-slot-rect ${slot.status}"
        x="${cx - half}" y="${cy - half}"
        width="${LOT_SLOT_SIZE}" height="${LOT_SLOT_SIZE}"
        rx="8"
      ></rect>
      <text class="lot-slot-label" x="${cx}" y="${cy - 2}">
        ${slot.id}${slot.is_accessible ? " ♿" : ""}
      </text>
      <text class="lot-slot-sub" x="${cx}" y="${cy + 14}">
        ${slot.vehicle_number ? slot.vehicle_number : statusLabels[slot.status]}
      </text>
    `);
  }

  for (const robot of placedRobots) {
    const cx = sx(robot.x);
    const cy = sy(robot.y);
    parts.push(`
      <circle
        class="lot-robot-dot ${robot.status}"
        cx="${cx}" cy="${cy}" r="${LOT_ROBOT_RADIUS}"
      ></circle>
      <text class="lot-robot-label" x="${cx}" y="${cy - LOT_ROBOT_RADIUS - 6}">
        ${robot.id} (${statusLabels[robot.status]})
      </text>
    `);
  }

  svg.innerHTML = parts.join("");
}

function renderSlots(slots) {
  document.getElementById("slotGrid").innerHTML = slots
    .map(
      (slot) => `
        <article class="slot-card ${slot.status}">
          <h3>${slot.id}</h3>
          <span class="badge ${slot.status}">
            ${statusLabels[slot.status]}
          </span>
          <p>${slot.vehicle_number || "차량 없음"}</p>
        </article>
      `
    )
    .join("");
}

function renderRequests(requests, system) {
  const container = document.getElementById("requestTable");
  const showMockControls = !system || system.mock_controls;

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
            <th>Mock 제어</th>
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
                  <td>${request.robot_id || "대기"}</td>
                  <td>
                    <span class="badge ${request.status}">
                      ${statusLabels[request.status]}
                    </span>
                  </td>
                  <td>
                    ${
                      showMockControls &&
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
                  </td>
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
    .getElementById("mockControls")
    .classList.toggle("hidden", !system.mock_controls);
}

function showMessage(message, isError = false) {
  const messageBox = document.getElementById("messageBox");

  messageBox.textContent = message;
  messageBox.classList.remove("hidden", "error");

  if (isError) {
    messageBox.classList.add("error");
  }
}

async function refreshDashboard() {
  try {
    const data = await apiRequest("/dashboard");

    renderSummary(data.summary);
    renderRobots(data.robots);
    renderSlots(data.slots);
    renderLotMap(data.slots, data.robots, data.map);
    renderRequests(data.requests, data.system);
    renderAlerts(data.alerts || []);
    renderSystem(data.system);
  } catch (error) {
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

    try {
      const result = await apiRequest("/requests", {
        method: "POST",
        body: JSON.stringify({
          request_type: requestType,
          vehicle_number: vehicleNumber,
        }),
      });

      showMessage(
        `${requestTypeLabels[result.request_type]} 요청 #${result.id}이 등록되었습니다.`
      );

      event.target.reset();
      await refreshDashboard();
    } catch (error) {
      showMessage(error.message, true);
    }
  });

document
  .getElementById("resetButton")
  .addEventListener("click", async () => {
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
  .getElementById("obstacleButton")
  .addEventListener("click", async () => {
    try {
      await apiRequest("/mock/obstacle", { method: "POST" });
      await refreshDashboard();
    } catch (error) {
      showMessage(error.message, true);
    }
  });

document
  .getElementById("errorButton")
  .addEventListener("click", async () => {
    try {
      await apiRequest("/mock/robot-error", { method: "POST" });
      await refreshDashboard();
    } catch (error) {
      showMessage(error.message, true);
    }
  });

window.advanceRequest = advanceRequest;
window.resolveAlert = resolveAlert;

refreshDashboard();
setInterval(refreshDashboard, 3000);

"use strict";

const qs = new URLSearchParams(location.search);
const queryToken = qs.get("token");
if (queryToken) localStorage.setItem("t170_token", queryToken);
const token = queryToken || localStorage.getItem("t170_token") || "";
if (queryToken && history.replaceState) history.replaceState(null, "", location.pathname);

const state = {
  online: false, busy: false, commandRunning: false, snapshot: {},
  pendingAction: null, logSeq: 0, startupShown: false, autoTimer: null, host: {},
};
const $ = (id) => document.getElementById(id);
const labels6 = ["X", "Y", "Z", "RX", "RY", "RZ"];

async function api(path, options = {}) {
  const headers = { "X-T170-Token": token, ...(options.headers || {}) };
  if (options.body && typeof options.body !== "string") {
    headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(options.body);
  }
  const response = await fetch(path, { ...options, headers, cache: "no-store" });
  let result;
  try { result = await response.json(); } catch { result = { ok: false, error: `HTTP ${response.status}` }; }
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

function toast(message, error = false) {
  const el = $("toast"); el.textContent = message; el.className = `toast show${error ? " error" : ""}`;
  clearTimeout(el.timer); el.timer = setTimeout(() => el.className = "toast", 3200);
}

function fmt(value, digits = 3) {
  const n = Number(value); return Number.isFinite(n) ? n.toFixed(digits) : "--";
}

function renderNumbers(id, values, labels) {
  const list = Array.isArray(values) ? values : [];
  $(id).innerHTML = labels.map((label, i) => `<div class="number-cell"><span>${label}</span><strong>${fmt(list[i], i < 3 && labels.length === 6 ? 4 : 2)}</strong></div>`).join("");
}

function setChip(el, mode, text) { el.className = `chip ${mode}`; el.innerHTML = `<i></i>${text}`; }

function renderStatus(data) {
  if (!data || !data.ok) return;
  state.snapshot = data.fresh_hardware ? data : { ...state.snapshot, ...data };
  state.busy = !!data.busy;
  setChip($("actionChip"), data.busy ? "busy" : "online", data.busy ? (data.action || "执行中") : "空闲");
  if (!data.fresh_hardware) return;

  renderNumbers("rightTcp", data.right_tcp_m_deg, labels6);
  renderNumbers("leftTcp", data.left_tcp_m_deg, labels6);
  renderNumbers("rightJoints", data.right_joints_deg, ["J1","J2","J3","J4","J5","J6","J7"]);
  renderNumbers("leftJoints", data.left_joints_deg, ["J1","J2","J3","J4","J5","J6","J7"]);
  const can = Array.isArray(data.can_channels) ? data.can_channels : [];
  const canCount = can.filter(Boolean).length;
  $("metricCan").textContent = `${canCount}/${can.length || 6}`;
  $("metricCanDetail").textContent = data.can_fault ? (data.can_fault_message || "CAN 故障") : "通信未报告故障";
  $("metricCan").style.color = data.can_fault || canCount < 6 ? "var(--red)" : "var(--green)";
  $("metricVision").textContent = data.pipeline_ready ? "就绪" : "未就绪";
  $("metricVision").style.color = data.pipeline_ready ? "var(--green)" : "var(--red)";
  $("metricGripper").textContent = data.gripper_running ? "运行中" : "异常";
  $("metricGripperDetail").textContent = data.gripper_connected ? "串口已连接" : "串口未连接";
  renderGripper("right", data.grippers?.right);
  renderGripper("left", data.grippers?.left);
  const hostCan = state.host.can_interfaces || {};
  const checks = [
    ["视觉管线", data.pipeline_ready], ["腰部位置同步", data.waist_synced],
    ["夹爪控制线程", data.gripper_running], ["CAN 无故障", !data.can_fault],
    ["头部动态外参（未接入）", false],
    ...can.map((ok, i) => [`can${i}`, ok]),
    ["夹爪串口×2", (state.host.serial_ports || []).length >= 2],
    ["YOLO 模型", (state.host.models || []).length > 0],
    ["主机磁盘", Number(state.host.disk_free_gb) > 2],
    ...Object.entries(hostCan).map(([name, value]) => [`${name} ${value}`, value === "up" || value === "unknown"]),
  ];
  $("deviceChecks").innerHTML = checks.map(([name, ok]) => `<div class="check ${ok ? "ok" : ""}"><b></b>${name}<span style="margin-left:auto">${ok ? "OK" : "检查"}</span></div>`).join("");
  $("lastUpdate").textContent = `更新于 ${new Date().toLocaleTimeString()}`;
}

function renderGripper(side, data = {}) {
  const title = side === "right" ? "right" : "left";
  const chip = $(`${title}GripState`);
  setChip(chip, data.have_feedback ? "online" : "offline", data.have_feedback ? "反馈正常" : "无反馈");
  $(`${title}GripMetrics`).innerHTML = [
    ["角度", `${fmt(data.angle_deg,1)}°`], ["力矩", `${fmt(data.torque_nm,2)} N·m`], ["速度", `${fmt(data.velocity_rad_s,2)} rad/s`]
  ].map(([k,v]) => `<div class="grip-metric"><span>${k}</span><strong>${v}</strong></div>`).join("");
}

async function pollHealth() {
  try {
    const health = await api("/api/health"); state.online = !!health.robot_online; state.host = health.host || {};
    setChip($("serviceChip"), state.online ? "online" : "offline", state.online ? "服务在线" : "服务离线");
    $("metricService").textContent = state.online ? "在线" : "离线";
    $("metricService").style.color = state.online ? "var(--green)" : "var(--red)";
  } catch (e) { state.online = false; setChip($("serviceChip"), "offline", e.message.includes("令牌") ? "令牌无效" : "Web 异常"); }
}

async function pollStatus(force = false) {
  if (!state.online || (state.commandRunning && !force)) return;
  try { renderStatus(await api("/api/status")); } catch (e) { if (force) toast(e.message, true); }
}

async function execute(payload, dangerous = false) {
  if (state.commandRunning && payload.cmd !== "abort") return;
  state.commandRunning = payload.cmd !== "abort";
  document.body.classList.toggle("working", state.commandRunning);
  try {
    if (dangerous) payload.confirm = payload.cmd;
    const result = await api("/api/command", { method: "POST", body: payload });
    $("motionResult").textContent = JSON.stringify(result, null, 2);
    $("motionResultState").textContent = result.ok ? "完成" : "失败";
    if (!result.ok) throw new Error(result.error || "命令失败");
    toast(`${payload.cmd} 已完成`);
    if (result.targets) renderDetections(result.targets, "yolo");
    if (result.markers) renderDetections(result.markers, "aruco");
    await refreshImages();
    await pollStatus(true);
    return result;
  } catch (e) { toast(e.message, true); throw e; }
  finally { state.commandRunning = false; document.body.classList.remove("working"); }
}

function askDangerous(payload, title, text) {
  state.pendingAction = payload; $("confirmTitle").textContent = title; $("confirmText").textContent = text;
  $("estopReady").checked = false; $("acceptConfirm").disabled = true;
  $("confirmModal").classList.add("open"); $("confirmModal").setAttribute("aria-hidden", "false");
}

function renderDetections(items = [], type = "yolo") {
  const el = $("detectionResults"); $("detectionSummary").textContent = `${items.length} 个目标`;
  if (!items.length) { el.className = "result-list empty-state"; el.textContent = "未检测到目标"; return; }
  el.className = "result-list";
  el.innerHTML = items.map((item, i) => {
    if (type === "aruco") return `<div class="result-row"><span>#${i+1}</span><span class="tag">ArUco ID ${item.id}</span><span class="confidence">${fmt(item.reproj_px,2)} px</span><span>相机 ${xyz(item.camera_m)}</span><span>基座 ${xyz(item.robot_m_deg)}</span></div>`;
    const selected = item.selected_for ? ` · ${item.selected_for === "right" ? "右手目标" : item.selected_for === "left" ? "左手目标" : "双手目标"}` : "";
    const column = item.tray_column ? `列${item.tray_column}` : "未分列";
    return `<div class="result-row"><span>#${i+1}</span><span class="tag">${escapeHtml(item.class_name || `class ${item.class_id}`)} · ${column}${selected}</span><span class="confidence">${fmt((item.confidence || 0)*100,1)}%</span><span>相机 ${xyz(item.camera_m)}</span><span>基座 ${item.robot_ok ? xyz(item.robot_m_deg) : "--"}</span></div>`;
  }).join("");
}

function xyz(p) { return p ? `(${fmt(p.x,4)}, ${fmt(p.y,4)}, ${fmt(p.z,4)})` : "--"; }
function escapeHtml(s) { return String(s).replace(/[&<>'"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","'":"&#39;",'"':"&quot;"}[c])); }

async function refreshImages() {
  const slot = $("cameraSelect").value;
  try {
    const meta = await api("/api/images");
    for (const kind of ["processed", "original"]) {
      const info = meta.images?.[slot]?.[kind]; const img = $(`${kind}Image`); const stamp = $(`${kind}Stamp`);
      if (info) {
        if (img.dataset.mtime !== String(info.mtime_ms)) {
          const response = await fetch(`/api/image/latest?slot=${slot}&kind=${kind}&t=${info.mtime_ms}`, {headers:{"X-T170-Token":token}, cache:"no-store"});
          if (!response.ok) throw new Error(`图像读取失败 HTTP ${response.status}`);
          const objectUrl = URL.createObjectURL(await response.blob());
          if (img.dataset.objectUrl) URL.revokeObjectURL(img.dataset.objectUrl);
          img.src = objectUrl; img.dataset.objectUrl = objectUrl; img.dataset.mtime = String(info.mtime_ms);
        }
        img.classList.add("ready"); stamp.textContent = info.name;
      }
      else { img.classList.remove("ready"); stamp.textContent = "无图像"; }
    }
  } catch { /* status area reports connectivity */ }
}

async function runDetection() {
  const slot = $("cameraSelect").value;
  try { await execute({ cmd: "vision_detect", slot }); } catch { /* toast already shown */ }
}

function updateAutoDetect() {
  clearInterval(state.autoTimer); state.autoTimer = null;
  if ($("autoDetect").checked) state.autoTimer = setInterval(() => { if (!state.commandRunning && state.online) runDetection(); }, 1800);
}

async function loadConfig() {
  try { const data = await api("/api/config"); $("configEditor").value = data.content; $("configPath").textContent = data.path; $("configDirty").textContent = "已同步"; $("configDirty").style.color = ""; }
  catch (e) { toast(e.message, true); }
}

async function saveConfig() {
  askDangerous({ special: "save_config" }, "保存并重载配置", "配置会立即影响之后的机器人动作。服务端会先备份，C++ 校验失败则自动恢复。请确认参数单位和安全边界。" );
}

async function doSaveConfig() {
  try { const result = await api("/api/config", { method: "POST", body: { content: $("configEditor").value, confirm: "save_config" } }); $("configDirty").textContent = `已保存 · ${result.backup}`; $("configDirty").style.color = "var(--green)"; toast("配置已保存并通过 C++ 校验"); }
  catch (e) { toast(e.message, true); }
}

async function pollLogs() {
  try {
    const data = await api(`/api/logs?since=${state.logSeq}`); state.logSeq = data.seq;
    const viewer = $("logViewer");
    if (!state.startupShown && data.startup_tail) {
      state.startupShown = true;
      for (const text of data.startup_tail.split(/\r?\n/).filter(Boolean)) appendLog(viewer, {time:"历史",source:"startup",text});
    }
    for (const line of data.lines) appendLog(viewer, line);
    if ($("followLogs").checked) viewer.scrollTop = viewer.scrollHeight;
  } catch { /* do not spam */ }
}

function appendLog(viewer, line) {
  const row = document.createElement("div"); row.className = `log-line ${String(line.source).includes("error") ? "error" : ""}`;
  row.innerHTML = `<span class="time">${escapeHtml(line.time)}</span><span class="source">${escapeHtml(line.source)}</span><span class="text">${escapeHtml(line.text)}</span>`;
  viewer.appendChild(row); while (viewer.children.length > 2500) viewer.firstChild.remove();
}

document.querySelectorAll(".nav-item").forEach(btn => btn.addEventListener("click", () => {
  document.querySelectorAll(".nav-item").forEach(x => x.classList.toggle("active", x === btn));
  document.querySelectorAll(".page").forEach(x => x.classList.toggle("active", x.id === `page-${btn.dataset.page}`));
  if (btn.dataset.page === "config" && !$("configEditor").value) loadConfig();
}));

$("stopButton").addEventListener("click", async () => { try { await execute({cmd:"abort"}); } catch {} });
$("refreshButton").addEventListener("click", () => pollStatus(true));
$("startServiceButton").addEventListener("click", async () => { try { await api("/api/service/start", {method:"POST",body:{}}); toast("启动请求已完成"); await pollHealth(); } catch(e) { toast(e.message,true); } });
$("detectButton").addEventListener("click", runDetection);
$("arucoButton").addEventListener("click", async () => { $("cameraSelect").value = "head"; try { const r = await execute({cmd:"aruco_detect"}); renderDetections(r.markers || [], "aruco"); } catch {} });
$("cameraSelect").addEventListener("change", refreshImages);
$("autoDetect").addEventListener("change", updateAutoDetect);
$("reloadConfigButton").addEventListener("click", loadConfig);
$("saveConfigButton").addEventListener("click", saveConfig);
$("configEditor").addEventListener("input", () => { $("configDirty").textContent = "未保存"; $("configDirty").style.color = "var(--amber)"; });
$("clearLogs").addEventListener("click", () => $("logViewer").replaceChildren());

document.querySelectorAll(".command").forEach(btn => btn.addEventListener("click", () => {
  const cmd = btn.dataset.command; const payload = {cmd}; if (btn.dataset.slot) payload.slot = btn.dataset.slot;
  if (["home","grasp_ready","grasp_ready1","grasp_ready2","grasp_ready3","grasp_ready6","tray2ready","tray2ready1","tray2ready2","tray2ready3","tray2ready6","tray2","tray2_place","vision_grasp","grasp_belt","grasp_belt1","waist1","waist2","belt","belt_ready","belt_place","belt_grasp_rpy","belt_grasp","belt2","belt2_ready","belt2_place","belt2_grasp_rpy","belt2_grasp"].includes(cmd)) {
    const titles = {home:"执行 home", grasp_ready:"去抓取准备姿态", grasp_ready1:"ready1 row3", grasp_ready2:"ready2 row2/5", grasp_ready3:"ready3 row1/4", grasp_ready6:"ready6 第6行", tray2ready:"料盘2准备姿态", tray2ready1:"tray2ready1", tray2ready2:"tray2ready2", tray2ready3:"tray2ready3", tray2ready6:"tray2ready6", tray2:"料盘2空孔放置", tray2_place:"料盘2空孔放置", vision_grasp:"开始完整抓取", grasp_belt:"抓取后去传送带放置", grasp_belt1:"料盘1和一号传送带循环", waist1:"腰平移到初始位", waist2:"腰平移到前伸位", belt:"一号皮带放置", belt_ready:"皮带准备姿态", belt_place:"皮带放置末端", belt_grasp_rpy:"皮带抓取手腕", belt_grasp:"皮带抓取", belt2:"二号皮带放置", belt2_ready:"二号皮带准备", belt2_place:"二号皮带放置", belt2_grasp_rpy:"二号皮带抓取手腕", belt2_grasp:"二号皮带抓取"};
    const texts = {
      home:"腰会升回测量高度，双臂回到 yaml standby。",
      grasp_ready:"头部标定姿态、腰同原 ready，手臂用 ready1 姿态。",
      grasp_ready1:"腰和身体不变，左手姿态用 row 3（ready1）。右臂锁定时只动左臂。",
      grasp_ready2:"腰和身体不变，左手姿态用 row 2/5。右臂锁定时只动左臂。",
      grasp_ready3:"腰和身体不变，左手姿态用 row 1/4。右臂锁定时只动左臂。",
      grasp_ready6:"腰和身体不变，左手姿态用第 6 行专用 RPY。右臂锁定时只动左臂。",
      tray2ready:"头/腰同 grasp ready，手臂用 yaml tray2_place 独立姿态。",
      tray2ready1:"腰和身体同 grasp ready，手臂用 tray2_place ready1（row3/6）。",
      tray2ready2:"腰和身体同 grasp ready，手臂用 tray2_place ready2（row2/5）。",
      tray2ready3:"腰和身体同 grasp ready，手臂用 tray2_place ready3（row4）。",
      tray2ready6:"腰和身体同 grasp ready，手臂用 tray2_place 最远行姿态。",
      tray2:"底盘先到 AP9，下蹲同 grasp，YOLO 认空孔放置，放完站起回 home_tcp。",
      tray2_place:"底盘先到 AP9，下蹲同 grasp，YOLO 认空孔放置，放完站起回 home_tcp。",
      vision_grasp:"底盘先到 AP4，再执行双臂视觉抓取。确认料盘、人员和障碍物状态。",
      grasp_belt:"闭环：AP7 抓→home→AP5 放置→皮带抓取→AP6 放置→第二皮带抓取→AP9 空孔放置→回 AP7 再抓。abort 才停。",
      grasp_belt1:"只在料盘1和一号传送带之间循环：AP7 抓→home→AP5 放置→回 AP7 再抓。不在传送带上抓，不去 AP6，不去 AP9。abort 才停。",
      waist1:"只动腰，平移回 layer3_home.x（后移 5cm 的初始位），手臂不动。",
      waist2:"只动腰，平移到前伸 25cm 位，手臂不动。",
      belt:"底盘先到 AP6，再按 conveyor 放置。不预开合爪。结束后停在准备 tcp。",
      belt2:"底盘先到 AP5，再按 conveyor2 放置。不预开合爪。结束后停在准备 tcp。",
      belt_ready:"腰/头到 conveyor，手臂走到 yaml conveyor.tcp（准备姿态）。",
      belt_place:"腰/头到 conveyor，手臂走到 yaml conveyor.place_tcp（放置末端，类似 ready）。",
      belt_grasp_rpy:"腰/头同 belt_ready，手臂停在 conveyor.tcp 的 XYZ，手腕换成 grasp_rpy_deg。不拍照、不夹取、不动底盘。",
      belt_grasp:"单独执行：先到 AP6，不转腰，拍照后抓取，回 grasp_tcp。左右手可过中轴。cycle 里接在第一次放置后。",
      belt2_ready:"底盘先到 AP5，腰/头到 conveyor2，手臂走到 yaml conveyor2.tcp（准备姿态）。",
      belt2_place:"底盘先到 AP5，准备后按 conveyor2 放置，结束后停在准备 tcp。左右手可过中轴。",
      belt2_grasp_rpy:"腰/头到 conveyor2 观察位，手臂停在 conveyor2.tcp 的 XYZ，手腕换成 grasp_rpy_deg。不拍照、不夹取、不动底盘。",
      belt2_grasp:"底盘先到 AP5，进入准备姿态，不转腰，拍照抓取，回 conveyor2.grasp_tcp。左右手可过中轴。"
    };
    askDangerous(payload, titles[cmd], texts[cmd]);
  }
  else execute(payload).catch(()=>{});
}));
document.querySelectorAll(".grip-command").forEach(btn => btn.addEventListener("click", () => {
  const payload = {cmd:btn.dataset.command, side:btn.dataset.side};
  if (payload.cmd === "gripper_grasp") payload.max_torque_nm = Number($("gripTorque").value);
  askDangerous(payload, payload.cmd === "gripper_open" ? "张开夹爪" : "力控夹取", `将控制${payload.side === "both" ? "双侧" : payload.side === "right" ? "右侧" : "左侧"}夹爪，请确保手指附近无人员或异物。`);
}));
$("estopReady").addEventListener("change", e => $("acceptConfirm").disabled = !e.target.checked);
$("cancelConfirm").addEventListener("click", () => { $("confirmModal").classList.remove("open"); state.pendingAction = null; });
$("acceptConfirm").addEventListener("click", async () => {
  const action = state.pendingAction; $("confirmModal").classList.remove("open"); state.pendingAction = null;
  if (!action) return; if (action.special === "save_config") await doSaveConfig(); else execute(action, true).catch(()=>{});
});

renderNumbers("rightTcp", [], labels6); renderNumbers("leftTcp", [], labels6);
renderNumbers("rightJoints", [], ["J1","J2","J3","J4","J5","J6","J7"]); renderNumbers("leftJoints", [], ["J1","J2","J3","J4","J5","J6","J7"]);
renderGripper("right"); renderGripper("left");
pollHealth().then(() => pollStatus(true)); refreshImages(); pollLogs();
setInterval(pollHealth, 3000); setInterval(pollStatus, 2000); setInterval(pollLogs, 1000); setInterval(refreshImages, 2500);

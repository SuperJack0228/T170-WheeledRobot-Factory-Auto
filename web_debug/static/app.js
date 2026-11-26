"use strict";

const qs = new URLSearchParams(location.search);
const queryToken = qs.get("token");
if (queryToken) localStorage.setItem("t170_token", queryToken);
const token = queryToken || localStorage.getItem("t170_token") || "";
if (queryToken && history.replaceState) history.replaceState(null, "", location.pathname);

const $ = (id) => document.getElementById(id);
/** 与 server.py 的 WEB_API_VERSION 对齐；小改页面时同时 bump UI_VERSION。 */
const WEB_API = 10;
const UI_VERSION = "2.3.0";
const robotPrefix = location.pathname.match(/^\/robots\/(TH-T170C-\d{4,8})(?=\/)/);
const targetRobot = robotPrefix?.[1] || "";
const apiPrefix = robotPrefix?.[0] || "";
const robotUrl = (path) => apiPrefix + path;
let fleetData = null;
let dispatchActive = false;
const esc = (value) => String(value ?? "").replace(/[&<>"']/g, (c) =>
  ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const files = [
  ["motion", "运动参数"],
  ["tray_code", "料盘二维码"],
  ["belt_code", "传送带二维码"],
  ["hand_eye", "手眼平移"],
  ["head_geom", "头部高度"],
];
const taskName = { load: "上料", transfer: "转运", unload: "下料" };
const plan = [];
let configFile = "motion";
let logSeq = 0;
let commandCapabilities = null;
let capabilitiesCheckedAt = 0;
let profileData = null;
let actionsMode = null;
let statusRefreshing = false;
let motionPending = false;
let remoteBusy = false;
let readyGroup = "1";
let workcellData = null;
let workcellPending = false;
const stationCommands = new Set(["vision_grasp", "vision_grasp_once", "vision_servo_dual_once",
  "tray2_auto", "belt_ready", "belt2_ready", "belt_grasp", "belt2_grasp", "belt", "belt2"]);
function workcellConfirmed() {
  return workcellData?.editable === true && workcellData.active_group === workcellData.selected_group;
}
function updateCommandAvailability() {
  document.querySelectorAll(".cmd[data-command]").forEach((button) => {
    button.disabled = motionPending || remoteBusy || workcellPending || !commandPermitted(button.dataset.command)
      || (dispatchActive && !["abort", "dispatch_exit"].includes(button.dataset.command))
      || (stationCommands.has(button.dataset.command) && !workcellConfirmed());
  });
  $("workcellGroup").disabled = motionPending || remoteBusy || dispatchActive || workcellPending || !workcellData?.editable;
  $("runPlan").disabled = commandCapabilities?.mode !== "maintenance" || dispatchActive || motionPending || remoteBusy || workcellPending || !workcellConfirmed();
}
function commandPermitted(cmd) {
  if (!commandCapabilities) return false;
  return commandCapabilities.commands.some((entry) => entry.name === cmd && entry.allowed);
}
async function refreshCapabilities() {
  if (Date.now() - capabilitiesCheckedAt < 5000) return;
  capabilitiesCheckedAt = Date.now();
  try {
    const data = await api("/api/capabilities");
    if (!Array.isArray(data.commands)) return;
    commandCapabilities = data;
    dispatchActive = data.dispatch_active === true;
    const production = data.mode === "production";
    applyMode(data.mode);
    if (data.mode === "maintenance" && !workcellPending) await refreshWorkcells();
    const notice = $("modeNotice");
    notice.hidden = false;
    notice.textContent = production
      ? "生产模式：开放调度运行控制和状态查询。手动动作、单步与标定请使用维护入口。"
      : dispatchActive ? "维护模式 · 调度测试运行中。退出调度后继续标定和单步。"
      : "维护模式：不会自动进入调度；可标定、单步，也可在任务联调页手动进入调度测试。";
    updateCommandAvailability();
    const signature = JSON.stringify([data.mode, data.commands]);
    if (actionsMode !== signature) {
      const catalog = await api("/api/actions");
      renderActions(catalog);
      actionsMode = signature;
    }
  } catch (_) { commandCapabilities = null; updateCommandAvailability(); }
}

function applyMode(mode) {
  const production = mode === "production";
  const maintenance = mode === "maintenance";
  document.body.dataset.mode = mode;
  $("modeChip").textContent = production ? "生产模式" : maintenance ? (dispatchActive ? "维护 · 调度测试" : "维护模式") : "模式未知";
  $("modeChip").className = `pill ${production ? "ok" : "maintenance"}`;
  $("taskTab").textContent = production ? "生产调度" : "任务联调";
  $("taskHeading").textContent = production ? "生产调度" : "任务联调";
  $("taskLead").textContent = production ? "查看调度任务和执行状态，由工厂调度分配任务。" : "维护模式下按顺序验证上料、转运与下料技能，不自动进入调度。";
  $("dispatchMonitor").hidden = !(production || maintenance);
  $("automaticTaskCard").hidden = !(production || maintenance);
  $("manualTaskCard").hidden = !maintenance || dispatchActive;
  $("maintenanceWorkcells").hidden = !maintenance || dispatchActive;
  document.querySelector('[data-page="motion"]').hidden = !maintenance;
  $("calibrationLink").hidden = !maintenance || dispatchActive;
  if (production && $("page-motion").classList.contains("active")) showPage("status");
  $("applyProfile").disabled = !maintenance;
  $("createProfile").disabled = !maintenance;
}

function renderWorkcells(data) {
  workcellData = data;
  $("workcellGroup").innerHTML = data.groups.map((row) =>
    `<option value="${row.group}" ${row.group === data.selected_group ? "selected" : ""}>第 ${row.group} 组</option>`).join("");
  const active = data.groups.find((row) => row.group === data.active_group);
  const row = active || data.groups.find((entry) => entry.group === data.selected_group);
  $("workcellStations").textContent = row
    ? `${active ? "当前生效" : "配置站点"} · 料盘一 ${row.tray_station}　传送带一 ${row.belt_station}　传送带二 ${row.out_station}　料盘二 ${row.tray2_station}` : "没有可用站点";
  $("workcellNote").textContent = data.active_group === data.selected_group
    ? `第 ${data.active_group} 组已生效。切换组别只应用配置，执行单步动作时才导航。`
    : `配置选择第 ${data.selected_group} 组，${data.active_group ? `当前仍生效第 ${data.active_group} 组` : "尚未确认生效站点"}。请重新选择并等待切换成功。`;
  updateCommandAvailability();
}

async function refreshWorkcells() {
  try { renderWorkcells(await api("/api/maintenance-workcells")); }
  catch (error) {
    workcellData = null;
    $("workcellStations").textContent = "机床组配置未加载";
    $("workcellNote").textContent = error.message;
    updateCommandAvailability();
  }
}

$("workcellGroup").addEventListener("change", async (event) => {
  const group = Number(event.target.value);
  workcellPending = true; updateCommandAvailability();
  $("workcellNote").textContent = `正在切换至第 ${group} 组…`;
  try {
    const result = await api("/api/maintenance-workcells", {method: "POST", body: {group}});
    renderWorkcells(result.workcells);
    toast(result.applied === true ? `第 ${group} 组已生效` : result.message || "尚未确认切换结果");
  } catch (error) {
    toast(error.message); await refreshWorkcells();
    $("workcellNote").textContent = `切换失败：${error.message}`;
  } finally { workcellPending = false; updateCommandAvailability(); }
});

function renderActions(catalog) {
  $("actionGroups").innerHTML = catalog.groups.map((group) => {
    const preparation = group.id === "preparation";
    const actions = preparation ? group.actions.filter((action) => !/^(grasp_ready|tray2ready)[23]$/.test(action.name)) : group.actions;
    const selector = preparation ? `<label class="ready-group">料盘准备行组<select id="readyGroup">${[["1","近排"],["2","中排"],["3","远排"]].map(([value,label])=>`<option value="${value}" ${readyGroup===value?"selected":""}>${label}</option>`).join("")}</select></label>` : "";
    const body = `<h2>${esc(group.title)}</h2><p class="hint">${esc(group.description)}</p>${selector}<div class="actions">${actions.map((action) => {
      const ready = preparation && /^(grasp_ready|tray2ready)1$/.test(action.name);
      const base = action.name.startsWith("grasp_ready") ? "grasp_ready" : "tray2ready";
      const name = ready ? base + readyGroup : action.name;
      const label = ready ? (base === "grasp_ready" ? "料盘抓取准备" : "料盘放置准备") : action.label;
      return `<button class="cmd" ${ready?`data-ready-base="${base}"`:""} data-command="${esc(name)}" title="${esc(action.description)}">${esc(label)}</button>`;
    }).join("")}</div>`;
    const colorClass = {preparation:"mod-pose",tray:"mod-tray",conveyor:"mod-belt"}[group.id] || "";
    return group.id === "service" ? `<details class="card"><summary>${esc(group.title)}</summary>${body}</details>` : `<article class="card ${colorClass}">${body}</article>`;
  }).join("");
  if ($("readyGroup")) $("readyGroup").onchange = (event) => {
    readyGroup = event.target.value;
    document.querySelectorAll("[data-ready-base]").forEach((button) => {button.dataset.command = button.dataset.readyBase + readyGroup;});
    updateCommandAvailability();
  };
  updateCommandAvailability();
}

function showActionOutcome(outcome, message) {
  $("actionNotice").hidden = false;
  $("actionNotice").dataset.outcome = outcome;
  $("actionNotice").textContent = message;
  $("motionStatus").dataset.outcome = outcome;
  $("motionStatus").textContent = message;
}

function toast(message) {
  const el = $("toast");
  el.textContent = message;
  el.className = "toast show";
  clearTimeout(el.timer);
  el.timer = setTimeout(() => { el.className = "toast"; }, 2800);
}

async function api(path, options = {}) {
  const headers = { "X-T170-Token": token, ...(options.headers || {}) };
  const central = path.startsWith("/api/fleet");
  const explicitTarget = path.match(/^\/robots\/(TH-T170C-\d{4,8})\//)?.[1];
  const expected = explicitTarget || targetRobot || commandCapabilities?.robot_id || fleetData?.local_robot_id;
  if (!central && expected) headers["X-T170-Expected-Robot"] = expected;
  if (options.body && typeof options.body !== "string") {
    if (!central && expected) options.body = {...options.body,expected_robot_id:expected};
    headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(options.body);
  }
  const url = central || path.startsWith("/robots/") ? path : robotUrl(path);
  const response = await fetch(url, { ...options, headers, cache: "no-store" });
  const result = await response.json().catch(() => ({ ok: false, error: `HTTP ${response.status}` }));
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

function fmt(value, digits) {
  if (value === null || value === undefined || value === "") return "—";
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "—";
}

function figures(id, pairs) {
  $(id).innerHTML = pairs.map(([label, value]) =>
    `<div><span>${label}</span><strong>${value}</strong></div>`).join("");
}

function showPage(name) {
  document.querySelectorAll(".tab").forEach((el) => el.classList.toggle("active", el.dataset.page === name));
  document.querySelectorAll(".page").forEach((el) => el.classList.toggle("active", el.id === `page-${name}`));
  if (name === "config") loadConfig();
  if (name === "vision") loadVision();
  if (name === "logs") loadLogs().catch(() => {});
  if (name === "dispatch") refreshDispatch();
  if (name === "profiles") loadProfiles(!profileData).catch((error) => toast(error.message));
}

function checkVersion(health) {
  const banner = $("versionBanner");
  const stale = health.api !== WEB_API;
  banner.hidden = !stale;
  if (stale) {
    banner.textContent = health.api === undefined
      ? "网页服务 server.py 是旧版本：参数页和调度监控用不了。请重启 server.py，再按 Ctrl+F5 刷新。"
      : `网页和 server.py 版本不一致（页面 ${WEB_API}，服务 ${health.api}）。请按 Ctrl+F5 强制刷新。`;
  }
  const versionEl = $("uiVersion");
  if (versionEl) {
    const serverUi = health.ui_version ? ` · server ${health.ui_version}` : "";
    versionEl.textContent = `T170C Web Debug  v${UI_VERSION}${serverUi}`;
  }
}

async function refreshStatus() {
  if (statusRefreshing) return;
  statusRefreshing = true;
  try {
    const health = await api("/api/health");
    checkVersion(health);
    setHandLivePreview(health.hand_live_preview === true);
    const chip = $("serviceChip");
    chip.textContent = health.robot_online ? "已连接" : "未连接";
    chip.className = `pill ${health.robot_online ? "ok" : "bad"}`;
    $("metricService").textContent = health.robot_online ? "在线" : "离线";
    if (!health.robot_online) {
      commandCapabilities = null; actionsMode = null;
      document.querySelectorAll(".cmd").forEach((button) => { button.disabled = true; });
      $("activeRobot").textContent = "机器人服务离线";
      $("modeChip").textContent = "离线";
      return;
    }
    await refreshCapabilities();
    const data = await api("/api/status");
    const job = await api("/api/job");
    remoteBusy = job.running === true || data.busy === true;
    updateCommandAvailability();
    if (data.ok !== true) { $("statusLead").textContent = data.error || "状态读取失败，保留上一次读数。"; return; }
    $("statusLead").textContent = data.busy
      ? `正在执行 ${data.action || job.step || "动作"}。关节读数暂停，避免打断电机。`
      : "空闲时每秒读取一次。动作进行中不打断电机，沿用上一次读数。";
    $("metricChassis").textContent = data.chassis_station || data.chassis_message || "—";
    const holdParts = [];
    if (data.grasped_right) holdParts.push("右");
    if (data.grasped_left) holdParts.push("左");
    const qBits = [];
    if (Number.isFinite(data.gripper_q_right)) qBits.push(`右q=${Number(data.gripper_q_right).toFixed(2)}`);
    if (Number.isFinite(data.gripper_q_left)) qBits.push(`左q=${Number(data.gripper_q_left).toFixed(2)}`);
    $("metricHold").textContent = holdParts.length
      ? `${holdParts.join("")}${qBits.length ? `（${qBits.join(" ")}）` : ""}`
      : (qBits.length ? `空（${qBits.join(" ")}）` : "空");
    $("metricWaist").textContent = fmt(data.waist_z, 3);
    const tcp = (arr) => ["X", "Y", "Z", "RX", "RY", "RZ"].map((label, i) => [label, fmt(arr?.[i], i < 3 ? 3 : 1)]);
    figures("rightTcp", tcp(data.right_tcp_m_deg));
    figures("leftTcp", tcp(data.left_tcp_m_deg));
    const joints = Array.isArray(data.waist_joints_deg) ? data.waist_joints_deg : [];
    figures("waistPose", [
      ["X", fmt(data.waist_x, 3)],
      ["Y", fmt(data.waist_y, 3)],
      ["Z", fmt(data.waist_z, 3)],
      ...["踝", "膝", "髋", "侧倾", "回转"].map((label, i) => [label, fmt(joints[i], 1)]),
    ]);
    $("jobLine").textContent = job.running
      ? `正在执行：${job.step}`
      : (job.error ? `上次失败：${job.error}` : (job.step === "完成" ? "上次执行完成。" : "当前没有编排在跑。"));
  } catch (error) {
    commandCapabilities = null; actionsMode = null;
    updateCommandAvailability();
    $("serviceChip").textContent = "未连接";
    $("serviceChip").className = "pill bad";
    $("metricService").textContent = "离线";
  } finally { statusRefreshing = false; }
}

function renderPlan() {
  $("plan").innerHTML = plan.map((step, index) =>
    `<li>${taskName[step]} <button type="button" data-remove="${index}">去掉</button></li>`).join("");
}

async function sendCommand(cmd) {
  if (!commandPermitted(cmd)) throw new Error("该命令仅在维护模式开放");
  if (workcellPending) throw new Error("请等待机床组切换完成");
  if (stationCommands.has(cmd) && !workcellConfirmed()) throw new Error("请先选择并应用维护机床组");
  showActionOutcome("running", `正在执行 ${cmd}，等待机器人返回实际结果…`);
  const result = await api("/api/command", { method: "POST", body: { cmd, confirm: cmd } });
  $("motionResult").textContent = JSON.stringify(result, null, 2);
  if (cmd === "dispatch" && result.status === "stopped") {
    showActionOutcome("stopped", commandCapabilities?.mode === "maintenance"
      ? "调度已停止；维护模式可继续执行单步与标定。" : "调度已停止，生产服务保持待命。");
    toast("调度已停止"); refreshStatus(); return;
  }
  const succeeded = result.ok === true;
  const message = succeeded ? `${cmd} 执行完成` : (result.error || `${cmd} 执行失败`);
  showActionOutcome(succeeded ? "succeeded" : result.status || "failed", message);
  toast(message);
  refreshStatus();
}

async function loadConfig() {
  const form = $("fieldForm");
  document.querySelectorAll(".file-tab").forEach((el) => el.classList.toggle("active", el.dataset.file === configFile));
  form.innerHTML = `<p class="hint">读取中…</p>`;
  let data;
  try {
    data = await api(`/api/fields?file=${encodeURIComponent(configFile)}`);
  } catch (error) {
    const old = /资源不存在|HTTP 404/.test(error.message);
    form.innerHTML = `<p class="form-error">参数读取失败：${esc(error.message)}${
      old ? "。server.py 是旧版本，重启它再按 Ctrl+F5 刷新。" : ""}</p>`;
    return;
  }
  if (!data.groups?.length) {
    form.innerHTML = `<p class="form-error">服务没有返回参数，检查 server.py 日志。</p>`;
    return;
  }
  $("configNote").textContent = data.note || "";
  const editable = data.editable === true && commandCapabilities?.mode === "maintenance";
  $("saveConfig").disabled = !editable;
  form.innerHTML = data.groups.map((group) => `
    <h2 class="group">${esc(group.title)}</h2>
    <article class="card">
      ${group.fields.map((field) => `
        <label class="field">
          <span>${esc(field.label)}${field.unit ? `<small>${esc(field.unit)}</small>` : ""}<small class="key">${esc(field.key)}</small></span>
          <input ${editable ? "" : "readonly"} data-key="${esc(field.key)}" data-orig="${esc(field.value ?? "")}" value="${esc(field.value ?? "")}"${
            field.value === null || field.value === undefined ? ` placeholder="文件里没有"` : ""}>
        </label>`).join("")}
    </article>`).join("");
}

const stateTone = (s) => (!s ? "idle" : s.startsWith("ERR_") ? "bad" : s === "WORKING" || s === "GO_CHARGE" || s === "commuting" ? "info" : s === "CHARGING" ? "wait" : "ok");

function tile(label, value, sub = "", tone = "idle", extra = "") {
  return `<div class="tile tone-${tone} ${extra}">
    <span class="tile-label">${esc(label)}</span>
    <strong class="tile-value">${esc(value || "—")}</strong>
    ${sub ? `<small class="tile-sub">${esc(sub)}</small>` : ""}
  </div>`;
}

function applyTile(a) {
  const what = [a.what, a.time].filter(Boolean).join(" · ");
  switch (a.state) {
    case "waiting": return tile("申请", "已发出，等待同意", `${a.name} → 等 ${a.grant} · ${what}`, "wait", "pulse");
    case "granted": return tile("申请", "已同意", `${a.name} → 收到 ${a.grant} · ${what}`, "ok");
    case "denied": return tile("申请", "超时", `${a.name} 20s 没有回复 · ${what}`, "bad");
    default: return tile("申请", "无", "这个任务还没发申请", "idle");
  }
}

function reportTile(r) {
  const sub = [r.name, r.what, r.time].filter(Boolean).join(" · ");
  switch (r.state) {
    case "pending": return tile("汇报", "发送中", sub, "wait", "pulse");
    case "sent": return tile("汇报", "已发送", sub, "ok");
    case "failed": return tile("汇报", "发送失败", sub, "bad");
    default: return tile("汇报", "无", "这个任务还没汇报", "idle");
  }
}

function trayCard(title, unit, t, side, takenLabel, flagOn, flagOff, flagName) {
  if (!t?.known) {
    return `<div class="tray"><div class="tray-head"><h3>${esc(title)}</h3><span class="chip idle">还没数过</span></div>
      <p class="hint">机器人还没在这个料盘拍照计数。</p></div>`;
  }
  const total = (t.right || 0) + (t.left || 0);
  const flag = t.reported
    ? `<span class="chip bad">${esc(flagOn)} · ${esc(flagName)}</span>`
    : `<span class="chip ok">${esc(flagOff)}</span>`;
  return `<div class="tray">
    <div class="tray-head"><h3>${esc(title)}</h3>${flag}</div>
    <div class="tray-total"><strong>${total}</strong><span>${esc(unit)}</span></div>
    <div class="tray-grid">
      <div><span>右侧${esc(side)}</span><strong>${t.right ?? 0}</strong></div>
      <div><span>左侧${esc(side)}</span><strong>${t.left ?? 0}</strong></div>
      <div><span>本次任务${esc(takenLabel)}</span><strong>${t.task ?? 0}</strong></div>
      <div><span>本轮累计${esc(takenLabel)}</span><strong>${t.session ?? 0}</strong></div>
    </div>
    <small class="tile-sub">上次计数 ${esc(t.time || "—")}</small>
  </div>`;
}

function stepsRow(d) {
  const steps = d.steps || [];
  if (!steps.length) return `<p class="hint">没有在执行的任务。</p>`;
  return `<ol class="steps">${steps.map((s, i) => {
    const cls = i < d.step_index ? "done" : i === d.step_index ? "current" : "";
    return `<li class="${cls}"><b>${i + 1}</b>${esc(s)}</li>`;
  }).join("")}</ol>`;
}

function renderDispatch(d) {
  const linkTone = /断|失败/.test(d.link || "") ? "bad" : d.link === "执行任务" ? "info" : d.link === "等待任务" ? "ok" : "idle";
  const station = d.target_station ? `→ ${d.target_station}` : (d.station || "—");
  const stationSub = d.target_station ? `行驶中${d.station ? `，从 ${d.station} 出发` : ""}` : "停在站点";
  const stateSub = [d.robot_state, d.robot_state_ok ? "" : "上报失败", d.robot_state_time].filter(Boolean).join(" · ");
  const events = (d.events || []).slice().reverse().slice(0, 30);
  return `
    <div class="tiles four">
      ${tile("调度模式", d.active ? "调度中" : "未进入", d.active ? `${d.server} · ${d.robot}` : "按下面“进入调度”开始", d.active ? "ok" : "idle")}
      ${tile("本机状态", d.robot_state_zh || "未记录", stateSub, d.robot_state_ok === false ? "bad" : stateTone(d.robot_state))}
      ${tile("调度连接", d.link || "—", d.started_at ? `进入于 ${d.started_at}` : "", linkTone)}
      ${tile("底盘位置", station, stationSub, d.target_station ? "info" : "idle")}
    </div>
    <div class="tiles two">
      ${tile("当前任务", d.mission_zh || "无", d.mission ? `${d.mission} · 开始于 ${d.mission_since}` : "等待调度下发", d.mission ? "info" : "idle", "big")}
      ${tile("收到的指令", d.last_command_zh || "无", d.last_command ? `${d.last_command} · ${d.last_command_time}` : "", d.last_command ? "info" : "idle", "big")}
    </div>
    <div class="tile tone-${d.mission ? "info" : "idle"} wide">
      <span class="tile-label">步骤${d.steps?.length && d.step_index >= 0 ? ` ${d.step_index + 1}/${d.steps.length}` : ""}</span>
      ${stepsRow(d)}
      <span class="tile-label">正在做</span>
      <strong class="tile-activity">${esc(d.activity || "—")}</strong>
    </div>
    <div class="tiles four">
      ${applyTile(d.apply || {})}
      ${reportTile(d.report || {})}
      ${tile("右手", d.holding_right ? "持件" : "空",
        Number.isFinite(d.gripper_q_right) ? `q=${Number(d.gripper_q_right).toFixed(2)}` : "",
        d.holding_right ? "info" : "idle")}
      ${tile("左手", d.holding_left ? "持件" : "空",
        Number.isFinite(d.gripper_q_left) ? `q=${Number(d.gripper_q_left).toFixed(2)}` : "",
        d.holding_left ? "info" : "idle")}
    </div>
    <div class="tiles two">
      ${trayCard("料盘一 · 毛坯", "件剩余", d.tray1, "剩余", "取走", "已报空", "未报空", "RAW_MATERIAL_EMPTY")}
      ${trayCard("料盘二 · 成品", "个空孔", d.tray2, "空孔", "放入", "已报满", "未报满", "WELL_DONE_MATERIAL_FULL")}
    </div>
    <div class="tiles four">
      ${tile("完成任务", String(d.tasks_done ?? 0), "本轮调度", "ok")}
      ${tile("未完成", String(d.tasks_failed ?? 0), "本轮调度", d.tasks_failed ? "bad" : "idle")}
      ${tile("上次结果", d.last_result || "—", "", d.last_result ? (d.last_result_ok ? "ok" : "bad") : "idle")}
      ${tile("最近错误", d.last_error || "无", "", d.last_error ? "bad" : "idle")}
    </div>
    <div class="card events">
      <h3>事件</h3>
      ${events.length ? `<ul>${events.map((e) =>
        `<li class="ev-${esc(e.level)}"><time>${esc(e.time)}</time>${esc(e.text)}</li>`).join("")}</ul>`
        : `<p class="hint">还没有事件。</p>`}
      ${d.log_path ? `<p class="hint">完整日志：${esc(d.log_path)}</p>` : ""}
    </div>`;
}

let dispatchBusy = false;
async function refreshDispatch() {
  if (dispatchBusy) return;
  dispatchBusy = true;
  try {
    const d = await api("/api/dispatch");
    if (!d.ok) throw new Error(d.error || "读取调度状态失败");
    dispatchActive = d.active === true;
    if (commandCapabilities) applyMode(commandCapabilities.mode);
    updateCommandAvailability();
    $("dmError").hidden = true;
    $("dmBody").innerHTML = renderDispatch(d);
    $("dmUpdated").textContent = `刷新于 ${new Date().toLocaleTimeString("zh-CN", { hour12: false })}`;
  } catch (error) {
    $("dmError").hidden = false;
    $("dmError").textContent = /API 不存在|资源不存在|HTTP 404/.test(error.message)
      ? "server.py 是旧版本，没有调度监控。重启 server.py 后按 Ctrl+F5 刷新。"
      : `调度状态读不到：${error.message}`;
  } finally {
    dispatchBusy = false;
  }
}

async function loadVision() {
  const image = $("visionImage");
  image.onload = () => image.classList.add("show");
  image.onerror = () => image.classList.remove("show");
  image.src = robotUrl(`/api/image/latest?slot=head&kind=processed&token=${encodeURIComponent(token)}&t=${Date.now()}`);
}

function reverseLines(text) {
  return String(text || "")
    .split("\n")
    .filter((line) => line.length)
    .reverse()
    .join("\n");
}

async function loadLogs() {
  const data = await api(`/api/logs?since=${logSeq}`);
  logSeq = data.seq || logSeq;
  const view = $("logView");
  // 最新在最上：新行插到顶部，有更新时滚回顶。
  const freshLines = (data.lines || []).map((line) => `${line.time}  ${line.text}`);
  if (!$("page-logs").dataset.seeded) {
    view.textContent = reverseLines(data.startup_tail);
    $("page-logs").dataset.seeded = "1";
  }
  if (freshLines.length) {
    const block = freshLines.slice().reverse().join("\n");
    view.textContent = view.textContent ? `${block}\n${view.textContent}` : block;
    view.scrollTop = 0;
  }
}

document.querySelectorAll(".tab[data-page]").forEach((el) => el.addEventListener("click", () => showPage(el.dataset.page)));

document.addEventListener("click", async (event) => {
  const el = event.target.closest(".cmd[data-command]");
  if (!el || el.disabled) return;
  const cmd = el.dataset.command;
  if (!window.confirm(`执行 ${el.textContent.trim()}？`)) return;
  motionPending = true;
  updateCommandAvailability();
  try { await sendCommand(cmd); }
  catch (error) { showActionOutcome("failed", error.message); toast(error.message); }
  finally { motionPending = false; updateCommandAvailability(); refreshStatus(); }
});

$("stopButton").addEventListener("click", async () => {
  try { await api("/api/command", { method: "POST", body: { cmd: "abort" } }); toast("已请求停止"); }
  catch (error) { toast(error.message); }
});

async function waitDispatchInactive(ms = 15000) {
  const deadline = Date.now() + ms;
  while (Date.now() < deadline) {
    try {
      const d = await api("/api/dispatch");
      if (d.ok && !d.active) return true;
    } catch (_) { /* 继续等 */ }
    await new Promise((r) => setTimeout(r, 400));
  }
  return false;
}

$("exitDispatch").addEventListener("click", async () => {
  if (!window.confirm("退出调度模式？将中止当前动作并离开自动调度。")) return;
  const btn = $("exitDispatch");
  btn.disabled = true;
  try {
    await api("/api/command", { method: "POST", body: { cmd: "dispatch_exit" } });
    toast("正在退出调度…");
    const ok = await waitDispatchInactive();
    await refreshDispatch();
    toast(ok ? "已退出调度模式" : "已发出退出请求，若仍显示调度中请稍候再刷新");
  } catch (error) {
    toast(error.message);
  } finally {
    btn.disabled = false;
  }
});

$("addLoad").addEventListener("click", () => { plan.push("load"); renderPlan(); });
$("addTransfer").addEventListener("click", () => { plan.push("transfer"); renderPlan(); });
$("addUnload").addEventListener("click", () => { plan.push("unload"); renderPlan(); });
$("clearPlan").addEventListener("click", () => { plan.length = 0; renderPlan(); });
$("plan").addEventListener("click", (event) => {
  const index = event.target?.dataset?.remove;
  if (index === undefined) return;
  plan.splice(Number(index), 1);
  renderPlan();
});
$("runPlan").addEventListener("click", async () => {
  if (!plan.length) { toast("先加上料、转运或下料"); return; }
  if (!window.confirm(`按顺序执行：${plan.map((step) => taskName[step]).join(" → ")}？`)) return;
  try {
    await api("/api/sequence", { method: "POST", body: { confirm: "sequence", steps: plan.slice() } });
    toast("编排已开始");
  } catch (error) { toast(error.message); }
});

$("fileTabs").innerHTML = files.map(([id, label]) =>
  `<button type="button" class="quiet file-tab${id === "motion" ? " active" : ""}" data-file="${id}">${label}</button>`).join("");
$("fileTabs").addEventListener("click", (event) => {
  const id = event.target?.dataset?.file;
  if (!id) return;
  configFile = id;
  loadConfig().catch((error) => toast(error.message));
});
$("reloadRobot").addEventListener("click", async () => {
  const btn = $("reloadRobot");
  btn.disabled = true;
  try {
    const result = await api("/api/command", { method: "POST", body: { cmd: "reload_config" } });
    toast(result.message || result.error || "应用配置完成");
  } catch (error) {
    toast(error.message);
  } finally {
    btn.disabled = false;
  }
});
$("reloadConfig").addEventListener("click", () => loadConfig().catch((error) => toast(error.message)));
$("saveConfig").addEventListener("click", async () => {
  const values = {};
  document.querySelectorAll("#fieldForm input").forEach((input) => {
    const value = input.value.trim();
    if (value !== input.dataset.orig) values[input.dataset.key] = value;
  });
  const keys = Object.keys(values);
  if (!keys.length) { toast("没有改动"); return; }
  if (!window.confirm(`保存这 ${keys.length} 项改动？\n${keys.map((k) => `${k} = ${values[k]}`).join("\n")}`)) return;
  try {
    const result = await api("/api/fields", {
      method: "POST",
      body: { file: configFile, values, confirm: "save_config" },
    });
    toast(result.message || result.error || "已保存，启动时加载");
    await loadConfig();
  } catch (error) { toast(error.message); }
});

const versionEl = $("uiVersion");
if (versionEl) versionEl.textContent = `T170C Web Debug  v${UI_VERSION}`;

function modelLabel(model) {
  return model === "t170c_v2_3kg" ? "T170C V2 · 3 kg" : model === "t170c_v2_5kg" ? "T170C V2 · 5 kg" : (model || "机型未知");
}
const robotLabel = (id) => id ? `No.${Number(id.split("-").at(-1))}` : "未知编号";
async function loadFleet() {
  fleetData = await api("/api/fleet");
  const selected = targetRobot || fleetData.local_robot_id;
  $("fleetRobot").innerHTML = fleetData.robots.map((r) => `<option value="${esc(r.robot_id)}" ${r.robot_id === selected ? "selected" : ""}>${robotLabel(r.robot_id)}${r.robot_id === fleetData.local_robot_id ? " · 本机" : ""}</option>`).join("");
  if (!selected) $("fleetRobot").selectedIndex = -1;
}
$("fleetRobot").onchange = async () => {
  const select = $("fleetRobot"), id = select.value;
  const robot = fleetData?.robots.find((r) => r.robot_id === id);
  if (!robot) return;
  if (id === fleetData.local_robot_id) { location.assign("/"); return; }
  if (!robot.token_saved) {
    showPage("profiles"); $("fleetSettings").open = true;
    $("fleetId").value = id; $("fleetHost").value = robot.host;
    $("fleetMessage").textContent = "首次连接，请保存该机器人访问令牌，然后重新选择连接。";
    select.value = targetRobot || fleetData.local_robot_id || "";
    return;
  }
  select.disabled = true;
  try {
    await api(`/robots/${id}/api/identity`);
    // A new page context discards old frames, logs, pending callbacks and plans.
    // Already accepted commands remain owned by their original robot.
    location.assign(`/robots/${id}/`);
  } catch (error) {
    toast(error.message); select.value = targetRobot || fleetData.local_robot_id || "";
    select.disabled = false;
  }
};
$("saveFleet").onclick = async () => {
  try {
    await api("/api/fleet", {method:"POST",body:{robot_id:$("fleetId").value.trim(),
      host:$("fleetHost").value.trim(),token:$("fleetToken").value.trim()}});
    $("fleetToken").value = "";
    $("fleetMessage").textContent = "连接已保存，可从顶部选择机器人。";
    await loadFleet();
  } catch (error) { $("fleetMessage").textContent = error.message; }
};
$("newRobotId").oninput = () => {
  const number = Number($("newRobotId").value.split("-").at(-1));
  $("newRobotWebHost").value = Number.isInteger(number) && number > 0 && number < 255 ? `192.168.113.${number}` : "";
};
function selectedProfileDetail() {
  const profile = profileData?.profiles.find((entry) => entry.manifest === $("profileSelect").value);
  if (!profile) return;
  $("profileModel").value = profile.robot_model;
  $("newRobotHost").value = profile.host;
  $("profileDetail").textContent = `底盘 ${profile.host} · 上位机 ${profile.web_host}:${profile.web_port} · 标定目录 ${profile.calibration_directory}` +
    (profile.missing_files.length ? `。启用前需导入本机文件：${profile.missing_files.join("；")}` : "");
}
async function loadProfiles(refreshForm = true) {
  const data = await api("/api/profiles");
  profileData = data;
  const active = data.active;
  const current = active ? `${active.robot_id || "当前服务"} · ${modelLabel(active.robot_model)}` : "主服务未运行";
  $("profileActive").textContent = current;
  $("activeRobot").textContent = active?.robot_id || (active ? "当前服务" : "机器人服务离线");
  $("activeModel").textContent = active ? modelLabel(active.robot_model) : "";
  $("connectedRobot").textContent = active ? `${robotLabel(active.robot_id)} · ${modelLabel(active.robot_model)}` : "机器人离线";
  const selected = data.profiles.find((entry) => entry.manifest === data.selected_manifest);
  $("profileSelected").textContent = selected ? `${selected.robot_id} · ${modelLabel(selected.robot_model)}` : "未选择";
  // Model edits are staged until restart. Compare runtime identity/model rather
  // than just manifest name: editing the same manifest must show a restart cue.
  if (active && selected && (active.robot_id !== selected.robot_id || active.robot_model !== selected.robot_model)) {
    $("profileSelected").textContent += " · 等待重启主服务";
  }
  $("applyProfile").disabled = !!active && active.mode !== "maintenance";
  $("createProfile").disabled = !!active && active.mode !== "maintenance";
  if (refreshForm) {
    $("profileSelect").innerHTML = data.profiles.map((profile) => `<option value="${esc(profile.manifest)}" ${profile.manifest === data.selected_manifest ? "selected" : ""}>${esc(profile.robot_id)} · ${modelLabel(profile.robot_model)}</option>`).join("");
    selectedProfileDetail();
  }
}
$("openProfiles").onclick = () => showPage("profiles");
$("profileSelect").onchange = selectedProfileDetail;
async function saveProfile(action) {
  if (!profileData) return;
  const payload = {action, expected_manifest: profileData.selected_manifest,
    manifest: $("profileSelect").value, robot_model: $("profileModel").value,
    template: $("profileSelect").value, robot_id: $("newRobotId").value.trim(), host: $("newRobotHost").value.trim()};
  payload.web_host = $("newRobotWebHost").value.trim();
  const button = $(action === "create" ? "createProfile" : "applyProfile");
  button.disabled = true;
  try {
    const result = await api("/api/profiles", {method: "POST", body: payload});
    $("profileMessage").textContent = result.message;
    toast(result.message);
    await loadProfiles();
    if (action === "create" && result.manifest) {
      try {
        await api("/api/fleet", {method:"POST",body:{robot_id:result.robot_id,
          host:result.web_host,port:result.web_port}});
        await loadFleet();
      } catch (error) { $("fleetMessage").textContent = `档案已创建，连接登记需补充：${error.message}`; }
      $("profileSelect").value = result.manifest;
      selectedProfileDetail();
    }
  } catch (error) { $("profileMessage").textContent = error.message; toast(error.message); }
  finally { button.disabled = profileData?.active?.mode === "production"; }
}
$("applyProfile").onclick = () => saveProfile("select");
$("createProfile").onclick = () => saveProfile("create");

function bindHandStream(id, slot, enabled) {
  const img = $(id);
  const empty = $(id + "Empty");
  if (!img) return;
  if (!enabled) {
    if (img.dataset.streaming === "1") {
      img.dataset.streaming = "0";
      img.removeAttribute("src");
      img.classList.remove("show");
    }
    if (empty) {
      empty.hidden = false;
      empty.textContent = "预览已关";
    }
    return;
  }
  if (empty && !img.classList.contains("show")) {
    empty.hidden = false;
    empty.textContent = "等待画面";
  }
  if (img.dataset.streaming === "1") return;
  img.dataset.streaming = "1";
  img.onload = () => {
    img.classList.add("show");
    if (empty) empty.hidden = true;
  };
  img.onerror = () => {
    img.dataset.streaming = "0";
    img.classList.remove("show");
    if (empty) {
      empty.hidden = false;
      empty.textContent = "画面中断";
    }
  };
  img.src = robotUrl(`/api/stream/live?slot=${slot}&token=${encodeURIComponent(token)}`);
}

function setHandLivePreview(enabled) {
  document.querySelector(".hand-live").hidden = !enabled;
  const note = $("handLiveNote");
  if (note) {
    note.textContent = enabled
      ? "视频流开着，按相机帧率推左右手彩色。不读头相机，不进调度运动循环。和影子同时开时，影子取帧会变慢。不用时把 cameras.hand_live_preview 改回 false，再重启服务。"
      : "实时预览已关，不从手相机抽帧。要看画面，把 cameras.hand_live_preview 改成 true，再重启服务。";
  }
  bindHandStream("liveLeft", "left_hand", enabled);
  bindHandStream("liveRight", "right_hand", enabled);
}

setInterval(refreshStatus, 1000);
setInterval(() => loadProfiles(false).catch(() => {}), 10000);
setInterval(() => { if ($("page-dispatch").classList.contains("active")) refreshDispatch(); }, 1000);
setInterval(() => { if ($("page-logs").classList.contains("active")) loadLogs().catch(() => {}); }, 1500);
refreshStatus();
loadProfiles().catch((error) => { $("profileMessage").textContent = error.message; });
loadFleet().catch((error) => { $("fleetMessage").textContent = error.message; });

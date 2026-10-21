"use strict";

const qs = new URLSearchParams(location.search);
const queryToken = qs.get("token");
if (queryToken) localStorage.setItem("t170_token", queryToken);
const token = queryToken || localStorage.getItem("t170_token") || "";
if (queryToken && history.replaceState) history.replaceState(null, "", location.pathname);

const $ = (id) => document.getElementById(id);
/** 与 server.py 的 WEB_API_VERSION 对齐；小改页面时同时 bump UI_VERSION。 */
const WEB_API = 4;
const UI_VERSION = "1.1.9";
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

function toast(message) {
  const el = $("toast");
  el.textContent = message;
  el.className = "toast show";
  clearTimeout(el.timer);
  el.timer = setTimeout(() => { el.className = "toast"; }, 2800);
}

async function api(path, options = {}) {
  const headers = { "X-T170-Token": token, ...(options.headers || {}) };
  if (options.body && typeof options.body !== "string") {
    headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(options.body);
  }
  const response = await fetch(path, { ...options, headers, cache: "no-store" });
  const result = await response.json().catch(() => ({ ok: false, error: `HTTP ${response.status}` }));
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

function fmt(value, digits) {
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
  try {
    const health = await api("/api/health");
    checkVersion(health);
    const chip = $("serviceChip");
    chip.textContent = health.robot_online ? "已连接" : "未连接";
    chip.className = `pill ${health.robot_online ? "ok" : "bad"}`;
    $("metricService").textContent = health.robot_online ? "在线" : "离线";
    if (!health.robot_online) return;
    const data = await api("/api/status");
    const job = await api("/api/job");
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
      : (job.error ? `上次停止：${job.error}` : (job.step === "完成" ? "编排已完成。" : "当前没有编排在跑。"));
  } catch (error) {
    $("serviceChip").textContent = "未连接";
    $("serviceChip").className = "pill bad";
    $("metricService").textContent = "离线";
  }
}

function renderPlan() {
  $("plan").innerHTML = plan.map((step, index) =>
    `<li>${taskName[step]} <button type="button" data-remove="${index}">去掉</button></li>`).join("");
}

async function sendCommand(cmd) {
  const result = await api("/api/command", { method: "POST", body: { cmd, confirm: cmd } });
  $("motionResult").textContent = JSON.stringify(result, null, 2);
  toast(result.ok ? "已完成" : (result.error || "没有完成"));
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
  form.innerHTML = data.groups.map((group) => `
    <h2 class="group">${esc(group.title)}</h2>
    <article class="card">
      ${group.fields.map((field) => `
        <label class="field">
          <span>${esc(field.label)}${field.unit ? `<small>${esc(field.unit)}</small>` : ""}<small class="key">${esc(field.key)}</small></span>
          <input data-key="${esc(field.key)}" data-orig="${esc(field.value ?? "")}" value="${esc(field.value ?? "")}"${
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
  image.src = `/api/image/latest?slot=head&kind=processed&token=${encodeURIComponent(token)}&t=${Date.now()}`;
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

document.querySelectorAll(".tab").forEach((el) => el.addEventListener("click", () => showPage(el.dataset.page)));

document.querySelectorAll(".cmd").forEach((el) => el.addEventListener("click", async () => {
  const cmd = el.dataset.command;
  if (!window.confirm(`执行 ${el.textContent.trim()}？`)) return;
  el.disabled = true;
  try { await sendCommand(cmd); }
  catch (error) { toast(error.message); }
  finally { el.disabled = false; }
}));

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
    toast(result.reload ? "已保存并重载" : (result.error || "已保存，重启后生效"));
    await loadConfig();
  } catch (error) { toast(error.message); }
});

const versionEl = $("uiVersion");
if (versionEl) versionEl.textContent = `T170C Web Debug  v${UI_VERSION}`;

function refreshHandLive() {
  const stamp = Date.now();
  for (const [id, slot] of [["liveRight", "right_hand"], ["liveLeft", "left_hand"]]) {
    const img = $(id);
    const empty = $(id + "Empty");
    if (!img || img.dataset.loading === "1") continue;
    img.dataset.loading = "1";
    const next = new Image();
    next.onload = () => {
      img.src = next.src;
      img.classList.add("show");
      if (empty) empty.hidden = true;
      img.dataset.loading = "0";
    };
    next.onerror = () => {
      if (!img.classList.contains("show") && empty) empty.hidden = false;
      img.dataset.loading = "0";
    };
    next.src = `/api/image/live?slot=${slot}&token=${encodeURIComponent(token)}&t=${stamp}`;
  }
}

setInterval(refreshHandLive, 200);
refreshHandLive();
setInterval(refreshStatus, 1000);
setInterval(() => { if ($("page-dispatch").classList.contains("active")) refreshDispatch(); }, 1000);
setInterval(() => { if ($("page-logs").classList.contains("active")) loadLogs().catch(() => {}); }, 1500);
refreshStatus();

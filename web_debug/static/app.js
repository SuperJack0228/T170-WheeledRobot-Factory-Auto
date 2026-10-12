"use strict";

const qs = new URLSearchParams(location.search);
const queryToken = qs.get("token");
if (queryToken) localStorage.setItem("t170_token", queryToken);
const token = queryToken || localStorage.getItem("t170_token") || "";
if (queryToken && history.replaceState) history.replaceState(null, "", location.pathname);

const $ = (id) => document.getElementById(id);
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
  if (name === "logs") loadLogs();
}

async function refreshStatus() {
  try {
    const health = await api("/api/health");
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
    const hold = `${data.grasped_right ? "右" : ""}${data.grasped_left ? "左" : ""}`;
    $("metricHold").textContent = hold || "空";
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
  const data = await api(`/api/fields?file=${encodeURIComponent(configFile)}`);
  $("configNote").textContent = data.note || "";
  document.querySelectorAll(".file-tab").forEach((el) => el.classList.toggle("active", el.dataset.file === configFile));
  $("fieldForm").innerHTML = (data.groups || []).map((group) => `
    <h2 class="group">${group.title}</h2>
    <article class="card">
      ${group.fields.map((field) => `
        <label class="field">
          <span>${field.label}${field.unit ? `<small>${field.unit}</small>` : ""}</span>
          <input data-key="${field.key}" value="${field.value ?? ""}">
        </label>`).join("")}
    </article>`).join("");
}

async function loadVision() {
  const image = $("visionImage");
  image.onload = () => image.classList.add("show");
  image.onerror = () => image.classList.remove("show");
  image.src = `/api/image/latest?slot=head&kind=processed&token=${encodeURIComponent(token)}&t=${Date.now()}`;
}

async function loadLogs() {
  const data = await api(`/api/logs?since=${logSeq}`);
  logSeq = data.seq || logSeq;
  const view = $("logView");
  const fresh = (data.lines || []).map((line) => `${line.time}  ${line.text}`).join("\n");
  if (!$("page-logs").dataset.seeded) {
    view.textContent = (data.startup_tail || "").trim();
    $("page-logs").dataset.seeded = "1";
  }
  if (fresh) view.textContent += (view.textContent ? "\n" : "") + fresh;
  view.scrollTop = view.scrollHeight;
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
  if (!window.confirm("保存这些参数？")) return;
  const values = {};
  document.querySelectorAll("#fieldForm input").forEach((input) => { values[input.dataset.key] = input.value.trim(); });
  try {
    const result = await api("/api/fields", {
      method: "POST",
      body: { file: configFile, values, confirm: "save_config" },
    });
    toast(result.reload ? "已保存并重载" : (result.error || "已保存，重启后生效"));
  } catch (error) { toast(error.message); }
});

setInterval(refreshStatus, 1000);
setInterval(() => { if ($("page-logs").classList.contains("active")) loadLogs().catch(() => {}); }, 1500);
refreshStatus();

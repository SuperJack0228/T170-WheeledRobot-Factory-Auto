(() => {
  const $ = (sel, el = document) => el.querySelector(sel);
  const $$ = (sel, el = document) => [...el.querySelectorAll(sel)];

  const state = {
    library: [],
    steps: [],
    selectedId: null,
    status: "idle",
    currentId: null,
    robot: null,
    logs: [],
    name: "untitled",
    editBoot: false,
    mainSteps: [],
    mainName: "untitled",
    approachPaths: [],
    pathTeach: { name: "left_pick", paths: {}, selectedWp: 0 },
  };

  const POSE_KEYS = ["x", "y", "z", "rx", "ry", "rz"];
  const DEF_L = { x: 0.45, y: 0.36, z: -0.15, rx: 90, ry: 0, rz: 0 };
  const DEF_R = { x: 0.45, y: -0.36, z: -0.15, rx: -90, ry: 0, rz: 0 };

  function toast(msg) {
    const el = $("#toast");
    el.textContent = msg;
    el.hidden = false;
    clearTimeout(toast._t);
    toast._t = setTimeout(() => { el.hidden = true; }, 3200);
  }

  async function api(path, opts) {
    const res = await fetch(path, {
      headers: { "Content-Type": "application/json" },
      ...opts,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      const err = data.error || res.statusText;
      throw new Error(err);
    }
    return data;
  }

  function kindClass(step) {
    if (step.kind === "arm") {
      if (step.left && step.right) return "arm both";
      if (step.left) return "arm left-only";
      return "arm";
    }
    return step.kind;
  }

  function labelOf(step) {
    if (step.kind === "arm") {
      if (step.left && step.right) return "双臂直线";
      if (step.left) return "左臂直线";
      return "右臂直线";
    }
    if (step.kind === "gripper") {
      const bits = [];
      if (step.left) bits.push("左" + (step.left === "open" ? "开" : "握"));
      if (step.right) bits.push("右" + (step.right === "open" ? "开" : "握"));
      return "夹爪 " + bits.join(" / ");
    }
    const map = {
      sleep: "等待",
      home: "双手回初始",
      vision_grasp: "视觉抓取",
      vision_detect: "视觉检测",
      vision_grasp_factory: "视觉抓取(新模型)",
      vision_detect_jindi: "视觉检测(金帝四类)",
      vision_grasp_jindi: "视觉抓取(金帝·毛胚)",
      aruco_detect: "二维码位姿",
      conveyor_goto: "传送带上方",
      aruco_above: "二维码上方",
      hand_grasp: "手部相机识别和抓取",
      hand_grasp_factory: "手部相机识别和抓取(小毛胚)",
      vision_place: "视觉放货",
      head: "头部姿态",
      modbus_wait: "等待机床信号",
      modbus_send: "发送机床信号",
    };
    if (step.kind === "waist") {
      const m = { rotate: "转腰", fold: "腰部折叠", lift: step.delta_m >= 0 ? "上升" : "下降", advance: "前进", goto: "腰到坐标" };
      return m[step.mode] || "腰部";
    }
    if (step.kind === "conveyor_goto") {
      const sideT = step.side === "left" ? "左侧" : "右侧";
      let t;
      if (step.arm === "left") t = `传送带${sideT}上方(左臂)`;
      else if (step.arm === "right") t = `传送带${sideT}上方(右臂)`;
      else t = `传送带${sideT}上方`;
      if (step.path) t += ` · ${step.path}`;
      return t;
    }
    if (step.kind === "aruco_above") {
      if (step.arm === "left") return "二维码上方(左臂)";
      if (step.arm === "right") return "二维码上方(右臂)";
      return "二维码上方";
    }
    if (step.kind === "modbus_wait") {
      return step.signal === 20 ? "等待下料许可(20)" : "等待上料许可(10)";
    }
    if (step.kind === "modbus_send") {
      return `发送机床信号(${step.value})`;
    }
    return map[step.kind] || step.kind;
  }

  function fmtPose(p) {
    if (!p) return "—";
    return `${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)}  /  ${p.rx}°, ${p.ry}°, ${p.rz}°`;
  }

  function poseInputs(prefix, pose) {
    return POSE_KEYS.map((k) => {
      const step = ["rx", "ry", "rz"].includes(k) ? "1" : "0.01";
      return `<label>${k}<input data-pose="${prefix}.${k}" type="number" step="${step}" value="${pose[k]}" draggable="false"></label>`;
    }).join("");
  }

  function renderLib() {
    const root = $("#libList");
    const groups = [];
    for (const item of state.library) {
      let g = groups.find((x) => x.name === item.group);
      if (!g) { g = { name: item.group, items: [] }; groups.push(g); }
      g.items.push(item);
    }
    root.innerHTML = groups.map((g) => `
      <div class="lib-group">${g.name}</div>
      ${g.items.map((it) => {
        let extra = it.kind;
        if (it.id === "arm_right") extra = "arm right";
        if (it.id === "arm_dual") extra = "arm dual";
        if (it.id === "arm_left") extra = "arm";
        return `<div class="lib-item ${extra}" data-lib="${it.id}">
          <b>${it.title}</b>
          <small>${it.hint}</small>
        </div>`;
      }).join("")}
    `).join("");
  }

  function renderSeq() {
    const rail = $("#seqRail");
    $("#seqMeta").textContent = `${state.steps.length} 步`;
    if (!state.steps.length) {
      rail.innerHTML = `<div class="empty">从左侧点选动作。左臂+右臂会自动合成一步并双线程执行。</div>`;
      return;
    }
    rail.innerHTML = state.steps.map((step, i) => {
      const sel = step.id === state.selectedId ? "sel" : "";
      const run = step.id === state.currentId ? "run" : "";
      return `<article class="card ${kindClass(step)} ${sel} ${run}" data-id="${step.id}">
        <div class="cue" draggable="true" title="拖这里排序">Q${String(i + 1).padStart(2, "0")}</div>
        <div class="card-h">
          <strong draggable="true" title="拖标题排序">${labelOf(step)}</strong>
          <button class="btn tiny" data-act="up">上</button>
          <button class="btn tiny" data-act="down">下</button>
          <button class="btn tiny" data-act="del">删</button>
        </div>
        <div class="chips-mini">${chips(step)}</div>
        ${editor(step)}
        <input class="note" data-field="note" placeholder="备注" value="${escapeAttr(step.note || "")}">
      </article>`;
    }).join("");
  }

  function chips(step) {
    if (step.kind === "arm") {
      const bits = [];
      if (step.left) bits.push(`<span class="tag">L ${fmtPose(step.left)}</span>`);
      if (step.right) bits.push(`<span class="tag">R ${fmtPose(step.right)}</span>`);
      bits.push(`<span class="tag">${step.speed} m/s</span>`);
      return bits.join("");
    }
    if (step.kind === "waist") {
      if (step.mode === "rotate" || step.mode === "fold")
        return `<span class="tag">${step.angle_deg}°</span><span class="tag">${step.speed_deg_s}°/s</span>`;
      if (step.mode === "goto" && step.pose)
        return `<span class="tag">${fmtPose(step.pose)}</span><span class="tag">${step.speed} m/s</span>`;
      return `<span class="tag">${step.delta_m} m</span><span class="tag">${step.speed} m/s</span>`;
    }
    if (step.kind === "sleep") return `<span class="tag">${step.seconds}s</span>`;
    if (step.kind === "head") return `<span class="tag">${step.m0}° / ${step.m1}° / ${step.m2}°</span>`;
    if (step.kind === "vision_grasp") return `<span class="tag">C++ 抓取流程</span>`;
    if (step.kind === "vision_detect") return `<span class="tag">单类 best.pt · 只识别</span>`;
    if (step.kind === "vision_grasp_factory") return `<span class="tag">单类 best.pt · 抓取</span>`;
    if (step.kind === "vision_detect_jindi") return `<span class="tag">金帝四类 · 毛胚/半加工/加工/孔</span>`;
    if (step.kind === "vision_grasp_jindi") return `<span class="tag">金帝四类 · 只抓毛胚</span>`;
    if (step.kind === "aruco_detect") return `<span class="tag">ArUco 码ID→边长 · PnP</span>`;
    if (step.kind === "conveyor_goto") {
      const side = step.side === "left" ? "码左侧" : "码右侧";
      const arm = step.arm === "left" ? "左臂" : (step.arm === "right" ? "右臂" : (step.hand === "empty" ? "空手自动" : "持料自动"));
      const bits = [`<span class="tag">${side}</span><span class="tag">${arm}</span>`];
      if (step.path) bits.push(`<span class="tag">路径 ${step.path}</span>`);
      return bits.join("");
    }
    if (step.kind === "aruco_above") {
      const arm = step.arm === "left" ? "左臂" : (step.arm === "right" ? "右臂" : (step.hand === "empty" ? "空手自动" : "持料自动"));
      const bits = [`<span class="tag">码正上方 ~5cm</span><span class="tag">${arm}</span>`];
      if (step.path) bits.push(`<span class="tag">路径 ${step.path}</span>`);
      return bits.join("");
    }
    if (step.kind === "hand_grasp") return `<span class="tag">手相机 · 4类 · 不回待命</span>`;
    if (step.kind === "hand_grasp_factory") return `<span class="tag">手相机 · 小毛胚 · 不回待命</span>`;
    if (step.kind === "vision_place") return `<span class="tag">4类 class1 · 料盘空位</span>`;
    if (step.kind === "modbus_wait") {
      const host = step.host || (step.signal === 20 ? "89" : "88");
      const addr = step.addr || (step.signal === 20 ? 2200 : 2100);
      return `<span class="tag">读 ${host}:${addr} = ${step.signal}</span><span class="tag">${step.timeout_sec}s</span>`;
    }
    if (step.kind === "modbus_send") {
      const bits = [`<span class="tag">写 ${step.value}</span>`];
      if (step.host) bits.push(`<span class="tag">${step.host}</span>`);
      if (step.addr) bits.push(`<span class="tag">reg ${step.addr}</span>`);
      return bits.join("");
    }
    return "";
  }

  function editor(step) {
    if (step.kind === "arm") {
      const speed = `<div class="fields"><label>速度 m/s<input data-field="speed" type="number" step="0.01" value="${step.speed}" draggable="false"></label></div>`;
      const l = step.left ? `<div class="side-block"><h4>左臂</h4><div class="fields">${poseInputs("left", step.left)}</div><div class="row"><button class="btn tiny" data-act="fillArm" data-side="left" type="button">填入当前左臂 TCP</button></div></div>` : "";
      const r = step.right ? `<div class="side-block"><h4>右臂</h4><div class="fields">${poseInputs("right", step.right)}</div><div class="row"><button class="btn tiny" data-act="fillArm" data-side="right" type="button">填入当前右臂 TCP</button></div></div>` : "";
      return speed + l + r;
    }
    if (step.kind === "gripper") {
      return `<div class="row">
        <label>左 <select data-field="left"><option value="">关</option>
          <option ${step.left === "open" ? "selected" : ""} value="open">松开</option>
          <option ${step.left === "close" ? "selected" : ""} value="close">握住</option></select></label>
        <label>右 <select data-field="right"><option value="">关</option>
          <option ${step.right === "open" ? "selected" : ""} value="open">松开</option>
          <option ${step.right === "close" ? "selected" : ""} value="close">握住</option></select></label>
      </div>`;
    }
    if (step.kind === "waist" && step.mode === "goto") {
      const p = step.pose || { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
      return `<div class="fields">${poseInputs("pose", p)}</div>
        <div class="fields"><label>速度 m/s<input data-field="speed" type="number" step="0.01" value="${step.speed || 0.1}" draggable="false"></label></div>
        <div class="row"><button class="btn tiny" data-act="fillWaist" type="button">填入当前腰坐标</button></div>`;
    }
    if (step.kind === "waist" && (step.mode === "rotate" || step.mode === "fold")) {
      return `<div class="fields">
        <label>角度 °<input data-field="angle_deg" type="number" step="1" value="${step.angle_deg}"></label>
        <label>速度 °/s<input data-field="speed_deg_s" type="number" step="1" value="${step.speed_deg_s}"></label>
      </div>`;
    }
    if (step.kind === "waist") {
      return `<div class="fields">
        <label>位移 m<input data-field="delta_m" type="number" step="0.01" value="${step.delta_m}"></label>
        <label>速度 m/s<input data-field="speed" type="number" step="0.01" value="${step.speed}"></label>
      </div>`;
    }
    if (step.kind === "sleep") {
      return `<div class="fields"><label>秒<input data-field="seconds" type="number" step="0.1" value="${step.seconds}"></label></div>`;
    }
    if (step.kind === "head") {
      return `<div class="fields">
        <label>M0 °<input data-field="m0" type="number" step="1" value="${step.m0}"></label>
        <label>M1 °<input data-field="m1" type="number" step="1" value="${step.m1}"></label>
        <label>M2 °<input data-field="m2" type="number" step="1" value="${step.m2}"></label>
      </div>`;
    }
    if (step.kind === "modbus_wait") {
      return `<div class="fields">
        <label>等待信号
        <select data-field="signal">
          <option ${Number(step.signal) === 10 ? "selected" : ""} value="10">10 机床可以上料 · 88+89 的 2100</option>
          <option ${Number(step.signal) === 20 ? "selected" : ""} value="20">20 机床可以下料 · 88+89 的 2200</option>
        </select></label>
        <label>超时秒<input data-field="timeout_sec" type="number" step="1" value="${step.timeout_sec}"></label>
        <label>主机覆盖<input data-field="host" placeholder="空=同时轮询 88 和 89" value="${escapeAttr(step.host || "")}"></label>
        <label>寄存器覆盖<input data-field="addr" type="number" step="1" value="${step.addr || 0}"></label>
      </div>`;
    }
    if (step.kind === "modbus_send") {
      return `<div class="fields">
        <label>写出数值
        <input data-field="value" type="number" step="1" value="${step.value}">
        </label>
        <label>主机
        <select data-field="host">
          <option ${!step.host ? "selected" : ""} value="">最近许可那台（没有则 88+89 都写）</option>
          <option ${step.host === "m88" || step.host === "load" ? "selected" : ""} value="m88">192.168.1.88</option>
          <option ${step.host === "m89" || step.host === "unload" ? "selected" : ""} value="m89">192.168.1.89</option>
        </select></label>
        <label>寄存器<input data-field="addr" type="number" step="1" value="${step.addr || 0}" placeholder="0=按数值自动"></label>
      </div>
      <small class="muted">常用：35 锁上料传送带 / 15 上料完成（2150）；45 锁下料传送带 / 25 下料完成（2250）。寄存器填 0 则按数值从表里选地址。</small>`;
    }
    if (step.kind === "conveyor_goto") {
      const names = state.approachPaths || [];
      const opts = [`<option value="">(无，直达几何点)</option>`]
        .concat(names.map((n) => `<option ${step.path === n ? "selected" : ""} value="${n}">${n}</option>`))
        .join("");
      return `<div class="fields">
        <label>码的哪一侧
        <select data-field="side">
          <option ${step.side === "right" ? "selected" : ""} value="right">右侧</option>
          <option ${step.side === "left" ? "selected" : ""} value="left">左侧</option>
        </select></label>
        <label>哪只臂
        <select data-field="arm">
          <option ${step.arm === "right" ? "selected" : ""} value="right">右臂</option>
          <option ${step.arm === "left" ? "selected" : ""} value="left">左臂</option>
          <option ${!step.arm || step.arm === "auto" ? "selected" : ""} value="auto">自动(按持料)</option>
        </select></label>
        <label>接近路径
        <select data-field="path">${opts}</select></label>
      </div>`;
    }
    if (step.kind === "aruco_above") {
      const names = state.approachPaths || [];
      const opts = [`<option value="">(无，直达几何点)</option>`]
        .concat(names.map((n) => `<option ${step.path === n ? "selected" : ""} value="${n}">${n}</option>`))
        .join("");
      return `<div class="fields">
        <label>哪只臂
        <select data-field="arm">
          <option ${step.arm === "right" ? "selected" : ""} value="right">右臂</option>
          <option ${step.arm === "left" ? "selected" : ""} value="left">左臂</option>
          <option ${!step.arm || step.arm === "auto" ? "selected" : ""} value="auto">自动(按持料)</option>
        </select></label>
        <label>接近路径
        <select data-field="path">${opts}</select></label>
      </div>`;
    }
    return "";
  }

  function escapeAttr(s) {
    return String(s).replace(/"/g, "&quot;");
  }

  function appendLog(item) {
    state.logs.push(item);
    if (state.logs.length > 600) state.logs.splice(0, 200);
    const view = $("#logView");
    const line = document.createElement("div");
    line.className = item.level;
    line.textContent = `${item.ts}  ${item.level.padEnd(5)}  ${item.source.padEnd(7)}  ${item.message}`;
    view.appendChild(line);
    if (view.childElementCount > 600) view.removeChild(view.firstChild);
    view.scrollTop = view.scrollHeight;
    $("#logCount").textContent = String(state.logs.length);
  }

  function renderLogs(items) {
    const view = $("#logView");
    view.innerHTML = "";
    state.logs = [];
    for (const it of items) appendLog(it);
  }

  function setStatus(snap) {
    state.status = snap.status || "idle";
    state.currentId = snap.current_id;
    state.robot = snap.robot;
    const chip = $("#statusChip");
    const inf = Number(snap.loop_total) === 0;
    let label = (state.status || "idle").toUpperCase();
    if (state.status === "running" && snap.loop_index) {
      label = inf
        ? `RUN · ${snap.loop_index}/∞`
        : (snap.loop_total > 1 ? `RUN · ${snap.loop_index}/${snap.loop_total}` : "RUN");
    }
    chip.textContent = label;
    chip.className = "chip dim" + (state.status === "running" ? " run" : "") + (state.status === "error" ? " err" : "");
    $("#btnRun").disabled = state.status === "running" || state.status === "stopping";
    $("#btnStop").disabled = state.status !== "running" && state.status !== "stopping";
    if (state.currentId) {
      const step = state.steps.find((s) => s.id === state.currentId);
      if (step && (step.kind === "vision_grasp" || step.kind === "vision_detect" || step.kind === "vision_grasp_factory" || step.kind === "vision_detect_jindi" || step.kind === "vision_grasp_jindi" || step.kind === "vision_place" || step.kind === "aruco_detect" || step.kind === "conveyor_goto" || step.kind === "aruco_above" || step.kind === "hand_grasp" || step.kind === "hand_grasp_factory")) openVision();
    }
    $$(".card").forEach((el) => el.classList.toggle("run", el.dataset.id === state.currentId));
  }

  function isSeqEditing() {
    const el = document.activeElement;
    return !!(el && $("#seqRail") && $("#seqRail").contains(el) &&
      /^(INPUT|SELECT|TEXTAREA)$/.test(el.tagName));
  }

  let pushTimer = null;
  async function pushSequence() {
    const path = state.editBoot ? "/api/boot_sequence" : "/api/sequence";
    const body = state.editBoot
      ? { steps: state.steps }
      : { steps: state.steps, name: $("#progName").value };
    await api(path, {
      method: "POST",
      body: JSON.stringify(body),
    });
  }
  function schedulePushSequence() {
    clearTimeout(pushTimer);
    pushTimer = setTimeout(() => {
      pushSequence().catch((err) => toast(err.message));
    }, 400);
  }

  function findStep(id) {
    return state.steps.find((s) => s.id === id);
  }

  function bindRange(id, vid) {
    const a = document.getElementById(id);
    const b = document.getElementById(vid);
    const sync = () => { b.textContent = a.value; };
    a.addEventListener("input", sync);
    sync();
  }

  function makePoseEditor(root, pose) {
    root.innerHTML = POSE_KEYS.map((k) => {
      const step = ["rx", "ry", "rz"].includes(k) ? "1" : "0.01";
      return `<label>${k}<input data-k="${k}" type="number" step="${step}" value="${pose[k]}" draggable="false"></label>`;
    }).join("");
  }

  function readPose(root) {
    const o = {};
    for (const k of POSE_KEYS) o[k] = Number($( `[data-k="${k}"]`, root).value);
    return o;
  }

  async function addFromLib(libId) {
    const item = state.library.find((x) => x.id === libId);
    if (!item) return;
    try {
      const data = await api(state.editBoot ? "/api/boot_sequence/add" : "/api/sequence/add", {
        method: "POST",
        body: JSON.stringify({ step: { ...item.seed }, target_id: state.selectedId }),
      });
      state.steps = data.steps;
      if (data.message) toast(data.message);
      const last = state.steps[state.steps.length - 1];
      state.selectedId = data.merged ? (state.selectedId || last.id) : last.id;
      renderSeq();
    } catch (err) {
      toast(err.message);
    }
  }

  function openVision() {
    const dlg = $("#visionDlg");
    if (!dlg.open) dlg.show();
    refreshVisImages();
  }
  function closeVision() {
    const dlg = $("#visionDlg");
    if (dlg.open) dlg.close();
  }

  function setImg(id, url) {
    const el = $(id);
    if (!el) return;
    if (url) el.src = url;
    else el.removeAttribute("src");
  }

  async function refreshVisImages() {
    try {
      const data = await api("/api/debug_vis/latest");
      const im = data.images || {};
      setImg("#visHead", im.head);
      setImg("#visHeadOrig", im.head_orig);
      setImg("#visRight", im.right_hand);
      setImg("#visRightOrig", im.right_hand_orig);
      setImg("#visLeft", im.left_hand);
      setImg("#visLeftOrig", im.left_hand_orig);
    } catch (_) { /* ignore */ }
    if ($("#visionDlg").open && state.status === "running")
      setTimeout(refreshVisImages, 800);
  }

  function connectSSE() {
    const es = new EventSource("/api/events");
    es.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.event === "log") appendLog(msg.data);
        if (msg.event === "state") setStatus(msg.data);
        if (msg.event === "sequence") {
          if (state.editBoot) return;
          state.steps = msg.data.steps;
          if (msg.data.program_name) {
            state.name = msg.data.program_name;
            if (document.activeElement !== $("#progName"))
              $("#progName").value = state.name;
          }
          if (!isSeqEditing()) renderSeq();
        }
        if (msg.event === "boot_sequence") {
          if (!state.editBoot) return;
          state.steps = msg.data.steps;
          if (!isSeqEditing()) renderSeq();
        }
        if (msg.event === "logs_cleared") renderLogs([]);
      } catch (_) { /* ignore malformed */ }
    };
  }

  async function init() {
    const meta = await api("/api/meta");
    state.library = meta.library;
    state.approachPaths = meta.approach_paths || [];
    $("#modeChip").textContent = (meta.mode || "sim").toUpperCase();
    renderLib();

    makePoseEditor($("#poseR"), DEF_R);
    makePoseEditor($("#poseL"), DEF_L);
    makePoseEditor($("#poseW"), { x: -0.133, y: 0, z: 0.642, rx: -90, ry: -90, rz: 180 });
    bindRange("manYaw", "manYawV");
    bindRange("manPitch", "manPitchV");
    bindRange("h0", "h0v");
    bindRange("h1", "h1v");
    bindRange("h2", "h2v");

    const snap = await api("/api/state");
    state.steps = snap.steps || [];
    $("#progName").value = snap.program_name || "untitled";
    setStatus(snap);
    renderSeq();
    try {
      const hw = await api("/api/hw/snapshot");
      if (hw.tcp && hw.tcp.right) makePoseEditor($("#poseR"), hw.tcp.right);
      if (hw.tcp && hw.tcp.left) makePoseEditor($("#poseL"), hw.tcp.left);
      if (hw.waist_tcp) makePoseEditor($("#poseW"), hw.waist_tcp);
    } catch (_) { /* 硬件未就绪则用默认 */ }

    const logs = await api("/api/logs");
    renderLogs(logs.logs || []);
    connectSSE();

    $("#libList").addEventListener("click", (e) => {
      const item = e.target.closest("[data-lib]");
      if (item) addFromLib(item.dataset.lib);
    });

    $("#seqRail").addEventListener("click", async (e) => {
      const card = e.target.closest(".card");
      if (!card) return;
      const id = card.dataset.id;
      const act = e.target.dataset.act;
      state.selectedId = id;
      const idx = state.steps.findIndex((s) => s.id === id);
      if (act === "del") {
        state.steps.splice(idx, 1);
        state.selectedId = null;
        await pushSequence();
        renderSeq();
        return;
      }
      if (act === "up" && idx > 0) {
        [state.steps[idx - 1], state.steps[idx]] = [state.steps[idx], state.steps[idx - 1]];
        await pushSequence();
        renderSeq();
        return;
      }
      if (act === "down" && idx < state.steps.length - 1) {
        [state.steps[idx + 1], state.steps[idx]] = [state.steps[idx], state.steps[idx + 1]];
        await pushSequence();
        renderSeq();
        return;
      }
      if (act === "fillArm") {
        e.preventDefault();
        const side = e.target.dataset.side;
        try {
          const hw = await api("/api/hw/snapshot");
          const tcp = hw.tcp && hw.tcp[side];
          if (!tcp) { toast("读不到当前臂 TCP"); return; }
          const step = findStep(id);
          if (!step) return;
          step[side] = tcp;
          await pushSequence();
          renderSeq();
          toast(side === "left" ? "已填入当前左臂 TCP（含 rpy）" : "已填入当前右臂 TCP（含 rpy）");
        } catch (err) { toast(err.message); }
        return;
      }
      if (act === "fillWaist") {
        e.preventDefault();
        try {
          const hw = await api("/api/hw/snapshot");
          const wt = hw.waist_tcp;
          if (!wt) { toast("读不到当前腰坐标"); return; }
          const step = findStep(id);
          if (!step) return;
          step.pose = wt;
          await pushSequence();
          renderSeq();
        } catch (err) { toast(err.message); }
        return;
      }
      $$(".card").forEach((el) => el.classList.toggle("sel", el === card));
    });

    $("#seqRail").addEventListener("input", (e) => {
      const card = e.target.closest(".card");
      if (!card) return;
      const step = findStep(card.dataset.id);
      if (!step) return;
      if (e.target.dataset.field) {
        const f = e.target.dataset.field;
        let v = e.target.value;
        if (["speed", "seconds", "angle_deg", "speed_deg_s", "delta_m", "x", "y", "z", "rx", "ry", "rz", "m0", "m1", "m2", "signal", "timeout_sec", "value", "addr"].includes(f)) {
          if (v === "" || v === "-" || v === "." || v === "-.") return;
          v = Number(v);
          if (!Number.isFinite(v)) return;
        }
        if ((f === "left" || f === "right") && step.kind === "gripper") v = v || null;
        step[f] = v;
      }
      if (e.target.dataset.pose) {
        const [side, key] = e.target.dataset.pose.split(".");
        if (!step[side]) step[side] = {};
        const raw = e.target.value;
        if (raw === "" || raw === "-" || raw === "." || raw === "-.") return;
        const n = Number(raw);
        if (!Number.isFinite(n)) return;
        step[side][key] = n;
      }
      schedulePushSequence();
    });
    $("#seqRail").addEventListener("focusout", (e) => {
      if (e.relatedTarget && $("#seqRail").contains(e.relatedTarget)) return;
      clearTimeout(pushTimer);
      pushSequence().catch((err) => toast(err.message));
    });

    let dragId = null;
    let lastDragY = 0;
    let dragScrollTimer = null;
    function startDragScroll() {
      if (dragScrollTimer) return;
      dragScrollTimer = setInterval(() => {
        const rail = $("#seqRail");
        if (!rail || !dragId) return;
        const r = rail.getBoundingClientRect();
        const margin = 64;
        if (lastDragY < r.top + margin) rail.scrollTop -= 28;
        else if (lastDragY > r.bottom - margin) rail.scrollTop += 28;
      }, 40);
    }
    function stopDragScroll() {
      if (dragScrollTimer) {
        clearInterval(dragScrollTimer);
        dragScrollTimer = null;
      }
    }
    $("#seqRail").addEventListener("dragstart", (e) => {
      if (e.target.closest("input, select, textarea, button")) {
        e.preventDefault();
        return;
      }
      const handle = e.target.closest(".cue, .card-h strong");
      const card = handle && handle.closest(".card");
      if (!handle || !card) {
        e.preventDefault();
        return;
      }
      dragId = card.dataset.id;
      card.classList.add("drag");
      lastDragY = e.clientY;
      startDragScroll();
    });
    document.addEventListener("dragover", (e) => {
      if (!dragId) return;
      lastDragY = e.clientY;
    });
    $("#seqRail").addEventListener("dragend", (e) => {
      const card = e.target.closest(".card");
      if (card) card.classList.remove("drag");
      $$(".card.drop-over").forEach((el) => el.classList.remove("drop-over"));
      dragId = null;
      stopDragScroll();
    });
    $("#seqRail").addEventListener("dragover", (e) => {
      e.preventDefault();
      const over = e.target.closest(".card");
      $$(".card").forEach((el) => el.classList.toggle("drop-over", over && el === over && el.dataset.id !== dragId));
    });
    $("#seqRail").addEventListener("drop", async (e) => {
      e.preventDefault();
      const over = e.target.closest(".card");
      $$(".card.drop-over").forEach((el) => el.classList.remove("drop-over"));
      stopDragScroll();
      if (!over || !dragId || over.dataset.id === dragId) return;
      const from = state.steps.findIndex((s) => s.id === dragId);
      const to = state.steps.findIndex((s) => s.id === over.dataset.id);
      if (from < 0 || to < 0) return;
      const [item] = state.steps.splice(from, 1);
      state.steps.splice(to, 0, item);
      dragId = null;
      await pushSequence();
      renderSeq();
    });

    function loopPayload() {
      if ($("#loopInf").checked) return 0;
      const n = Number($("#loopCount").value || 1);
      return Number.isFinite(n) && n > 0 ? Math.floor(n) : 1;
    }
    $("#loopInf").addEventListener("change", () => {
      $("#loopCount").disabled = $("#loopInf").checked;
    });
    $("#btnRun").onclick = async () => {
      try {
        await pushSequence();
        if (state.editBoot) {
          await api("/api/boot_sequence/run", { method: "POST" });
        } else {
          await api("/api/sequence/run", {
            method: "POST",
            body: JSON.stringify({ loops: loopPayload() }),
          });
        }
      } catch (err) { toast(err.message); }
    };
    $("#btnFromHere").onclick = async () => {
      if (state.editBoot) { toast("启动流程请用「试跑启动流程」"); return; }
      if (!state.selectedId) { toast("先选中一步"); return; }
      try {
        await pushSequence();
        await api("/api/sequence/run", {
          method: "POST",
          body: JSON.stringify({ loops: 1, from_id: state.selectedId }),
        });
      } catch (err) { toast(err.message); }
    };
    $("#btnStop").onclick = () => api("/api/sequence/stop", { method: "POST" });
    $("#btnClear").onclick = async () => {
      state.steps = [];
      state.selectedId = null;
      await pushSequence();
      renderSeq();
    };
    $("#btnSave").onclick = async () => {
      if (state.editBoot) {
        try {
          await pushSequence();
          toast("启动初始动作已保存");
        } catch (err) { toast(err.message); }
        return;
      }
      const name = $("#progName").value.trim() || "untitled";
      try {
        await pushSequence();
        await api("/api/programs", { method: "POST", body: JSON.stringify({ name }) });
        toast("已保存到 orchestrator/sequences/");
      } catch (err) { toast(err.message); }
    };
    $("#btnDownload").onclick = () => {
      const doc = { schema: 1, name: $("#progName").value, chassis: null, steps: state.steps };
      const blob = new Blob([JSON.stringify(doc, null, 2)], { type: "application/json" });
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = `${doc.name || "sequence"}.json`;
      a.click();
    };
    $("#btnLoad").onclick = async () => {
      const data = await api("/api/programs");
      const sel = $("#progList");
      sel.innerHTML = (data.programs || []).map((n) => `<option value="${n}">${n}</option>`).join("");
      $("#loadDlg").showModal();
    };
    $("#btnLoadServer").onclick = async () => {
      const name = $("#progList").value;
      if (!name) return;
      const data = await api("/api/programs/" + encodeURIComponent(name));
      state.steps = data.doc.steps;
      $("#progName").value = data.doc.name;
      $("#loadDlg").close();
      renderSeq();
    };
    $("#fileLocal").onchange = async (e) => {
      const file = e.target.files[0];
      if (!file) return;
      const text = await file.text();
      const doc = JSON.parse(text);
      state.steps = doc.steps || [];
      $("#progName").value = doc.name || file.name.replace(/\.json$/, "");
      await pushSequence();
      renderSeq();
      $("#loadDlg").close();
    };
    $("#btnLogClear").onclick = () => api("/api/logs/clear", { method: "POST" }).then(() => renderLogs([]));
    $("#btnLogExport").onclick = () => {
      const text = state.logs.map((l) => `${l.ts}\t${l.level}\t${l.source}\t${l.message}`).join("\n");
      const a = document.createElement("a");
      a.href = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
      a.download = "orchestrator.log";
      a.click();
    };
    $("#btnVisClose").onclick = () => closeVision();
    $("#btnVisOpen").onclick = () => openVision();
    $("#btnReadWaist").onclick = async () => {
      try {
        const hw = await api("/api/hw/snapshot");
        if (!hw.waist_tcp) { toast("读不到当前腰坐标（硬件未连接？）"); return; }
        makePoseEditor($("#poseW"), hw.waist_tcp);
        toast("已填入当前腰 TCP");
      } catch (err) { toast(err.message); }
    };

    async function openGraspDlg() {
      const data = await api("/api/grasp_params");
      const root = $("#graspFields");
      let group = "";
      root.innerHTML = "";
      for (const f of data.fields || []) {
        if (f.group !== group) {
          group = f.group;
          const h = document.createElement("div");
          h.className = "grasp-group";
          h.textContent = group;
          root.appendChild(h);
        }
        const val = data.values[f.id];
        const row = document.createElement("div");
        row.className = "grasp-row";
        let control = "";
        if (f.kind === "enum" && f.options) {
          const opts = f.options.map((o) => {
            const sel = String(val) === String(o.value) ? " selected" : "";
            return `<option value="${o.value}"${sel}>${o.label}</option>`;
          }).join("");
          control = `<select data-gp="${f.id}" data-kind="enum">${opts}</select>`;
        } else {
          control = `<input data-gp="${f.id}" type="number" step="${f.step}" value="${val ?? ""}">`;
        }
        row.innerHTML = `<label>${f.label}<span class="unit">${f.unit || ""}</span></label>
          ${control}
          <div class="hint">${f.hint}</div>`;
        root.appendChild(row);
      }
      $("#graspDlg").showModal();
    }
    $("#btnGraspParams").onclick = () => openGraspDlg().catch((err) => toast(err.message));
    $("#btnGraspSave").onclick = async () => {
      const values = {};
      $$("[data-gp]").forEach((el) => {
        values[el.dataset.gp] = el.dataset.kind === "enum" || el.tagName === "SELECT"
          ? el.value
          : Number(el.value);
      });
      try {
        await api("/api/grasp_params", { method: "POST", body: JSON.stringify({ values }) });
        toast("抓取参数已保存并让硬件重载");
        $("#graspDlg").close();
      } catch (err) { toast(err.message); }
    };

    function fmt4(n) {
      const v = Number(n);
      return Number.isFinite(v) ? v.toFixed(4) : "0";
    }
    function currentTeachPath() {
      return state.pathTeach.paths[state.pathTeach.name] || null;
    }
    function fillPathSelect() {
      const sel = $("#pathSel");
      const names = Object.keys(state.pathTeach.paths || {});
      state.approachPaths = names;
      sel.innerHTML = names.map((n) => `<option value="${n}">${n}</option>`).join("");
      if (!names.includes(state.pathTeach.name) && names.length)
        state.pathTeach.name = names[0];
      sel.value = state.pathTeach.name;
    }
    function renderPathWaypoints() {
      const p = currentTeachPath();
      const body = $("#pathWpBody");
      if (!p) { body.innerHTML = ""; return; }
      const wps = p.waypoints || [];
      body.innerHTML = wps.map((w, i) => {
        const sel = i === state.pathTeach.selectedWp ? "sel" : "";
        return `<tr class="${sel}" data-wp="${i}">
          <td>${i + 1}</td>
          <td><input class="name" data-wk="name" value="${escapeAttr(w.name || "")}"></td>
          <td><input data-wk="x" type="number" step="0.001" value="${fmt4(w.x)}"></td>
          <td><input data-wk="y" type="number" step="0.001" value="${fmt4(w.y)}"></td>
          <td><input data-wk="z" type="number" step="0.001" value="${fmt4(w.z)}"></td>
          <td><input data-wk="rx" type="number" step="0.1" value="${fmt4(w.rx)}"></td>
          <td><input data-wk="ry" type="number" step="0.1" value="${fmt4(w.ry)}"></td>
          <td><input data-wk="rz" type="number" step="0.1" value="${fmt4(w.rz)}"></td>
          <td><button class="btn tiny" data-wpdel="${i}" type="button">删</button></td>
        </tr>`;
      }).join("");
    }
    function applyPathMetaToForm() {
      const p = currentTeachPath() || {};
      $("#pathArm").value = p.arm === "right" ? "right" : "left";
      $("#pathSide").value = p.side || "left";
      $("#pathMidA").value = p.marker_id || 0;
      $("#pathMidB").value = p.marker_id_b || 0;
      $("#pathPosTh").value = p.dual_pos_thresh_m ?? 0.015;
      $("#pathRotTh").value = p.dual_rot_thresh_deg ?? 3;
      $("#pathIncTgt").checked = p.include_target !== false;
      renderPathWaypoints();
    }
    function metaFromForm() {
      const p = currentTeachPath() || { waypoints: [] };
      return {
        note: p.note || "",
        arm: $("#pathArm").value,
        side: $("#pathSide").value,
        include_target: $("#pathIncTgt").checked,
        marker_id: Number($("#pathMidA").value) || 0,
        marker_id_b: Number($("#pathMidB").value) || 0,
        dual_pos_thresh_m: Number($("#pathPosTh").value),
        dual_rot_thresh_deg: Number($("#pathRotTh").value),
        dual_rel: p.dual_rel || null,
        waypoints: p.waypoints || [],
      };
    }
    async function loadTeachPaths() {
      const data = await api("/api/approach_paths");
      state.pathTeach.paths = data.paths || {};
      fillPathSelect();
      applyPathMetaToForm();
    }
    async function openPathDlg() {
      await loadTeachPaths();
      $("#pathDlg").showModal();
    }
    $("#btnPathTeach").onclick = () => openPathDlg().catch((err) => toast(err.message));
    $("#btnPathClose").onclick = () => $("#pathDlg").close();
    $("#pathSel").onchange = () => {
      state.pathTeach.name = $("#pathSel").value;
      state.pathTeach.selectedWp = 0;
      applyPathMetaToForm();
    };
    $("#btnPathCreate").onclick = async () => {
      const name = ($("#pathNewName").value || "").trim();
      if (!name) { toast("先填新路径名"); return; }
      try {
        const data = await api("/api/approach_paths", {
          method: "POST",
          body: JSON.stringify({ name, arm: $("#pathArm").value, side: $("#pathSide").value }),
        });
        state.pathTeach.paths[name] = data.path;
        state.pathTeach.name = name;
        fillPathSelect();
        applyPathMetaToForm();
        toast("已新建 " + name);
      } catch (err) { toast(err.message); }
    };
    $("#btnPathDelete").onclick = async () => {
      const name = state.pathTeach.name;
      if (!name) return;
      try {
        const data = await api("/api/approach_paths/delete", { method: "POST", body: JSON.stringify({ name }) });
        state.pathTeach.paths = data.paths || {};
        fillPathSelect();
        applyPathMetaToForm();
        toast("已删除 " + name);
      } catch (err) { toast(err.message); }
    };
    $("#btnPathSaveMeta").onclick = async () => {
      try {
        const data = await api("/api/approach_paths", {
          method: "POST",
          body: JSON.stringify({ name: state.pathTeach.name, path: metaFromForm() }),
        });
        state.pathTeach.paths[state.pathTeach.name] = data.path;
        toast("路径设置已保存");
      } catch (err) { toast(err.message); }
    };
    function setPathStatus(rec) {
      if (!rec) return;
      const ids = (rec.markers || []).map((m) => m.id).join(",");
      let t = `主码 ${rec.marker_id}`;
      if (rec.have_b) t += ` 副码 ${rec.marker_id_b}`;
      t += ` 可见 [${ids || "无"}]`;
      const rel = rec.rel || {};
      t += ` 当前相对 (${fmt4(rel.x)}, ${fmt4(rel.y)}, ${fmt4(rel.z)})`;
      $("#pathStatus").textContent = t;
    }
    $("#btnPathDetect").onclick = async () => {
      try {
        const data = await api("/api/approach_paths/detect", {
          method: "POST",
          body: JSON.stringify({
            arm: $("#pathArm").value,
            marker_id: Number($("#pathMidA").value) || 0,
            marker_id_b: Number($("#pathMidB").value) || 0,
          }),
        });
        setPathStatus(data.record);
        if (data.record && data.record.marker_id > 0 && Number($("#pathMidA").value) === 0)
          $("#pathMidA").value = data.record.marker_id;
        if (data.record && data.record.have_b && Number($("#pathMidB").value) === 0)
          $("#pathMidB").value = data.record.marker_id_b;
        toast("已拍码");
      } catch (err) { toast(err.message); }
    };
    $("#btnPathRecord").onclick = async () => {
      try {
        await api("/api/approach_paths", {
          method: "POST",
          body: JSON.stringify({ name: state.pathTeach.name, path: metaFromForm() }),
        });
        const data = await api("/api/approach_paths/record", {
          method: "POST",
          body: JSON.stringify({
            name: state.pathTeach.name,
            arm: $("#pathArm").value,
            marker_id: Number($("#pathMidA").value) || 0,
            marker_id_b: Number($("#pathMidB").value) || 0,
            waypoint_name: ($("#pathWpName").value || "").trim(),
          }),
        });
        state.pathTeach.paths[state.pathTeach.name] = data.path;
        state.pathTeach.selectedWp = Math.max(0, (data.path.waypoints || []).length - 1);
        applyPathMetaToForm();
        setPathStatus(data.record);
        $("#pathWpName").value = "";
        toast("已记录相对点");
      } catch (err) { toast(err.message); }
    };
    $("#pathWpBody").addEventListener("click", async (e) => {
      const tr = e.target.closest("tr");
      if (tr && tr.dataset.wp != null) state.pathTeach.selectedWp = Number(tr.dataset.wp);
      const del = e.target.dataset.wpdel;
      if (del != null) {
        const p = currentTeachPath();
        if (!p) return;
        p.waypoints.splice(Number(del), 1);
        try {
          const data = await api("/api/approach_paths", {
            method: "POST",
            body: JSON.stringify({ name: state.pathTeach.name, path: p }),
          });
          state.pathTeach.paths[state.pathTeach.name] = data.path;
          applyPathMetaToForm();
        } catch (err) { toast(err.message); }
        return;
      }
      renderPathWaypoints();
    });
    $("#pathWpBody").addEventListener("change", async (e) => {
      const tr = e.target.closest("tr");
      const p = currentTeachPath();
      if (!tr || !p) return;
      const i = Number(tr.dataset.wp);
      const key = e.target.dataset.wk;
      if (!key || !p.waypoints[i]) return;
      p.waypoints[i][key] = key === "name" ? e.target.value : Number(e.target.value);
      try {
        const data = await api("/api/approach_paths", {
          method: "POST",
          body: JSON.stringify({ name: state.pathTeach.name, path: p }),
        });
        state.pathTeach.paths[state.pathTeach.name] = data.path;
      } catch (err) { toast(err.message); }
    });
    $("#btnPathReplayOne").onclick = async () => {
      try {
        await api("/api/approach_paths/replay", {
          method: "POST",
          body: JSON.stringify({
            name: state.pathTeach.name,
            index: state.pathTeach.selectedWp,
            arm: $("#pathArm").value,
            side: $("#pathSide").value,
          }),
        });
        toast("已走到选中点");
      } catch (err) { toast(err.message); }
    };
    $("#btnPathReplay").onclick = async () => {
      try {
        await api("/api/approach_paths", {
          method: "POST",
          body: JSON.stringify({ name: state.pathTeach.name, path: metaFromForm() }),
        });
        await api("/api/approach_paths/replay", {
          method: "POST",
          body: JSON.stringify({
            name: state.pathTeach.name,
            arm: $("#pathArm").value,
            side: $("#pathSide").value,
            include_target: $("#pathIncTgt").checked,
          }),
        });
        toast("试走完成");
      } catch (err) { toast(err.message); }
    };

    function syncEditModeUi() {
      document.body.classList.toggle("boot-edit", state.editBoot);
      $("#progName").disabled = state.editBoot;
      $("#btnRun").textContent = state.editBoot ? "试跑启动流程" : "GO";
      $("#seqTitle").textContent = state.editBoot ? "启动初始动作" : "序列";
      $("#seqHint").textContent = state.editBoot
        ? "改双手位置会立刻写入待机位，手臂马上走过去；抓取结束也会回到这里。点「试跑启动流程」可整段重跑。"
        : "拖卡片左侧 Q 序号，或拖标题，可跨很远排序；靠近顶部/底部时列表会跟着滚。";
      $("#btnBootBack").hidden = !state.editBoot;
      $("#btnBootReset").hidden = !state.editBoot;
      $("#btnLoad").hidden = state.editBoot;
      $("#btnDownload").hidden = state.editBoot;
      $("#btnFromHere").hidden = state.editBoot;
      $$(".loop").forEach((el) => { el.hidden = state.editBoot; });
      $("#btnBootEdit").disabled = state.editBoot;
    }
    async function enterBootEdit() {
      if (state.editBoot) return;
      try {
        await pushSequence();
        state.mainSteps = state.steps;
        state.mainName = $("#progName").value;
        const data = await api("/api/boot_sequence");
        state.steps = data.steps || [];
        state.selectedId = null;
        state.editBoot = true;
        $("#progName").value = "启动初始动作";
        syncEditModeUi();
        renderSeq();
      } catch (err) { toast(err.message); }
    }
    async function leaveBootEdit() {
      if (!state.editBoot) return;
      try {
        await pushSequence();
      } catch (err) { toast(err.message); }
      state.editBoot = false;
      state.steps = state.mainSteps;
      $("#progName").value = state.mainName;
      state.selectedId = null;
      syncEditModeUi();
      renderSeq();
    }
    $("#btnBootEdit").onclick = () => enterBootEdit();
    $("#btnBootBack").onclick = () => leaveBootEdit();
    $("#btnBootReset").onclick = async () => {
      try {
        const data = await api("/api/boot_sequence/reset", { method: "POST" });
        state.steps = data.steps || [];
        renderSeq();
        toast("已恢复默认启动动作");
      } catch (err) { toast(err.message); }
    };
    syncEditModeUi();

    document.addEventListener("click", async (e) => {
      const man = e.target.dataset.man;
      if (!man) return;
      try {
        if (man === "yaw") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "waist_rotate", angle_deg: Number($("#manYaw").value), speed_deg_s: 30 }) });
        if (man === "pitch") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "waist_fold", angle_deg: Number($("#manPitch").value), speed_deg_s: 30 }) });
        if (man === "up") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "waist_lift", delta_m: 0.05, speed: 0.1 }) });
        if (man === "down") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "waist_lift", delta_m: -0.05, speed: 0.1 }) });
        if (man === "fwd") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "waist_advance", delta_m: 0.06, speed: 0.1 }) });
        if (man === "waistGoto") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "waist_goto", pose: readPose($("#poseW")), speed: 0.1 }) });
        if (man === "head") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "head", m0: Number($("#h0").value), m1: Number($("#h1").value), m2: Number($("#h2").value) }) });
        if (man === "armR") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "arm", side: "right", pose: readPose($("#poseR")), speed: 0.2 }) });
        if (man === "armL") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "arm", side: "left", pose: readPose($("#poseL")), speed: 0.2 }) });
        if (man === "openR") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "right", cmd: "open" }) });
        if (man === "closeR") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "right", cmd: "close" }) });
        if (man === "openL") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "left", cmd: "open" }) });
        if (man === "closeL") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "left", cmd: "close" }) });
        if (man === "openBoth") {
          await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "right", cmd: "open" }) });
          await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "left", cmd: "open" }) });
        }
        if (man === "closeBoth") {
          await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "right", cmd: "close" }) });
          await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "gripper", side: "left", cmd: "close" }) });
        }
        if (man === "home") await api("/api/manual", { method: "POST", body: JSON.stringify({ target: "home" }) });
      } catch (err) { toast(err.message); }
    });
  }

  init().catch((err) => toast("启动失败: " + err.message));
})();

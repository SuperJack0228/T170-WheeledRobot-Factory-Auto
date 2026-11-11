"use strict";
const $ = id => document.getElementById(id);
const params = new URLSearchParams(location.search);
const supplied = params.get("token");
if (supplied) {localStorage.setItem("t170_token", supplied);history.replaceState(null,"",location.pathname);}
const token = supplied || localStorage.getItem("t170_token") || "";
const labels = ["选择与准备", "装入零件", "识别目标 · 去D点", "初始引导补偿", "C点精确对准", "记录孔心参考", "持件回放验证", "保存完成"];
let state = {active:false, step:0, phase:"idle"}, busy=false, viewStep=null, selected=null, choice="", jogSize="0.2";
const esc = v => String(v??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
const fmt = (n,d=2)=>Number.isFinite(Number(n))?Number(n).toFixed(d):"—";
async function api(path,body) {
 const r=await fetch(path,{method:body?"POST":"GET",headers:{"X-T170-Token":token,...(body?{"Content-Type":"application/json"}:{})},body:body?JSON.stringify(body):undefined,cache:"no-store"});
 const data=await r.json();if(!r.ok)throw Error(data.error||`HTTP ${r.status}`);return data;
}
function error(message="") {$("error").hidden=!message;$("error").textContent=message;}
function button(action,text,primary=false,disabled=false) {return `<button data-action="${action}" class="${primary?"primary":""}" ${disabled?"disabled":""}>${text}</button>`;}
function confirm(id,text) {return `<label class="checks"><input type="checkbox" id="${id}">${text}</label>`;}
function image(interactive=true) {return `<div class="image-wrap"><img id="frame" style="cursor:${interactive?"crosshair":"default"}" alt="保持位姿后拍摄的固定标定图；点击选择目标孔"><canvas id="cross"></canvas></div>`;}
function observation() {return `<div class="card"><h3>手相机 · 最近固定画面</h3><p class="muted">每次微调后自动更新。画面只供观察，此处不标参考点。</p>${button("observe","刷新手相机画面")}${state.image_revision?image(false):""}</div>`;}
function metrics() {return `<div class="grid"><div><div class="label">D点记录补偿 · X / Y</div><div class="metric">${(state.coarse_xy_mm||[]).map(v=>fmt(v)).join(" / ")} <small>mm</small></div></div><div><div class="label">最终对准补偿 · X / Y</div><div class="metric">${(state.compensation_xy_mm||[]).map(v=>fmt(v)).join(" / ")} <small>mm</small></div></div></div>`;}
function jog() {return `<div class="field"><label for="stepSize">固定TCP姿态，仅平移基座XY</label><select id="stepSize">${["0.2","1","2"].map(v=>`<option value="${v}" ${jogSize===v?"selected":""}>${v} mm</option>`).join("")}</select></div><div class="jog"><span></span><button data-jog="x,1">前 +X</button><span></span><button data-jog="y,1">左 +Y</button><span class="space">TCP</span><button data-jog="y,-1">右 −Y</button><span></span><button data-jog="x,-1">后 −X</button><span></span></div><p class="muted">每次点击执行一个位移并停稳，不使用长按连续运动。</p>`;}
function render() {
 const page=viewStep??state.step??0, readonly=!!state.active&&page!==state.step;
 $("steps").innerHTML=labels.map((s,i)=>`<li class="${i===page?"active":i<(state.step||0)?"done":""}"><i>${i<(state.step||0)?"✓":i+1}</i><span>${s}</span></li>`).join("");
 $("sessionBar").innerHTML=state.active?`<span>${state.hand?"右手":"左手"}${state.row?` · 第${state.row}行 第${state.col}列`:""} · 会话 ${esc(state.session)} · ${busy?"动作执行中":"等待操作"}</span><button id="exit" class="quiet">退出标定</button>`:"<span>机器人本地标定 · 不移动底盘</span>";
 $("progress").textContent=`${page+1} / ${labels.length}`;$("back").disabled=page===0;$("resume").hidden=viewStep===null;
 let body=`<div class="eyebrow">STEP ${String(page+1).padStart(2,"0")}</div><h2>${labels[page]}</h2>`;
 if(readonly)body+='<div class="readOnly">正在查看步骤说明。动作以机器人当前步骤为准，点击底部返回当前步骤。</div>';
 if(state.active&&state.interrupted)body+='<div class="readOnly">本会话已停止。手臂不会自动回位，夹爪不会自动松开。请退出会话后检查现场。</div>';
 const phase=state.phase;
 if(page===0) {
  body+='<p class="lead">机器人已位于料盘二工位。选择一只手和实际生产使用的放置姿态，另一只手不下发手臂轨迹；准备过程会调整头部和腰部。</p>';
  if(!state.active)body+=`<div class="card grid"><div class="field"><label for="hand">标定手臂</label><select id="hand"><option value="1">右手</option><option value="0">左手</option></select></div><div class="field"><label for="group">放置姿态组</label><select id="group"><option value="1">ready1 · 第3/6行（按远近）</option><option value="2">ready2 · 第2/5行</option><option value="3">ready3 · 第4行</option><option value="6">ready6 · 最远第1行</option></select></div><div class="field"><label for="zone">观察区域</label><select id="zone"><option value="near">近三排</option><option value="far">远三排（腰前伸）</option></select></div><div class="field"><label for="clearance">D在生产C上方</label><select id="clearance"><option value="20">20 mm（默认）</option><option value="15">15 mm</option><option value="10">10 mm</option></select></div></div>${confirm("workspace","已确认机器人位于料盘二工位，当前姿态与活动空间适合标定。")}<div class="actions">${state.software_stopped?button("reset_stop","确认现场后解除软件停止",true):button("begin","建立标定会话",true)}</div>`;
  else body+=`<div class="card"><p>先到所选手的放置准备姿态，再调整腰部到观察位，最后张开所选夹爪。</p><p class="muted">D点默认上抬距离：${fmt(state.d_clearance_mm,0)} mm。准备期间不要装入零件。</p>${button("prepare","移动至准备位并张爪",true)}</div>`;
 } else if(page===1) body+=`<p class="lead">人工将实际生产零件放入夹爪，夹持深度和方向与生产一致。点击后按反馈力矩夹持，接触后保持当前位置。</p><div class="card">${confirm("inserted","零件已放好，可以合爪。")}${button("grip","按反馈合爪并检查持件",true)}</div>`;
 else if(page===2) body+=`<p class="lead">先识别料盘二维码与空孔，人工确认目标行列。机器人先到观察高度，再竖直到D；不使用历史经验XY补偿，不自动松爪。</p><div class="card">${button("detect","重新识别料盘")}${state.holes?.length?`<div class="field"><label for="hole">选择目标空孔（实际确认该孔为空）</label><select id="hole">${state.holes.filter(h=>h.class_id===1).map(h=>`<option value="${h.row},${h.col}" ${choice===`${h.row},${h.col}`?"selected":""}>第${h.row}行 第${h.col}列 · X ${fmt(h.x*1000,1)} / Y ${fmt(h.y*1000,1)} mm</option>`).join("")}</select></div>${button("move_d","移动到所选孔上方D点",true)}`:'<p>识别后将在这里列出空孔。行列编号与料盘二维码定义一致。</p>'}</div>`;
 else if(page===3) body+=`<p class="lead">当前停在C上方 ${fmt(state.d_clearance_mm,0)} mm 的D点。只做XY初步对准；这里记录的是初始引导补偿，随后C点精调结果会纳入最终补偿。</p><div class="card">${jog()}${button("record_coarse","记录D点初始补偿",true)}</div>`;
 else if(page===4) body+=`<p class="lead">使用固定的生产C高度，保持TCP姿态。先下放，再精调XY，确认零件与实际目标孔居中，不以“能掉进去”代替居中。</p><div class="card">${metrics()}${phase==="coarse_recorded"?`<div class="actions">${button("lower_c","保持当前XY，下放到C",true)}</div>`:`${jog()}${confirm("centered","已确认零件与目标孔实际居中。")}${button("record_aligned","记录正确对准位姿",true)}`}</div>`;
 else if(page===5) body+=`<p class="lead">保持已记录的手臂位置，松爪并取走零件，让目标孔可见。点击图像选择孔，程序定位橙色孔环中心，自动记录像素和实际TCP高度。</p><div class="card">${phase==="aligned"?button("release","保持手臂，张爪",true):`${confirm("removed","零件已取走，未推动夹爪，目标孔可见。")}${button("capture",phase==="captured"?"重新拍摄固定图像":"拍摄固定标定图",true)}`}${phase==="captured"?`${image()}<p id="selection" class="muted">点击与刚才对准的同一个目标孔中心附近。</p>${button("reference","定位并记录孔心参考",true,!selected)}`:""}</div>`;
 else if(page===6) {
  body+=`<p class="lead">草案已生成，尚未替换正式配置。重新装件，用草案补偿和参考执行一次生产伺服链路，再确认真实放置结果。</p><div class="card">${metrics()}<p>参考像素：<b>${(state.reference_px||[]).map(v=>fmt(v)).join(", ")}</b> · 参考TCP距盘面：<b>${fmt(state.reference_tcp_above_plane_mm)} mm</b></p>`;
  if(phase==="reference")body+=button("verify_return","竖直抬离并返回准备位",true);
  else if(phase==="verify_ready")body+=`${confirm("inserted","已重新放入同类零件，可以合爪。")}${button("grip","按反馈合爪",true)}`;
  else if(phase==="verify_held")body+=button("verify_run","重新识别并执行单手伺服放置",true);
  else if(phase==="verify_at_c")body+=`<p>回放已到C，仍保持夹持。可以先观察对准情况，再张爪检查实物。</p>${button("verify_release","保持位置，张爪检查",true)}`;
  else if(phase==="verify_released")body+=`${confirm("verified","实际零件放置居中，本次回放验证通过。")}${button("confirm_verified","确认通过并抬离返回",true)}`;
  else if(phase==="verified")body+=`<div class="success">持件回放与实物确认已通过。</div><div class="actions">${button("save","保存并应用这只手的标定",true)}</div>`;
  body+='</div><div class="note">如果实物仍偏，请退出并重新示教，不把“伺服执行成功”当作“标定已准确”。</div>';
 } else body+=`<div class="success"><h3>${state.hand?"右手":"左手"}标定已保存并加载</h3><p>旧标定已备份。生产参数文件保持原样，独立标定文件在启动和重新加载配置时生效。</p></div><div class="card">${metrics()}<p>像素参考：${(state.reference_px||[]).map(v=>fmt(v)).join(", ")}；TCP距盘面 ${fmt(state.reference_tcp_above_plane_mm)} mm。</p><p class="muted">本次标定限定于所验证的俯仰/横滚姿态。改变相机、工具或夹持方式后应重新标定。</p><p>标定文件：<code>${esc(state.saved_file)}</code></p><p>日志：<code>${esc(state.log_directory)}</code></p>${button("end","结束本只手标定",true)}</div>`;
 if(page===2&&state.head_image_revision)body+=`<div class="card"><h3>头相机 · 目标行列参考</h3><img style="width:100%;height:auto" alt="头相机料盘孔位编号图" src="/api/calibration/image?kind=head&session=${encodeURIComponent(state.session)}&revision=${state.head_image_revision}&token=${encodeURIComponent(token)}"></div>`;
 if(page===3||(page===4&&phase==="at_c"))body+=observation();
 if(state.active&&state.current_tcp)body+=`<div class="muted">最近实际TCP · X ${fmt(state.current_tcp[0]*1000)} / Y ${fmt(state.current_tcp[1]*1000)} / Z ${fmt(state.current_tcp[2]*1000)} mm</div>`;
 $("content").innerHTML=body;
 if($("stepSize"))$("stepSize").onchange=e=>{jogSize=e.target.value;};
 document.querySelectorAll("[data-action]").forEach(b=>{b.disabled ||= busy||readonly||(state.active&&state.interrupted);b.onclick=()=>run(b.dataset.action);});
 document.querySelectorAll("[data-jog]").forEach(b=>{b.disabled=busy||readonly||(state.active&&state.interrupted);b.onclick=()=>{const [axis,sign]=b.dataset.jog.split(",");run("jog",{[axis==="x"?"dx_mm":"dy_mm"]:Number(sign)*Number($("stepSize").value)});};});
 if($("exit"))$("exit").onclick=()=>$("exitDialog").showModal();
 if($("hole"))$("hole").onchange=e=>{choice=e.target.value;};
 if($("frame")) {
  $("frame").src=`/api/calibration/image?session=${encodeURIComponent(state.session)}&revision=${state.image_revision}&token=${encodeURIComponent(token)}`;
  $("frame").onload=draw;
  $("frame").onclick=e=>{if(busy||readonly||(state.active&&state.interrupted)||state.phase!=="captured")return;const box=e.currentTarget.getBoundingClientRect();selected={u:(e.clientX-box.left)*state.width/box.width,v:(e.clientY-box.top)*state.height/box.height,revision:state.image_revision};draw();$("selection").textContent=`已选 ${fmt(selected.u,1)}, ${fmt(selected.v,1)}，将自动定位附近孔环中心。`;document.querySelector('[data-action="reference"]').disabled=false;};
 }
}
function draw() {const c=$("cross"),img=$("frame");if(!c||!img)return;c.width=state.width;c.height=state.height;const ctx=c.getContext("2d");if(selected?.revision===state.image_revision){ctx.strokeStyle="#00e4ce";ctx.lineWidth=3;ctx.beginPath();ctx.arc(selected.u,selected.v,18,0,2*Math.PI);ctx.moveTo(selected.u-25,selected.v);ctx.lineTo(selected.u+25,selected.v);ctx.moveTo(selected.u,selected.v-25);ctx.lineTo(selected.u,selected.v+25);ctx.stroke();}}
async function run(action,extra={}) {
 if(busy)return;error();
 try {
  if(action==="reset_stop") {if(!$("workspace")?.checked)throw Error("请先确认现场与机器人状态");extra.acknowledged=true;}
  if(action==="begin") {if(!$("workspace").checked)throw Error("请先确认工位和活动空间");extra={hand:Number($("hand").value),row_group:Number($("group").value),clearance_mm:Number($("clearance").value),far_rows:$("zone").value==="far"};}
  if(action==="grip"&&!$("inserted")?.checked)throw Error("请确认零件已放好");
  if(action==="move_d") {const [row,col]=($("hole")?.value||"").split(",").map(Number);if(!row||!col)throw Error("请选择目标空孔");extra={row,col};}
  if(action==="record_aligned") {if(!$("centered")?.checked)throw Error("请先确认实物居中");extra.physically_centered=true;}
  if(action==="capture"&&!$("removed")?.checked)throw Error("请确认零件已取走且未推动夹爪");
  if(action==="reference") {if(!selected)throw Error("请先点击目标孔");extra={u:selected.u,v:selected.v,image_revision:selected.revision};}
  if(action==="confirm_verified") {if(!$("verified")?.checked)throw Error("请先确认实物回放结果");extra.physically_centered=true;}
  const request={action,session:state.session,request_id:globalThis.crypto?.randomUUID?.()||`${Date.now()}-${Math.random()}`,...extra};
  busy=true;render();const result=await api("/api/calibration",request);
  if(result.session)state=result;if(!result.ok)throw Error(result.error||"动作未完成");
  viewStep=null;if(action==="capture")selected=null;
 } catch(e){error(e.message);} finally {busy=false;render();}
}
async function refresh() {
 try {const next=await api("/api/calibration");if(!next.ok)throw Error(next.error||"机器人版本不支持标定，请重新编译并启动");
  $("connection").textContent=`机器人已连接 · ${next.active?"标定会话有效":"空闲"}`;$("connection").className="connection";
  if(!busy){const changed=next.phase!==state.phase||next.session!==state.session||next.interrupted!==state.interrupted||next.software_stopped!==state.software_stopped;state=next;if(changed){selected=null;render();}}
 } catch(e){$("connection").textContent=e.message;$("connection").className="connection bad";}
}
$("stop").onclick=async()=>{try{await api("/api/command",{cmd:"abort"});error("已请求停止。不会自动松爪或回位；请等待动作结束，再退出当前标定会话。");await refresh();}catch(e){error(e.message);}};
$("back").onclick=()=>{viewStep=Math.max(0,(viewStep??state.step)-1);render();};
$("resume").onclick=()=>{viewStep=null;render();};
$("cancelExit").onclick=()=>$("exitDialog").close();
$("confirmExit").onclick=()=>{$("exitDialog").close();run("end");};
render();refresh();setInterval(refresh,1500);

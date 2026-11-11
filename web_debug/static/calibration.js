"use strict";
const $ = id => document.getElementById(id);
const params = new URLSearchParams(location.search), supplied = params.get("token");
if (supplied) {localStorage.setItem("t170_token", supplied);history.replaceState(null,"",location.pathname);}
const token = supplied || localStorage.getItem("t170_token") || "";
const labels = ["选择任务与准备", "识别固定空孔 · 去D点", "D点XY对准", "保持手臂 · 准备参考物", "点击视觉参考点", "保存与完成"];
let state={active:false,step:0,phase:"idle"},busy=false,viewStep=null,selected=null,jogSize="0.2",taskChoice="grasp",handChoice="1";
const esc=v=>String(v??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
const fmt=(n,d=2)=>Number.isFinite(Number(n))?Number(n).toFixed(d):"—";
const grasp=()=> (state.active?state.task:taskChoice)==="grasp";
const taskName=()=>grasp()?"料盘抓取":"料盘放置";
async function api(path,body) {
 const r=await fetch(path,{method:body?"POST":"GET",headers:{"X-T170-Token":token,...(body?{"Content-Type":"application/json"}:{})},body:body?JSON.stringify(body):undefined,cache:"no-store"});
 const data=await r.json();if(!r.ok)throw Error(data.error||`HTTP ${r.status}`);return data;
}
function error(message="") {$("error").hidden=!message;$("error").textContent=message;}
function button(action,text,primary=false,disabled=false) {return `<button data-action="${action}" class="${primary?"primary":""}" ${disabled?"disabled":""}>${text}</button>`;}
function confirm(id,text) {return `<label class="checks"><input type="checkbox" id="${id}">${text}</label>`;}
function image(interactive=true) {return `<div class="image-wrap"><img id="frame" style="cursor:${interactive?"crosshair":"default"}" alt="当前位姿的固定手相机标定图"><canvas id="cross"></canvas></div>${interactive?'<p class="muted">移动鼠标查看局部放大；点击中心。抓取记录人工点击的顶面圆心，放置自动精定位所选孔环。</p><canvas id="zoom" width="320" height="240" style="max-width:100%;border:1px solid #ccd5dd"></canvas>':""}`;}
function jog() {return `<div class="field"><label for="stepSize">保持TCP姿态与Z，只平移基座XY</label><select id="stepSize">${["0.2","1","2","5"].map(v=>`<option value="${v}" ${jogSize===v?"selected":""}>${v} mm</option>`).join("")}</select></div><div class="jog"><span></span><button data-jog="x,1">前 +X</button><span></span><button data-jog="y,1">左 +Y</button><span class="space">TCP</span><button data-jog="y,-1">右 −Y</button><span></span><button data-jog="x,-1">后 −X</button><span></span></div><p class="muted">每次点击只执行一步，完成后再接受下一步。5mm粗调，0.2/1mm精调。</p>`;}
function metrics() {return `<p>料盘引导补偿 X / Y：<b>${(state.compensation_xy_mm||[]).map(v=>fmt(v)).join(" / ")} mm</b></p><p>参考像素：<b>${(state.reference_px||[]).map(v=>fmt(v)).join(", ")}</b>；参考TCP距${grasp()?"零件顶面":"孔口平面"}：${fmt(state.reference_tcp_above_plane_mm)} mm。</p>`;}
function render() {
 const page=viewStep??state.step??0,readonly=!!state.active&&page!==state.step,phase=state.phase;
 $("steps").innerHTML=labels.map((s,i)=>`<li class="${i===page?"active":i<(state.step||0)?"done":""}"><i>${i<(state.step||0)?"✓":i+1}</i><span>${s}</span></li>`).join("");
 $("sessionBar").innerHTML=state.active?`<span>${taskName()} · ${state.hand?"右手":"左手"} · 第五行 / 从左数第${state.hand?5:2}孔 · ${busy?"动作执行中":"等待操作"}</span><button id="exit" class="quiet">退出标定</button>`:"<span>仅用于料盘 · 抓取与放置独立保存 · 不修改传送带参数</span>";
 $("progress").textContent=`${page+1} / ${labels.length}`;$("back").disabled=page===0;$("resume").hidden=viewStep===null;
 let body=`<div class="eyebrow">STEP ${String(page+1).padStart(2,"0")}</div><h2>${labels[page]}</h2>`;
 if(readonly)body+='<div class="readOnly">正在查看步骤说明。动作以机器人当前步骤为准，请返回当前步骤操作。</div>';
 if(state.active&&state.interrupted)body+='<div class="readOnly">本会话已停止。不会自动回位或松爪。请退出会话后检查现场。</div>';
 if(page===0) {
  if(!state.active) body+=`<div class="card grid"><div class="field"><label for="task">标定任务</label><select id="task"><option value="grasp" ${taskChoice==="grasp"?"selected":""}>料盘抓取 · 新机器人先做</option><option value="placement" ${taskChoice==="placement"?"selected":""}>料盘放置 · 从实际持件开始</option></select></div><div class="field"><label for="hand">标定手臂</label><select id="hand"><option value="1" ${handChoice==="1"?"selected":""}>右手 · 第五行从左数第五孔</option><option value="0" ${handChoice==="0"?"selected":""}>左手 · 第五行从左数第二孔</option></select></div><div class="field"><label for="clearance">D点在生产C上方</label><select id="clearance"><option value="20">20 mm（默认）</option><option value="15">15 mm</option><option value="10">10 mm</option></select></div></div>`;
  body+=grasp()?'<p class="lead">机器人先到料盘抓取工位，第五行清空。所选夹爪不能夹零件。准备时空爪闭合，使用第五行抓取姿态；其余行抓取姿态保持原样。</p>':'<p class="lead">先退出标定，按实际传送带抓取流程拿到零件，再到料盘放置工位。禁止人工喂件。准备动作保持夹持，所有行放置统一采用第五行姿态。</p>';
  if(!state.active) body+=`${confirm("workspace",grasp()?"已确认料盘抓取工位、第五行无零件、所选夹爪为空且活动空间足够。":"已确认料盘放置工位、第五行无零件，所选手已通过实际传送带抓取持件且活动空间足够。")}${button(state.software_stopped?"reset_stop":"begin",state.software_stopped?"确认现场后解除软件停止":"建立标定会话",true)}`;
  else body+=`<div class="card">${confirm("setup","第五行已清空、空间足够；所选手满足当前任务的空爪/实际持件条件。")}${button("prepare",grasp()?"进入准备位并空爪闭合":"保持持件，进入准备位",true)}</div>`;
 } else if(page===1) {
  body+=`<p class="lead">头相机识别空孔。自动选择机器人视角第五行、从左数第${state.hand?5:2}孔，不需要选择姿态组或行列。不加入旧XY经验偏置；Z保持当前生产配置。</p><div class="card">${button("detect","识别固定标定孔",true)}${state.holes?.length?`<p>已锁定：机器人第五行 / 从左第${state.hand?5:2}孔；内部编号 row=${state.row}, col=${state.col}。</p>${button("move_d","接近并停在D点",true)}`:""}</div>`;
 } else if(page===2) {
  body+=`<p class="lead">当前保持在C上方 ${fmt(state.d_clearance_mm,0)} mm 的D点。只微调XY，确认竖直下降的工作中心对准孔；不要仅用斜视画面里的手指中点判断。确认后不再下到C点。</p><div class="card">${jog()}${confirm("centered","已确认保持当前XY竖直下降，可以居中抓取/放置。")}${button("record_aligned","记录D点对准与XY补偿",true)}</div><div class="card"><h3>手相机 · 最近固定画面</h3>${button("observe","刷新观察画面")}${state.image_revision?image(false):""}</div>`;
 } else if(page===3) {
  body+=`<p class="lead">${grasp()?"保持手臂不动，原地张爪，再将一个零件居中放入刚才的目标孔。宽松孔中零件可能偏心，优先使用居中套。":"保持手臂不动，张开夹爪，人工取走刚才夹着的零件，露出同一个目标孔。"}</p><div class="card">${phase==="aligned"?button("release","保持手臂，原地张爪",true):`${confirm("scene",grasp()?"零件已居中放入同一目标孔，顶面清晰，未碰动手臂或相机。":"零件已取走，同一目标孔清晰可见，未碰动手臂或相机。")}${button("capture","拍摄固定参考图",true)}`}</div>`;
 } else if(page===4) {
  body+=`<p class="lead">点击${grasp()?"零件顶面真正的圆心（不是孔心，也不是检测框中心）":"刚才对准的目标孔中心附近，程序将拟合橙色孔环"}。图像和实际TCP位姿一起记录。</p><div class="card">${image()}<p id="selection" class="muted">请选择正确目标中心。</p>${button("reference",grasp()?"记录零件顶面圆心参考":"精定位并记录孔心参考",true,!selected)}${confirm("scene","参考物仍在原位，未碰动手臂或相机。")}${button("capture","重新拍摄固定参考图")}</div>`;
 } else {
  body+=`<div class="card">${metrics()}${state.image_revision?image(false):""}<p>单份数据用于本只手的全部料盘行；不会修改传送带偏置或参考。</p>`;
  if(phase==="reference")body+=`${confirm("reference_ok","已确认D点对准、参考对象及中心点正确。")}${button("save","保存并应用料盘标定",true)}`;
  else if(phase==="saved"||phase==="returned")body+=`<div class="success">${taskName()}标定已保存并加载；已完成人工标定，尚未验证实际作业精度。</div><p>文件：<code>${esc(state.saved_file)}</code></p><p>日志：<code>${esc(state.log_directory)}</code></p>${phase==="saved"?button("return_ready","抬离并返回准备位（可选）"):""}${button("end","结束本次标定",true)}`;
  body+='</div><div class="note">无强制C点精调或重新装件回放。保存后用实际料盘抓放验证效果。相机、工具或夹持方式改变后重新标定。</div>';
 }
 if(page===1&&state.head_image_revision) body+=`<div class="card"><h3>头相机 · 固定目标核对</h3><img style="width:100%;height:auto" alt="料盘孔位编号" src="/api/calibration/image?kind=head&session=${encodeURIComponent(state.session)}&revision=${state.head_image_revision}&token=${encodeURIComponent(token)}"></div>`;
 if(state.active&&state.current_tcp)body+=`<p class="muted">最近实际TCP X / Y / Z：${state.current_tcp.slice(0,3).map(v=>fmt(v*1000)).join(" / ")} mm</p>`;
 $("content").innerHTML=body;
 if($("task"))$("task").onchange=e=>{taskChoice=e.target.value;render();};
 if($("hand"))$("hand").onchange=e=>{handChoice=e.target.value;};
 if($("stepSize"))$("stepSize").onchange=e=>{jogSize=e.target.value;};
 document.querySelectorAll("[data-action]").forEach(b=>{b.disabled ||= busy||readonly||(state.active&&state.interrupted);b.onclick=()=>run(b.dataset.action);});
 document.querySelectorAll("[data-jog]").forEach(b=>{b.disabled=busy||readonly||(state.active&&state.interrupted);b.onclick=()=>{const [axis,sign]=b.dataset.jog.split(",");run("jog",{[axis==="x"?"dx_mm":"dy_mm"]:Number(sign)*Number($("stepSize").value)});};});
 if($("exit"))$("exit").onclick=()=>$("exitDialog").showModal();
 if($("frame")) {
  const img=$("frame");img.src=`/api/calibration/image?session=${encodeURIComponent(state.session)}&revision=${state.image_revision}&token=${encodeURIComponent(token)}`;
  img.onload=draw;
  const point=e=>{const box=img.getBoundingClientRect();return {u:(e.clientX-box.left)*state.width/box.width,v:(e.clientY-box.top)*state.height/box.height,revision:state.image_revision};};
  img.onmousemove=e=>zoom(point(e));
  img.onclick=e=>{if(busy||readonly||(state.active&&state.interrupted)||phase!=="captured")return;selected=point(e);draw();$("selection").textContent=`已选 ${fmt(selected.u,1)}, ${fmt(selected.v,1)}。${grasp()?"将记录此顶面圆心。":"将精定位附近孔环。"}`;document.querySelector('[data-action="reference"]').disabled=false;};
 }
}
function zoom(p) {
 const c=$("zoom"),img=$("frame");if(!c||!img?.complete||!img.naturalWidth)return;
 const ctx=c.getContext("2d");ctx.clearRect(0,0,c.width,c.height);ctx.drawImage(img,p.u-40,p.v-30,80,60,0,0,320,240);
 ctx.strokeStyle="#00e4ce";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(145,120);ctx.lineTo(175,120);ctx.moveTo(160,105);ctx.lineTo(160,135);ctx.stroke();
}
function draw() {
 const c=$("cross");if(!c)return;c.width=state.width;c.height=state.height;const ctx=c.getContext("2d");
 const marked=state.have_reference?{u:state.reference_px[0],v:state.reference_px[1],revision:state.image_revision}:selected;
 if(marked?.revision===state.image_revision){const selected=marked;ctx.strokeStyle="#00e4ce";ctx.lineWidth=2;ctx.beginPath();ctx.arc(selected.u,selected.v,12,0,2*Math.PI);ctx.moveTo(selected.u-22,selected.v);ctx.lineTo(selected.u+22,selected.v);ctx.moveTo(selected.u,selected.v-22);ctx.lineTo(selected.u,selected.v+22);ctx.stroke();zoom(selected);}
}
async function run(action,extra={}) {
 if(busy)return;error();
 try {
  if(action==="reset_stop") {if(!$("workspace")?.checked)throw Error("请先确认现场与机器人状态");extra.acknowledged=true;}
  if(action==="begin") {if(!$("workspace")?.checked)throw Error("请先确认工位和当前夹持状态");extra={hand:Number($("hand").value),task:$("task").value,clearance_mm:Number($("clearance").value)};}
  if(action==="prepare") {if(!$("setup")?.checked)throw Error("请确认第五行清空及夹持状态");extra.setup_confirmed=true;}
  if(action==="record_aligned") {if(!$("centered")?.checked)throw Error("请先确认D点对准");extra.physically_centered=true;}
  if(action==="capture") {if(!$("scene")?.checked)throw Error("请确认参考物及手臂未被碰动");extra.scene_ready=true;}
  if(action==="reference") {if(!selected)throw Error("请先点击正确的中心点");extra={u:selected.u,v:selected.v,image_revision:selected.revision};}
  if(action==="save") {if(!$("reference_ok")?.checked)throw Error("请先确认补偿与参考点");extra.reference_confirmed=true;}
  const request={action,session:state.session,request_id:globalThis.crypto?.randomUUID?.()||`${Date.now()}-${Math.random()}`,...extra};
  busy=true;render();const result=await api("/api/calibration",request);
  if(result.session)state=result;if(!result.ok)throw Error(result.error||"动作未完成");
  viewStep=null;if(action==="capture")selected=null;
 } catch(e){error(e.message);}finally{busy=false;render();}
}
async function refresh() {
 try {const next=await api("/api/calibration");if(!next.ok)throw Error(next.error||"请重新编译并启动支持料盘标定的机器人程序");
  $("connection").textContent=`机器人已连接 · ${next.active?"标定会话有效":"空闲"}`;$("connection").className="connection";
  if(!busy){const changed=next.phase!==state.phase||next.session!==state.session||next.interrupted!==state.interrupted||next.software_stopped!==state.software_stopped;state=next;if(changed){selected=null;render();}}
 }catch(e){$("connection").textContent=e.message;$("connection").className="connection bad";}
}
$("stop").onclick=async()=>{try{await api("/api/command",{cmd:"abort"});error("已请求停止；不会自动松爪或回位。等待动作结束后退出会话。");await refresh();}catch(e){error(e.message);}};
$("back").onclick=()=>{viewStep=Math.max(0,(viewStep??state.step)-1);render();};
$("resume").onclick=()=>{viewStep=null;render();};
$("cancelExit").onclick=()=>$("exitDialog").close();
$("confirmExit").onclick=()=>{$("exitDialog").close();run("end");};
render();refresh();setInterval(refresh,1500);

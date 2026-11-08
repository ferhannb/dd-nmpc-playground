/* Panamax DD-NMPC replay viewer: plays frozen closed-loop results.
   The controller, the solver and the vessel model are not part of this site. */
const $ = (id) => document.getElementById(id);
const routeNames = {straight: 'Straight passage', s_curve: 'Waypoint S-curve', complex: 'Complex harbour route'};
const routeHints = {straight: '6.0 km · Straight-line path following', s_curve: '≈ 6.1 km · Curvature-continuous waypoints', complex: '≈ 14.3 km · Smoothed harbour waypoints'};
let run = {parameters: null, reference: null, waypoints: [], cycles: [], summary: null, initial: null, initialProgress: 0, length: 0};
let currentIndex = -1, playing = false, playTimer = null, loadToken = 0;
let zoom = 1;
let followCamera = true;
const plotHeights = {error:165,rudder:260,rpm:260};
let mapHeight = 205, compactMapHeight = 205;

function fmt(n, digits = 1) {
  return typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '—';
}

function setStatus(message, kind = '') {
  $('statusText').textContent = message;
  $('liveLight').className = 'live-light' + (kind ? ' ' + kind : '');
  $('resultState').textContent = message;
}

function setConfigFeedback(title, description, kind = '') {
  $('configFeedback').className = 'config-feedback' + (kind ? ' ' + kind : '');
  $('configFeedbackTitle').textContent = title;
  $('configFeedbackText').textContent = description;
}

function showRecordedSettings(p) {
  $('setHorizon').textContent = `${fmt(p.horizon_m, 0)} m`;
  $('setRpm').textContent = `${fmt(p.rpm, 0)} RPM`;
  $('setCrossTrack').textContent = `${fmt(p.cross_track_m, 0)} m`;
  $('setHeading').textContent = `${fmt(p.heading_error_deg, 0)}°`;
  $('setCurrent').textContent = `${fmt(p.current_east_mps, 1)} / ${fmt(p.current_north_mps, 1)} m/s`;
}

async function loadRun() {
  const token = ++loadToken;
  const route = $('route').value, [N, M] = $('preset').value.split('/').map(Number);
  stopPlayback();
  $('routeHint').textContent = routeHints[route];
  $('scenarioLabel').textContent = `/ ${routeNames[route]}`;
  setStatus('Loading recorded run…', 'active');
  try {
    const response = await fetch(`./data/${route}-n${N}-m${M}.json`);
    if (!response.ok) throw new Error(`Recorded run not found (${response.status})`);
    const data = await response.json();
    if (token !== loadToken) return;
    run = {parameters: data.parameters, reference: data.reference, waypoints: data.waypoints, cycles: data.cycles,
      summary: data.summary, initial: data.initial, initialProgress: data.initial_progress_m, length: data.length_m};
    currentIndex = 0; zoom = 1; followCamera = true;
    $('timeline').disabled = false;
    ['prevBtn', 'playBtn', 'nextBtn'].forEach(id => $(id).disabled = false);
    $('waypointLegendLabel').textContent = `${data.waypoints_are_nominal ? 'Nominal WPs' : 'WPs'} (${run.waypoints.length})`;
    $('mapInfo').textContent = `${fmt(run.length / 1000, 2)} km reference · ${run.waypoints.length} WPs · N=${N}, M=${M}`;
    showRecordedSettings(run.parameters);
    $('summaryRmse').textContent = `${fmt(run.summary.cross_track_rmse_m, 2)} m`;
    $('summaryMax').textContent = `${fmt(run.summary.max_abs_cross_track_m, 2)} m`;
    const ending = data.termination === 'arrival' ? 'route end reached' : `stopped: ${data.termination}`;
    setStatus(`Recorded run · ${run.summary.cycles} cycles · ${ending}`);
    setConfigFeedback('Recorded result', `${routeNames[route]} · N=${N}, M=${M} · ${ending}. Press play to step through the control cycles.`);
    updateSelectedCycle();
  } catch (err) {
    if (token !== loadToken) return;
    console.error(err);
    setStatus(err.message || 'Could not load the recorded run', 'error');
    setConfigFeedback('Recorded run not available', err.message || 'Reload the page.', 'error');
  }
}

function selectCycle(i) {
  if (!run.cycles.length) return;
  currentIndex = Math.max(0, Math.min(i, run.cycles.length - 1));
  updateSelectedCycle();
}

function updateSelectedCycle() {
  const n = run.cycles.length;
  $('seekText').textContent = n ? `${Math.max(0, currentIndex + 1)} / ${n}` : '0 / 0';
  $('timeline').max = String(Math.max(0, n - 1));
  $('timeline').value = String(Math.max(0, currentIndex));
  const evt = run.cycles[currentIndex];
  const l = evt?.log;
  $('timeInfo').textContent = l ? `t = ${fmt(evt.elapsed_plant_time_s, 1)} s` : 't = 0 s';
  $('metricEy').innerHTML = l ? `${fmt(l.ey_m, 2)}<small>m</small>` : '—<small>m</small>';
  $('metricHeading').textContent = fmt(l?.echi_deg, 2);
  $('metricSpeed').textContent = fmt(l?.V_mps, 2);
  $('metricRudder').textContent = fmt(l?.delta_cmd_deg, 1);
  $('metricRpm').textContent = fmt(l?.rpm_actual, 1);
  $('metricDs').textContent = l ? `${fmt(l.first_shooting_interval_m, 2)} m` : '— m';
  $('metricApplied').textContent = l ? `${fmt(l.apply_ds_m, 2)} m` : '— m';
  $('metricHorizon').textContent = l ? `${fmt(l.S_H_m, 0)} m` : '— m';
  $('metricMode').textContent = l ? l.mode.toUpperCase() : '—';
  $('metricStatus').textContent = l ? (l.fallback_used ? 'FALLBACK' : (l.solver_success ? 'SUCCESS' : 'ACCEPTED')) : '—';
  drawAll();
}

function stopPlayback() {
  playing = false; clearInterval(playTimer); playTimer = null; $('playBtn').textContent = '▶';
}

function togglePlayback() {
  if (!run.cycles.length) return;
  if (playing) { stopPlayback(); return; }
  if (currentIndex >= run.cycles.length - 1) selectCycle(0);
  playing = true; $('playBtn').textContent = 'Ⅱ';
  playTimer = setInterval(() => {
    if (currentIndex + 1 >= run.cycles.length) { stopPlayback(); return; }
    selectCycle(currentIndex + 1);
  }, 550);
}

function canvasSize(canvas) {
  const rect = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio || 1, 2);
  const w = Math.max(1, rect.width), h = Math.max(1, rect.height);
  canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0);
  return {ctx, w, h};
}

function getCamera(w, h) {
  const all = (run.reference || [[0,0],[0,1000]])
    .concat(run.initial ? [[run.initial[0],run.initial[1]]] : [])
    .concat(run.waypoints.map(wp => wp.xy));
  const xs = all.map(p => p[0]), ys = all.map(p => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const xmid = (minX+maxX)/2, ymid = (minY+maxY)/2;
  const width = Math.max(maxX - minX, 600), height = Math.max(maxY - minY, 600);
  const fitScale = Math.min((w-95)/width, (h-75)/height);
  const state = currentIndex >= 0 ? run.cycles[currentIndex].state_after : run.initial;
  const centerX = followCamera && state ? state[0] : xmid;
  const centerY = followCamera && state ? state[1] : ymid;
  // A consistent local viewing window avoids a tiny ship on long routes.
  // Fit explicitly returns to the whole-route view.
  const localScale = Math.min((w-95)/1400,(h-75)/900);
  const scale = followCamera ? localScale * zoom : fitScale;
  return {scale, point:(p) => [w/2 + (p[0]-centerX)*scale, h/2 - (p[1]-centerY)*scale]};
}

function linePath(ctx, points, camera, color, width, dash = []) {
  if (!points || points.length < 2) return;
  ctx.save();ctx.beginPath();ctx.setLineDash(dash);
  for (let i=0;i<points.length;i++) {
    const [x,y] = camera.point(points[i]);
    if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle = color;ctx.lineWidth = width;ctx.lineJoin='round';ctx.stroke();ctx.restore();
}

function drawShip(ctx, camera, state) {
  if (!state) return;
  const [x,y]=camera.point(state);const chi=state[2];
  ctx.save();ctx.translate(x,y);ctx.rotate(chi);
  ctx.shadowBlur=15;ctx.shadowColor='#46dfc277';
  ctx.beginPath();ctx.moveTo(0,-16);ctx.lineTo(8,8);ctx.lineTo(0,4);ctx.lineTo(-8,8);ctx.closePath();
  ctx.fillStyle='#46dfc2';ctx.fill();ctx.shadowBlur=0;ctx.strokeStyle='#d5fff5';ctx.lineWidth=1.3;ctx.stroke();ctx.restore();
  ctx.beginPath();ctx.arc(x,y,21,0,2*Math.PI);ctx.strokeStyle='#46dfc250';ctx.lineWidth=1;ctx.stroke();
}

function drawWaypoints(ctx, camera, w, h) {
  if (!$('showWaypoints').checked) return;
  for (const wp of run.waypoints) {
    const [x,y] = camera.point(wp.xy);
    if (x < -70 || x > w+70 || y < -35 || y > h+35) continue;
    ctx.save();
    ctx.fillStyle='#172739';ctx.strokeStyle='#e7b666';ctx.lineWidth=2;
    ctx.beginPath();ctx.arc(x,y,5,0,2*Math.PI);ctx.fill();ctx.stroke();
    ctx.beginPath();ctx.arc(x,y,1.6,0,2*Math.PI);ctx.fillStyle='#f8d5a1';ctx.fill();
    const label = `WP${wp.number}${wp.role==='start'?' START':wp.role==='goal'?' GOAL':''}`;
    ctx.font='10px ui-monospace, monospace';
    const labelWidth=ctx.measureText(label).width+13;
    const lx=Math.max(5,Math.min(w-labelWidth-5,x+11));
    const ly=Math.max(5,Math.min(h-22,y+(wp.number%2? -23:10)));
    ctx.fillStyle='#1b2938eb';ctx.strokeStyle='#785b37';ctx.lineWidth=1;
    ctx.beginPath();ctx.roundRect(lx,ly,labelWidth,19,4);ctx.fill();ctx.stroke();
    ctx.textAlign='left';ctx.textBaseline='middle';ctx.fillStyle='#ffd7a0';ctx.fillText(label,lx+6,ly+10);
    ctx.restore();
  }
}

function drawMap() {
  const {ctx,w,h}=canvasSize($('mapCanvas'));
  ctx.clearRect(0,0,w,h);
  if (!run.reference) {
    ctx.font='12px DM Mono, monospace';ctx.textAlign='center';ctx.fillStyle='#66849a';
    ctx.fillText('LOADING RECORDED RUN',w/2,h/2);return;
  }
  const camera=getCamera(w,h);
  // World-coordinate axes (north is visually up).
  linePath(ctx, [[0,-100000],[0,100000]],camera,'#34506654',1,[4,7]);
  linePath(ctx, [[-100000,0],[100000,0]],camera,'#34506654',1,[4,7]);
  linePath(ctx,run.reference,camera,'#829aac',2,[8,5]);
  drawWaypoints(ctx,camera,w,h);
  const visited=[run.initial ? [run.initial[0],run.initial[1]] : null, ...run.cycles.slice(0,currentIndex+1).map(e=>[e.state_after[0],e.state_after[1]])].filter(Boolean);
  linePath(ctx,visited,camera,'#47dfc2',2.5);
  const evt=run.cycles[currentIndex];
  if (evt?.prediction_state_nodes) {
    const pred=evt.prediction_state_nodes.slice(evt.applied_stages).map(z=>[z[0],z[1]]);
    linePath(ctx,pred,camera,'#f4b37a',1.8,[5,5]);
    ctx.fillStyle='#f4b37a';
    for (const point of pred.slice(1)) {const [x,y]=camera.point(point);ctx.beginPath();ctx.arc(x,y,2.8,0,2*Math.PI);ctx.fill();}
  }
  drawShip(ctx,camera,evt ? evt.state_after : run.initial);
  const scaleDistance=zoom>1.05?500:1000;const len=scaleDistance*camera.scale;
  if (len > 20 && len < w*.4) {
    ctx.strokeStyle='#a5bacaaa';ctx.lineWidth=1.3;ctx.beginPath();ctx.moveTo(18,h-21);ctx.lineTo(18+len,h-21);ctx.moveTo(18,h-25);ctx.lineTo(18,h-17);ctx.moveTo(18+len,h-25);ctx.lineTo(18+len,h-17);ctx.stroke();
    ctx.fillStyle='#97adbf';ctx.font='9px DM Mono, monospace';ctx.fillText(`${scaleDistance} m`,18,h-32);
  }
}

function drawChart(id, series, {zeroLine=false, legend=false}={}) {
  const {ctx,w,h}=canvasSize($(id));ctx.clearRect(0,0,w,h);
  const left=33,right=9,top=16,bottom=22;const pw=w-left-right,ph=h-top-bottom;
  ctx.strokeStyle='#24374a';ctx.lineWidth=1;ctx.font='9px DM Mono, monospace';ctx.fillStyle='#7290a6';ctx.textAlign='right';
  const logs=run.cycles.map(evt=>evt.log);
  let values=series.flatMap(s=>logs.map(l=>l[s.key]).filter(Number.isFinite));
  values.push(0); let ymin=Math.min(...values),ymax=Math.max(...values);
  const pad=Math.max(1,(ymax-ymin)*.15);ymin-=pad;ymax+=pad;
  const xPositions=logs.map(z=>z.sigma_m);
  const xmin=0,xmax=Math.max(run.length || 1,...xPositions,1);
  for(let i=0;i<=3;i++) {
    const val=ymin+(ymax-ymin)*i/3;const y=top+ph-(i/3)*ph;
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();
    ctx.fillText(fmt(val,0),left-5,y+3);
  }
  ctx.textAlign='center';ctx.fillStyle='#72889d';ctx.fillText('PROGRESS (km)',left+pw/2,h-2);
  ctx.textAlign='left';ctx.fillText('0',left,h-12);ctx.textAlign='right';ctx.fillText(fmt(xmax/1000,1),w-right,h-12);
  if(zeroLine && ymin < 0 && ymax > 0) {const y=top+ph-(0-ymin)/(ymax-ymin)*ph;ctx.strokeStyle='#69809966';ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();ctx.setLineDash([]);}
  for (const s of series) {
    ctx.strokeStyle=s.color;ctx.lineWidth=1.9;ctx.beginPath();
    logs.forEach((l,i)=>{const x=left+pw*l.sigma_m/xmax,y=top+ph-ph*(l[s.key]-ymin)/(ymax-ymin);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke();
  }
  if(currentIndex>=0 && logs[currentIndex]) {
    const x=left+pw*logs[currentIndex].sigma_m/xmax;
    ctx.strokeStyle='#e5eef77a';ctx.lineWidth=1;ctx.setLineDash([2,3]);ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,top+ph);ctx.stroke();ctx.setLineDash([]);
    for (const s of series) { const y=top+ph-ph*(logs[currentIndex][s.key]-ymin)/(ymax-ymin);ctx.fillStyle=s.color;ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fill(); }
  }
  if(legend) {
    ctx.textAlign='right';ctx.font='9px DM Mono, monospace';let x=w-10;
    for(const s of [...series].reverse()) {ctx.fillStyle=s.color;ctx.fillText(s.name,x,11);x-=ctx.measureText(s.name).width+13;}
  }
}

function remainingInputPlan(evt, controlColumn) {
  const W = evt?.optimized_block_commands;
  const ds = evt?.stage_distances_m;
  const block = evt?.block_map;
  if (!Array.isArray(W) || !Array.isArray(ds) || !Array.isArray(block)
      || ds.length !== block.length || !ds.length) return [];
  // The first block has already been applied at the displayed plant state. Only
  // the remaining stages of this plan are projected to the right of NOW.
  const segments = [];
  let x = evt.progress_after_m;
  for (let stage = evt.applied_stages; stage < ds.length; stage++) {
    const cmd = W[block[stage]];
    if (!cmd || !Number.isFinite(cmd[controlColumn]) || !(ds[stage] > 0)) return [];
    segments.push({start: x, end: x + ds[stage], value: cmd[controlColumn]});
    x += ds[stage];
  }
  return segments;
}

function remainingPredictedActuator(evt, stateIndex) {
  const states = evt?.prediction_state_nodes;
  const ds = evt?.stage_distances_m;
  if (!Array.isArray(states) || !Array.isArray(ds)
      || states.length !== ds.length + 1 || !ds.length) return [];
  // Node k holds the predicted actuator states; the node at the end of the
  // applied block corresponds to the displayed NOW.
  // Do NOT substitute the actual plant state: its model mismatch is useful.
  const points = [];
  let distance = evt.progress_after_m;
  for (let node = evt.applied_stages; node < states.length; node++) {
    if (node > evt.applied_stages) distance += ds[node - 1];
    const value = states[node]?.[stateIndex];
    if (!Number.isFinite(value)) return [];
    points.push([distance, value]);
  }
  return points;
}

function drawInputChart(canvasId, controlColumn) {
  const {ctx,w,h} = canvasSize($(canvasId));
  ctx.clearRect(0,0,w,h);
  const isRpm = controlColumn === 0;
  const pastCommandKey = isRpm ? 'n_cmd_rpm' : 'delta_cmd_deg';
  const actualKey = isRpm ? 'rpm_actual' : 'rudder_actual_deg';
  const actualInitialIndex = isRpm ? 3 : 4;  // initial = [X, Y, course, RPM, rudder]
  const selected = run.cycles[currentIndex];
  if (!selected) {
    ctx.textAlign='center';ctx.font='10px ui-monospace,monospace';ctx.fillStyle='#688499';
    ctx.fillText('Loading recorded run',w/2,h/2);return;
  }

  const left=36,right=10,top=17,bottom=23,pw=Math.max(1,w-left-right),ph=Math.max(1,h-top-bottom);
  const now=selected.progress_after_m;
  const horizon=Math.max(1,selected.log.S_H_m);
  const future=remainingInputPlan(selected,controlColumn);
  const predictedActual=remainingPredictedActuator(selected,isRpm?2:3)  // nodes = [X, Y, RPM, rudder];
  const futureEnd=future.length ? future[future.length-1].end : now+horizon;
  const xmin=Math.max(0,now-Math.max(horizon,350));
  const xmax=Math.max(now+100,futureEnd,now+0.45*horizon);
  const X=s=>left+pw*(s-xmin)/(xmax-xmin);
  const history=run.cycles.slice(0,currentIndex+1);
  const initialActual=run.initial?.[actualInitialIndex];
  const targetRpm=run.parameters?.rpm;
  const candidates=[...history.flatMap(e=>[e.log[pastCommandKey],e.log[actualKey]]),
    ...future.map(seg=>seg.value),...predictedActual.map(point=>point[1]),initialActual];
  if (isRpm) candidates.push(targetRpm);
  else candidates.push(0);
  const finite=candidates.filter(Number.isFinite);
  let ymin=Math.min(...finite),ymax=Math.max(...finite);
  const ypad=Math.max(isRpm?3:2,(ymax-ymin)*0.18);
  ymin-=ypad;ymax+=ypad;
  const Y=v=>top+ph-ph*(v-ymin)/(ymax-ymin);

  // Distinguish already observed history from hypothetical open-loop future.
  const nowX=X(now);
  ctx.fillStyle='#d1a25e0c';ctx.fillRect(nowX,top,w-right-nowX,ph);
  ctx.lineWidth=1;ctx.strokeStyle='#293b4c';ctx.font='9px ui-monospace,monospace';
  for(let i=0;i<=3;i++) {
    const val=ymin+(ymax-ymin)*i/3,y=Y(val);
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();
    ctx.textAlign='right';ctx.fillStyle='#7d96a9';ctx.fillText(fmt(val,isRpm?0:1),left-5,y+3);
  }
  ctx.fillStyle='#7d96a9';ctx.textAlign='left';ctx.fillText(`${fmt(xmin/1000,2)} km`,left,h-7);
  ctx.textAlign='right';ctx.fillText(`${fmt(xmax/1000,2)} km`,w-right,h-7);

  ctx.save();ctx.beginPath();ctx.rect(left,top,pw,ph);ctx.clip();

  if (isRpm && Number.isFinite(targetRpm)) {
    ctx.save();ctx.strokeStyle='#9c9df0';ctx.lineWidth=1.5;ctx.setLineDash([2,4]);
    ctx.beginPath();ctx.moveTo(left,Y(targetRpm));ctx.lineTo(w-right,Y(targetRpm));ctx.stroke();ctx.restore();
  } else if (!isRpm && ymin < 0 && ymax > 0) {
    ctx.save();ctx.strokeStyle='#647c9075';ctx.setLineDash([2,4]);
    ctx.beginPath();ctx.moveTo(left,Y(0));ctx.lineTo(w-right,Y(0));ctx.stroke();ctx.restore();
  }

  // Realized actuator response: sampled after each physical plant transition.
  const actualPoints=[];
  if (Number.isFinite(initialActual)) actualPoints.push([run.initialProgress,initialActual]);
  for(const evt of history) actualPoints.push([evt.progress_after_m,evt.log[actualKey]]);
  ctx.strokeStyle='#46dfc2';ctx.lineWidth=2;ctx.beginPath();
  actualPoints.forEach(([s,v],i)=>{if(i===0)ctx.moveTo(X(s),Y(v));else ctx.lineTo(X(s),Y(v));});ctx.stroke();

  // Command history is held constant during the applied shooting distance.
  ctx.strokeStyle='#f4b37a';ctx.lineWidth=1.8;ctx.beginPath();
  history.forEach((evt,i)=>{
    const v=evt.log[pastCommandKey],x0=X(evt.progress_before_m),x1=X(evt.progress_after_m);
    if(i===0)ctx.moveTo(x0,Y(v));else ctx.lineTo(x0,Y(v));
    ctx.lineTo(x1,Y(v));
  });ctx.stroke();

  // The remainder of the *last accepted optimization* is not a measurement.
  if(future.length) {
    ctx.save();ctx.setLineDash([5,4]);ctx.strokeStyle='#ffce77';ctx.lineWidth=2;
    ctx.beginPath();
    future.forEach((seg,i)=>{
      if(i===0)ctx.moveTo(X(seg.start),Y(seg.value));
      else ctx.lineTo(X(seg.start),Y(seg.value));
      ctx.lineTo(X(seg.end),Y(seg.value));
    });ctx.stroke();ctx.restore();
  }

  // Blue: predicted *actual actuator states*, not commanded controls. These
  // originate from the same accepted FATROP X_phys nodes as the map forecast.
  if(predictedActual.length > 1) {
    ctx.save();ctx.setLineDash([2,4]);ctx.strokeStyle='#77b9ff';ctx.lineWidth=2;
    ctx.beginPath();predictedActual.forEach(([s,v],i)=>{
      if(i===0)ctx.moveTo(X(s),Y(v));else ctx.lineTo(X(s),Y(v));
    });ctx.stroke();ctx.restore();
  }

  ctx.restore();

  ctx.save();ctx.strokeStyle='#f1f5ff';ctx.lineWidth=1.2;ctx.setLineDash([3,3]);
  ctx.beginPath();ctx.moveTo(nowX,top);ctx.lineTo(nowX,top+ph);ctx.stroke();ctx.restore();
  const nowLabel='NOW';
  ctx.font='bold 9px ui-monospace,monospace';
  const labelWidth=ctx.measureText(nowLabel).width+10;
  const lx=Math.max(left,Math.min(w-right-labelWidth,nowX-labelWidth/2));
  ctx.fillStyle='#e9eff4';ctx.fillRect(lx,0,labelWidth,15);
  ctx.fillStyle='#13212d';ctx.textAlign='center';ctx.fillText(nowLabel,lx+labelWidth/2,11);

}

function drawAll() {
  drawMap();
  drawChart('errorCanvas',[{key:'ey_m',color:'#46dfc2',name:'error'}],{zeroLine:true});
  drawInputChart('commandCanvas',1);
  drawInputChart('rpmCanvas',0);
}

// State wiring.
$('route').addEventListener('change',loadRun);
$('preset').addEventListener('change',loadRun);
$('prevBtn').addEventListener('click',()=>{stopPlayback();selectCycle(currentIndex-1);});
$('nextBtn').addEventListener('click',()=>{stopPlayback();selectCycle(currentIndex+1);});
$('playBtn').addEventListener('click',togglePlayback);
$('timeline').addEventListener('input',()=>{stopPlayback();selectCycle(Number($('timeline').value));});
$('mapCanvas').addEventListener('wheel',e=>{e.preventDefault();if(!followCamera){followCamera=true;zoom=1;}zoom=Math.max(.25,Math.min(16,zoom*(e.deltaY<0?1.27:1/1.27)));drawMap();},{passive:false});
$('mapCanvas').addEventListener('dblclick',()=>{followCamera=true;zoom=1;drawMap();});
$('zoomOut').addEventListener('click',()=>{if(!followCamera){followCamera=true;zoom=1;}zoom=Math.max(.25,zoom/1.5);drawMap();});
$('zoomFit').addEventListener('click',()=>{followCamera=false;zoom=1;drawMap();});
$('zoomIn').addEventListener('click',()=>{if(!followCamera){followCamera=true;zoom=1;}zoom=Math.min(16,zoom*1.5);drawMap();});
function setMapHeight(height){
  mapHeight=Math.max(160,Math.min(720,height));
  $('mapCard').style.setProperty('--map-height',mapHeight+'px');
  $('mapHeightLabel').textContent=mapHeight+' px';
  requestAnimationFrame(drawAll);
}
$('mapHeightDown').addEventListener('click',()=>setMapHeight(mapHeight-40));
$('mapHeightUp').addEventListener('click',()=>setMapHeight(mapHeight+40));
$('toggleMapSize').addEventListener('click',()=>{
  const expanded=$('mapCard').classList.toggle('expanded-map');
  if(expanded){compactMapHeight=mapHeight;setMapHeight(Math.max(470,mapHeight));}
  else setMapHeight(compactMapHeight);
  $('toggleMapSize').textContent=expanded?'Compact map':'Expand map';
  $('toggleMapSize').setAttribute('aria-pressed',String(expanded));
  $('toggleMapSize').title=expanded?'Reduce the X-Y map':'Enlarge the X-Y map';
});
$('showWaypoints').addEventListener('change',drawMap);
for(const [name,height] of Object.entries(plotHeights)){
  $(name+'Card').style.height=height+'px';
  $(name+'HeightLabel').textContent=height+' px';
}
document.querySelectorAll('.chart-resize button').forEach(button=>{
  button.addEventListener('click',()=>{
    const name=button.parentElement.dataset.chart;
    const next=Math.max(150,Math.min(500,plotHeights[name]+Number(button.dataset.step)));
    plotHeights[name]=next;
    $(name+'Card').style.height=next+'px';
    $(name+'HeightLabel').textContent=next+' px';
    requestAnimationFrame(drawAll);
  });
});
new ResizeObserver(drawAll).observe($('mapCanvas').parentElement);
window.addEventListener('resize',drawAll);
setMapHeight(mapHeight);
drawAll();
loadRun();

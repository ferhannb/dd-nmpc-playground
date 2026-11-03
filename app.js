/* Panamax DD-NMPC public browser research demonstration. */
const $ = (id) => document.getElementById(id);
const routeNames = {straight: 'Straight passage', s_curve: 'Waypoint S-curve', complex: 'Complex harbour route'};
const routeHints = {straight: '6.0 km · Straight-line path following', s_curve: '≈ 6.1 km · Curvature-continuous waypoints', complex: '≈ 14.2 km · Smoothed harbour waypoints'};
const numericIds = ['N', 'M', 'horizon_m', 'rpm', 'max_cycles', 'cross_track_m', 'heading_error_deg', 'current_east_mps', 'current_north_mps'];
let run = {parameters: null, applied: null, reference: null, waypoints: [], cycles: [], summary: null, initial: null, initialProgress: 0, length: 0};
let currentIndex = -1, followLive = true, playing = false, playTimer = null, abortController = null, running = false;
let zoom = 1;
let followCamera = true;
const plotHeights = {error:165,rudder:260,rpm:260};
let mapHeight = 205, compactMapHeight = 205;
let browserSolverWorker = null;

function setEngineMode() {
  const wasm=$('engine').value==='wasm';
  $('max_cycles').max=wasm?'100':'160';
  for(const id of numericIds)$(id).disabled=running;
  for(const id of ['route','cross_track_slider','heading_slider'])$(id).disabled=running;
  $('engineHint').textContent=wasm?
    'Browser FATROP supports Straight, S-curve and Harbour; compiled N/M pairs: 8/2, 8/4, 12/4. Other controls are experimental; 1–100 cycles.':
    'Python computes the full controller and streams the result locally.';
  $('forecastNote').textContent=wasm?
    'NOW = independent plant state after the applied stage. Dashed map/actuator forecasts come from the last accepted FATROP solution, not measured data; fallback cycles have no forecast.':
    'NOW = state after first applied stage. Forecasts show remaining stages of the last accepted MPC solution; future predicted states are model outputs, not measurements.';
  setConfigFeedback(wasm?'Browser FATROP ready':'Python FATROP ready',
    wasm?'Choose a compiled N/M pair. Scenario, Smax, RPM, initial errors and current change the browser simulation.':'All editable parameters apply to the local Python solver.');
}

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

function reportPendingChanges() {
  if (running) return;
  let params;
  try { params = parameterValues(); }
  catch (err) {
    setConfigFeedback('Invalid parameter', err.message, 'error');
    return;
  }
  if (run.parameters && JSON.stringify(params) === JSON.stringify(run.parameters) && run.applied) {
    showAppliedConfiguration(run.applied);
    return;
  }
  setConfigFeedback('Changes pending', 'New values will take effect when you click Apply & run.', 'pending');
  if (run.cycles.length) setStatus('Settings changed · previous run shown', '');
}

function showAppliedConfiguration(applied) {
  const details = `N=${applied.N}, M=${applied.M}, S_H=${fmt(applied.horizon_m, 0)} m, RPM=${fmt(applied.rpm, 0)}, e_y0=${fmt(applied.cross_track_m, 0)} m, ψ₀ error=${fmt(applied.heading_error_deg, 0)}°; current E/N=${fmt(applied.current_east_mps, 1)}/${fmt(applied.current_north_mps, 1)} m/s`;
  setConfigFeedback('Applied to Python solver', details);
}

function parameterValues() {
  const data = {route: $('route').value};
  for (const id of numericIds) {
    if ($(id).value.trim() === '') throw new Error(`${id}: enter a value`);
    data[id] = Number($(id).value);
    if (!Number.isFinite(data[id])) throw new Error(`${id}: invalid number`);
  }
  if (data.M > data.N) throw new Error('Control blocks M cannot exceed shooting stages N');
  if($('engine').value==='wasm'){
    if(![[8,2],[8,4],[12,4]].some(([n,m])=>data.N===n&&data.M===m))
      throw new Error('WASM graph supports only N/M = 8/2, 8/4 or 12/4');
    for(const [key,minimum,maximum] of [
      ['max_cycles',1,100],['horizon_m',600,1500],['rpm',40,100],
      ['cross_track_m',-100,100],['heading_error_deg',-30,30],
      ['current_east_mps',-1,1],['current_north_mps',-1,1]]){
      if(data[key]<minimum||data[key]>maximum)throw new Error(`${key}: browser bounds ${minimum} to ${maximum}`);
    }
  }
  return data;
}

function switchRunning(next) {
  running = next;
  $('runBtn').classList.toggle('hidden', next);
  $('stopBtn').classList.toggle('hidden', !next);
  for (const id of numericIds) $(id).disabled = next;
  for (const id of ['route','cross_track_slider','heading_slider']) $(id).disabled = next;
  $('engine').disabled=true;
  if(!next)setEngineMode();
}

function resetRun(params) {
  stopPlayback();
  run = {parameters: params, applied: null, reference: null, waypoints: [], cycles: [], summary: null, initial: null, initialProgress: 0, length: 0, diagnostics: [], lastStage: null};
  currentIndex = -1; followLive = true; zoom = 1; followCamera = true;
  $('scenarioLabel').textContent = `/ ${routeNames[params.route]}`;
  $('summaryRmse').textContent = '—'; $('summarySolve').textContent = '—'; $('summaryFallback').textContent = '—';
  $('exportBtn').disabled = true;
  setConfigFeedback('Sending parameters', 'Waiting for Python to confirm the effective configuration…');
  updateSelectedCycle();
}

function handleEvent(evt) {
  if (evt.type === 'status') { setStatus(evt.message, 'active'); return; }
  if (evt.type === 'configuration') {
    run.applied = evt.applied;
    showAppliedConfiguration(evt.applied);
    return;
  }
  if (evt.type === 'reference') {
    run.reference = evt.xy;
    run.waypoints = evt.waypoints || [];
    run.length = evt.length_m;
    run.initial = evt.initial_state;
    run.initialProgress = evt.initial_progress_m;
    $('waypointLegendLabel').textContent = evt.waypoints_are_nominal ? 'Nominal WPs' : 'WPs';
    $('mapInfo').textContent = `${fmt(run.length / 1000, 2)} km reference · ${run.waypoints.length} WPs · N=${evt.N}, M=${evt.M}`;
    drawAll();
    return;
  }
  if (evt.type === 'cycle') {
    run.cycles.push(evt);
    const n = run.cycles.length;
    $('timeline').max = String(n - 1);
    $('timeline').disabled = false;
    ['prevBtn','playBtn','nextBtn'].forEach(id => $(id).disabled = false);
    $('exportBtn').disabled = false;
    if (followLive) currentIndex = n - 1;
    setStatus(`Cycle ${n} / ${run.parameters.max_cycles} · ${fmt(evt.log.sigma_m / 1000, 2)} km`, 'active');
    updateSelectedCycle();
    return;
  }
  if (evt.type === 'complete') {
    run.summary = evt.summary;
    $('summaryRmse').textContent = `${fmt(evt.summary.cross_track_rmse_m, 2)} m`;
    $('summarySolve').textContent = `${fmt((evt.summary.median_solver_time_s || 0) * 1000, 1)} ms`;
    $('summaryFallback').textContent = `${evt.summary.fallback_cycles} / ${evt.summary.cycles}`;
    const label = evt.summary.stream_interrupted ? 'Interrupted' : `Finished · ${evt.summary.cycles} cycles`;
    setStatus(label, '');
    return;
  }
  if (evt.type === 'error') throw new Error(evt.message);
}

async function runSimulation() {
  if (running) return;
  if($('engine').value==='wasm')return runBrowserSimulation();
  let params;
  try { params = parameterValues(); }
  catch (err) { setStatus(err.message, 'error'); return; }
  resetRun(params);
  switchRunning(true);
  abortController = new AbortController();
  setStatus('Starting Python solver…', 'active');
  try {
    const response = await fetch('/api/run', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(params), signal: abortController.signal,
    });
    const responseType = response.headers.get('content-type') || '';
    if (!response.ok) {
      if (responseType.includes('application/json')) {
        const error = await response.json();
        throw new Error(error.error || `Server error ${response.status}`);
      }
      throw new Error('Simulation API unavailable. Start the local Python dashboard_server.py and open http://127.0.0.1:8765.');
    }
    if (!responseType.includes('application/x-ndjson')) {
      throw new Error('This page is only a static preview. Run dashboard_server.py to apply parameters and solve the NMPC.');
    }
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    while (true) {
      const {value, done} = await reader.read();
      buffer += decoder.decode(value || new Uint8Array(), {stream: !done});
      let nextBreak;
      while ((nextBreak = buffer.indexOf('\n')) !== -1) {
        const line = buffer.slice(0, nextBreak).trim();
        buffer = buffer.slice(nextBreak + 1);
        if (line) handleEvent(JSON.parse(line));
      }
      if (done) break;
    }
    if (!run.summary) setStatus(`Connection closed · ${run.cycles.length} cycles recorded`, '');
  } catch (err) {
    if (err.name === 'AbortError') setStatus(`Stopped · ${run.cycles.length} cycles recorded`, '');
    else {
      console.error(err);
      setStatus(err.message || 'Simulation error', 'error');
      setConfigFeedback('Simulation did not start', err.message || 'Check Python server output.', 'error');
    }
  } finally {
    abortController = null;
    switchRunning(false);
  }
}

async function runBrowserSimulation() {
  let params;
  try{params=parameterValues();}
  catch(e){setStatus(e.message,'error');return;}
  const cycles=params.max_cycles;
  resetRun(params);switchRunning(true);
  setStatus('Loading browser FATROP · WebAssembly…','active');
  browserSolverWorker?.terminate();

  const worker=new Worker('./wasm/autonomous-worker.js?v=cycle-diagnostics-1');


  browserSolverWorker=worker;
  run.applied={...params,apply_policy:'first_shooting_interval'};
  const applyCycle=item=>{
    const beforeProgress=run.cycles.length?run.cycles.at(-1).progress_after_m:run.initialProgress;
    const state=item.stateAfter;
    handleEvent({type:'cycle',state_after:state,
      progress_before_m:beforeProgress,progress_after_m:item.sigma_m,
      elapsed_plant_time_s:item.elapsedPlantTime_s,
      prediction_state_nodes:item.prediction_state_nodes||null,
      optimized_block_commands:item.optimized_block_commands||null,
      stage_distances_m:item.stage_distances_m||null,
      block_map:item.block_map||null,
      log:{sigma_m:item.sigma_m,ey_m:item.ey_m,
        echi_deg:item.headingError_deg,V_mps:state[3],
        delta_cmd_deg:item.commandRudder_deg,rpm_actual:state[7],
        rudder_actual_deg:state[8],n_cmd_rpm:item.commandRpm,
        solve_time_s:item.solveMs/1000,
        first_shooting_interval_m:item.applyDistance_m,
        apply_ds_m:item.applyDistance_m,S_H_m:item.horizon_m,
        mode:'tracking',fallback_used:item.fallback,
        solver_success:item.accepted,
        solver_max_constraint_violation:item.constraintViolation}});
    drawAll();
  };
  worker.onmessage=({data})=>{
    if(data.type==='progress'){setStatus(data.message,'active');return;}
    if(data.type==='stage'){
      run.lastStage=data;
      run.diagnostics.push(data);
      setStatus(`Cycle ${data.cycle}/${data.total} · ${data.stage} · ${run.cycles.length} recorded`,'active');
      return;
    }
    if(data.type==='initial'){
      run.reference=data.referencePlot;
      run.length=data.referenceLength_m;
      run.initial=data.initialState;
      run.initialProgress=data.initialProgress_m;
      run.waypoints=data.waypoints||[];
      $('waypointLegendLabel').textContent=`${data.waypointsAreNominal?'Nominal WPs':'WPs'} (${run.waypoints.length})`;
      $('mapInfo').textContent=`${fmt(run.length/1000,2)} km reference · ${run.waypoints.length} WPs · WASM N=${params.N}, M=${params.M}`;
      drawAll();return;
    }
    if(data.type==='cycle'){applyCycle(data.item);return;}
    worker.terminate();browserSolverWorker=null;switchRunning(false);
    if(data.type==='error'){
      const last=run.lastStage;
      setStatus(`${data.message.split('\n')[0]} · ${run.cycles.length} recorded${last?` · last: cycle ${last.cycle} ${last.stage}`:''}`,'error');
      setConfigFeedback('Browser solve failed',data.message.split('\n')[0],'error');return;
    }
    if(!run.reference){
      run.reference=data.referencePlot;run.length=data.referenceLength_m;
      run.initial=data.initialState;run.initialProgress=data.trajectory[0]?.sigma_m||0;
      run.waypoints=data.waypoints||[];
    }
    if(!run.cycles.length)for(const item of data.rows)applyCycle(item);
    handleEvent({type:'complete',summary:{cycles:data.cyclesCompleted,
      cross_track_rmse_m:data.rmseEy_m,median_solver_time_s:data.medianSolveMs/1000,
      fallback_cycles:data.fallbackCount,stream_interrupted:data.termination!=='cycle_limit'}});
    setConfigFeedback('Applied to browser FATROP',
      `${routeNames[params.route]} · N=${params.N}, M=${params.M}, Smax=${params.horizon_m} m, RPM=${params.rpm} · ${data.acceptedCount}/${data.cyclesCompleted} accepted · ${data.fallbackCount} fallback(s)`);
    setStatus(`Browser run: ${data.cyclesCompleted}/${cycles} cycles · ${data.termination} · ${data.fallbackCount} fallback(s)`);
    drawAll();
  };
  worker.onerror=e=>{worker.terminate();browserSolverWorker=null;
    switchRunning(false);setStatus(`${e.message||'Browser worker error'} · ${run.cycles.length} recorded`,'error');};
  worker.postMessage({...params,cycles});
}

function selectCycle(i, live = false) {
  if (!run.cycles.length) return;
  currentIndex = Math.max(0, Math.min(i, run.cycles.length - 1));
  followLive = live;
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
  $('metricSolver').textContent = l ? `${fmt(1000 * l.solve_time_s, 1)} ms` : '— ms';
  $('metricDs').textContent = l ? `${fmt(l.first_shooting_interval_m, 2)} m` : '— m';
  $('metricApplied').textContent = l ? `${fmt(l.apply_ds_m, 2)} m` : '— m';
  $('metricHorizon').textContent = l ? `${fmt(l.S_H_m, 0)} m` : '— m';
  $('metricMode').textContent = l ? l.mode.toUpperCase() : '—';
  $('metricStatus').textContent = l ? (l.fallback_used ? 'FALLBACK' : (l.solver_success ? 'SUCCESS' : 'ACCEPTED')) : '—';
  $('metricViolation').textContent = l ? Number(l.solver_max_constraint_violation).toExponential(1) : '—';
  if (n && !run.summary) {
    const logs = run.cycles.map(c => c.log);
    const rmse = Math.sqrt(logs.reduce((acc, z) => acc + z.ey_m ** 2, 0) / logs.length);
    const solves = logs.map(z => z.solve_time_s).sort((a,b) => a-b);
    $('summaryRmse').textContent = `${fmt(rmse, 2)} m`;
    $('summarySolve').textContent = `${fmt(solves[Math.floor(solves.length / 2)] * 1000, 1)} ms`;
    $('summaryFallback').textContent = `${logs.filter(z => z.fallback_used).length} / ${n}`;
  }
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
    ctx.fillText('RUN A SIMULATION TO DISPLAY THE REFERENCE ROUTE',w/2,h/2);return;
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
    const pred=evt.prediction_state_nodes.map(z=>[z[0],z[1]]);
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
  // Stage 0 has already been applied at the displayed plant state. Only the
  // remaining stages from this (old) plan are projected to the right of NOW.
  const segments = [];
  let x = evt.progress_after_m;
  for (let stage = 1; stage < ds.length; stage++) {
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
  // X_phys[k] is the predicted actuator state at shooting node k. Node 1
  // corresponds to the end of the first stage, i.e. the displayed NOW.
  // Do NOT substitute the actual plant state: its model mismatch is useful.
  const points = [];
  let distance = evt.progress_after_m;
  for (let node = 1; node < states.length; node++) {
    if (node > 1) distance += ds[node - 1];
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
  const actualInitialIndex = isRpm ? 7 : 8;
  const selected = run.cycles[currentIndex];
  if (!selected) {
    ctx.textAlign='center';ctx.font='10px ui-monospace,monospace';ctx.fillStyle='#688499';
    ctx.fillText('Run simulation to view applied and planned inputs',w/2,h/2);return;
  }

  const left=36,right=10,top=17,bottom=23,pw=Math.max(1,w-left-right),ph=Math.max(1,h-top-bottom);
  const now=selected.progress_after_m;
  const horizon=Math.max(1,selected.log.S_H_m);
  const future=remainingInputPlan(selected,controlColumn);
  const predictedActual=remainingPredictedActuator(selected,actualInitialIndex);
  const futureEnd=future.length ? future[future.length-1].end : now+horizon;
  const xmin=Math.max(0,now-Math.max(horizon,350));
  const xmax=Math.max(now+100,futureEnd,now+0.45*horizon);
  const X=s=>left+pw*(s-xmin)/(xmax-xmin);
  const history=run.cycles.slice(0,currentIndex+1);
  const initialActual=run.initial?.[actualInitialIndex];
  const targetRpm=run.applied?.rpm ?? run.parameters?.rpm;
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

function exportRun() {
  if (!run.reference || !run.cycles.length) return;
  const report = {
    format: 'panamax-ddnmpc-dashboard-v1',
    parameter_policy: 'first_shooting_interval',
    parameters: run.parameters, reference_xy_m: run.reference, initial_state: run.initial,
    waypoints: run.waypoints,
    cycles: run.cycles, summary: run.summary,
    browser_diagnostics: run.diagnostics,
    last_worker_stage: run.lastStage,
  };
  const blob = new Blob([JSON.stringify(report,null,2)],{type:'application/json'});
  const url=URL.createObjectURL(blob),link=document.createElement('a');
  link.href=url;link.download=`panamax_ddnmpc_${run.parameters.route}_${new Date().toISOString().replace(/[:.]/g,'-')}.json`;
  document.body.appendChild(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
}

// State wiring.
$('route').addEventListener('change',()=>{
  $('routeHint').textContent=routeHints[$('route').value];
  $('scenarioLabel').textContent=`/ ${routeNames[$('route').value]}`;
});
for (const id of ['route', ...numericIds]) {
  $(id).addEventListener('input', reportPendingChanges);
  $(id).addEventListener('change', reportPendingChanges);
}
for (const [slider,field] of [['cross_track_slider','cross_track_m'],['heading_slider','heading_error_deg']]) {
  $(slider).addEventListener('input',()=>{$(field).value=$(slider).value;reportPendingChanges();});
  $(field).addEventListener('input',()=>$(slider).value=$(field).value);
}
$('runBtn').addEventListener('click',runSimulation);
$('stopBtn').addEventListener('click',()=>{
  abortController?.abort();
  if(browserSolverWorker){browserSolverWorker.terminate();browserSolverWorker=null;
    switchRunning(false);
    const last=run.lastStage;
    setStatus(`Stopped · ${run.cycles.length} recorded cycles${last?` · last: cycle ${last.cycle} ${last.stage}`:''}`);}
});
$('engine').addEventListener('change',setEngineMode);
$('exportBtn').addEventListener('click',exportRun);
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
setEngineMode();
setMapHeight(mapHeight);
drawAll();

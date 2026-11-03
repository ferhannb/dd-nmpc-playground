/* Browser-native tracking-mode DD-NMPC research simulator.
   Python provides only the immutable OCP graph, route, coefficients and initial
   numerical guess. No per-cycle Python states, controls or grids are used.
   Acceptance/fallback commands follow the Python tracking-mode policy;
   the warm-start and retry initialization are browser prototype variants. */
self.__DDNMPC_LIBRARY_MODE__=true;
importScripts('./reference-worker.js','./plant-worker.js');
const CDN='https://unpkg.com/@casadi/casadi-wasm@3.8.1/';
const stateScaling=[100,100,1,10,.30,.002,1,50,35];
const numberBounds=x=>Number.isFinite(x)?x:(x>0?1e20:-1e20);
const maxFinite=(array)=>array.every(Number.isFinite);
async function moduleCjs(name,requireFn){
  const response=await fetch(CDN+name);if(!response.ok)throw Error(name+' '+response.status);
  const module={exports:{}};
  return new Function('module','exports','require','__dirname','__filename',
    (await response.text())+'\n;return module.exports;')(
    module,module.exports,requireFn,CDN.replace(/\/$/,''),CDN+name);
}
function horizonFor(ref,sigma,cfg){
  let cap=cfg.S_H_max;
  for(let i=0;i<30;i++){
    try{const result=grid(ref,sigma,{...cfg,S_H_max:cap});return {...result,cap};}
    catch(e){if(!String(e).includes('Infeasible bounded horizon'))throw e;
      cap=Math.max(cfg.N*cfg.ds_min,cfg.S_arrival,cap*.95);}
  }
  throw Error('Could not build a feasible fixed-N horizon');
}
function maxViolation(values,lo,hi){
  return Math.max(0,...values.map((v,i)=>Math.max(
    Number.isFinite(lo[i])?lo[i]-v:0,Number.isFinite(hi[i])?v-hi[i]:0)));
}
function fallbackCommand(ref,nodes,state,coeff,bounds,blocks){
  // Mirrors _fresh_guess('tracking')/steady_targets_np lookup from Python:
  // 7001 rudder samples between -35 and 35 deg, minimum curvature mismatch.
  const mids=[];
  for(let k=0;k<blocks.length;k++)if(blocks[k]===0)
    mids.push(curvature((nodes[k]+nodes[k+1])/2,ref));
  const target=mids.reduce((a,b)=>a+b,0)/mids.length;
  let bestDelta=0,bestError=Infinity;
  for(let i=0;i<=7000;i++){
    const delta=-35+i*.01,u=delta/35,u2=u*u;
    const k=coeff.K1*u+coeff.K3*u*u2+coeff.K5*u*u2*u2+coeff.K7*u*u2*u2*u2;
    const error=Math.abs(k-target);
    if(error<bestError){bestError=error;bestDelta=delta;}
  }
  return [clip(state[7],bounds.rpm_cmd_min,bounds.rpm_cmd_max),
    clip(bestDelta,-bounds.rudder_cmd_abs_max_deg,bounds.rudder_cmd_abs_max_deg)];
}
function errorMetrics(state,sigma,ref){
  const [xr,yr]=sample(sigma,ref),[dx,dy]=sample(sigma,ref,1);
  const chi=Math.atan2(dx,dy);
  const ey=(state[0]-xr)*Math.cos(chi)-(state[1]-yr)*Math.sin(chi);
  let ech=state[2]-chi;
  ech=((ech+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
  return {ey_m:ey,headingError_deg:ech*180/Math.PI};
}
function percentile(xs,p){const s=xs.slice().sort((a,b)=>a-b);
  if(!s.length)return null;const pos=(s.length-1)*p,lo=Math.floor(pos),hi=Math.ceil(pos);
  if(lo===hi)return s[lo];
  return s[lo]*(hi-pos)+s[hi]*(pos-lo);}
function decodePrediction(vars,commands,graph,ds,blockMap){
  // CasADi Opti declaration order is X[0], U[0], X[1], U[1], ...
  // X[k][0:9] is normalized physical state; control/arc variables are separate.
  if(!Array.isArray(vars)||vars.length!==graph.nVariables ||
     commands.length!==2*(Math.max(...blockMap)+1))throw Error('Unexpected FATROP solution layout');
  let offset=0;
  const states=[];
  for(let k=0;k<graph.nx.length;k++){
    const nState=graph.nx[k],nInput=graph.nu[k];
    const scaled=vars.slice(offset,offset+9);
    if(scaled.length!==9)throw Error('Missing prediction state at stage '+k);
    states.push(scaled.map((v,i)=>v*stateScaling[i]));
    offset+=nState+nInput;
  }
  if(offset!==vars.length||!states.every(maxFinite))throw Error('Invalid prediction state extraction');
  const blocks=[];
  for(let j=0;j<commands.length;j+=2)blocks.push([commands[j],commands[j+1]]);
  return {prediction_state_nodes:states,optimized_block_commands:blocks,
    stage_distances_m:ds.slice(),block_map:blockMap.slice()};
}
async function runNative(options){
  const maxCycles=options.cycles;
  const route=options.route,N=options.N,M=options.M;
  if(!['straight','s_curve','complex'].includes(route)||
     ![[8,2],[8,4],[12,4]].some(([n,m])=>n===N&&m===M))
    throw Error('Unsupported browser route or FATROP graph dimensions');
  const fixture=await (await fetch(`./${route}-n${N}-m${M}.json`)).json();
  if(fixture.N!==N||fixture.M!==M||fixture.route!==route)throw Error('Graph preset mismatch');
  const ref=fixture.reference,graph=fixture.graphs.tracking;
  const runtime=fixture.controllerRuntime;
  if(!runtime||!runtime.trackingOnly||runtime.applyPolicy!=='first_shooting_interval')
    throw Error('Only tracking-only first-shooting-interval runtime is supported');
  postMessage({type:'progress',message:'Loading browser FATROP and dynamic bound graph…'});
  const wasm=await moduleCjs('casadi_wasm.js',()=>{throw Error('Unexpected require');});
  const create=await moduleCjs('casadi.js',path=>{
    if(path.endsWith('casadi_wasm.js'))return wasm;
    if(path==='path')return {join:(...a)=>a.filter(Boolean).join('/').replace(/([^:])\/{2,}/g,'$1/')};
    throw Error('Dependency '+path);
  });
  const ca=await create();await ca.load_nlpsol('fatrop');
  const fn=ca.Function.deserialize(graph.serialized);
  const boundsFn=ca.Function.deserialize(graph.constraintBoundsSerialized);
  const w=ca.MX.sym('w_native',graph.nVariables),p=ca.MX.sym('p_native',fixture.attempts[0].p.length);
  const evalFn=fn.call({x:w,p});
  const solver=ca.nlpsol('native_loop','fatrop',{x:w,p,f:evalFn.f,g:evalFn.g},graph.solverOptions);
  const config={...fixture.horizonConfigurations.tracking,S_H_max:options.horizon_m};
  const windows=fixture.projectionConfigurations.tracking;
  let state=fixture.cycles[0].stateBefore.slice();
  const initialSigma=project(state[0],state[1],ref,null);
  const [tx,ty]=sample(initialSigma,ref,1),chi=Math.atan2(tx,ty);
  const initialEy=errorMetrics(state,initialSigma,ref).ey_m;
  const eyDelta=options.cross_track_m-initialEy;
  state[0]+=eyDelta*Math.cos(chi);
  state[1]-=eyDelta*Math.sin(chi);
  state[2]+=((options.heading_error_deg-2)*Math.PI/180);
  state[7]=options.rpm;
  const physicalPlant={...fixture.plant,environment:{...fixture.plant.environment,
    current_east_mps:options.current_east_mps,
    current_north_mps:options.current_north_mps}};
  const initialState=state.slice();
  let sigma=project(state[0],state[1],ref,null);
  postMessage({type:'initial',initialState:initialState.slice(),initialProgress_m:sigma,
    referencePlot:Array.from({length:301},(_,i)=>sample(ref.length*i/300,ref)),
    referenceLength_m:ref.length,waypoints:fixture.waypoints||[],
    waypointsAreNominal:Boolean(fixture.waypointsAreNominal)});
  let uprev=[state[7],state[8]];
  let guess=fixture.attempts[0].x0.slice();
  for(let i=0;i<9;i++)guess[i]=state[i]/stateScaling[i];
  const rows=[],trajectory=[{cycle:0,x:state[0],y:state[1],sigma_m:sigma}];
  const fallbackMax=runtime.solverFailure.tracking_fallback_apply_max_m;
  const tolerance=runtime.solverFailure.tracking_candidate_violation_tol;
  let fallbackCount=0,retryCount=0,acceptedCount=0,failedSolveCount=0;
  let elapsedPlantTime=0;
  let termination='cycle_limit';
  for(let cycle=0;cycle<maxCycles;cycle++){
    if(ref.length-sigma<=runtime.arrival_m){termination='arrival';break;}
    const stage=(name,extra={})=>postMessage({type:'stage',cycle:cycle+1,total:maxCycles,
      stage:name,sigma_m:sigma,rpm_actual:state[7],...extra});
    stage('reference projection');
    const controlSigma=project(state[0],state[1],ref,sigma,
      windows.backwardWindow_m,windows.forwardWindow_m);
    const projected=project(state[0],state[1],ref,controlSigma);
    let horizon;
    stage('horizon generation');
    try{horizon=horizonFor(ref,projected,config);}
    catch(e){termination='horizon_infeasible';break;}
    stage('constraint assembly',{horizon_m:horizon.nodes.at(-1)-horizon.nodes[0]});
    const parameters=[...state,...horizon.ds,...horizon.nodes,...uprev,
      options.rpm*fixture.plant.coefficients.kV0];

    const bounds=boundsFn.call({p:ca.DM(parameters)});
    const rawL=bounds.lbg.nonzeros(),rawU=bounds.ubg.nonzeros();
    const low=ca.DM(rawL.map(numberBounds)),high=ca.DM(rawU.map(numberBounds));
    let solution,commands,cost=NaN,violation=Infinity,vars,solveMs=0;
    const attempt=x0=>{
      const start=performance.now();
      const answer=solver.call({x0:ca.DM(x0),p:ca.DM(parameters),lbg:low,ubg:high});
      solveMs+=performance.now()-start;
      const evaluated=fn.call({x:answer.x,p:ca.DM(parameters)});
      return {vars:answer.x.nonzeros(),commands:evaluated.commands.nonzeros(),
        cost:evaluated.f.nonzeros()[0],
        violation:maxViolation(evaluated.g.nonzeros(),rawL,rawU)};
    };
    let candidate=null;
    stage('FATROP solve',{vref_mps:parameters.at(-1),previous_command:uprev.slice()});
    try{candidate=attempt(guess);}catch(e){failedSolveCount++;}
    stage('candidate acceptance',{solve_ms:solveMs});
    const acceptable=c=>c&&maxFinite(c.vars)&&maxFinite(c.commands)&&
      Number.isFinite(c.cost)&&c.violation<=tolerance;
    if(!acceptable(candidate)){
      retryCount++;
      // One independent retry with a new guess generated from the current
      // previous solution/first measured state; not Python geometry init.
      const retry=guess.slice();
      for(let j=0;j<9;j++)retry[j]=state[j]/stateScaling[j];
      try{const r=attempt(retry);
        if(acceptable(r)||!candidate||r.violation<candidate.violation)candidate=r;
      }catch(e){failedSolveCount++;}
    }
    const accepted=acceptable(candidate);
    const prediction=accepted?decodePrediction(candidate.vars,candidate.commands,
      graph,horizon.ds,runtime.blockMap):null;
    if(accepted){acceptedCount++;vars=candidate.vars;commands=candidate.commands;
      cost=candidate.cost;violation=candidate.violation;guess=vars.slice();}
    else{fallbackCount++;violation=candidate?.violation??Infinity;}
    const first=accepted?commands.slice(0,2):fallbackCommand(ref,horizon.nodes,state,
      fixture.plant.coefficients,runtime.bounds,runtime.blockMap);
    const cmd=[clip(first[0],runtime.bounds.rpm_cmd_min,runtime.bounds.rpm_cmd_max),
      clip(first[1],-runtime.bounds.rudder_cmd_abs_max_deg,runtime.bounds.rudder_cmd_abs_max_deg)];
    const ds=Math.min(horizon.ds[0],ref.length-sigma,accepted?Infinity:fallbackMax);
    if(!(ds>0)){termination='nonpositive_apply_distance';break;}
    let plant;
    stage('plant propagation',{command:cmd.slice(),apply_distance_m:ds});
    try{plant=propagate(state,cmd,ds,physicalPlant,sigma,.1);}
    catch(e){termination='plant_guard: '+String(e);break;}
    state=plant.state;
    elapsedPlantTime+=plant.elapsedTime_s;
    const sigmaNext=project(state[0],state[1],ref,sigma,
      windows.backwardWindow_m,Math.max(windows.forwardWindow_m,ds+windows.backwardWindow_m));
    sigma=sigmaNext;uprev=cmd;
    const {ey_m,headingError_deg}=errorMetrics(state,sigma,ref);
    rows.push({cycle:cycle+1,accepted,fallback:!accepted,solveMs,
      cost:Number.isFinite(cost)?cost:null,
      constraintViolation:Number.isFinite(violation)?violation:null,
      ey_m,headingError_deg,commandRpm:cmd[0],commandRudder_deg:cmd[1],
      x:state[0],y:state[1],sigma_m:sigma,applyDistance_m:ds,
      stateAfter:state.slice(),elapsedPlantTime_s:elapsedPlantTime,
      horizon_m:horizon.nodes.at(-1)-horizon.nodes[0],
      initialProgress_m:horizon.nodes[0],
      ...(prediction||{})});
    postMessage({type:'cycle',item:rows.at(-1)});
    trajectory.push({cycle:cycle+1,x:state[0],y:state[1],sigma_m:sigma});
    if((cycle+1)%10===0)postMessage({type:'progress',message:`${cycle+1} cycles: ${acceptedCount} accepted, ${fallbackCount} fallback`});
  }
  const times=rows.map(r=>r.solveMs),errors=rows.map(r=>r.ey_m);
  const heading=rows.map(r=>Math.abs(r.headingError_deg));
  postMessage({type:'result',phase:'browser-native-dd-nmpc-tracking',
    pass:rows.length>0&&fallbackCount===0&&termination==='cycle_limit',
    scope:'Independent browser tracking-only closed loop; original graph and initial guess from Python; browser-specific warm start/retry, Python-derived deterministic curvature fallback. No robustness or safety certification claim.',
    termination,cyclesCompleted:rows.length,requestedCycles:maxCycles,acceptedCount,
    fallbackCount,retryCount,failedSolveCount,
    rmseEy_m:errors.length?Math.sqrt(errors.reduce((a,v)=>a+v*v,0)/errors.length):null,
    maxAbsEy_m:errors.length?Math.max(...errors.map(Math.abs)):null,
    maxAbsHeadingError_deg:heading.length?Math.max(...heading):null,
    medianSolveMs:percentile(times,.5),p95SolveMs:percentile(times,.95),
    progress_m:sigma,referenceLength_m:ref.length,initialState,rows,trajectory,
    waypoints:fixture.waypoints||[],waypointsAreNominal:Boolean(fixture.waypointsAreNominal),route,N,M,
    referencePlot:Array.from({length:301},(_,i)=>sample(ref.length*i/300,ref))});
}
self.onmessage=({data})=>{
  const options={cycles:Number(data?.cycles),route:String(data?.route||'complex'),
    N:Number(data?.N??8),M:Number(data?.M??4),horizon_m:Number(data?.horizon_m??1500),
    rpm:Number(data?.rpm??60),cross_track_m:Number(data?.cross_track_m??10),
    heading_error_deg:Number(data?.heading_error_deg??2),
    current_east_mps:Number(data?.current_east_mps??0),
    current_north_mps:Number(data?.current_north_mps??0)};
  const limits=[['cycles',1,100],['horizon_m',600,1500],['rpm',40,100],
    ['cross_track_m',-100,100],['heading_error_deg',-30,30],
    ['current_east_mps',-1,1],['current_north_mps',-1,1]];
  if(limits.some(([k,lo,hi])=>!Number.isFinite(options[k])||options[k]<lo||options[k]>hi)||
     !Number.isInteger(options.cycles)){
    postMessage({type:'error',message:'Browser input outside supported prototype bounds'});return;
  }
  runNative(options).catch(e=>postMessage({type:'error',message:String(e?.stack||e)}));
};

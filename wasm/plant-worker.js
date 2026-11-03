/* Browser-side independent implementation of the original SciPy truth-plant
   spatial RHS. Integrates with fixed-step RK4, not the NMPC prediction Fs.
   Verification target: Python DOP853 plant_transition_to_distance(). */
function clip(v,lo,hi){return Math.min(Math.max(v,lo),hi);}
function actuator(n0,d0,nCmd,dCmd,t,a){
  const nTarget=clip(nCmd,a.rpm_min,a.rpm_max);
  const dTarget=clip(dCmd,a.rudder_min_deg,a.rudder_max_deg);
  const n=clip(n0+clip(nTarget-n0,-a.rpm_rate*Math.max(0,t),a.rpm_rate*Math.max(0,t)),a.rpm_min,a.rpm_max);
  const d=clip(d0+clip(dTarget-d0,-a.rudder_rate_deg_s*Math.max(0,t),a.rudder_rate_deg_s*Math.max(0,t)),a.rudder_min_deg,a.rudder_max_deg);
  return [n,d];
}
function current(cfg,progress){
  const env=cfg.environment||{};
  const e=Number(env.current_east_mps||0),n=Number(env.current_north_mps||0);
  if((env.profile||'constant')==='constant')return [e,n];
  if(env.profile!=='sinusoidal_reverse')throw new Error('Unsupported environment profile');
  const f=clip(progress,0,1),envelope=Math.abs(Math.sin(2*Math.PI*f)),sign=f<.5?1:-1;
  return [sign*envelope*e,sign*envelope*n];
}
function rhs(s,z,x0,cmd,plant,sigma){
  const c=plant.coefficients,a=plant.actuators,g=plant.transition;
  const [, ,chi,V,beta,xi,D,tau]=z;
  if(V<=g.V_min_guard)throw new Error(`V=${V} violates plant guard`);
  const [n,d]=actuator(x0[7],x0[8],cmd[0],cmd[1],tau,a);
  const u=d/35,A=Math.abs(u),u2=u*u,u3=u2*u,u5=u3*u2,u7=u5*u2;
  const A2=A*A,A4=A2*A2,A6=A4*A2,A8=A4*A4;
  const lambdaV=1+c.A2*A2+c.A4*A4+c.A6*A6+c.A8*A8;
  const kInf=c.K1*u+c.K3*u3+c.K5*u5+c.K7*u7;
  const betaInf=c.B1*u+c.B3*u3+c.B5*u5+c.B7*u7;
  const lambdaBeta=c.lambda_beta0+c.lambda_beta_delta*A;
  const Lz=c.Lz0*Math.exp(c.gamma_Lz*A);
  const lambdaD=Math.exp(c.p0+c.p_delta*A);
  const G=Math.exp(c.ga*A);
  const dInf=1/(lambdaV*lambdaV*Math.cos(betaInf)**2);
  const betaPrime=lambdaBeta*(betaInf-beta);
  const xiPrime=c.lambda_kappa*(kInf-xi);
  const kappa=xi-Lz*c.lambda_kappa*(kInf-xi);
  const dPrime=lambdaD*(dInf-D);
  const cosB=Math.cos(beta);
  if(Math.abs(cosB)<g.cos_beta_guard)throw new Error('Sideslip cosine violates guard');
  const U=V*cosB;
  const Udot=G*(c.an2*n*n-c.cV2*D*U*Math.abs(U));
  const betaDot=V*betaPrime;
  const Vprime=(Udot+V*Math.sin(beta)*betaDot)/cosB/V;
  const fraction=(sigma+s)/Math.max(plant.referenceLength_m,1);
  const [east,north]=current(plant,fraction);
  const speed=Math.max(V,g.V_min_guard);
  return [Math.sin(chi)+east/speed,Math.cos(chi)+north/speed,kappa,
    Vprime,betaPrime,xiPrime,dPrime,1/V];
}
function propagate(state,cmd,ds,plant,sigma,maxStep){
  if(!(ds>0))throw new Error('Nonpositive application distance');
  let z=[...state.slice(0,7),0],s=0;
  const steps=Math.ceil(ds/maxStep),h=ds/steps;
  const add=(base,delta,factor)=>base.map((v,i)=>v+factor*delta[i]);
  for(let i=0;i<steps;i++){
    const k1=rhs(s,z,state,cmd,plant,sigma);
    const k2=rhs(s+h/2,add(z,k1,h/2),state,cmd,plant,sigma);
    const k3=rhs(s+h/2,add(z,k2,h/2),state,cmd,plant,sigma);
    const k4=rhs(s+h,add(z,k3,h),state,cmd,plant,sigma);
    z=z.map((v,j)=>v+h*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6);
    s+=h;
  }
  return {state:[...z.slice(0,7),...actuator(state[7],state[8],cmd[0],cmd[1],z[7],plant.actuators)],
    elapsedTime_s:z[7]};
}
async function run(){
  const response=await fetch('./panamax-ten-cycles.json');
  if(!response.ok)throw new Error('Cycle fixture HTTP '+response.status);
  const fixture=await response.json();
  if(fixture.cycles.length!==10)throw new Error('Expected ten original plant steps');
  const rows=[];
  const chained=[];
  let browserState=fixture.cycles[0].stateBefore.slice();
  for(const step of fixture.cycles){
    // Fixed spatial RK4 = independent integrator, not DOP853.
    const result=propagate(step.stateBefore,step.appliedCommand,
      step.applyDistance_m,fixture.plant,step.progressBefore_m,0.1);
    const errors=result.state.map((v,i)=>Math.abs(v-step.stateAfter[i]));
    const xyError_m=Math.hypot(errors[0],errors[1]);
    const elapsedTimeError_s=Math.abs(result.elapsedTime_s-step.elapsedTime_s);
    // Test boundaries: ≤1 cm inertial position, ≤1e-4 s time, ≤1e-4
    // on other model state components (mixed units, explicitly per component).
    const otherStateMaxError=Math.max(...errors.slice(2));
    rows.push({cycle:step.cycle+1,xyError_m,elapsedTimeError_s,
      otherStateMaxError,pass:xyError_m<=0.01&&elapsedTimeError_s<=1e-4&&
        otherStateMaxError<=1e-4});
    // Critical second experiment: use the PREVIOUS BROWSER STATE, not the
    // Python stateBefore reset at every cycle. Commands and distance remain
    // from the native reference; projection remains Python-owned.
    const accumulated=propagate(browserState,step.appliedCommand,
      step.applyDistance_m,fixture.plant,step.progressBefore_m,0.1);
    browserState=accumulated.state;
    const accumulatedStateError=browserState.map((v,i)=>Math.abs(v-step.stateAfter[i]));
    const chainedXYError_m=Math.hypot(accumulatedStateError[0],accumulatedStateError[1]);
    chained.push({cycle:step.cycle+1,xyError_m:chainedXYError_m,
      otherStateMaxError:Math.max(...accumulatedStateError.slice(2)),
      pass:chainedXYError_m<=.01&&Math.max(...accumulatedStateError.slice(2))<=1e-4});
  }
  postMessage({type:'result',pass:rows.every(r=>r.pass)&&chained.every(r=>r.pass),
    phase:'browser-plant-rk4-vs-python-dop853',
    rows,chained,passCount:rows.filter(r=>r.pass).length,
    chainedPassCount:chained.filter(r=>r.pass).length,
    scope:'Independent browser RK4 compared against ten Python DOP853 steps, including chained browser-state propagation. Applied controls, distances and reference progress are Python-provided.'});
}
if(!self.__DDNMPC_LIBRARY_MODE__)
  run().catch(e=>postMessage({type:'error',message:String(e?.stack||e)}));

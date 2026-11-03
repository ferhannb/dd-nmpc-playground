/* Independent browser implementation of SciPy natural cubic reference
   evaluation, local projection and geometry-aware fixed-N horizon grid. */
const clamp=(x,l,h)=>Math.min(Math.max(x,l),h);
function location(s,ref){
  const knots=ref.sigma;
  const x=clamp(s,0,ref.length);
  let low=0,high=knots.length-1;
  while(low+1<high){const m=(low+high)>>1;if(knots[m]<=x)low=m;else high=m;}
  return [low,x-knots[low]];
}
function evalPoly(c,i,dx,order){
  if(order===0)return ((c[0][i]*dx+c[1][i])*dx+c[2][i])*dx+c[3][i];
  if(order===1)return (3*c[0][i]*dx+2*c[1][i])*dx+c[2][i];
  return 6*c[0][i]*dx+2*c[1][i];
}
function sample(s,ref,order=0){
  if(ref.type==='piecewise'){
    const value=clamp(s,0,ref.length);
    // The Python searchsorted(..., side='right') owns the join point.
    let i=ref.pieces.findIndex(piece=>value<piece.end);
    if(i<0)i=ref.pieces.length-1;
    const piece=ref.pieces[i],local=value-piece.start;
    if(piece.type==='turn')return sample(local,piece.spline,order);
    if(order===0){const f=clamp(local/piece.length,0,1);
      return piece.p0.map((v,j)=>v+f*(piece.p1[j]-v));}
    if(order===1)return piece.p0.map((v,j)=>(piece.p1[j]-v)/piece.length);
    return [0,0];
  }
  const [i,dx]=location(s,ref);
  return [evalPoly(ref.xCoefficients,i,dx,order),evalPoly(ref.yCoefficients,i,dx,order)];
}
function curvature(s,ref){
  if(ref.type==='piecewise'){
    const value=clamp(s,0,ref.length);
    let i=ref.pieces.findIndex(piece=>value<piece.end);
    if(i<0)i=ref.pieces.length-1;
    const piece=ref.pieces[i];
    if(piece.type==='straight')return 0;
    return curvature(value-piece.start,piece.spline);
  }
  const [dx,dy]=sample(s,ref,1),[ddx,ddy]=sample(s,ref,2);
  return (dy*ddx-dx*ddy)/Math.max(dx*dx+dy*dy,1e-14)**1.5;
}
function curvatureDerivative(s,ref){
  const h=Math.max(.001,.0001*ref.length);
  const sm=clamp(s-h,0,ref.length),sp=clamp(s+h,0,ref.length);
  return (curvature(sp,ref)-curvature(sm,ref))/Math.max(sp-sm,1e-12);
}
function distanceSquared(s,x,y,ref){
  const [px,py]=sample(s,ref);return (px-x)**2+(py-y)**2;
}
function project(x,y,ref,hint,back=25,forward=150){
  const lo=hint===null?0:Math.max(0,hint-back);
  const hi=hint===null?ref.length:Math.min(ref.length,hint+forward);
  if(hi<=lo)return lo;
  const count=300,step=(hi-lo)/(count-1);
  let best=0,min=Infinity;
  for(let i=0;i<count;i++){
    const d=distanceSquared(lo+i*step,x,y,ref);
    if(d<min){min=d;best=i;}
  }
  let a=lo+Math.max(0,best-2)*step;
  let b=lo+Math.min(count-1,best+2)*step;
  if(b-a<1e-10)return lo+best*step;
  // Match SciPy bounded minimizer's golden-section core; xatol default 1e-5.
  const invphi=(3-Math.sqrt(5))/2;
  let c=a+invphi*(b-a),d=b-invphi*(b-a);
  let fc=distanceSquared(c,x,y,ref),fd=distanceSquared(d,x,y,ref);
  for(let j=0;j<120;j++){
    const mid=(a+b)/2;
    if(b-a<=1e-5+1e-12*Math.abs(mid))break;
    if(fc<=fd){b=d;d=c;fd=fc;c=a+invphi*(b-a);fc=distanceSquared(c,x,y,ref);}
    else{a=c;c=d;fc=fd;d=b-invphi*(b-a);fd=distanceSquared(d,x,y,ref);}
  }
  return clamp((a+b)/2,lo,hi);
}
function boxSum(desired,lower,upper,total){
  const N=desired.length;
  const lsum=lower.reduce((a,b)=>a+b,0),usum=upper.reduce((a,b)=>a+b,0);
  if(total<lsum-1e-10||total>usum+1e-10)throw new Error('Infeasible bounded horizon');
  let lo=Math.min(...lower.map((v,i)=>v-desired[i]))-Math.abs(total)-1;
  let hi=Math.max(...upper.map((v,i)=>v-desired[i]))+Math.abs(total)+1;
  const values=lam=>desired.map((v,i)=>clamp(v+lam,lower[i],upper[i]));
  for(let i=0;i<200;i++){
    const mid=(lo+hi)/2,x=values(mid),sum=x.reduce((a,b)=>a+b,0);
    if(Math.abs(sum-total)<=1e-10)return x;
    if(sum<total)lo=mid;else hi=mid;
  }
  const x=values((lo+hi)/2),sum=x.reduce((a,b)=>a+b,0);
  if(Math.abs(sum-total)>1e-7)throw new Error('Box projection closure');
  return x;
}
function grid(reference,sigmaCurrent,config){
  const N=config.N,remaining=Math.max(0,reference.length-sigmaCurrent);
  if(remaining<Math.max(config.S_arrival,N*config.ds_min))return {arrival:true,ds:[],nodes:[sigmaCurrent]};
  const S=Math.min(config.S_H_max,remaining),rho=config.rho;
  const base=Array.from({length:N},(_,i)=>Math.abs(rho-1)<1e-12?S/N:
    S*(rho-1)/(rho**N-1)*rho**i);
  const lower=Array(N).fill(config.ds_min);
  let ds=base.slice();
  const metrics=x=>{
    let running=0;
    return x.map(d=>{const mid=clamp(sigmaCurrent+running+d/2,0,reference.length);
      running+=d;
      return [Math.abs(curvature(mid,reference)),Math.abs(curvatureDerivative(mid,reference))];});
  };
  const upperFor=mt=>mt.map(([k])=>Math.min(config.ds_max_global,
    Math.sqrt(8*config.epsilon_sag/Math.max(k,1e-12))));
  for(let iteration=0;iteration<8;iteration++){
    const mt=metrics(ds),ku=mt.map(x=>x[0]),du=mt.map(x=>x[1]);
    const km=Math.max(...ku),dm=Math.max(...du);
    const desired=base.map((v,i)=>v/(1+
      config.curvature_weight*(km>1e-14?ku[i]/km:0)+
      config.curvature_rate_weight*(dm>1e-14?du[i]/dm:0)));
    const next=boxSum(desired,lower,upperFor(mt),S);
    const delta=Math.max(...next.map((v,i)=>Math.abs(v-ds[i])));
    ds=next;if(delta<1e-8)break;
  }
  const finalUpper=upperFor(metrics(ds));
  if(ds.some((v,i)=>v>finalUpper[i]+1e-7))ds=boxSum(ds,lower,finalUpper,S);
  let accumulated=0;
  const nodes=[sigmaCurrent];
  for(const interval of ds){accumulated+=interval;nodes.push(clamp(sigmaCurrent+accumulated,0,reference.length));}
  nodes[N]=clamp(sigmaCurrent+S,0,reference.length);
  return {arrival:false,ds,nodes};
}
async function main(){
  const response=await fetch('./panamax-ten-cycles.json');
  if(!response.ok)throw new Error('Fixture HTTP '+response.status);
  const data=await response.json(),ref=data.reference,results=[];
  for(let i=0;i<data.cycles.length;i++){
    const cycle=data.cycles[i],attempt=data.attempts[i],config=data.horizonConfigurations[cycle.mode];
    const windows=data.projectionConfigurations[cycle.mode];
    const x=cycle.stateBefore[0],y=cycle.stateBefore[1];
    const sigmaControl=project(x,y,ref,cycle.progressBefore_m,
      windows.backwardWindow_m,windows.forwardWindow_m);
    const sigmaGrid=project(x,y,ref,sigmaControl);
    const predicted=grid(ref,sigmaGrid,{...config,S_H_max:cycle.effectiveHorizonMax_m});
    if(predicted.arrival)throw new Error('Unexpected terminal grid at cycle '+i);
    const distanceError=Math.max(...predicted.ds.map((v,j)=>Math.abs(v-attempt.shootingDistances[j])));
    const projectionError=Math.abs(sigmaGrid-attempt.nominalSigmas[0]);
    const nodeError=Math.max(...predicted.nodes.map((v,j)=>Math.abs(v-attempt.nominalSigmas[j])));
    results.push({cycle:i+1,projectionError_m:projectionError,
      gridMaxError_m:distanceError,nodeMaxError_m:nodeError,
      pass:projectionError<=0.02&&distanceError<=0.05&&nodeError<=0.05});
  }
  postMessage({type:'result',pass:results.every(x=>x.pass),phase:'browser-reference-projection-grid',
    rows:results,passCount:results.filter(x=>x.pass).length,
    scope:'Natural cubic spline, local projection and fixed-N grid computed in browser; Python initial states and prior-progress hints provided.'});
}
if(!self.__DDNMPC_LIBRARY_MODE__)
  main().catch(e=>postMessage({type:'error',message:String(e?.stack||e)}));

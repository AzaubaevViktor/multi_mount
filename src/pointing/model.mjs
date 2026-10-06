// Coordinates and their observed rates; these are never motor commands.
const rad = Math.PI / 180;
export const add = (a, b) => a.map((v, i) => v + b[i]);
export const scale = (a, k) => a.map(v => v * k);
export const norm = a => Math.hypot(...a);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
export const wrapDelta = (a, b) => ((a - b + 540) % 360 + 360) % 360 - 180;

export function formatRA(hours) {
  if (!Number.isFinite(hours)) return '—';
  const ticks=((Math.round(hours*360000)%8640000)+8640000)%8640000;
  return `${String(Math.floor(ticks/360000)).padStart(2,'0')}:${String(Math.floor(ticks/6000)%60).padStart(2,'0')}:${((ticks%6000)/100).toFixed(2).padStart(5,'0')}`;
}

export function direction(basis, longitude, latitude) {
  const a = longitude * rad, b = latitude * rad;
  return add(scale(add(scale(basis[0], Math.cos(a)), scale(basis[1], Math.sin(a))), Math.cos(b)), scale(basis[2], Math.sin(b)));
}

export function frames(sample) {
  const {alt_deg: alt, az_deg: az} = sample.altaz;
  const {ra_hours: ra, dec_deg: dec} = sample.equatorial;
  const phi = sample.calibration.site.latitude_deg * rad;
  const polar = [0, Math.cos(phi), Math.sin(phi)], meridian = [0, -Math.sin(phi), Math.cos(phi)];
  const tube = direction([[0,1,0],[1,0,0],[0,0,1]], az, alt);
  const hourAngle = Math.atan2(-tube[0], dot(tube, meridian));
  const lst = hourAngle + ra * 15 * rad;
  return [
    {name:'AZ / ALT', basis:[[0,1,0],[1,0,0],[0,0,1]], lon:az, lat:alt, keys:['az','alt'], pole:'Z / AZ'},
    {name:'RA / DEC', basis:[add(scale(meridian,Math.cos(lst)),[-Math.sin(lst),0,0]), add(scale(meridian,Math.sin(lst)),[Math.cos(lst),0,0]),polar], lon:ra*15, lat:dec, keys:['ra','dec'], pole:'P / RA'},
  ];
}

export class MotionSamples {
  previous = null;
  update(sample) {
    const rates = {az:null, alt:null, ra:null, dec:null};
    if (!sample?.altaz || !sample.equatorial || !sample.calibration.site) {
      this.previous = null;
      return {sample:null, rates};
    }
    const previous = this.previous, time = Date.parse(sample.sensor.timestamp);
    const elapsed = previous ? (time - Date.parse(previous.sensor.timestamp)) / 1000 : 0;
    const sequenceDelta = previous ? (sample.sensor.sequence - previous.sensor.sequence + 2**32) % 2**32 : 0;
    if (previous && sample.calibration.revision === previous.calibration.revision && elapsed >= .2 && elapsed <= 5 && sequenceDelta > 0 && sequenceDelta < 2**31) {
      rates.alt = (sample.altaz.alt_deg - previous.altaz.alt_deg) * 3600 / elapsed;
      rates.dec = (sample.equatorial.dec_deg - previous.equatorial.dec_deg) * 3600 / elapsed;
      if (Math.abs(Math.cos(sample.altaz.alt_deg * rad)) > 1e-6 && Math.abs(Math.cos(previous.altaz.alt_deg * rad)) > 1e-6) {
        rates.az = wrapDelta(sample.altaz.az_deg, previous.altaz.az_deg) * 3600 / elapsed;
      }
      if (Math.abs(Math.cos(sample.equatorial.dec_deg * rad)) > 1e-6 && Math.abs(Math.cos(previous.equatorial.dec_deg * rad)) > 1e-6) {
        rates.ra = wrapDelta(sample.equatorial.ra_hours * 15, previous.equatorial.ra_hours * 15) * 3600 / elapsed;
      }
    }
    if (!previous || elapsed >= .2 || elapsed < 0 || sample.calibration.revision !== previous.calibration.revision || sequenceDelta === 0 || sequenceDelta >= 2**31) this.previous = sample;
    return {sample, rates};
  }
}

// Tangents in ENU, per second, for the two coordinate components of a frame.
export function velocities(frame, rates) {
  const a = frame.lon * rad, b = frame.lat * rad;
  const lon = add(scale(frame.basis[0],-Math.sin(a)),scale(frame.basis[1],Math.cos(a)));
  const lat = add(scale(add(scale(frame.basis[0],Math.cos(a)),scale(frame.basis[1],Math.sin(a))),-Math.sin(b)),scale(frame.basis[2],Math.cos(b)));
  return [rates[frame.keys[0]] == null ? null : scale(lon,Math.cos(b)*rates[frame.keys[0]]*rad/3600), rates[frame.keys[1]] == null ? null : scale(lat,rates[frame.keys[1]]*rad/3600)];
}

const colors = ['#72cced','#ffc16b','#9aeab0'];
export class OrbitView {
  yaw = 35;
  elevation = 25;
  zoom = 1;
  project([e,n,u]) {
    const yaw=this.yaw*rad, elevation=this.elevation*rad, size=94*this.zoom;
    const across=Math.cos(yaw)*e-Math.sin(yaw)*n, toward=Math.sin(yaw)*e+Math.cos(yaw)*n;
    return [220+size*across,150-size*(Math.cos(elevation)*u-Math.sin(elevation)*toward),Math.cos(elevation)*toward+Math.sin(elevation)*u];
  }
}
const views = new WeakMap();
const ns = 'http://www.w3.org/2000/svg';
const farOpacity = .38, farWidth = .55;
function element(svg, tag, attrs) {
  const item = document.createElementNS(ns, tag);
  for (const [key,value] of Object.entries(attrs)) item.setAttribute(key,value);
  svg.append(item);
  return item;
}
function path(svg, points, color, width=1, dash='', backDash=dash) {
  const projected=points.map(point=>views.get(svg).project(point)), segments=['',''];
  for (let i=1;i<projected.length;i++) {
    const a=projected[i-1], b=projected[i];
    const pieces=[];
    if ((a[2]<0)!==(b[2]<0)) {
      // Clip exactly at the camera-facing plane through the model centre.
      // A crossing line keeps its bright foreground half as the view rotates.
      const t=a[2]/(a[2]-b[2]), crossing=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),0];
      pieces.push([a,crossing],[crossing,b]);
    } else pieces.push([a,b]);
    for (const [start,end] of pieces) {
      const side=(start[2]+end[2])>=0?1:0;
      segments[side]+=`M${start[0]},${start[1]} L${end[0]},${end[1]} `;
    }
  }
  for (const [side,d] of segments.entries()) {
    if (d) element(svg,'path',{d,fill:'none',stroke:color,'stroke-width':width*(side?1:farWidth),'stroke-opacity':side?1:farOpacity,'stroke-dasharray':side?dash:backDash,'data-depth':side?'near':'far'});
  }
}
function label(svg, position, text, color='#82948a') {
  const [x,y,depth] = views.get(svg).project(position);
  const fontSize=Number(svg.dataset.fontSize), gap=fontSize*1.35;
  const width = text.length * fontSize*.6, left = Math.max(8,Math.min(432-width,x+5));
  const occupied = [...svg.querySelectorAll('text')].map(item => ({x:Number(item.getAttribute('x')),y:Number(item.getAttribute('y')),width:item.textContent.length*fontSize*.6}));
  let top = Math.max(16,Math.min(284,y-5));
  for (const offset of [0,-gap,gap,-gap*2,gap*2,-gap*3,gap*3,-gap*4,gap*4]) {
    const candidate = Math.max(16,Math.min(284,y-5+offset));
    if (!occupied.some(item => Math.abs(item.y-candidate)<gap && left<item.x+item.width+4 && left+width+4>item.x)) {top=candidate; break;}
  }
  if (Math.abs(top-(y-5))>8 || Math.abs(left-(x+5))>8) element(svg,'path',{d:`M${x},${y} L${left-2},${top-4}`,fill:'none',stroke:color,'stroke-width':.6*(depth<0?farWidth:1),'stroke-opacity':depth<0?farOpacity:1});
  element(svg,'text',{x:left,y:top,style:`fill:${color}`}).textContent = text;
}
function arrow(svg, start, end, color, text='') {
  path(svg,[start,end],color,2);
  const [x,y,depth] = views.get(svg).project(end), [sx,sy] = views.get(svg).project(start), angle = Math.atan2(y-sy,x-sx);
  const head = Math.min(8,Math.hypot(x-sx,y-sy)/3);
  element(svg,'path',{d:`M${x-head*Math.cos(angle-.45)},${y-head*Math.sin(angle-.45)} L${x},${y} L${x-head*Math.cos(angle+.45)},${y-head*Math.sin(angle+.45)}`,fill:'none',stroke:color,'stroke-width':2*(depth<0?farWidth:1),'stroke-opacity':depth<0?farOpacity:1});
  if (text) label(svg,end,text,color);
}

function cylinder(svg, start, end, radius, color) {
  const axis=add(end,scale(start,-1)), unit=scale(axis,1/norm(axis));
  const reference=Math.abs(unit[2])<.9 ? [0,0,1] : [0,1,0];
  const perpendicular=[unit[1]*reference[2]-unit[2]*reference[1],unit[2]*reference[0]-unit[0]*reference[2],unit[0]*reference[1]-unit[1]*reference[0]];
  const a=scale(perpendicular,radius/norm(perpendicular)), b=[unit[1]*a[2]-unit[2]*a[1],unit[2]*a[0]-unit[0]*a[2],unit[0]*a[1]-unit[1]*a[0]];
  const ring=Array.from({length:33},(_,i)=>add(scale(a,Math.cos(i*Math.PI/16)),scale(b,Math.sin(i*Math.PI/16))));
  for (const center of [start,end]) path(svg,ring.map(point=>add(center,point)),color,1.3);
  for (const i of [0,8,16,24]) path(svg,[add(start,ring[i]),add(end,ring[i])],color,1.3);
}

export class TelescopeModel {
  samples = new MotionSamples();
  view = new OrbitView();
  motion = {sample:null,rates:{az:null,alt:null,ra:null,dec:null}};
  site = null;
  constructor(root) {
    this.root = root;
    let drag=null;
    for (const svg of root.querySelectorAll('[data-model-view]')) {
      svg.addEventListener('pointerdown',event=>{
        if (event.button!==0 || drag) return;
        drag={id:event.pointerId,x:event.clientX,y:event.clientY,touch:event.pointerType==='touch',started:event.pointerType!=='touch'};
        svg.setPointerCapture(event.pointerId);
        if (!drag.touch) event.preventDefault();
      });
      svg.addEventListener('pointermove',event=>{
        if (!drag || drag.id!==event.pointerId) return;
        if (!drag.started) {
          const dx=event.clientX-drag.x, dy=event.clientY-drag.y;
          if (Math.hypot(dx,dy)<6) return;
          if (Math.abs(dy)>Math.abs(dx)) {drag=null; svg.releasePointerCapture(event.pointerId); return;}
          drag.started=true;
        }
        this.view.yaw=(this.view.yaw+(event.clientX-drag.x)*.4+360)%360;
        if (!drag.touch) this.view.elevation=Math.max(-85,Math.min(85,this.view.elevation-(event.clientY-drag.y)*.4));
        drag.x=event.clientX; drag.y=event.clientY; this.render();
      });
      svg.addEventListener('lostpointercapture',()=>{drag=null;});
      for (const type of ['pointerup','pointercancel']) svg.addEventListener(type,event=>{
        if (drag?.id===event.pointerId) {drag=null; svg.releasePointerCapture(event.pointerId);}
      });
    }
    for (const button of root.querySelectorAll('[data-view-action]')) button.addEventListener('click',()=>{
      const action=button.dataset.viewAction;
      if (action==='reset') this.view=new OrbitView();
      else if (action==='in' || action==='out') this.view.zoom=Math.max(.7,Math.min(1.6,this.view.zoom*(action==='in'?1.15:1/1.15)));
      else {
        this.view.yaw=(this.view.yaw+(action==='left'?-15:action==='right'?15:0)+360)%360;
        this.view.elevation=Math.max(-85,Math.min(85,this.view.elevation+(action==='up'?15:action==='down'?-15:0)));
      }
      this.render();
    });
    if (typeof ResizeObserver!=='undefined') {
      this.resizeObserver=new ResizeObserver(()=>this.render());
      for (const svg of root.querySelectorAll('[data-model-view]')) this.resizeObserver.observe(svg);
    }
  }
  update(data) {
    this.motion = this.samples.update(data);
    this.site = data?.calibration?.site ?? null;
    const {sample,rates} = this.motion;
    const status = this.root.querySelector('[data-model-status]');
    status.textContent = sample ? (rates.alt == null ? 'ДАТЧИК / ОЖИДАНИЕ СКОРОСТИ' : 'ДАТЧИК / Δ КООРДИНАТ / Δ t') : 'НЕТ РЕШЕНИЯ ДАТЧИКА';
    for (const key of ['az','alt','ra','dec']) {
      const value = sample ? (key === 'az' || key === 'alt' ? sample.altaz[`${key}_deg`] : sample.equatorial[key === 'ra' ? 'ra_hours' : 'dec_deg']) : null;
      this.root.querySelector(`[data-angle="${key}"]`).textContent = key === 'ra' ? formatRA(value) : value == null ? '—' : value.toFixed(3);
      this.root.querySelector(`[data-rate="${key}"]`).textContent = rates[key] == null ? '—' : `${rates[key] < -.005 ? '−' : rates[key] > .005 ? '+' : ''}${Math.abs(rates[key]).toFixed(2)}`;
      this.root.querySelector(`[data-direction="${key}"]`).textContent = rates[key] == null ? 'НЕИЗВ.' : Math.abs(rates[key]) < .005 ? '0' : (rates[key] > 0 ? {az:'N → E',alt:'ВВЕРХ',ra:'+ RA',dec:'К +90°'} : {az:'N → W',alt:'ВНИЗ',ra:'− RA',dec:'К −90°'})[key];
    }
    this.render();
  }
  render() {
    const {sample,rates} = this.motion;
    const modelFrames = sample ? frames(sample) : [
      {name:'AZ / ALT', basis:[[0,1,0],[1,0,0],[0,0,1]], lon:0,lat:0,keys:['az','alt'],pole:'Z / AZ'},
      {name:'RA / DEC', basis:null, lon:0,lat:0,keys:['ra','dec'],pole:'P / RA'},
    ];
    const allVelocities = modelFrames.flatMap(frame => frame.basis ? velocities(frame,rates) : []);
    const maxSpeed = Math.max(1e-12,...allVelocities.filter(v=>v).map(norm));
    const equatorial=modelFrames[1], polar=this.site ? [0,Math.cos(this.site.latitude_deg*rad),Math.sin(this.site.latitude_deg*rad)] : null;
    for (const [index,frame] of modelFrames.entries()) {
      const svg = this.root.querySelectorAll('[data-model-view]')[index];
      const bounds=svg.getBoundingClientRect(), screenScale=Math.max(.1,Math.min(bounds.width/440,bounds.height/300));
      const fontSize=Math.min(18,Math.max(12,12*screenScale))/screenScale;
      svg.dataset.fontSize=fontSize; svg.style.fontSize=`${fontSize}px`;
      views.set(svg,this.view);
      svg.replaceChildren();
      // A dim, star-free celestial grid surrounds the instrument. Dashed lines
      // belong to the far hemisphere; its frame is shared by both diagrams.
      const sphereBasis=equatorial.basis ?? [[0,1,0],[1,0,0],[0,0,1]], grid=[];
      for (const latitude of [-60,-30,0,30,60]) grid.push(Array.from({length:73},(_,i)=>scale(direction(sphereBasis,i*5,latitude),1.35)));
      for (const longitude of [0,30,60,90,120,150]) grid.push(Array.from({length:73},(_,i)=>add(scale(direction(sphereBasis,longitude,0),1.35*Math.cos(i*5*rad)),scale(sphereBasis[2],1.35*Math.sin(i*5*rad)))));
      element(svg,'circle',{cx:220,cy:150,r:1.35*94*this.view.zoom,fill:'none',stroke:'#34483c','stroke-width':.7});
      for (const points of grid) path(svg,points,'#34483c',.65,'','3 4');
      element(svg,'text',{x:8,y:290,style:'font-size:.85em'}).textContent=equatorial.basis ? 'СФЕРА / СЕТКА RA·DEC' : 'СЕТКА ENU / НЕТ ОРИЕНТАЦИИ RA·DEC';
      // ENU ground, pedestal and tripod remain visible when no trustworthy pose exists.
      for (const axis of [[1.35,0,0],[0,1.35,0],[0,0,1.35]]) arrow(svg,[0,0,0],axis,'#34483c');
      for (const [position,text] of [[[1.35,0,0],'E'],[[0,1.35,0],'N'],[[0,0,1.35],'Z']]) label(svg,position,text);
      for (const foot of [[.35,0,-.95],[-.2,.3,-.95],[-.2,-.3,-.95]]) path(svg,[[0,0,-.5],foot],'#526557');
      path(svg,[[0,0,-.6],[0,0,0]],'#526557',3);
      if (polar) {
        const poleTip=scale(polar,1.35);
        path(svg,[[0,0,0],poleTip],'#c6b3ec',1,'4 4');
        label(svg,poleTip,'NCP / ОСЬ RA','#c6b3ec');
      }
      if (equatorial.basis) {
        // Approximate Polaris catalogue direction (ICRS J2000, SIMBAD).
        // It is a directional marker, not a star field or an alignment solution.
        const polaris=scale(direction(equatorial.basis,(2+31/60+49.09456/3600)*15,89+15/60+50.7923/3600),1.35);
        arrow(svg,[0,0,0],polaris,'#d5c5f4');
        label(svg,polaris,'ПОЛЯРНАЯ ≈','#d5c5f4');
      }
      if (!frame.basis) continue;
      const ring = Array.from({length:73},(_,i)=>direction(frame.basis,i*5,0));
      path(svg,ring,'#34483c');
      if (index===0) arrow(svg,[0,0,0],scale(frame.basis[2],1.25),'#526557',frame.pole);
      label(svg,frame.basis[0],index === 0 ? 'AZ 0°' : 'RA 00:00:00');
      if (!sample) continue;
      const tip = direction(frame.basis,frame.lon,frame.lat);
      const planar = scale(direction(frame.basis,frame.lon,0),Math.cos(frame.lat*rad));
      const axial = scale(frame.basis[2],Math.sin(frame.lat*rad));
      const transverse = add(scale(frame.basis[0],-Math.sin(frame.lon*rad)),scale(frame.basis[1],Math.cos(frame.lon*rad)));
      arrow(svg,scale(transverse,-.4),scale(transverse,index === 0 ? .8 : -.8),'#526557',index === 0 ? 'ALT' : 'DEC');
      // The pointing vector is the sum of its plane and normal components.
      // Keep these position vectors visible even when the telescope is still.
      if (norm(planar) > 1e-6) arrow(svg,[0,0,0],planar,colors[0]);
      if (norm(axial) > 1e-6) arrow(svg,planar,tip,colors[1]);
      const lonArc = Array.from({length:49},(_,i)=>direction(frame.basis,frame.lon*i/48,0));
      const latArc = Array.from({length:25},(_,i)=>direction(frame.basis,frame.lon,frame.lat*i/24));
      path(svg,lonArc,colors[0]); path(svg,latArc,colors[1]);
      label(svg,scale(planar,.65),frame.keys[0].toUpperCase(),colors[0]);
      label(svg,add(planar,scale(axial,.55)),frame.keys[1].toUpperCase(),colors[1]);
      // Cylindrical optical tube and separate schematic equatorial joints.
      cylinder(svg,scale(tip,-.25),scale(tip,.9),.105,colors[2]);
      cylinder(svg,scale(tip,.86),scale(tip,.9),.125,colors[2]);
      if (polar && equatorial.basis) {
        const a=equatorial.lon*rad;
        const decAxis=add(scale(equatorial.basis[0],-Math.sin(a)),scale(equatorial.basis[1],Math.cos(a)));
        cylinder(svg,scale(polar,-.5),scale(polar,-.12),.09,colors[0]);
        cylinder(svg,scale(polar,-.36),scale(polar,-.26),.135,colors[0]);
        const center=scale(polar,-.1);
        cylinder(svg,add(center,scale(decAxis,-.28)),add(center,scale(decAxis,.28)),.085,colors[1]);
        cylinder(svg,add(center,scale(decAxis,-.06)),add(center,scale(decAxis,.06)),.125,colors[1]);
        path(svg,[scale(polar,-.5),[0,0,-.6]],'#526557',3);
        label(svg,scale(polar,-.4),'RA / УЗЕЛ',colors[0]);
        label(svg,add(center,scale(decAxis,.3)),'DEC / УЗЕЛ',colors[1]);
      }
      arrow(svg,[0,0,0],tip,colors[2]);
      label(svg,scale(tip,1.08),'LOS',colors[2]);
      const parts = velocities(frame,rates);
      for (const [i,v] of parts.entries()) {
        if (v && norm(v) > 1e-9) arrow(svg,tip,add(tip,scale(v,.55/maxSpeed)),colors[i],`v${frame.keys[i].toUpperCase()}`);
        // Small arc and arrow show the sign around each coordinate axis.
        if (v && norm(v) > 1e-9) {
          const sign = Math.sign(rates[frame.keys[i]]);
          const arc = Array.from({length:13},(_,j)=>direction(frame.basis,frame.lon+(i===0?sign*j:0),frame.lat+(i===1?sign*j:0)));
          path(svg,arc,colors[i],2);
          arrow(svg,arc.at(-2),arc.at(-1),colors[i]);
        }
      }
    }
  }
}

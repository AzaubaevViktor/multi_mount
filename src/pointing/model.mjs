// Coordinates and their observed rates; these are never motor commands.
const rad = Math.PI / 180;
export const add = (a, b) => a.map((v, i) => v + b[i]);
export const scale = (a, k) => a.map(v => v * k);
export const norm = a => Math.hypot(...a);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
export const wrapDelta = (a, b) => ((a - b + 540) % 360 + 360) % 360 - 180;

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
const project = ([e,n,u]) => [220 + 112*(.82*e-.57*n), 160 + 112*(.28*e+.4*n-.87*u)];
const ns = 'http://www.w3.org/2000/svg';
function element(svg, tag, attrs) {
  const item = document.createElementNS(ns, tag);
  for (const [key,value] of Object.entries(attrs)) item.setAttribute(key,value);
  svg.append(item);
  return item;
}
function path(svg, points, color, width=1, dash='') {
  return element(svg,'path',{d:points.map((p,i)=>`${i?'L':'M'}${project(p).join(',')}`).join(' '),fill:'none',stroke:color,'stroke-width':width,'stroke-dasharray':dash});
}
function label(svg, position, text, color='#82948a') {
  const [x,y] = project(position);
  element(svg,'text',{x:x+5,y:y-5,style:`fill:${color}`}).textContent = text;
}
function arrow(svg, start, end, color, text='') {
  path(svg,[start,end],color,2);
  const [x,y] = project(end), [sx,sy] = project(start), angle = Math.atan2(y-sy,x-sx);
  element(svg,'path',{d:`M${x-8*Math.cos(angle-.45)},${y-8*Math.sin(angle-.45)} L${x},${y} L${x-8*Math.cos(angle+.45)},${y-8*Math.sin(angle+.45)}`,fill:'none',stroke:color,'stroke-width':2});
  if (text) label(svg,end,text,color);
}

export class TelescopeModel {
  samples = new MotionSamples();
  constructor(root) { this.root = root; }
  update(data) {
    const {sample,rates} = this.samples.update(data);
    const status = this.root.querySelector('[data-model-status]');
    status.textContent = sample ? (rates.alt == null ? 'ДАТЧИК / ОЖИДАНИЕ СКОРОСТИ' : 'ДАТЧИК / Δ КООРДИНАТ / Δ t') : 'НЕТ РЕШЕНИЯ ДАТЧИКА';
    for (const key of ['az','alt','ra','dec']) {
      const value = sample ? (key === 'az' || key === 'alt' ? sample.altaz[`${key}_deg`] : sample.equatorial[key === 'ra' ? 'ra_hours' : 'dec_deg']) : null;
      this.root.querySelector(`[data-angle="${key}"]`).textContent = value == null ? '—' : value.toFixed(key === 'ra' ? 5 : 3);
      this.root.querySelector(`[data-rate="${key}"]`).textContent = rates[key] == null ? '—' : `${rates[key] < -.005 ? '−' : rates[key] > .005 ? '+' : ''}${Math.abs(rates[key]).toFixed(2)}`;
      this.root.querySelector(`[data-direction="${key}"]`).textContent = rates[key] == null ? 'НЕИЗВ.' : Math.abs(rates[key]) < .005 ? '0' : (rates[key] > 0 ? {az:'N → E',alt:'ВВЕРХ',ra:'+ RA',dec:'К +90°'} : {az:'N → W',alt:'ВНИЗ',ra:'− RA',dec:'К −90°'})[key];
    }
    const modelFrames = sample ? frames(sample) : [
      {name:'AZ / ALT', basis:[[0,1,0],[1,0,0],[0,0,1]], lon:0,lat:0,keys:['az','alt'],pole:'Z / AZ'},
      {name:'RA / DEC', basis:null, lon:0,lat:0,keys:['ra','dec'],pole:'P / RA'},
    ];
    const allVelocities = modelFrames.flatMap(frame => frame.basis ? velocities(frame,rates) : []);
    const maxSpeed = Math.max(1e-12,...allVelocities.filter(v=>v).map(norm));
    for (const [index,frame] of modelFrames.entries()) {
      const svg = this.root.querySelectorAll('[data-model-view]')[index];
      svg.replaceChildren();
      // ENU ground, pedestal and tripod remain visible when no trustworthy pose exists.
      for (const axis of [[1.35,0,0],[0,1.35,0],[0,0,1.35]]) arrow(svg,[0,0,0],axis,'#34483c');
      for (const [position,text] of [[[1.35,0,0],'E'],[[0,1.35,0],'N'],[[0,0,1.35],'Z']]) label(svg,position,text);
      for (const foot of [[.35,0,-.95],[-.2,.3,-.95],[-.2,-.3,-.95]]) path(svg,[[0,0,-.5],foot],'#526557');
      path(svg,[[0,0,-.6],[0,0,0]],'#526557',3);
      if (!frame.basis) continue;
      const ring = Array.from({length:73},(_,i)=>direction(frame.basis,i*5,0));
      path(svg,ring,'#34483c');
      arrow(svg,[0,0,0],scale(frame.basis[2],1.25),'#526557',frame.pole);
      label(svg,frame.basis[0],index === 0 ? 'AZ 0°' : 'RA 0h');
      if (!sample) continue;
      const tip = direction(frame.basis,frame.lon,frame.lat);
      const transverse = add(scale(frame.basis[0],-Math.sin(frame.lon*rad)),scale(frame.basis[1],Math.cos(frame.lon*rad)));
      arrow(svg,scale(transverse,-.4),scale(transverse,index === 0 ? .8 : -.8),'#526557',index === 0 ? 'ALT' : 'DEC');
      path(svg,[[0,0,0],direction(frame.basis,frame.lon,0),tip],'#526557',1,'3 3');
      const lonArc = Array.from({length:49},(_,i)=>direction(frame.basis,frame.lon*i/48,0));
      const latArc = Array.from({length:25},(_,i)=>direction(frame.basis,frame.lon,frame.lat*i/24));
      path(svg,lonArc,colors[0]); path(svg,latArc,colors[1]);
      label(svg,direction(frame.basis,frame.lon*.5,0),`${frame.keys[0].toUpperCase()} ${index === 0 ? frame.lon.toFixed(1)+'°' : (frame.lon/15).toFixed(2)+'h'}`,colors[0]);
      label(svg,direction(frame.basis,frame.lon,frame.lat*.55),`${frame.keys[1].toUpperCase()} ${frame.lat.toFixed(1)}°`,colors[1]);
      // Wireframe tube, with its open end on the pointing side.
      const cross = scale(transverse,.075);
      const side = scale(add(scale(direction(frame.basis,frame.lon,0),-Math.sin(frame.lat*rad)),scale(frame.basis[2],Math.cos(frame.lat*rad))),.055);
      const corners = [add(cross,side),add(scale(cross,-1),side),add(scale(cross,-1),scale(side,-1)),add(cross,scale(side,-1)),add(cross,side)];
      for (const k of [-.25,.9]) path(svg,corners.map(c=>add(scale(tip,k),c)),colors[2]);
      for (const corner of corners.slice(0,4)) path(svg,[add(scale(tip,-.25),corner),add(scale(tip,.9),corner)],colors[2]);
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

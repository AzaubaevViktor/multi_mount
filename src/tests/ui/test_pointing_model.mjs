import {test} from 'node:test';
import assert from 'node:assert/strict';
import {MotionSamples, OrbitView, TelescopeModel, direction, formatRA, frames, norm, velocities} from '../../pointing/model.mjs';

function sample(seconds, {az=0,alt=30,ra=6,dec=20,revision=0,sequence=seconds+1,latitude=43}={}) {
  return {altaz:{az_deg:az,alt_deg:alt}, equatorial:{ra_hours:ra,dec_deg:dec}, calibration:{revision,site:{latitude_deg:latitude}}, sensor:{timestamp:new Date(Date.UTC(2026,9,6,0,0,seconds)).toISOString(),sequence}};
}
function close(actual, expected, tolerance=1e-8) {
  assert.ok(Math.abs(actual-expected) < tolerance, `${actual} != ${expected}`);
}

test('RA clock formatting carries seconds and minutes and wraps at 24 hours',()=>{
  assert.equal(formatRA(2+31/60+49.09456/3600),'02:31:49.09');
  assert.equal(formatRA((59.996)/3600),'00:01:00.00');
  assert.equal(formatRA((3599.996)/3600),'01:00:00.00');
  assert.equal(formatRA(23.999999),'00:00:00.00');
  assert.equal(formatRA(null),'—');
  assert.equal(formatRA(undefined),'—');
});

for (const [name, start, end, expected] of [
  ['stationary',sample(0),sample(2),{az:0,alt:0,ra:0,dec:0}],
  ['east and up',sample(0),sample(2,{az:.01,alt:30.02,ra:6.001,dec:20.03}),{az:18,alt:36,ra:27,dec:54}],
  ['west and down',sample(0),sample(2,{az:359.99,alt:29.98,ra:5.999,dec:19.97}),{az:-18,alt:-36,ra:-27,dec:-54}],
  ['wrap north and RA zero',sample(0,{az:359.99,ra:23.999}),sample(2,{az:.01,ra:.001}),{az:36,alt:0,ra:54,dec:0}],
  ['reverse wrap',sample(0,{az:.01,ra:.001}),sample(2,{az:359.99,ra:23.999}),{az:-36,alt:0,ra:-54,dec:0}],
  ['sidereal coordinate change on a fixed tube',sample(0),sample(2,{ra:6+30.082/54000}),{az:0,alt:0,ra:15.041,dec:0}],
]) {
  test(name,()=>{
    const motion = new MotionSamples();
    assert.equal(motion.update(start).rates.az,null);
    const {rates} = motion.update(end);
    for (const key of Object.keys(expected)) close(rates[key],expected[key]);
  });
}

for (const [name,end] of [
  ['calibration changed',sample(2,{revision:1})],
  ['long gap',sample(6)],
  ['clock went backwards',sample(-2,{sequence:3})],
  ['repeated sequence',sample(2,{sequence:1})],
  ['device sequence restarted',sample(2,{sequence:0})],
]) {
  test(`${name} does not imply zero or a spike`,()=>{
    const motion = new MotionSamples(); motion.update(sample(0));
    assert.deepEqual(motion.update(end).rates,{az:null,alt:null,ra:null,dec:null});
    const next = structuredClone(end); next.sensor.timestamp = new Date(Date.parse(end.sensor.timestamp)+2000).toISOString(); next.sensor.sequence++;
    assert.equal(motion.update(next).rates.alt,0);
  });
}

test('sequence wraps at the wire u32 limit',()=>{
  const motion = new MotionSamples(); motion.update(sample(0,{sequence:2**32-1}));
  assert.equal(motion.update(sample(2,{sequence:0})).rates.az,0);
});

test('loss of coordinates or connection clears the previous pose',()=>{
  for (const missing of [null,{...sample(1),altaz:null},{...sample(1),equatorial:null}]) {
    const motion = new MotionSamples(); motion.update(sample(0));
    assert.equal(motion.update(missing).sample,null);
    assert.equal(motion.update(sample(2)).rates.alt,null);
    assert.equal(motion.update(sample(4)).rates.alt,0);
  }
});

test('fast extra refreshes do not erase the useful baseline',()=>{
  const motion = new MotionSamples(); motion.update(sample(0));
  const extra = sample(0,{sequence:2}); extra.sensor.timestamp = new Date(Date.parse(extra.sensor.timestamp)+100).toISOString();
  assert.equal(motion.update(extra).rates.alt,null);
  close(motion.update(sample(2,{alt:30.01})).rates.alt,18);
});

for (const [alt,dec] of [[90,90],[-90,-90]]) {
  test(`coordinate singularities at ${alt} degrees retain unknown rates`,()=>{
    const motion = new MotionSamples(); motion.update(sample(0,{alt,dec}));
    const {rates} = motion.update(sample(2,{alt,dec}));
    assert.equal(rates.az,null); assert.equal(rates.ra,null);
    assert.equal(rates.alt,0); assert.equal(rates.dec,0);
  });
}

// Independent spherical geometry: east/up at HA=-90°, DEC=0, latitude=0.
// RA=0 is therefore east; its positive tangent points down. A positive DEC
// tangent points north. The horizontal positive AZ tangent points south.
test('both frames agree on the optical axis and have correct tangent signs',()=>{
  const pose = sample(0,{az:90,alt:0,ra:0,dec:0,latitude:0});
  const [horizontal,equatorial] = frames(pose);
  for (const frame of [horizontal,equatorial]) {
    const tube = direction(frame.basis,frame.lon,frame.lat);
    close(tube[0],1); close(tube[1],0); close(tube[2],0);
  }
  const [vAz,vAlt] = velocities(horizontal,{az:3600,alt:3600});
  assert.ok(vAz[1]<0); assert.ok(vAlt[2]>0);
  const [vRA,vDEC] = velocities(equatorial,{ra:3600,dec:3600});
  assert.ok(vRA[2]<0); assert.ok(vDEC[1]>0);
  close(norm(vRA),Math.PI/180);
  assert.deepEqual(velocities(equatorial,{ra:null,dec:null}),[null,null]);
});

test('longitude velocity narrows with latitude, without faking pole motion',()=>{
  const frame = frames(sample(0,{az:0,alt:60}))[0];
  close(norm(velocities(frame,{az:3600,alt:0})[0]),Math.PI/360);
  frame.lat=90;
  close(norm(velocities(frame,{az:3600,alt:0})[0]),0);
});

test('camera rotation changes the view, preserving the 3D direction and radius',()=>{
  const view=new OrbitView(), point=[.3,.4,.5], original=[...point];
  for (const yaw of [0,35,90,180,270]) for (const elevation of [-85,0,25,85]) for (const roll of [0,45,90,180]) {
    view.yaw=yaw; view.elevation=elevation; view.roll=roll;
    const [x,y,depth]=view.project(point);
    close(((x-220)/94)**2+((150-y)/94)**2+depth**2,norm(point)**2);
    assert.deepEqual(point,original);
  }
  view.yaw=0; view.elevation=0; view.roll=0;
  assert.deepEqual(view.project([1,0,0]),[314,150,0]);
  assert.deepEqual(view.project([0,1,0]),[220,150,1]);
  assert.deepEqual(view.project([0,0,1]),[220,56,0]);
  view.yaw=90;
  close(view.project([1,0,0])[0],220);
  close(view.project([1,0,0])[2],1);
  close(view.project([0,1,0])[0],126);
});

test('camera zoom changes only screen distances; opposite hemispheres stay opposite',()=>{
  const view=new OrbitView(), direction=[0,Math.cos(43*Math.PI/180),Math.sin(43*Math.PI/180)];
  const initial=view.project(direction);
  view.zoom=1.6;
  const enlarged=view.project(direction), opposite=view.project(direction.map(v=>-v));
  close(enlarged[0]-220,(initial[0]-220)*1.6);
  close(enlarged[1]-150,(initial[1]-150)*1.6);
  close(enlarged[2],initial[2]);
  close(enlarged[0]+opposite[0],440);
  close(enlarged[1]+opposite[1],300);
  close(enlarged[2]+opposite[2],0);
});

test('drag capture, cancellation and view reset preserve the telemetry baseline',()=>{
  const targets=[{},{},{viewAction:'reset'}].map(dataset=>({
    dataset, listeners:new Map(), captured:null,
    addEventListener(type,callback) {this.listeners.set(type,callback);},
    setPointerCapture(id) {this.captured=id;},
    releasePointerCapture(id) {this.captured=null; this.listeners.get('lostpointercapture')({pointerId:id});},
  }));
  const model=new TelescopeModel({querySelectorAll:selector=>selector==='[data-model-view]'?targets.slice(0,2):targets.slice(2)});
  let paints=0;
  model.render=()=>{paints++;};
  model.samples.update(sample(0));
  model.motion=model.samples.update(sample(2,{az:.01}));
  const baseline=model.samples.previous, motion=model.motion;
  const event={pointerId:4,button:0,clientX:100,clientY:100,preventDefault() {}};
  targets[0].listeners.get('pointerdown')(event);
  assert.equal(targets[0].captured,4);
  targets[0].listeners.get('pointermove')({...event,pointerId:9,clientX:180});
  assert.equal(paints,0);
  targets[0].listeners.get('pointermove')({...event,clientX:180,clientY:150});
  close(model.view.yaw,3); close(model.view.elevation,45);
  assert.equal(paints,1);
  targets[0].listeners.get('pointercancel')(event);
  assert.equal(targets[0].captured,null);
  targets[0].listeners.get('pointermove')({...event,clientX:300});
  assert.equal(paints,1);
  targets[1].listeners.get('pointerdown')({...event,pointerId:5});
  targets[1].listeners.get('lostpointercapture')({pointerId:5});
  targets[1].listeners.get('pointermove')({...event,pointerId:5,clientX:300});
  assert.equal(paints,1);
  targets[2].listeners.get('click')();
  close(model.view.yaw,35); close(model.view.elevation,25); close(model.view.zoom,1);
  assert.equal(model.samples.previous,baseline);
  assert.equal(model.motion,motion);
  close(model.motion.rates.az,18);
  const touch={...event,pointerType:'touch',pointerId:6};
  targets[0].listeners.get('pointerdown')(touch);
  targets[0].listeners.get('pointermove')({...touch,clientY:150});
  close(model.view.yaw,35); close(model.view.elevation,45);
  assert.equal(targets[0].captured,6);
  targets[0].listeners.get('pointerup')(touch);
  targets[1].listeners.get('pointerdown')({...touch,pointerId:7});
  targets[1].listeners.get('pointermove')({...touch,pointerId:7,clientX:180,clientY:110});
  close(model.view.yaw,3); close(model.view.elevation,49);
  targets[1].listeners.get('pointerup')({...touch,pointerId:7});
  assert.equal(targets[1].captured,null);
  assert.equal(model.samples.previous,baseline);
  assert.equal(model.motion,motion);
  // Clockwise finger twist produces counterclockwise projected rotation.
  targets[0].listeners.get('pointerdown')({...touch,pointerId:8,clientX:100,clientY:100});
  targets[0].listeners.get('pointerdown')({...touch,pointerId:9,clientX:180,clientY:100});
  const yaw=model.view.yaw, elevation=model.view.elevation;
  targets[0].listeners.get('pointermove')({...touch,pointerId:9,clientX:100,clientY:180});
  close(model.view.roll,90); close(model.view.yaw,yaw); close(model.view.elevation,elevation);
  const right=model.view.project([1,0,0]);
  model.view.roll=0;
  const before=model.view.project([1,0,0]);
  close(right[0]-220,before[1]-150); close(right[1]-150,-(before[0]-220));
  targets[0].listeners.get('pointercancel')({...touch,pointerId:8});
  targets[0].listeners.get('pointerup')({...touch,pointerId:9});
});

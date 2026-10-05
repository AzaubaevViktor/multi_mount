import {test} from 'node:test';
import assert from 'node:assert/strict';
import {MotionSamples, direction, frames, norm, velocities} from '../../pointing/model.mjs';

function sample(seconds, {az=0,alt=30,ra=6,dec=20,revision=0,sequence=seconds+1,latitude=43}={}) {
  return {altaz:{az_deg:az,alt_deg:alt}, equatorial:{ra_hours:ra,dec_deg:dec}, calibration:{revision,site:{latitude_deg:latitude}}, sensor:{timestamp:new Date(Date.UTC(2026,9,6,0,0,seconds)).toISOString(),sequence}};
}
function close(actual, expected, tolerance=1e-8) {
  assert.ok(Math.abs(actual-expected) < tolerance, `${actual} != ${expected}`);
}

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

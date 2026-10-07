import {test} from 'node:test';
import assert from 'node:assert/strict';
import {MountPanel} from '../../pointing/telemetry.mjs';

// Only the text/attribute DOM surface used by telemetry; no browser or layout emulation.
class Element {
  textContent=''; dataset={}; children=[]; selectors=new Map();
  querySelector(selector) {
    if (!this.selectors.has(selector)) this.selectors.set(selector,new Element());
    return this.selectors.get(selector);
  }
  append(...nodes) {this.children.push(...nodes);}
  replaceChildren(...nodes) {this.children=nodes;}
  setAttribute() {}
}
globalThis.document={createElement:()=>new Element(),createElementNS:()=>new Element()};

function snapshot() {
  const axis={available:true,error:null,mode:'stop',rates:{},movement:{},processed:[],motor:{direction:'stop',motion_mode:'idle',position_native:0,speed_sps:0,power_v:12.34,initialized:true,protocol:{}}};
  return {state:'idle',clock:'2026-10-07T00:00:00Z',ra:structuredClone(axis),dec:structuredClone(axis)};
}

test('live Arduino can report a failed TMC UART while RA and gravity stay healthy',()=>{
  const root=new Element(), mount=snapshot();
  mount.dec.motor.protocol={driver_uart_connected:false,driver_flags:0x80,safety:'shutdown',enabled:false};
  new MountPanel(root).update(mount,{timestamp:mount.clock,channels:{gravity:'available',magnetic:'invalid_data'}});
  const ra=root.querySelector('[data-device="ra"]'), dec=root.querySelector('[data-device="dec"]');
  assert.equal(ra.dataset.health,'online');
  assert.equal(dec.dataset.health,'online');
  assert.equal(dec.querySelector('[data-device-link]').textContent,'ПОДКЛ.');
  assert.match(dec.querySelector('[data-device-driver]').textContent,/UART ОШИБКА/);
  assert.equal(dec.querySelector('[data-device-driver]').dataset.health,'error');
  assert.match(dec.querySelector('[data-device-diagnostic]').textContent,/FLAGS 0x80/);
  assert.equal(root.querySelector('[data-sensor-device="gravity"]').dataset.health,'online');
  assert.equal(root.querySelector('[data-sensor-device="magnetic"]').dataset.health,'error');
});

test('zero voltage is a reading, missing diagnostics and charge remain unknown',()=>{
  const root=new Element(), mount=snapshot();
  mount.ra.motor.power_v=0;
  mount.ra.motor.protocol={battery_v:'0.00',usb_v:'5.00'};
  mount.dec.motor.power_v=null;
  new MountPanel(root).update(mount);
  const ra=root.querySelector('[data-device="ra"]'), dec=root.querySelector('[data-device="dec"]');
  assert.equal(ra.querySelector('[data-device-power]').textContent,'PWR 0.00 V');
  assert.equal(ra.querySelector('[data-device-battery]').textContent,'BAT 0.00 V / USB 5.00 V / ЗАРЯД —');
  assert.equal(dec.querySelector('[data-device-power]').textContent,'PWR — V');
  assert.match(dec.querySelector('[data-device-driver]').textContent,/UART —/);
  assert.match(dec.querySelector('[data-device-diagnostic]').textContent,/FLAGS —/);
  assert.equal(root.querySelector('[data-sensor-device="gravity"]').dataset.health,'unknown');
});

test('polls retain the latest processed command and its full parameters',()=>{
  const root=new Element(), mount=snapshot(), panel=new MountPanel(root);
  mount.ra.processed=[{command:'Slew Forward 2.00',age_s:10},{command:'Slew Backward 16.00',age_s:2}];
  panel.update(mount);
  const card=root.querySelector('[data-device="ra"]');
  assert.equal(card.querySelector('[data-device-command]').textContent,'CMD SLEW BACKWARD / 2.0s ▾');
  assert.equal(card.querySelector('[data-device-command-full]').textContent,'Slew Backward 16.00');
  mount.ra.processed[1].age_s=4;
  panel.update(mount);
  assert.match(card.querySelector('[data-device-command]').textContent,/4.0s/);
  assert.equal(card.querySelector('[data-device-command-full]').textContent,'Slew Backward 16.00');
});

test('axis failure and HTTP loss clear stale hardware readings independently',()=>{
  const root=new Element(), mount=snapshot(), panel=new MountPanel(root);
  panel.update(mount,{timestamp:mount.clock,channels:{gravity:'available',magnetic:'available'}});
  mount.dec={...mount.dec,available:false,error:'serial timeout',motor:null};
  panel.update(mount,{timestamp:mount.clock,channels:{gravity:'transport_error',magnetic:'transport_error'}});
  const ra=root.querySelector('[data-device="ra"]'), dec=root.querySelector('[data-device="dec"]');
  assert.equal(ra.dataset.health,'online');
  assert.equal(dec.dataset.health,'error');
  assert.equal(dec.querySelector('[data-device-power]').textContent,'PWR — V');
  assert.match(dec.querySelector('[data-device-driver]').textContent,/UART —/);
  assert.equal(dec.querySelector('[data-device-error]').textContent,'serial timeout');
  assert.equal(root.querySelector('[data-sensor-device="gravity"]').dataset.health,'error');
  panel.update(null);
  assert.equal(ra.dataset.health,'unknown');
  assert.equal(ra.querySelector('[data-device-power]').textContent,'PWR — V');
  assert.equal(dec.querySelector('[data-device-error]').textContent,'');
  assert.equal(root.querySelector('[data-sensor-device="gravity"]').textContent,'MPU6050 / НЕИЗВ.');
  assert.match(root.querySelector('[data-sensor-poll]').textContent,/SENSOR —/);
});

const ns = 'http://www.w3.org/2000/svg';
const value = item => item == null || item === '-' ? '—' : String(item);
const number = (item,digits=2) => item == null ? '—' : Number(item).toFixed(digits);
const flag = item => item == null ? '—' : item ? 'y' : 'n';
const age = item => item == null ? '—' : `${number(item,1)}s`;
const rate = (item,axis) => item == null ? '—' : `${item >= 0 ? '+' : ''}${number(item,axis==='ra'?3:2)} ${axis==='ra'?'sRA/s':'″/s'}`;

function table(target,head,rows) {
  target.replaceChildren();
  const thead=document.createElement('thead'), tbody=document.createElement('tbody');
  const tr=document.createElement('tr');
  for (const text of head) {const th=document.createElement('th'); th.textContent=text; tr.append(th);}
  thead.append(tr);
  for (const row of rows) {
    const tr=document.createElement('tr');
    for (const [i,text] of row.entries()) {const cell=document.createElement(i===0?'th':'td'); cell.textContent=value(text); tr.append(cell);}
    tbody.append(tr);
  }
  target.append(thead,tbody);
}
function svgNode(svg,tag,attrs,text='') {
  const node=document.createElementNS(ns,tag);
  for (const [key,value] of Object.entries(attrs)) node.setAttribute(key,value);
  node.textContent=text; svg.append(node); return node;
}

export class MountPanel {
  constructor(root) {this.root=root;}
  update(mount) {
    this.root.querySelector('[data-mount-state]').textContent = mount ? `MODE ${mount.state.toUpperCase()} / RA ${mount.ra.available?'ON':'?'} / DEC ${mount.dec.available?'ON':'?'}` : 'НЕТ ДАННЫХ';
    this.root.querySelector('[data-mount-clock]').textContent = mount?.clock ?? '—';
    this.root.querySelector('[data-mount-guide]').textContent = `GUIDE ${age(mount?.polar?.guide_age_s)}`;
    this.root.querySelector('[data-mount-polar]').textContent = `POLAR ${mount?.polar?.status.toUpperCase() ?? '—'}`;
    const server=mount?.lx200?.server;
    this.root.querySelector('[data-lx200-server]').textContent = server ? `TCP ${server.listening?'ON':'OFF'} / ${server.host}:${server.port} / CLIENT ${server.client_connected?'ON':'OFF'} / REFUSED ${server.refused_clients}${server.error ? ' / '+server.error : ''}` : 'TCP —';
    const snapshots = ['ra','dec'].map(name=>mount?.[name]);
    const rows = [
      ['ОСЬ / ПОЗИЦИЯ',...snapshots.map(axis=>axis?.mount_position_text)],
      ['ОСЬ / SKY',...snapshots.map((axis,i)=>rate(axis?.sky_speed_native,i?'dec':'ra'))],
      ['ОСЬ / MOUNT 1s',...snapshots.map((axis,i)=>rate(axis?.rates.mount_native_s,i?'dec':'ra'))],
      ['ОЧЕРЕДЬ',...snapshots.map(axis=>axis?.queue_size)],
      ['МОТОР / ПОЗИЦИЯ',...snapshots.map(axis=>axis?.motor?.position_text)],
      ['ОСЬ / РЕЖИМ',...snapshots.map(axis=>axis?.mode)],
      ['МОТОР / РЕЖИМ',...snapshots.map(axis=>axis?.motor?.motion_mode)],
      ['SPEED MODE',...snapshots.map(axis=>axis?.motor?.protocol?.speed_mode)],
      ['HIGHSPEED RATIO',...snapshots.map(axis=>axis?.motor?.protocol?.highspeed_ratio)],
      ['INITIALIZED',...snapshots.map(axis=>axis?.motor?.protocol?.initialized ?? (axis?.motor ? flag(axis.motor.initialized) : null))],
      ['НАПРАВЛЕНИЕ',...snapshots.map(axis=>axis?.motor?.direction)],
      ['МОТОР / MOTOR 1s',...snapshots.map((axis,i)=>rate(axis?.rates.motor_native_s,i?'dec':'ra'))],
      ['SET / sps',...snapshots.map(axis=>number(axis?.motor?.speed_sps,0))],
      ['RAW / ШАГИ',...snapshots.map(axis=>axis?.motor?.steps)],
      ['ПИТАНИЕ / V',...snapshots.map(axis=>number(axis?.motor?.power_v))],
      ['BATTERY / USB · V',...snapshots.map(axis=>axis?.motor ? `${value(axis.motor.protocol?.battery_v)} / ${value(axis.motor.protocol?.usb_v)}` : null)],
      ['ДВИЖЕНИЕ / DIR',...snapshots.map(axis=>axis?.movement.direction)],
      ['ДВИЖЕНИЕ / VSET',...snapshots.map((axis,i)=>rate(axis?.motor?.speed_native,i?'dec':'ra'))],
      ['GOTO / ЦЕЛЬ',...snapshots.map(axis=>axis?.movement.target_text)],
      ['GOTO / ОСТАЛОСЬ',...snapshots.map((axis,i)=>axis?.movement.remaining_native == null ? null : `${number(axis.movement.remaining_native)} ${i?'″':'sRA'}`)],
      ['TRACK',...snapshots.map((axis,i)=>rate(axis?.sky_speed_native,i?'dec':'ra'))],
      ['ОШИБКА',...snapshots.map(axis=>axis?.error)],
    ];
    table(this.root.querySelector('[data-axis-table]'),['ОСИ / МОТОРЫ','RA · sRA = секунды RA','DEC · ″ = угл. секунды'],rows);
    const polar = ['ra','dec'].map(name=>mount?.polar?.[name]);
    table(this.root.querySelector('[data-polar-table]'),['ПОЛЯРНОЕ СОПРОВОЖДЕНИЕ','RA','DEC'],[
      ['AVG',polar[0] ? `${number(polar[0].average_sidereal,3)}x sid` : null, rate(polar[1]?.average_native,'dec')],
      ['SAMPLES',...polar.map(axis=>axis?.samples)],
      ['PULSE',...polar.map(axis=>age(axis?.pulse_age_s))],
      ['EXTERNAL / e',...polar.map(axis=>flag(axis?.external))],
      ['STABLE / s',...polar.map(axis=>flag(axis?.stable))],
      ['AXIS EXTERNAL / a',...polar.map(axis=>flag(axis?.axis_external))],
      ['CURRENT',...polar.map((axis,i)=>axis?.current_native == null ? null : `${number(axis.current_native)} ${i?'″':'sRA'}`)],
      ['EPS',...polar.map((axis,i)=>axis?.eps_native == null ? null : `${number(axis.eps_native)} ${i?'″':'sRA'}`)],
      ['STABLE COUNT',...polar.map(axis=>axis?.stable_count)],
      ['Δ / %',...polar.map(axis=>number(axis?.delta_percent))],
    ]);
    const stats=mount?.lx200?.stats ?? [];
    this.root.querySelector('[data-lx200-count]').textContent = `${stats.length} ТИПОВ / ${stats.reduce((n,s)=>n+s.count,0)} КОМАНД`;
    table(this.root.querySelector('[data-lx200-table]'),['КОМАНДА','КОЛИЧЕСТВО','ДАВНОСТЬ','АРГУМЕНТ'],stats.length ? stats.map(s=>[s.command,s.count,age(s.age_s),s.argument || '—']) : [['НЕТ КОМАНД','—','—','—']]);
    for (const [index,name] of ['ra','dec'].entries()) {
      const axis=snapshots[index], motor=axis?.motor;
      const svg=this.root.querySelector(`[data-motor-view="${name}"]`); svg.replaceChildren();
      const color=motor ? (index ? '#ffc16b' : '#72cced') : '#82948a';
      svgNode(svg,'circle',{cx:45,cy:45,r:27,fill:'none',stroke:'#34483c'});
      for (let i=0;i<12;i++) {const a=i*Math.PI/6; svgNode(svg,'path',{d:`M${45+29*Math.sin(a)},${45-29*Math.cos(a)} L${45+33*Math.sin(a)},${45-33*Math.cos(a)}`,stroke:'#34483c'});}
      if (motor) {
        const angle=(motor.position_native * (index ? 1 : 15) / 3600) * Math.PI/180;
        const x=45+25*Math.sin(angle),y=45-25*Math.cos(angle);
        const heading=angle-Math.PI/2;
        svgNode(svg,'path',{d:`M45,45 L${x},${y} M${x-6*Math.cos(heading-.5)},${y-6*Math.sin(heading-.5)} L${x},${y} L${x-6*Math.cos(heading+.5)},${y-6*Math.sin(heading+.5)}`,fill:'none',stroke:color,'stroke-width':2});
        if (motor.direction !== 'stop' && motor.speed_sps > 0 && motor.motion_mode !== 'idle') {
          const forward=motor.direction === 'forward';
          svgNode(svg,'path',{d:forward ? 'M21,20 A34,34 0 0 1 76,43 M71,36 L76,43 L82,37' : 'M76,43 A34,34 0 0 0 21,20 M20,28 L21,20 L29,22',fill:'none',stroke:color,'stroke-width':2});
        }
      }
      svgNode(svg,'text',{x:92,y:28,style:`fill:${color};font-size:12px`},`${name.toUpperCase()} / ${motor?.direction.toUpperCase() ?? 'НЕИЗВ.'}`);
      svgNode(svg,'text',{x:92,y:45},`${axis?.mode?.toUpperCase() ?? '—'} / ${motor?.motion_mode.toUpperCase() ?? '—'}`);
      svgNode(svg,'text',{x:92,y:62},`SET ${number(motor?.speed_sps,0)}sps / ${number(motor?.power_v)}V`);
    }
  }
}

'use strict';
const byId = id => document.getElementById(id);
const ids = ['wheel', 'lidar', 'fused'];
const colours = {wheel: '#63baff', lidar: '#f6ba63', fused: '#6be0b4'};
const arrowVisible = {wheel: true, lidar: true, fused: true};
const arrowLabels = {wheel: 'Wheel', lidar: 'RF2O', fused: 'EKF'};
const sources = Object.fromEntries(ids.map(id => [id, {trail: [], latest: null, sequence: 0, epoch: -1, session: '', status: 'waiting'}]));
const canvas = byId('plot');
const context = canvas.getContext('2d');
let connected = false, healthyInputs = false, view = null;
const fmt = (value, digits = 3) => Number.isFinite(value)
  ? (Math.abs(value) < 0.5 * 10 ** -digits ? 0 : value).toFixed(digits) : '—';
const ageText = value => Number.isFinite(value) ? `${fmt(value, 2)} s` : 'No messages';
const validPose = pose => pose && [pose.x, pose.y, pose.yaw].every(Number.isFinite);

function badge(element, text, state) {
  element.textContent = text;
  element.className = `badge ${state || ''}`;
}

function acceptSource(id, incoming) {
  const source = sources[id];
  const data = incoming || {};
  if (source.session !== data.session || source.epoch !== data.epoch) source.trail = [];
  source.session = typeof data.session === 'string' ? data.session : '';
  source.epoch = Number.isFinite(data.epoch) ? data.epoch : -1;
  source.sequence = Number.isFinite(data.sequence) ? data.sequence : 0;
  source.latest = validPose(data.latest) ? data.latest : null;
  source.status = ['live', 'waiting', 'stale'].includes(data.status) ? data.status : 'waiting';
  const limit = Math.min(6000, Math.max(1, Number(data.max_points) || 6000));
  if (Array.isArray(data.points)) {
    for (const point of data.points) if (validPose(point)) source.trail.push(point);
  }
  if (source.trail.length > limit) source.trail.splice(0, source.trail.length - limit);
  const predictionOnly = id === 'fused' && !healthyInputs && source.latest;
  const label = predictionOnly ? 'PREDICTION ONLY' : source.status.toUpperCase();
  badge(byId(`${id}-status`), label, predictionOnly ? 'warning' : source.status);
  byId(`${id}-card`).classList.toggle('untrusted', source.status !== 'live' || Boolean(predictionOnly));
  const pose = source.latest;
  for (const [metric, value, digits] of [
    ['x', pose?.x, 4], ['y', pose?.y, 4], ['yaw', pose ? pose.yaw * 180 / Math.PI : null, 1],
    ['vx', pose?.linear_x, 3], ['wz', pose?.angular_z, 3]
  ]) byId(`${id}-${metric}`).textContent = fmt(value, digits);
  if (typeof data.topic === 'string') byId(`${id}-topic`).textContent = data.topic;
  byId(`${id}-freshness`).textContent = `${fmt(data.rate_hz, 1)} Hz · age ${ageText(data.age_s)} · ${Number(data.received) || 0} samples`;
  byId(`${id}-session`).textContent = predictionOnly
    ? 'Sensor inputs are unavailable; this is not a current fused measurement'
    : `${source.trail.length} visible points · source session ${source.epoch < 0 ? '—' : source.epoch}`;
}

function niceStep(span) {
  const raw = Math.max(span / 8, 1e-6);
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map(value => value * magnitude).find(value => value >= raw);
}

function fittedView(width, height) {
  let minX = 0, minY = 0, maxX = 0, maxY = 0;
  for (const id of ids) {
    const points = sources[id].latest ? [...sources[id].trail, sources[id].latest] : sources[id].trail;
    for (const point of points) {
      minX = Math.min(minX, point.x); maxX = Math.max(maxX, point.x);
      minY = Math.min(minY, point.y); maxY = Math.max(maxY, point.y);
    }
  }
  return {cx: (minX + maxX) / 2, cy: (minY + maxY) / 2,
    metresPerPixel: Math.max(4 / width, 4 / height, (maxX - minX) * 1.25 / width, (maxY - minY) * 1.25 / height)};
}

function draw() {
  const box = canvas.getBoundingClientRect();
  if (box.width < 1 || box.height < 1) return;
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.max(1, Math.round(box.width * ratio));
  canvas.height = Math.max(1, Math.round(box.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = box.width, height = box.height, pad = 43;
  const innerWidth = Math.max(1, width - 2 * pad), innerHeight = Math.max(1, height - 2 * pad);
  if (byId('auto-fit').checked || !view) view = fittedView(innerWidth, innerHeight);
  const scale = 1 / view.metresPerPixel;
  const map = point => [width / 2 + (point.x - view.cx) * scale, height / 2 - (point.y - view.cy) * scale];
  const xMin = view.cx - innerWidth / (2 * scale), xMax = view.cx + innerWidth / (2 * scale);
  const yMin = view.cy - innerHeight / (2 * scale), yMax = view.cy + innerHeight / (2 * scale);
  const step = niceStep(Math.max(xMax - xMin, yMax - yMin));
  context.fillStyle = '#131e2b'; context.fillRect(0, 0, width, height);
  context.font = '11px system-ui'; context.lineWidth = 1;
  context.textAlign = 'center';
  for (let x = Math.ceil(xMin / step) * step, guard = 0; x <= xMax && guard < 100; x += step, guard++) {
    const px = map({x, y: 0})[0];
    context.strokeStyle = Math.abs(x) < step / 100 ? '#52677e' : '#26374b';
    context.beginPath(); context.moveTo(px, pad); context.lineTo(px, height - pad); context.stroke();
    context.fillStyle = '#9db1c8'; context.fillText(fmt(x, step < 1 ? 1 : 0), px, height - 20);
  }
  context.textAlign = 'right';
  for (let y = Math.ceil(yMin / step) * step, guard = 0; y <= yMax && guard < 100; y += step, guard++) {
    const py = map({x: 0, y})[1];
    context.strokeStyle = Math.abs(y) < step / 100 ? '#52677e' : '#26374b';
    context.beginPath(); context.moveTo(pad, py); context.lineTo(width - pad, py); context.stroke();
    context.fillStyle = '#9db1c8'; context.fillText(fmt(y, step < 1 ? 1 : 0), pad - 7, py + 4);
  }
  context.fillStyle = '#b4c6da'; context.fillText('X (m)', width - 8, height - 6);
  context.textAlign = 'left'; context.fillText('Y (m)', 9, 18);
  context.save(); context.beginPath(); context.rect(pad, pad, innerWidth, innerHeight); context.clip();
  let count = 0;
  for (const id of ids) {
    const source = sources[id];
    const degraded = !connected || source.status !== 'live' || (id === 'fused' && !healthyInputs);
    context.strokeStyle = colours[id]; context.lineWidth = id === 'fused' ? 2.4 : 1.7;
    context.globalAlpha = degraded ? 0.45 : 0.9; context.setLineDash(degraded ? [5, 5] : []);
    if (source.trail.length) {
      count += source.trail.length;
      context.beginPath();
      source.trail.forEach((point, index) => { const [x, y] = map(point); index ? context.lineTo(x, y) : context.moveTo(x, y); });
      context.stroke();
    }
    if (source.latest && arrowVisible[id]) {
      const [x, y] = map(source.latest); context.save(); context.translate(x, y); context.rotate(-source.latest.yaw);
      context.setLineDash([]); context.fillStyle = colours[id]; context.strokeStyle = '#101720'; context.lineWidth = 1.5;
      const size = id === 'fused' ? 14 : id === 'lidar' ? 11 : 8;
      context.beginPath(); context.moveTo(size, 0); context.lineTo(-size * 0.65, -size * 0.65); context.lineTo(-size * 0.35, 0); context.lineTo(-size * 0.65, size * 0.65); context.closePath(); context.fill(); context.stroke(); context.restore();
    }
  }
  context.restore(); context.globalAlpha = 1; context.setLineDash([]);
  if (!count && !ids.some(id => sources[id].latest)) {
    context.textAlign = 'center'; context.fillStyle = '#9fb1c7'; context.font = '14px system-ui';
    context.fillText('Waiting for real odometry measurements', width / 2, height / 2);
  }
}

function show(data) {
  connected = true;
  healthyInputs = data.input_health === true && !data.fault;
  for (const id of ids) acceptSource(id, data.sources?.[id]);
  const fusedLive = sources.fused.status === 'live';
  const ready = healthyInputs && fusedLive;
  const banner = byId('banner'); banner.className = `banner ${data.fault ? 'error' : ready ? '' : 'warning'}`;
  badge(byId('overall-badge'), data.fault ? 'FAULT' : ready ? 'FUSION LIVE' : 'INPUT CHECK', data.fault ? 'error' : ready ? 'live' : 'warning');
  byId('overall-message').textContent = data.fault || (ready ? 'Receiving both sensor inputs and the EKF estimate' : 'Fusion is not validated: check input freshness and alignment below');
  const alignment = data.alignment || {};
  byId('alignment-status').textContent = alignment.ready ? 'Fixed for this session' : 'Not aligned yet';
  byId('raw-separation').textContent = `${fmt(alignment.initial_disagreement_m, 3)} m · ${fmt(Number.isFinite(alignment.initial_disagreement_rad) ? alignment.initial_disagreement_rad * 180 / Math.PI : null, 1)} deg`;
  byId('alignment-detail').textContent = alignment.ready ? `Δx ${fmt(alignment.x, 3)} m · Δy ${fmt(alignment.y, 3)} m · Δyaw ${fmt(Number.isFinite(alignment.yaw) ? alignment.yaw * 180 / Math.PI : null, 1)}°` : '—';
  byId('alignment-reason').textContent = alignment.reason || 'The initial paired poses define one fixed SE(2) transform. Subsequent laser drift is not continually realigned to the wheels.';
  byId('raw-lidar-age').textContent = ageText(data.raw_lidar_age_s);
  byId('scan-age').textContent = ageText(data.scan_age_s);
  byId('input-health').textContent = healthyInputs ? 'Fresh and aligned' : 'Unavailable, stale or not aligned';
  const diagnostics = Array.isArray(data.diagnostics) ? data.diagnostics : [];
  byId('diagnostics').textContent = diagnostics.map(item => typeof item === 'string' ? item : JSON.stringify(item)).join('\n');
  const count = ids.reduce((sum, id) => sum + sources[id].trail.length, 0);
  byId('plot-state').textContent = `${count} visible points · refresh 5 Hz · ${ready ? 'both inputs fresh' : 'check source status; dashed traces are not current fused measurements'}`;
  draw();
}

async function poll() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3500);
  try {
    const query = new URLSearchParams();
    for (const id of ids) {
      query.set(`${id}_after`, sources[id].sequence);
      query.set(`${id}_epoch`, sources[id].epoch);
      query.set(`${id}_session`, sources[id].session);
    }
    const response = await fetch(`/api/state?${query}`, {cache: 'no-store', signal: controller.signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    show(await response.json());
  } catch (error) {
    connected = false; healthyInputs = false;
    byId('banner').className = 'banner error';
    badge(byId('overall-badge'), 'OFFLINE', 'offline');
    byId('overall-message').textContent = `Viewer unavailable — check the SSH tunnel (${error.name === 'AbortError' ? 'request timed out' : error.message})`;
    for (const id of ids) { badge(byId(`${id}-status`), 'OFFLINE', 'offline'); byId(`${id}-card`).classList.add('untrusted'); }
    byId('input-health').textContent = 'Not available';
    byId('plot-state').textContent = 'Connection lost. Retained values and trails are the last received measurements.';
    draw();
  } finally {
    clearTimeout(timeout);
    setTimeout(poll, 200);
  }
}

byId('fit').addEventListener('click', () => { view = null; draw(); });
byId('clear').addEventListener('click', () => {
  for (const id of ids) sources[id].trail = [];
  view = null; draw();
  byId('plot-state').textContent = 'Visible trails cleared in this browser. Robot odometry, EKF and server CSV history were not reset.';
});
byId('auto-fit').addEventListener('change', draw);
for (const id of ids) {
  const button = byId(`${id}-arrow-toggle`);
  button.addEventListener('click', () => {
    arrowVisible[id] = !arrowVisible[id];
    button.setAttribute('aria-pressed', String(arrowVisible[id]));
    button.textContent = `${arrowLabels[id]} arrow: ${arrowVisible[id] ? 'ON' : 'OFF'}`;
    draw();
  });
}
window.addEventListener('resize', draw);
draw(); poll();

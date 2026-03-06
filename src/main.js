/**
 * main.js
 *
 * アプリケーションエントリポイント
 * UI との接続、レンダーループ管理
 */

import { QuadrotorModel }    from './physics/QuadrotorModel.js';
import { RK4Integrator }     from './physics/Integrator.js';
import { FlightController }  from './control/FlightController.js';
import { DisturbanceModel }  from './disturbance/DisturbanceModel.js';
import { Simulator }         from './simulation/Simulator.js';
import { DroneVisualizer }   from './visualization/DroneVisualizer.js';

// ── インスタンス生成 ──────────────────────────────────────────────────────
const model       = new QuadrotorModel();
const integrator  = new RK4Integrator();
const disturbance = new DisturbanceModel();
const controller  = new FlightController(model);
const simulator   = new Simulator({ model, integrator, controller, disturbance });

const container   = document.getElementById('canvas-container');
const visualizer  = new DroneVisualizer(container, model);

// デバッグ用にグローバルへ公開
window._sim = simulator;

// ── UI 要素の取得 ──────────────────────────────────────────────────────────
const $ = (id) => document.getElementById(id);

const btnStart   = $('btn-start');
const btnPause   = $('btn-pause');
const btnReset   = $('btn-reset');

const spX   = $('sp-x');
const spY   = $('sp-y');
const spZ   = $('sp-z');
const spPsi = $('sp-psi');

const tTime  = $('t-time');
const tX = $('t-x');   const tY  = $('t-y');     const tZ     = $('t-z');
const tVx = $('t-vx'); const tVy = $('t-vy');    const tVz    = $('t-vz');
const tPhi = $('t-phi'); const tTheta = $('t-theta'); const tPsi = $('t-psi');
const tM1 = $('t-m1'); const tM2 = $('t-m2');    const tM3    = $('t-m3'); const tM4 = $('t-m4');

// PIDゲインスライダ
const gainFields = [
  'pos-xy-kp', 'pos-xy-ki', 'pos-xy-kd',
  'pos-z-kp',  'pos-z-ki',  'pos-z-kd',
  'att-rp-kp', 'att-rp-ki', 'att-rp-kd',
  'att-yaw-kp','att-yaw-ki','att-yaw-kd',
];

// スライダ + 数値入力を同期してセット
function setGainVal(id, val) {
  const slider = $(id);
  const numEl  = $(`${id}-num`);
  if (slider) slider.value = val;
  if (numEl)  numEl.value  = parseFloat(val).toFixed(3);
}

// 初期値を UI に反映
function syncGainsToUI() {
  const g = controller.getGains();
  setGainVal('pos-xy-kp', g.pos.xy.kp);
  setGainVal('pos-xy-ki', g.pos.xy.ki);
  setGainVal('pos-xy-kd', g.pos.xy.kd);
  setGainVal('pos-z-kp',  g.pos.z.kp);
  setGainVal('pos-z-ki',  g.pos.z.ki);
  setGainVal('pos-z-kd',  g.pos.z.kd);
  setGainVal('att-rp-kp', g.att.roll.kp);
  setGainVal('att-rp-ki', g.att.roll.ki);
  setGainVal('att-rp-kd', g.att.roll.kd);
  setGainVal('att-yaw-kp',g.att.yaw.kp);
  setGainVal('att-yaw-ki',g.att.yaw.ki);
  setGainVal('att-yaw-kd',g.att.yaw.kd);
}

// ── イベントハンドラ ──────────────────────────────────────────────────────

btnStart.addEventListener('click', () => {
  simulator.start();
  btnStart.disabled = true;
  btnPause.disabled = false;
});

btnPause.addEventListener('click', () => {
  simulator.pause();
  btnStart.disabled = false;
  btnPause.disabled = true;
});

btnReset.addEventListener('click', () => {
  simulator.reset({ z: 0 });
  visualizer.clearTrajectory();
  btnStart.disabled = false;
  btnPause.disabled = true;
  updateSetpoint();
});

// セットポイント変更
[spX, spY, spZ, spPsi].forEach((el) => el && el.addEventListener('input', updateSetpoint));

function updateSetpoint() {
  const x   = parseFloat(spX?.value   ?? 0);
  const y   = parseFloat(spY?.value   ?? 0);
  const z   = parseFloat(spZ?.value   ?? 1);
  const psi = parseFloat(spPsi?.value ?? 0) * Math.PI / 180;
  controller.setSetpoint({ x, y, z, psi });

  // number input に同期 (スライダ → 数値入力)
  const nx = $('sp-x-num');   if (nx) nx.value = x.toFixed(1);
  const ny = $('sp-y-num');   if (ny) ny.value = y.toFixed(1);
  const nz = $('sp-z-num');   if (nz) nz.value = z.toFixed(1);
  const np = $('sp-psi-num'); if (np) np.value = (psi * 180 / Math.PI).toFixed(0);
}

// 数値入力 → スライダ双方向同期
[['sp-x-num', spX], ['sp-y-num', spY], ['sp-z-num', spZ], ['sp-psi-num', spPsi]].forEach(([numId, rangeEl]) => {
  $(`${numId}`)?.addEventListener('input', () => {
    const v = $(`${numId}`).value;
    if (rangeEl) rangeEl.value = v;
    updateSetpoint();
  });
});

// PID ゲイン スライダ ↔ 数値入力 双方向同期
gainFields.forEach((id) => {
  const slider = $(id);
  const numEl  = $(`${id}-num`);
  if (slider) {
    slider.addEventListener('input', () => {
      if (numEl) numEl.value = parseFloat(slider.value).toFixed(3);
      applyGainsFromUI();
    });
  }
  if (numEl) {
    numEl.addEventListener('input', () => {
      if (slider) slider.value = numEl.value;
      applyGainsFromUI();
    });
  }
});

function applyGainsFromUI() {
  const v = (id) => parseFloat($(id)?.value ?? 0);
  controller.setGains({
    pos: {
      xy: { kp: v('pos-xy-kp'), ki: v('pos-xy-ki'), kd: v('pos-xy-kd') },
      z:  { kp: v('pos-z-kp'),  ki: v('pos-z-ki'),  kd: v('pos-z-kd')  },
    },
    att: {
      roll:  { kp: v('att-rp-kp'),  ki: v('att-rp-ki'),  kd: v('att-rp-kd')  },
      pitch: { kp: v('att-rp-kp'),  ki: v('att-rp-ki'),  kd: v('att-rp-kd')  },
      yaw:   { kp: v('att-yaw-kp'), ki: v('att-yaw-ki'), kd: v('att-yaw-kd') },
    },
  });
}

// 外乱ボタン
$('btn-wind-x')?.addEventListener('click', () => disturbance.setWind(3, 0, 0));
$('btn-wind-y')?.addEventListener('click', () => disturbance.setWind(0, 3, 0));
$('btn-gust')?.addEventListener('click', () => {
  const sign  = () => Math.random() < 0.5 ? 1 : -1;
  const mag   = 4 + Math.random() * 12;          // 4〜16 N
  const angle = Math.random() * Math.PI * 2;     // 水平方向ランダム
  disturbance.addGust({
    fx:       Math.cos(angle) * mag * sign(),
    fy:       Math.sin(angle) * mag * sign(),
    fz:       (Math.random() - 0.3) * 4,          // 上下方向も少し
    tz:       (Math.random() - 0.5) * 0.4,        // ヨー方向トルク
    duration: 0.1 + Math.random() * 0.4,          // 0.1〜0.5 s
  });
});
$('btn-turb')  ?.addEventListener('click', () => {
  const cur = disturbance._turbulence.intensity;
  disturbance.setTurbulence(cur > 0 ? 0 : 0.5);
  const el = $('btn-turb');
  el.textContent = 'Turb';
  el.classList.toggle('on', cur === 0);
});
$('btn-clear-dist')?.addEventListener('click', () => {
  disturbance.clearAll();
  const el = $('btn-turb');
  if (el) { el.textContent = 'Turb'; el.classList.remove('on'); }
});

// ── ゲイン エクスポート / インポート ──────────────────────────────────────
$('btn-gains-export')?.addEventListener('click', () => {
  const json = JSON.stringify(controller.getGains(), null, 2);
  const url  = URL.createObjectURL(new Blob([json], { type: 'application/json' }));
  Object.assign(document.createElement('a'), { href: url, download: 'xdrone-gains.json' }).click();
  URL.revokeObjectURL(url);
});

$('btn-gains-import')?.addEventListener('click', () => $('gains-file-input')?.click());

$('gains-file-input')?.addEventListener('change', (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (ev) => {
    try {
      controller.setGains(JSON.parse(ev.target.result));
      syncGainsToUI();
    } catch { /* invalid JSON — 無視 */ }
  };
  reader.readAsText(file);
  e.target.value = '';
});

// ── カメラコントロール ─────────────────────────────────────────────────────
$('cam-xy')?.addEventListener('click', () => visualizer.setCameraView('xy'));
$('cam-xz')?.addEventListener('click', () => visualizer.setCameraView('xz'));
$('cam-yz')?.addEventListener('click', () => visualizer.setCameraView('yz'));
$('cam-fit')?.addEventListener('click', () => visualizer.fitView(simulator.state));
$('cam-follow')?.addEventListener('click', () => {
  const enabled = !visualizer._autoFollow;
  visualizer.setAutoFollow(enabled);
  $('cam-follow').classList.toggle('on', enabled);
});

// ── レンダーループ ────────────────────────────────────────────────────────

let prevTime = null;
let trajCounter = 0;

function animate(timestamp) {
  requestAnimationFrame(animate);

  if (prevTime === null) { prevTime = timestamp; }
  const elapsed = Math.min((timestamp - prevTime) / 1000, 0.05); // 最大50ms
  prevTime = timestamp;

  // 物理シミュレーションを進める
  simulator.runFor(elapsed, 200);

  // 可視化更新
  visualizer.update(simulator.state, simulator.omega2, controller.setpoint, elapsed);

  // トラジェクトリ記録 (5フレームに1点)
  trajCounter++;
  if (trajCounter >= 3) {
    trajCounter = 0;
    const s = simulator.state;
    visualizer.addTrajectoryPoint(s[0], s[1], s[2]);
  }

  // HUD 更新
  updateHUD();

  visualizer.render();
}

function updateHUD() {
  const snap = simulator.getSnapshot();
  const fmt3 = (v) => (v >= 0 ? '+' : '') + v.toFixed(3);
  const fmt1 = (v) => (v >= 0 ? '+' : '') + v.toFixed(2);

  if (tTime)  tTime.textContent  = snap.time.toFixed(2);
  if (tX)     tX.textContent     = fmt3(snap.position.x);
  if (tY)     tY.textContent     = fmt3(snap.position.y);
  if (tZ)     tZ.textContent     = fmt3(snap.position.z);
  if (tVx)    tVx.textContent    = fmt3(snap.velocity.x);
  if (tVy)    tVy.textContent    = fmt3(snap.velocity.y);
  if (tVz)    tVz.textContent    = fmt3(snap.velocity.z);
  if (tPhi)   tPhi.textContent   = fmt1(snap.attitude.phi);
  if (tTheta) tTheta.textContent = fmt1(snap.attitude.theta);
  if (tPsi)   tPsi.textContent   = fmt1(snap.attitude.psi);
  if (tM1)    tM1.textContent    = snap.omega[0].toFixed(0);
  if (tM2)    tM2.textContent    = snap.omega[1].toFixed(0);
  if (tM3)    tM3.textContent    = snap.omega[2].toFixed(0);
  if (tM4)    tM4.textContent    = snap.omega[3].toFixed(0);
}

// ── 初期化 ────────────────────────────────────────────────────────────────
syncGainsToUI();
updateSetpoint();
requestAnimationFrame(animate);

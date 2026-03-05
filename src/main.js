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

const infoTime  = $('info-time');
const infoPos   = $('info-pos');
const infoVel   = $('info-vel');
const infoAtt   = $('info-att');
const infoOmega = $('info-omega');

// PIDゲインスライダ
const gainFields = [
  'pos-xy-kp', 'pos-xy-ki', 'pos-xy-kd',
  'pos-z-kp',  'pos-z-ki',  'pos-z-kd',
  'att-rp-kp', 'att-rp-ki', 'att-rp-kd',
  'att-yaw-kp','att-yaw-ki','att-yaw-kd',
];

// 初期値を UI に反映
function syncGainsToUI() {
  const g = controller.getGains();
  setSliderVal('pos-xy-kp', g.pos.xy.kp);
  setSliderVal('pos-xy-ki', g.pos.xy.ki);
  setSliderVal('pos-xy-kd', g.pos.xy.kd);
  setSliderVal('pos-z-kp',  g.pos.z.kp);
  setSliderVal('pos-z-ki',  g.pos.z.ki);
  setSliderVal('pos-z-kd',  g.pos.z.kd);
  setSliderVal('att-rp-kp', g.att.roll.kp);
  setSliderVal('att-rp-ki', g.att.roll.ki);
  setSliderVal('att-rp-kd', g.att.roll.kd);
  setSliderVal('att-yaw-kp',g.att.yaw.kp);
  setSliderVal('att-yaw-ki',g.att.yaw.ki);
  setSliderVal('att-yaw-kd',g.att.yaw.kd);
}

function setSliderVal(id, val) {
  const el = $(id);
  if (el) { el.value = val; updateSliderLabel(el); }
}

function updateSliderLabel(slider) {
  const label = document.querySelector(`label[for="${slider.id}"] .val`);
  if (label) label.textContent = parseFloat(slider.value).toFixed(3);
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

  // ラベル更新
  const lx = $('sp-x-val');   if (lx) lx.textContent = x.toFixed(1);
  const ly = $('sp-y-val');   if (ly) ly.textContent = y.toFixed(1);
  const lz = $('sp-z-val');   if (lz) lz.textContent = z.toFixed(1);
  const lp = $('sp-psi-val'); if (lp) lp.textContent = (psi * 180 / Math.PI).toFixed(0);
}

// PID ゲインスライダ変更
gainFields.forEach((id) => {
  const el = $(id);
  if (!el) return;
  el.addEventListener('input', () => {
    updateSliderLabel(el);
    applyGainsFromUI();
  });
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
$('btn-gust')  ?.addEventListener('click', () => disturbance.addGust({ fx: 5, duration: 0.3 }));
$('btn-turb')  ?.addEventListener('click', () => {
  const cur = disturbance._turbulence.intensity;
  disturbance.setTurbulence(cur > 0 ? 0 : 0.5);
  const el = $('btn-turb');
  el.textContent = cur > 0 ? '乱流 OFF' : '乱流 ON';
  el.classList.toggle('active', cur === 0);
});
$('btn-clear-dist')?.addEventListener('click', () => {
  disturbance.clearAll();
  const el = $('btn-turb');
  if (el) { el.textContent = '乱流 OFF'; el.classList.remove('active'); }
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
  if (infoTime)  infoTime.textContent  = snap.time.toFixed(2) + ' s';
  if (infoPos)   infoPos.textContent   =
    `x:${snap.position.x.toFixed(2)} y:${snap.position.y.toFixed(2)} z:${snap.position.z.toFixed(2)}`;
  if (infoVel)   infoVel.textContent   =
    `vx:${snap.velocity.x.toFixed(2)} vy:${snap.velocity.y.toFixed(2)} vz:${snap.velocity.z.toFixed(2)}`;
  if (infoAtt)   infoAtt.textContent   =
    `φ:${snap.attitude.phi.toFixed(1)}° θ:${snap.attitude.theta.toFixed(1)}° ψ:${snap.attitude.psi.toFixed(1)}°`;
  if (infoOmega) infoOmega.textContent =
    snap.omega.map((o) => o.toFixed(0)).join(' / ') + ' rad/s';
}

// ── 初期化 ────────────────────────────────────────────────────────────────
syncGainsToUI();
updateSetpoint();
requestAnimationFrame(animate);

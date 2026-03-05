/**
 * Simulator.js
 *
 * メインシミュレーションループ
 *
 * 責務:
 *   - 物理モデル・積分器・コントローラ・外乱モデルを統合
 *   - 固定時間刻みで状態を更新
 *   - シミュレーション時間管理
 *   - ログ記録
 */

import { QuadrotorModel, STATE_IDX } from '../physics/QuadrotorModel.js';
import { RK4Integrator }             from '../physics/Integrator.js';
import { FlightController }          from '../control/FlightController.js';
import { DisturbanceModel }          from '../disturbance/DisturbanceModel.js';

/** デフォルト設定 */
const DEFAULT_CONFIG = {
  /** 物理シミュレーション周波数 [Hz] */
  physicsHz: 500,
  /** 制御器更新周波数 [Hz] (physics以下, physicsの約数を推奨) */
  controlHz: 100,
  /** トラジェクトリ記録の間引き (N物理ステップごとに1点) */
  logDecimate: 10,
  /** トラジェクトリ最大点数 */
  maxLogPoints: 2000,
};

export class Simulator {
  /**
   * @param {object} options
   * @param {import('../physics/QuadrotorModel.js').QuadrotorModel} [options.model]
   * @param {import('../physics/Integrator.js').RK4Integrator}     [options.integrator]
   * @param {import('../control/FlightController.js').FlightController} [options.controller]
   * @param {import('../disturbance/DisturbanceModel.js').DisturbanceModel} [options.disturbance]
   * @param {typeof DEFAULT_CONFIG} [options.config]
   */
  constructor(options = {}) {
    this.model       = options.model       ?? new QuadrotorModel();
    this.integrator  = options.integrator  ?? new RK4Integrator();
    this.controller  = options.controller  ?? new FlightController(this.model);
    this.disturbance = options.disturbance ?? new DisturbanceModel();
    this.config      = { ...DEFAULT_CONFIG, ...(options.config ?? {}) };

    /** 現在の状態ベクトル */
    this.state = this.model.createState({ z: 0 });

    /** 最後に計算したローター角速度二乗 */
    this.omega2 = [0, 0, 0, 0];

    /** シミュレーション時刻 [s] */
    this.time = 0;

    /** 実行中フラグ */
    this.running = false;

    /** 制御器の内部カウンタ (制御器更新タイミング管理) */
    this._controlCounter = 0;
    this._controlStep = Math.round(this.config.physicsHz / this.config.controlHz);

    /** トラジェクトリログ */
    this._logCounter = 0;
    this.trajectory  = [];

    /** 状態更新コールバック */
    this.onStep = null;
  }

  /**
   * シミュレーションを開始
   */
  start() {
    this.running = true;
  }

  /**
   * シミュレーションを一時停止
   */
  pause() {
    this.running = false;
  }

  /**
   * シミュレーションをリセット
   * @param {{ x?, y?, z?, psi? }} initState
   */
  reset(initState = {}) {
    this.running = false;
    this.state   = this.model.createState(initState);
    this.omega2  = [0, 0, 0, 0];
    this.time    = 0;
    this._controlCounter = 0;
    this._logCounter     = 0;
    this.trajectory      = [];
    this.controller.reset();
    this.disturbance.clearAll();
  }

  /**
   * 1物理ステップを進める
   * 通常は runSteps() か requestAnimationFrame ループから呼ぶ
   */
  step() {
    if (!this.running) return;

    const dt = 1 / this.config.physicsHz;
    const S  = STATE_IDX;

    // ── 制御器更新 ──────────────────────────────────────────────────────
    this._controlCounter++;
    if (this._controlCounter >= this._controlStep) {
      this._controlCounter = 0;
      const ctrlDt = this._controlStep / this.config.physicsHz;
      this.omega2 = this.controller.compute(this.state, ctrlDt);
    }

    // ── 外乱計算 ────────────────────────────────────────────────────────
    const vel = [this.state[S.VX], this.state[S.VY], this.state[S.VZ]];
    const d = this.disturbance.compute(dt, vel);
    this.model.disturbance = d;

    // ── 物理積分 ────────────────────────────────────────────────────────
    const derivFn = (s, u) => this.model.derivative(s, u);
    this.state = this.integrator.integrate(derivFn, this.state, this.omega2, dt);

    // ピッチ特異点回避: theta を [-π/2+ε, π/2-ε] にクランプ
    const EPS = 0.01;
    this.state[S.THETA] = Math.max(-Math.PI / 2 + EPS, Math.min(Math.PI / 2 - EPS, this.state[S.THETA]));

    // 地面コリジョン
    if (this.state[S.Z] < 0) {
      this.state[S.Z]  = 0;
      this.state[S.VZ] = Math.max(0, this.state[S.VZ]);
    }

    this.time += dt;

    // ── ログ記録 ────────────────────────────────────────────────────────
    this._logCounter++;
    if (this._logCounter >= this.config.logDecimate) {
      this._logCounter = 0;
      this._recordLog();
    }

    if (this.onStep) this.onStep(this.state, this.time);
  }

  /**
   * 指定時間分のステップを実行 (フレームレート非依存の進行に使用)
   * @param {number} elapsedSec  経過実時間 [s]
   * @param {number} maxSteps    最大ステップ数 (スパイラル防止)
   */
  runFor(elapsedSec, maxSteps = 100) {
    if (!this.running) return;
    const stepsNeeded = Math.round(elapsedSec * this.config.physicsHz);
    const steps = Math.min(stepsNeeded, maxSteps);
    for (let i = 0; i < steps; i++) this.step();
  }

  /**
   * 状態の見やすいスナップショットを返す (UI・可視化用)
   * @returns {object}
   */
  getSnapshot() {
    const S = STATE_IDX;
    const s = this.state;
    const sp = this.controller.setpoint;
    return {
      time: this.time,
      position:     { x: s[S.X],   y: s[S.Y],     z: s[S.Z]     },
      velocity:     { x: s[S.VX],  y: s[S.VY],    z: s[S.VZ]    },
      attitude:     {
        phi:   s[S.PHI]   * (180 / Math.PI),
        theta: s[S.THETA] * (180 / Math.PI),
        psi:   s[S.PSI]   * (180 / Math.PI),
      },
      angularRate:  { p: s[S.P],   q: s[S.Q],     r: s[S.R]     },
      omega2:       [...this.omega2],
      omega:        this.omega2.map(Math.sqrt),
      setpoint:     { ...sp },
      disturbance:  this.disturbance.getStatus(),
    };
  }

  /**
   * トラジェクトリ点を記録
   */
  _recordLog() {
    const S = STATE_IDX;
    const s = this.state;
    this.trajectory.push({
      t: this.time,
      x: s[S.X], y: s[S.Y], z: s[S.Z],
    });
    if (this.trajectory.length > this.config.maxLogPoints) {
      this.trajectory.shift();
    }
  }
}

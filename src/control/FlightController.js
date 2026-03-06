/**
 * FlightController.js
 *
 * カスケード PID フライトコントローラ
 *
 * 制御構造:
 *   ┌─────────────────┐     ┌─────────────────────┐     ┌──────────────┐
 *   │  位置ループ      │     │  姿勢ループ (内側)   │     │  制御割り当て │
 *   │  (外側, 低速)   │ --> │  (内側, 高速)        │ --> │  omega²      │
 *   │  x, y, z        │     │  phi, theta, psi     │     │              │
 *   └─────────────────┘     └─────────────────────┘     └──────────────┘
 *
 * 座標系: ENU (East-North-Up)
 */

import { PIDController } from './PIDController.js';
import { STATE_IDX } from '../physics/QuadrotorModel.js';

/** デフォルト PID ゲイン */
export const DEFAULT_GAINS = {
  // 位置ループ (外側)
  pos: {
    xy: { kp: 0.6,  ki: 2.95, kd: 3.0  },
    z:  { kp: 14.7, ki: 2.95, kd: 4.8  },
  },
  // 姿勢ループ (内側)
  att: {
    roll:  { kp: 8.8,  ki: 1.85, kd: 3.0  },
    pitch: { kp: 8.8,  ki: 1.85, kd: 3.0  },
    // ヨー: 物理的に出せる最大トルクが ~0.1 N·m のため、
    // outputMax を合わせてスケールしたゲインを使う
    yaw:   { kp: 3.0,  ki: 1.33, kd: 1.85 },
  },
};

export class FlightController {
  /**
   * @param {import('../physics/QuadrotorModel.js').QuadrotorModel} model
   * @param {typeof DEFAULT_GAINS} gains
   */
  constructor(model, gains = DEFAULT_GAINS) {
    this.model = model;

    // PID コントローラを生成
    // 物理限界から出力上限を導出
    // ロール/ピッチ最大トルク ≈ kT * l * 2 * ω²hover (ホバー時ペア変動分)
    //   = 1.269e-5 * 0.225 * 2 * (mg/4kT) = ml²g/2 ≈ 1.1 N·m → マージン込み 1.5
    // ヨー最大トルク ≈ 4 * kQ * ω²hover = 4 * kQ * mg/(4kT) = kQ*mg/kT
    //   = 1.269e-7 * 9.81 / 1.269e-5 ≈ 0.098 N·m → マージン込み 0.08
    const { kT, kQ, armLength: l } = model;
    const hoverW2 = (model.mass * 9.81) / (4 * kT);
    const maxRollPitchTau = kT * l * 2 * hoverW2 * 1.4; // 1.4 倍マージン
    const maxYawTau       = 4 * kQ * hoverW2 * 0.8;     // 0.8 倍マージン (保守的)

    this._pid = {
      x:     new PIDController({ ...gains.pos.xy, outputMin: -Math.PI/6, outputMax: Math.PI/6 }),
      y:     new PIDController({ ...gains.pos.xy, outputMin: -Math.PI/6, outputMax: Math.PI/6 }),
      z:     new PIDController({ ...gains.pos.z,  outputMin: -model.mass * 9.81 * 0.9, outputMax: model.mass * 9.81 * 2 }),
      roll:  new PIDController({ ...gains.att.roll,  outputMin: -maxRollPitchTau, outputMax: maxRollPitchTau }),
      pitch: new PIDController({ ...gains.att.pitch, outputMin: -maxRollPitchTau, outputMax: maxRollPitchTau }),
      yaw:   new PIDController({ ...gains.att.yaw,   outputMin: -maxYawTau,       outputMax: maxYawTau }),
    };

    /** セットポイント */
    this.setpoint = { x: 0, y: 0, z: 1, psi: 0 };

    /** 最大傾斜角 [rad] (~30度) */
    this.maxTiltAngle = Math.PI / 6;
  }

  /**
   * セットポイントを更新
   * @param {{ x?, y?, z?, psi? }} sp
   */
  setSetpoint(sp) {
    Object.assign(this.setpoint, sp);
  }

  /**
   * PID ゲインを更新
   * @param {Partial<typeof DEFAULT_GAINS>} gains
   */
  setGains(gains) {
    const g = gains;
    if (g.pos?.xy) { this._pid.x.setGains(g.pos.xy); this._pid.y.setGains(g.pos.xy); }
    if (g.pos?.z)  this._pid.z.setGains(g.pos.z);
    if (g.att?.roll)  this._pid.roll.setGains(g.att.roll);
    if (g.att?.pitch) this._pid.pitch.setGains(g.att.pitch);
    if (g.att?.yaw)   this._pid.yaw.setGains(g.att.yaw);
  }

  /**
   * 制御演算: 状態 → ローター角速度二乗
   *
   * @param {number[]} state 12次元状態ベクトル
   * @param {number}   dt    制御周期 [s]
   * @returns {number[]} [ω1², ω2², ω3², ω4²]
   */
  compute(state, dt) {
    const S = STATE_IDX;
    const { x, y, z, psi } = this.setpoint;
    const { mass: m } = this.model;
    const g = 9.81;

    // ── 外側ループ: 位置 → 目標姿勢角 ──────────────────────────────────
    // 高度制御: z誤差 → 推力補正
    const T_fb = this._pid.z.update(z, state[S.Z], dt);
    const T_total = m * g + T_fb; // ホバリング推力 + フィードバック

    // 水平位置制御: x,y 誤差 → 目標ロール/ピッチ角
    // 小角近似: ax = T/m * sin(theta) ≈ T/m * theta → theta_des = m/T * ax_des
    const ax_des = this._pid.x.update(x, state[S.X], dt);
    const ay_des = this._pid.y.update(y, state[S.Y], dt);

    // ワールドフレームの加速度指令をボディ傾斜角に変換
    // 注意: ヨー角を考慮して回転
    const cy = Math.cos(state[S.PSI]);
    const sy = Math.sin(state[S.PSI]);
    const theta_des = ( ax_des * cy + ay_des * sy) * m / Math.max(T_total, 1.0);
    const phi_des   = ( ax_des * sy - ay_des * cy) * m / Math.max(T_total, 1.0);

    // 傾斜角クランプ
    const clampAngle = (v) => Math.max(-this.maxTiltAngle, Math.min(this.maxTiltAngle, v));

    // ── 内側ループ: 姿勢 → トルク ────────────────────────────────────────
    const tauX = this._pid.roll.update(clampAngle(phi_des),   state[S.PHI],   dt);
    const tauY = this._pid.pitch.update(clampAngle(theta_des), state[S.THETA], dt);
    const tauZ = this._pid.yaw.update(psi, state[S.PSI], dt);

    // ── 制御割り当て: [T, τ] → [ω1², ω2², ω3², ω4²] ───────────────────
    return this.model.controlAllocation(T_total, tauX, tauY, tauZ);
  }

  /**
   * 全 PID をリセット
   */
  reset() {
    Object.values(this._pid).forEach((pid) => pid.reset());
  }

  /**
   * デバッグ用: 各 PID の内部状態を取得
   */
  getDebugInfo() {
    return Object.fromEntries(
      Object.entries(this._pid).map(([k, pid]) => [k, pid.getState()])
    );
  }

  /**
   * 現在のゲインを取得 (UI表示用)
   */
  getGains() {
    return {
      pos: {
        xy: { kp: this._pid.x.kp, ki: this._pid.x.ki, kd: this._pid.x.kd },
        z:  { kp: this._pid.z.kp, ki: this._pid.z.ki, kd: this._pid.z.kd },
      },
      att: {
        roll:  { kp: this._pid.roll.kp,  ki: this._pid.roll.ki,  kd: this._pid.roll.kd  },
        pitch: { kp: this._pid.pitch.kp, ki: this._pid.pitch.ki, kd: this._pid.pitch.kd },
        yaw:   { kp: this._pid.yaw.kp,   ki: this._pid.yaw.ki,   kd: this._pid.yaw.kd   },
      },
    };
  }
}

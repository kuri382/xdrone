/**
 * MPCController.js
 *
 * 線形 MPC (Model Predictive Control) フライトコントローラ
 *
 * アプローチ:
 *   - ホバリング平衡点まわりで運動方程式を線形化 (小角近似)
 *   - Euler 法で離散化: Ad = I + Ac·dt, Bd = Bc·dt
 *   - 非制約有限ホライゾン LQR 問題をオフラインで解いてゲイン行列を事前計算
 *   - オンライン: u* = -K_mpc · e  (行列ベクトル積のみ)
 *
 * 制御入力: u = [δT, τx, τy, τz]
 *   δT   = T_total - m·g  (ホバリングからの推力偏差)
 *   τx, τy, τz = ボディフレームトルク [N·m]
 *
 * ヨー対応:
 *   位置・速度誤差をボディフレームに回転してから適用することで、
 *   ヨー=0 の線形化を任意のヨー角に対して有効化する。
 *
 * 制限事項:
 *   - 線形化は小角度・低速時に有効 (非線形な大変位には追従精度が落ちる)
 *   - 入力制約は明示的に扱わない (コスト重みで間接的に制限)
 */

import { zeros, eye, copy, get, set, transpose, matmul, matvec,
         matAdd, scale, inv, blockDiag } from './MatrixMath.js';
import { STATE_IDX } from '../physics/QuadrotorModel.js';

const S  = STATE_IDX;
const NX = 12;  // 状態次元
const NU = 4;   // 入力次元: [δT, τx, τy, τz]

/** デフォルト MPC チューニングパラメータ */
export const DEFAULT_MPC_PARAMS = {
  /** 予測ホライゾン ステップ数 */
  horizon: 15,
  /** 予測ステップ幅 [s] (制御周期と合わせること) */
  dt: 0.01,
  /**
   * 状態コスト対角成分 Q: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
   * 大きいほど対応する状態誤差を強く抑制する
   */
  Q: [12, 12, 25, 4, 4, 8, 10, 10, 4, 0.5, 0.5, 0.5],
  /**
   * 入力コスト対角成分 R: [δT, τx, τy, τz]
   * 大きいほど制御入力が抑制される
   */
  R: [0.04, 0.6, 0.6, 6.0],
};

export class MPCController {
  /**
   * @param {import('../physics/QuadrotorModel.js').QuadrotorModel} model
   * @param {Partial<typeof DEFAULT_MPC_PARAMS>} options
   */
  constructor(model, options = {}) {
    this.model = model;
    const p    = { ...DEFAULT_MPC_PARAMS, ...options };
    this.N     = p.horizon;
    this.dt    = p.dt;
    this._Q    = p.Q.slice();
    this._R    = p.R.slice();

    /** セットポイント */
    this.setpoint = { x: 0, y: 0, z: 1, psi: 0 };

    // 物理パラメータから出力上限を計算 (FlightController と同じ基準)
    const { kT, kQ, armLength: l, mass: m } = model;
    const g       = 9.81;
    const hoverW2 = (m * g) / (4 * kT);
    this._maxRollPitchTau = kT * l * 2 * hoverW2 * 1.4;
    this._maxYawTau       = 4 * kQ * hoverW2 * 0.8;
    this._mg              = m * g;

    // ゲイン行列をオフラインで事前計算
    this._buildGain();
  }

  // ─── ゲイン事前計算 ──────────────────────────────────────────────────────

  /**
   * MPC 状態フィードバックゲイン K_mpc (NU × NX) を構築する。
   *
   * 手順:
   *   1. 連続線形モデル (Ac, Bc) をホバリング点で構築
   *   2. Euler 法で離散化 → (Ad, Bd)
   *   3. 予測行列 Ψ (N·NX × NX) と Φ (N·NX × N·NU) を構築
   *   4. コスト行列 Qbar, Rbar を構築
   *   5. H = Φ'·Qbar·Φ + Rbar を逆行列計算
   *   6. K_full = H^{-1}·Φ'·Qbar·Ψ (N·NU × NX)
   *   7. K_mpc = K_full の先頭 NU 行のみ取り出す
   */
  _buildGain() {
    const { model, N, dt, _Q, _R } = this;
    const { mass: m, Ixx, Iyy, Izz } = model;
    const g = 9.81;

    // ── 1. 連続線形モデル ────────────────────────────────────────────────
    // Ac (NX × NX): ホバリング小角近似 (ヨー=0)
    // 状態: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
    const Ac = zeros(NX, NX);
    // 位置キネマティクス: dx/dt = vx, dy/dt = vy, dz/dt = vz
    set(Ac, 0, 3, 1); set(Ac, 1, 4, 1); set(Ac, 2, 5, 1);
    // 並進ダイナミクス (小角近似, ヨー=0)
    //   dvx/dt ≈ g·θ,  dvy/dt ≈ -g·φ
    set(Ac, 3, 7,  g);
    set(Ac, 4, 6, -g);
    // 姿勢キネマティクス: dφ/dt = p, dθ/dt = q, dψ/dt = r
    set(Ac, 6,  9, 1); set(Ac, 7, 10, 1); set(Ac, 8, 11, 1);
    // 回転ダイナミクス: dp/dt, dq/dt, dr/dt は入力のみ (Bc で記述)

    // Bc (NX × NU): u = [δT, τx, τy, τz]
    const Bc = zeros(NX, NU);
    set(Bc,  5, 0, 1 / m);    // dvz/dt = δT/m
    set(Bc,  9, 1, 1 / Ixx);  // dp/dt  = τx/Ixx
    set(Bc, 10, 2, 1 / Iyy);  // dq/dt  = τy/Iyy
    set(Bc, 11, 3, 1 / Izz);  // dr/dt  = τz/Izz

    // ── 2. Euler 離散化: Ad = I + Ac·dt, Bd = Bc·dt ─────────────────────
    const Ad = matAdd(eye(NX), scale(Ac, dt));
    const Bd = scale(Bc, dt);

    // ── 3. 予測行列の構築 ────────────────────────────────────────────────
    // Ad の累乗: AdPow[k] = Ad^k (k = 0 .. N)
    const AdPow = [eye(NX)];
    for (let k = 1; k <= N; k++) AdPow.push(matmul(Ad, AdPow[k - 1]));

    // Ψ (N·NX × NX): Ψ[(k-1)*NX : k*NX, :] = Ad^k
    const Psi = zeros(N * NX, NX);
    for (let k = 0; k < N; k++) {
      const Ak1 = AdPow[k + 1];
      for (let i = 0; i < NX; i++)
        for (let j = 0; j < NX; j++)
          set(Psi, k * NX + i, j, get(Ak1, i, j));
    }

    // Φ (N·NX × N·NU): 下三角ブロック
    //   Φ[k·NX:(k+1)·NX, j·NU:(j+1)·NU] = Ad^(k-j)·Bd  (k >= j)
    const Phi = zeros(N * NX, N * NU);
    for (let k = 0; k < N; k++) {
      for (let j = 0; j <= k; j++) {
        const M = matmul(AdPow[k - j], Bd);
        for (let i = 0; i < NX; i++)
          for (let l = 0; l < NU; l++)
            set(Phi, k * NX + i, j * NU + l, get(M, i, l));
      }
    }

    // ── 4. コスト行列 ────────────────────────────────────────────────────
    const Qmat = zeros(NX, NX);
    for (let i = 0; i < NX; i++) set(Qmat, i, i, _Q[i]);
    const Qbar = blockDiag(Array.from({ length: N }, () => copy(Qmat)));

    const Rmat = zeros(NU, NU);
    for (let i = 0; i < NU; i++) set(Rmat, i, i, _R[i]);
    const Rbar = blockDiag(Array.from({ length: N }, () => copy(Rmat)));

    // ── 5. オフラインゲイン計算 ──────────────────────────────────────────
    // H = Φ'·Qbar·Φ + Rbar  (N·NU × N·NU = 60×60)
    const PhiT     = transpose(Phi);
    const PhiTQbar = matmul(PhiT, Qbar);
    const H        = matAdd(matmul(PhiTQbar, Phi), Rbar);

    const Hinv = inv(H);
    if (!Hinv) {
      console.error('[MPC] H 行列が特異 — ゲインをゼロに設定');
      this._K = zeros(NU, NX);
      return;
    }

    // K_full = H^{-1}·Φ'·Qbar·Ψ  (N·NU × NX = 60×12)
    const K_full = matmul(matmul(Hinv, PhiTQbar), Psi);

    // K_mpc: 先頭 NU 行のみ取り出す (NU × NX = 4×12)
    this._K = zeros(NU, NX);
    for (let i = 0; i < NU; i++)
      for (let j = 0; j < NX; j++)
        set(this._K, i, j, get(K_full, i, j));

    console.log('[MPC] ゲイン計算完了. K[0] (dT row):',
      Array.from(this._K.data.slice(0, NX)).map(v => v.toFixed(3)).join(' '));
  }

  // ─── 制御演算 (オンライン) ───────────────────────────────────────────────

  /**
   * 制御演算: 状態 → ローター角速度二乗
   *
   * @param {number[]} state  12次元状態ベクトル
   * @param {number}   dt     制御周期 (本実装では未使用; ゲインは事前計算済み)
   * @returns {number[]} [ω1², ω2², ω3², ω4²]
   */
  compute(state, _dt) {
    const sp  = this.setpoint;
    const psi = state[S.PSI];
    const cy  = Math.cos(psi);
    const sy  = Math.sin(psi);

    // ── 誤差ベクトル (ボディフレーム回転適用) ──────────────────────────
    // ヨー=0 で線形化したモデルを任意のヨー角に対応させるため、
    // 位置・速度の水平成分誤差をボディフレームへ回転する
    const ex_w = sp.x - state[S.X];
    const ey_w = sp.y - state[S.Y];

    const e = new Float64Array(NX);
    e[S.X]     =  ex_w * cy + ey_w * sy;           // ボディフレーム x 誤差
    e[S.Y]     = -ex_w * sy + ey_w * cy;           // ボディフレーム y 誤差
    e[S.Z]     = sp.z - state[S.Z];
    e[S.VX]    = -(state[S.VX] * cy + state[S.VY] * sy);   // 目標速度 = 0
    e[S.VY]    = -(-state[S.VX] * sy + state[S.VY] * cy);
    e[S.VZ]    = -state[S.VZ];
    e[S.PHI]   = -state[S.PHI];     // 目標ロール = 0
    e[S.THETA] = -state[S.THETA];   // 目標ピッチ = 0
    // ヨー誤差を [-π, π] に正規化
    let ePsi = sp.psi - state[S.PSI];
    ePsi = ((ePsi + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
    e[S.PSI] = ePsi;
    e[S.P]   = -state[S.P];
    e[S.Q]   = -state[S.Q];
    e[S.R]   = -state[S.R];

    // u* = K · e  (K は事前計算済み)
    const u = matvec(this._K, e);

    // ── 物理入力へのマッピング ────────────────────────────────────────
    const T_total = Math.max(0, this._mg + u[0]);

    const clampRP  = v => Math.max(-this._maxRollPitchTau, Math.min(this._maxRollPitchTau, v));
    const clampYaw = v => Math.max(-this._maxYawTau,       Math.min(this._maxYawTau, v));

    return this.model.controlAllocation(
      T_total,
      clampRP(u[1]),
      clampRP(u[2]),
      clampYaw(u[3]),
    );
  }

  // ─── インターフェース互換メソッド ────────────────────────────────────────

  setSetpoint(sp) {
    Object.assign(this.setpoint, sp);
  }

  /** PID 積分器なし — リセット不要 */
  reset() {}

  /** UI 表示用: MPC パラメータを返す */
  getGains() {
    return {
      mpc: {
        horizon: this.N,
        dt:      this.dt,
        Q:       this._Q.slice(),
        R:       this._R.slice(),
      },
    };
  }
}

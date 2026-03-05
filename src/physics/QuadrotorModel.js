/**
 * QuadrotorModel.js
 *
 * 6自由度クアッドロータ運動力学モデル
 *
 * 座標系: ENU (East-North-Up)
 *   ワールドフレーム: x=東, y=北, z=上
 *   ボディフレーム:  x=前, y=左, z=上
 *
 * 状態ベクトル (12次元):
 *   [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
 *    0  1  2   3   4   5   6     7     8   9  10 11
 *
 * ローター配置 (+ コンフィグ、上から見た図):
 *         M1(前,CCW)
 *            |
 *   M2(左,CW)---+---M4(右,CW)
 *            |
 *         M3(後,CCW)
 *
 * 入力: 各ローターの角速度の二乗 [ω1², ω2², ω3², ω4²] [rad²/s²]
 */

export const STATE_IDX = {
  X: 0, Y: 1, Z: 2,
  VX: 3, VY: 4, VZ: 5,
  PHI: 6, THETA: 7, PSI: 8,
  P: 9, Q: 10, R: 11,
  SIZE: 12,
};

/** デフォルト機体パラメータ (DJI F450クラス相当) */
export const DEFAULT_PARAMS = {
  mass:      1.0,      // 質量 [kg]
  armLength: 0.225,    // アーム長 (中心〜モータ中心) [m]
  Ixx:       0.0196,   // ロール慣性モーメント [kg·m²]
  Iyy:       0.0196,   // ピッチ慣性モーメント [kg·m²]
  Izz:       0.0264,   // ヨー慣性モーメント [kg·m²]
  kT:        1.269e-5, // 推力係数 [N/(rad/s)²]
  kQ:        1.269e-7, // トルク係数 [N·m/(rad/s)²]
  omegaMin:  0,        // 最小ローター角速度 [rad/s]
  omegaMax:  1200,     // 最大ローター角速度 [rad/s]
};

export class QuadrotorModel {
  /**
   * @param {Partial<typeof DEFAULT_PARAMS>} params
   */
  constructor(params = {}) {
    Object.assign(this, { ...DEFAULT_PARAMS, ...params });

    /** 外乱: 外部から注入する力・トルク (ワールドフレーム, ボディフレーム) */
    this.disturbance = { fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0 };
  }

  /**
   * ホバリングに必要なローター角速度 [rad/s]
   */
  get hoverOmega() {
    return Math.sqrt((this.mass * 9.81) / (4 * this.kT));
  }

  /**
   * ボディ→ワールドへの回転行列 R (ZYX オイラー角)
   * ワールド上のベクトル v_w = R * v_b
   *
   * @param {number} phi   ロール  [rad]
   * @param {number} theta ピッチ  [rad]
   * @param {number} psi   ヨー    [rad]
   * @returns {number[][]} 3x3 回転行列
   */
  rotationMatrix(phi, theta, psi) {
    const cp = Math.cos(phi),   sp = Math.sin(phi);
    const ct = Math.cos(theta), st = Math.sin(theta);
    const cy = Math.cos(psi),   sy = Math.sin(psi);

    return [
      [ cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp ],
      [ sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp ],
      [ -st,    ct*sp,             ct*cp             ],
    ];
  }

  /**
   * ローター推力とトルクをボディフレームで計算
   *
   * @param {number[]} omega2 各ローター角速度の二乗 [rad²/s²]
   * @returns {{ T: number, tauX: number, tauY: number, tauZ: number }}
   */
  rotorForcesTorques(omega2) {
    const { kT, kQ, armLength: l } = this;
    const [w1, w2, w3, w4] = omega2;

    const T    = kT * (w1 + w2 + w3 + w4);
    const tauX = kT * l * (w4 - w2);           // ロール:  右 - 左
    const tauY = kT * l * (w1 - w3);           // ピッチ: 前 - 後
    const tauZ = kQ * (-w1 + w2 - w3 + w4);   // ヨー:   CCW+ CW- CCW+ CW-

    return { T, tauX, tauY, tauZ };
  }

  /**
   * 状態方程式 (連続時間) f(x, u) = dx/dt
   *
   * @param {number[]} state  12次元状態ベクトル
   * @param {number[]} omega2 4次元ローター角速度二乗ベクトル
   * @returns {number[]} 12次元状態微分ベクトル
   */
  derivative(state, omega2) {
    const [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] = state;
    const { mass: m, Ixx, Iyy, Izz } = this;
    const g = 9.81;
    const d = this.disturbance;

    // ボディ→ワールド 回転行列
    const R = this.rotationMatrix(phi, theta, psi);

    // ローター合力・合トルク (ボディフレーム)
    const { T, tauX, tauY, tauZ } = this.rotorForcesTorques(omega2);

    // 並進加速度 (ワールドフレーム)
    // 推力をボディz軸方向に作用 → ワールドフレームに変換
    const ax = (R[0][2] * T + d.fx) / m;
    const ay = (R[1][2] * T + d.fy) / m;
    const az = (R[2][2] * T + d.fz) / m - g;

    // オイラー角キネマティクス: ボディ角速度 → オイラー角速度
    const sp = Math.sin(phi), cp = Math.cos(phi);
    const ct = Math.cos(theta), tt = Math.tan(theta);

    const dPhi   = p + (q * sp + r * cp) * tt;
    const dTheta = q * cp - r * sp;
    const dPsi   = (q * sp + r * cp) / ct;

    // オイラー方程式: I * dω/dt = τ - ω × (I·ω) + 外乱トルク
    const dp = ((Iyy - Izz) * q * r + tauX + d.tx) / Ixx;
    const dq = ((Izz - Ixx) * p * r + tauY + d.ty) / Iyy;
    const dr = ((Ixx - Iyy) * p * q + tauZ + d.tz) / Izz;

    return [vx, vy, vz, ax, ay, az, dPhi, dTheta, dPsi, dp, dq, dr];
  }

  /**
   * 制御割り当て: [T, tauX, tauY, tauZ] → 各ローターのω² (クランプ付き)
   *
   * 逆混合行列 (+ コンフィグ):
   *   ω1² = T/(4kT) + tauY/(2kT·l) - tauZ/(4kQ)
   *   ω2² = T/(4kT) - tauX/(2kT·l) + tauZ/(4kQ)
   *   ω3² = T/(4kT) - tauY/(2kT·l) - tauZ/(4kQ)
   *   ω4² = T/(4kT) + tauX/(2kT·l) + tauZ/(4kQ)
   *
   * @param {number} T    全推力 [N]
   * @param {number} tauX ロールトルク [N·m]
   * @param {number} tauY ピッチトルク [N·m]
   * @param {number} tauZ ヨートルク  [N·m]
   * @returns {number[]} [ω1², ω2², ω3², ω4²]
   */
  controlAllocation(T, tauX, tauY, tauZ) {
    const { kT, kQ, armLength: l } = this;
    const wMin2 = this.omegaMin ** 2;
    const wMax2 = this.omegaMax ** 2;
    const clamp = (v) => Math.max(wMin2, Math.min(wMax2, v));

    return [
      clamp(T / (4 * kT) + tauY / (2 * kT * l) - tauZ / (4 * kQ)),
      clamp(T / (4 * kT) - tauX / (2 * kT * l) + tauZ / (4 * kQ)),
      clamp(T / (4 * kT) - tauY / (2 * kT * l) - tauZ / (4 * kQ)),
      clamp(T / (4 * kT) + tauX / (2 * kT * l) + tauZ / (4 * kQ)),
    ];
  }

  /**
   * 初期状態ベクトルを生成
   * @param {{ x?, y?, z?, psi? }} init
   * @returns {number[]}
   */
  createState({ x = 0, y = 0, z = 0, psi = 0 } = {}) {
    const s = new Array(STATE_IDX.SIZE).fill(0);
    s[STATE_IDX.X]   = x;
    s[STATE_IDX.Y]   = y;
    s[STATE_IDX.Z]   = z;
    s[STATE_IDX.PSI] = psi;
    return s;
  }
}

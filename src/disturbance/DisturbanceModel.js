/**
 * DisturbanceModel.js
 *
 * 外乱モデル — シミュレータに外乱を注入するためのモジュール
 *
 * 外乱の種類:
 *   - 定常風 (constant wind)    : 一定速度の風
 *   - 突風   (gust)             : 有限時間の突発的な力
 *   - 乱流   (turbulence)       : ランダムな力・トルク
 *   - インパルス (impulse)      : 単一フレームの衝撃力
 *
 * 出力: { fx, fy, fz, tx, ty, tz } [N, N·m]  ワールドフレーム力 + ボディフレームトルク
 */

/** ガウス乱数 (Box-Muller法) */
function randn() {
  let u, v;
  do { u = Math.random(); } while (u === 0);
  do { v = Math.random(); } while (v === 0);
  return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}

export class DisturbanceModel {
  constructor() {
    /** 定常風 [m/s] → 力に変換 (drag係数で調整) */
    this._wind = { vx: 0, vy: 0, vz: 0 };

    /** 乱流設定 */
    this._turbulence = { intensity: 0, enabled: false };

    /** アクティブな突風リスト: { fx, fy, fz, tx, ty, tz, remaining } */
    this._gusts = [];

    /** インパルス (次のステップのみ) */
    this._impulse = null;

    /** 空気抵抗係数 (簡易モデル) */
    this.dragCoeff = 0.1; // [N/(m/s)]
  }

  /**
   * 定常風を設定
   * @param {number} vx [m/s] 東向き
   * @param {number} vy [m/s] 北向き
   * @param {number} vz [m/s] 上向き
   */
  setWind(vx, vy, vz) {
    this._wind = { vx, vy, vz };
  }

  /**
   * 定常風をクリア
   */
  clearWind() {
    this._wind = { vx: 0, vy: 0, vz: 0 };
  }

  /**
   * 突風を追加
   * @param {object} params
   * @param {number} params.fx [N] 東方向力
   * @param {number} params.fy [N] 北方向力
   * @param {number} params.fz [N] 上方向力
   * @param {number} params.tx [N·m] ロールトルク (ボディフレーム)
   * @param {number} params.ty [N·m] ピッチトルク
   * @param {number} params.tz [N·m] ヨートルク
   * @param {number} params.duration [s] 持続時間
   */
  addGust({ fx = 0, fy = 0, fz = 0, tx = 0, ty = 0, tz = 0, duration = 0.5 }) {
    this._gusts.push({ fx, fy, fz, tx, ty, tz, remaining: duration });
  }

  /**
   * 単発インパルス (1ステップだけ力を加える)
   * @param {{ fx?, fy?, fz?, tx?, ty?, tz? }} params
   */
  addImpulse({ fx = 0, fy = 0, fz = 0, tx = 0, ty = 0, tz = 0 }) {
    this._impulse = { fx, fy, fz, tx, ty, tz };
  }

  /**
   * 乱流を有効化/設定
   * @param {number} intensity  強度 [N] (0 = 無効)
   */
  setTurbulence(intensity) {
    this._turbulence.intensity = intensity;
    this._turbulence.enabled   = intensity > 0;
  }

  /**
   * 全外乱をクリア
   */
  clearAll() {
    this._wind       = { vx: 0, vy: 0, vz: 0 };
    this._turbulence = { intensity: 0, enabled: false };
    this._gusts      = [];
    this._impulse    = null;
  }

  /**
   * 外乱を計算して返す (毎シミュレーションステップ呼ぶ)
   *
   * @param {number} dt [s]
   * @param {number[]} [droneVelocity]  ドローン速度 [vx, vy, vz] (空気抵抗計算用)
   * @returns {{ fx, fy, fz, tx, ty, tz }}
   */
  compute(dt, droneVelocity = [0, 0, 0]) {
    let fx = 0, fy = 0, fz = 0;
    let tx = 0, ty = 0, tz = 0;

    // 定常風による力: F = dragCoeff * (v_wind - v_drone)
    fx += this.dragCoeff * (this._wind.vx - droneVelocity[0]);
    fy += this.dragCoeff * (this._wind.vy - droneVelocity[1]);
    fz += this.dragCoeff * (this._wind.vz - droneVelocity[2]);

    // 突風
    for (let i = this._gusts.length - 1; i >= 0; i--) {
      const g = this._gusts[i];
      fx += g.fx; fy += g.fy; fz += g.fz;
      tx += g.tx; ty += g.ty; tz += g.tz;
      g.remaining -= dt;
      if (g.remaining <= 0) this._gusts.splice(i, 1);
    }

    // 乱流
    if (this._turbulence.enabled) {
      const s = this._turbulence.intensity;
      fx += randn() * s;
      fy += randn() * s;
      fz += randn() * s * 0.5;
      tx += randn() * s * 0.01;
      ty += randn() * s * 0.01;
      tz += randn() * s * 0.01;
    }

    // インパルス (1ステップのみ)
    if (this._impulse) {
      fx += this._impulse.fx; fy += this._impulse.fy; fz += this._impulse.fz;
      tx += this._impulse.tx; ty += this._impulse.ty; tz += this._impulse.tz;
      this._impulse = null;
    }

    return { fx, fy, fz, tx, ty, tz };
  }

  /**
   * 現在の外乱設定を取得 (UI表示用)
   */
  getStatus() {
    return {
      wind: { ...this._wind },
      turbulenceIntensity: this._turbulence.intensity,
      activeGusts: this._gusts.length,
    };
  }
}

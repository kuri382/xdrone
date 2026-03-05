/**
 * PIDController.js
 *
 * 汎用 PID コントローラ
 *
 * 実装上の工夫:
 *   - 微分先行型 (Derivative on Measurement): セットポイント急変による微分キックを防止
 *   - アンチワインドアップ: 積分項をクランプして飽和を防止
 *   - 出力リミット: 制御出力の上下限クランプ
 */
export class PIDController {
  /**
   * @param {object} params
   * @param {number} params.kp          比例ゲイン
   * @param {number} params.ki          積分ゲイン
   * @param {number} params.kd          微分ゲイン
   * @param {number} [params.outputMin] 出力下限 (デフォルト: -Infinity)
   * @param {number} [params.outputMax] 出力上限 (デフォルト: +Infinity)
   * @param {number} [params.iMin]      積分項下限 (デフォルト: outputMin)
   * @param {number} [params.iMax]      積分項上限 (デフォルト: outputMax)
   */
  constructor({ kp, ki, kd, outputMin = -Infinity, outputMax = Infinity, iMin, iMax } = {}) {
    this.kp = kp ?? 1;
    this.ki = ki ?? 0;
    this.kd = kd ?? 0;
    this.outputMin = outputMin;
    this.outputMax = outputMax;
    this.iMin = iMin ?? outputMin;
    this.iMax = iMax ?? outputMax;

    this._integral    = 0;
    this._prevMeasure = null; // 微分先行型: 前回の測定値を保持
  }

  /**
   * PID 演算
   *
   * @param {number} setpoint  目標値
   * @param {number} measure   現在の測定値
   * @param {number} dt        時間刻み [s]
   * @returns {number} 制御出力
   */
  update(setpoint, measure, dt) {
    const error = setpoint - measure;

    // 積分項 (アンチワインドアップ付き)
    this._integral += error * dt;
    this._integral = Math.max(this.iMin, Math.min(this.iMax, this._integral));

    // 微分項: 微分先行型 (測定値の変化率を使用)
    let derivative = 0;
    if (this._prevMeasure !== null && dt > 0) {
      derivative = -(measure - this._prevMeasure) / dt;
    }
    this._prevMeasure = measure;

    const output = this.kp * error + this.ki * this._integral + this.kd * derivative;
    return Math.max(this.outputMin, Math.min(this.outputMax, output));
  }

  /**
   * 内部状態をリセット
   */
  reset() {
    this._integral    = 0;
    this._prevMeasure = null;
  }

  /**
   * ゲインを動的に変更
   * @param {{ kp?, ki?, kd? }} gains
   */
  setGains({ kp, ki, kd } = {}) {
    if (kp !== undefined) this.kp = kp;
    if (ki !== undefined) this.ki = ki;
    if (kd !== undefined) this.kd = kd;
  }

  /** デバッグ用: 内部状態を取得 */
  getState() {
    return { integral: this._integral, prevMeasure: this._prevMeasure };
  }
}

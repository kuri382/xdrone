/**
 * Integrator.js
 *
 * 常微分方程式の数値積分器
 *
 * 共通インターフェース:
 *   integrate(f, state, input, dt) -> newState
 *
 *   f: (state: number[], input: any) -> dState: number[]
 */

/**
 * 4次ルンゲ・クッタ法 (RK4)
 *
 * 精度: O(dt^4)
 * 安定性: 明示的積分法で最良クラス
 * 用途: 本シミュレータのデフォルト積分器
 */
export class RK4Integrator {
  /**
   * @param {(state: number[], input: any) => number[]} f  状態方程式
   * @param {number[]} state  現在状態
   * @param {any}      input  制御入力
   * @param {number}   dt     時間刻み [s]
   * @returns {number[]} 次の状態
   */
  integrate(f, state, input, dt) {
    const k1 = f(state, input);
    const s2 = state.map((s, i) => s + 0.5 * dt * k1[i]);

    const k2 = f(s2, input);
    const s3 = state.map((s, i) => s + 0.5 * dt * k2[i]);

    const k3 = f(s3, input);
    const s4 = state.map((s, i) => s + dt * k3[i]);

    const k4 = f(s4, input);

    return state.map((s, i) =>
      s + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
    );
  }
}

/**
 * オイラー法 (1次)
 *
 * 精度: O(dt)
 * 用途: デバッグ・比較用 (精度よりも速度優先の場面)
 */
export class EulerIntegrator {
  integrate(f, state, input, dt) {
    const k = f(state, input);
    return state.map((s, i) => s + dt * k[i]);
  }
}

/**
 * ホイン法 (2次ルンゲ・クッタ, 台形則)
 *
 * 精度: O(dt^2)
 * 用途: RK4より軽量で Euler より精度が必要な場面
 */
export class HeunIntegrator {
  integrate(f, state, input, dt) {
    const k1 = f(state, input);
    const sPred = state.map((s, i) => s + dt * k1[i]);

    const k2 = f(sPred, input);

    return state.map((s, i) => s + (dt / 2) * (k1[i] + k2[i]));
  }
}

/**
 * MatrixMath.js
 *
 * 密行列演算ユーティリティ (MPC実装用)
 * 行優先 (row-major) Float64Array で格納
 */

/** ゼロ行列を生成 */
export function zeros(r, c) {
  return { r, c, data: new Float64Array(r * c) };
}

/** 単位行列を生成 */
export function eye(n) {
  const A = zeros(n, n);
  for (let i = 0; i < n; i++) A.data[i * n + i] = 1;
  return A;
}

/** コピー */
export function copy(A) {
  return { r: A.r, c: A.c, data: A.data.slice() };
}

export function get(A, i, j) { return A.data[i * A.c + j]; }
export function set(A, i, j, v) { A.data[i * A.c + j] = v; }

/** 転置 */
export function transpose(A) {
  const B = zeros(A.c, A.r);
  for (let i = 0; i < A.r; i++)
    for (let j = 0; j < A.c; j++)
      set(B, j, i, get(A, i, j));
  return B;
}

/** 行列積 A * B */
export function matmul(A, B) {
  if (A.c !== B.r) throw new Error(`matmul: dim mismatch ${A.r}x${A.c} * ${B.r}x${B.c}`);
  const C = zeros(A.r, B.c);
  for (let i = 0; i < A.r; i++)
    for (let k = 0; k < A.c; k++) {
      const aik = A.data[i * A.c + k];
      if (aik === 0) continue;
      const rowC = i * C.c;
      const rowB = k * B.c;
      for (let j = 0; j < B.c; j++)
        C.data[rowC + j] += aik * B.data[rowB + j];
    }
  return C;
}

/** 行列ベクトル積 A * v → Float64Array */
export function matvec(A, v) {
  const y = new Float64Array(A.r);
  for (let i = 0; i < A.r; i++) {
    let s = 0;
    const row = i * A.c;
    for (let j = 0; j < A.c; j++) s += A.data[row + j] * v[j];
    y[i] = s;
  }
  return y;
}

/** 要素和 A + B */
export function matAdd(A, B) {
  const C = copy(A);
  for (let i = 0; i < C.data.length; i++) C.data[i] += B.data[i];
  return C;
}

/** スカラー倍 */
export function scale(A, s) {
  const B = copy(A);
  for (let i = 0; i < B.data.length; i++) B.data[i] *= s;
  return B;
}

/**
 * Gauss-Jordan 逆行列
 * @returns {object|null} 逆行列、特異の場合 null
 */
export function inv(A) {
  const n = A.r;
  // 拡大係数行列 [A | I] をフラット配列で
  const M = new Float64Array(n * 2 * n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) M[i * 2 * n + j] = get(A, i, j);
    M[i * 2 * n + n + i] = 1;
  }

  for (let col = 0; col < n; col++) {
    // 部分ピボット
    let maxVal = Math.abs(M[col * 2 * n + col]);
    let maxRow = col;
    for (let row = col + 1; row < n; row++) {
      const v = Math.abs(M[row * 2 * n + col]);
      if (v > maxVal) { maxVal = v; maxRow = row; }
    }
    if (maxVal < 1e-14) return null;

    if (maxRow !== col) {
      const w = 2 * n;
      for (let j = 0; j < w; j++) {
        const tmp = M[col * w + j];
        M[col * w + j] = M[maxRow * w + j];
        M[maxRow * w + j] = tmp;
      }
    }

    // ピボット行を正規化
    const w = 2 * n;
    const pivot = M[col * w + col];
    for (let j = 0; j < w; j++) M[col * w + j] /= pivot;

    // 列消去
    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const factor = M[row * w + col];
      if (factor === 0) continue;
      for (let j = 0; j < w; j++) M[row * w + j] -= factor * M[col * w + j];
    }
  }

  const Inv = zeros(n, n);
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++)
      set(Inv, i, j, M[i * 2 * n + n + j]);
  return Inv;
}

/**
 * ブロック対角行列を構築
 * @param {Array<object>} mats 行列の配列
 */
export function blockDiag(mats) {
  const totalR = mats.reduce((s, M) => s + M.r, 0);
  const totalC = mats.reduce((s, M) => s + M.c, 0);
  const D = zeros(totalR, totalC);
  let rOff = 0, cOff = 0;
  for (const M of mats) {
    for (let i = 0; i < M.r; i++)
      for (let j = 0; j < M.c; j++)
        set(D, rOff + i, cOff + j, get(M, i, j));
    rOff += M.r;
    cOff += M.c;
  }
  return D;
}

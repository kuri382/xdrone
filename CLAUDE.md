# CLAUDE.md — xdrone プロジェクト

## プロジェクト概要

ブラウザで動作する 6DOF クアッドロータ飛行シミュレータ。
Three.js による 3D 可視化と、カスケード PID 制御を実装。

## ファイル構成

```
index.html                      # エントリポイント・全UI定義
src/
  main.js                       # アプリ初期化・UIイベント・レンダーループ
  physics/
    QuadrotorModel.js           # 6DOF運動力学モデル (ENU座標系)
    Integrator.js               # RK4数値積分
  control/
    FlightController.js         # カスケードPIDコントローラ
    PIDController.js            # 汎用PIDコントローラ (anti-windup付き)
  disturbance/
    DisturbanceModel.js         # 風・突風・乱流モデル
  simulation/
    Simulator.js                # シミュレーションループ管理
  visualization/
    DroneVisualizer.js          # Three.js 3D可視化・カメラ制御・ターゲットドラッグ
```

## 座標系

- **ワールドフレーム**: ENU (East-North-Up) — x=東, y=北, z=上
- **Three.js内部**: x=東, y=上, z=-北 (変換注意: physics(x,y,z) → three(x,z,-y))
- **状態ベクトル (12次元)**: `[x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]`

## 制御構造

カスケード PID (外側→内側):

1. **位置ループ** (外側): x,y,z 誤差 → 目標姿勢角・推力
2. **姿勢ループ** (内側): phi,theta,psi 誤差 → ロール/ピッチ/ヨートルク
3. **制御割り当て**: [T, τx, τy, τz] → [ω1², ω2², ω3², ω4²]

## デフォルト PID ゲイン (検証済み)

```json
{
  "pos": {
    "xy": { "kp": 3.1,  "ki": 0.7,  "kd": 6.4 },
    "z":  { "kp": 14.7, "ki": 2.95, "kd": 4.8 }
  },
  "att": {
    "roll":  { "kp": 8.8, "ki": 1.85, "kd": 3.0 },
    "pitch": { "kp": 8.8, "ki": 1.85, "kd": 3.0 },
    "yaw":   { "kp": 3.0, "ki": 1.33, "kd": 1.85 }
  }
}
```

## 重要な実装上の注意

### PID 出力上限は物理パラメータから導出すること

ヨー PID の `outputMax` を大きく設定するとモーターが飽和し、高度スパイクが発生する。
`FlightController.js` コンストラクタで `hoverW2` から計算している値を変更しないこと:

```js
const maxRollPitchTau = kT * l * 2 * hoverW2 * 1.4;  // ~1.1 N·m
const maxYawTau       = 4 * kQ * hoverW2 * 0.8;       // ~0.08 N·m
```

### Three.js ↔ 物理座標変換

```js
// Physics → Three.js
mesh.position.set(state.x, state.z, -state.y);

// Three.js raycast hit → Physics (ENU)
physics_x =  hit.x;
physics_y = -hit.z;
physics_z =  hit.y;
```

### ターゲットドラッグ (MOVE ボタン)

`DroneVisualizer._initTargetDrag()` が実装。
ドラッグ中は OrbitControls を無効化し、`THREE.Plane` へのレイキャストでセットポイントを更新。
`this._targetPlane.constant = -setpoint.z` で平面をターゲット高度に追従させる。

## 開発メモ

- サーバー不要: `index.html` を直接ブラウザで開くか、`python3 -m http.server` 等で配信
- ES modules 使用のため、ローカルファイルアクセスには HTTP サーバーが必要
- デバッグ: `window._sim` でシミュレータインスタンスにアクセス可能

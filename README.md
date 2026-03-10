# XDRONE — Flight Simulator

ブラウザで動作する 6DOF クアッドロータ飛行シミュレータ。
Three.js による 3D 可視化と、カスケード PID / 線形 MPC の 2 種類のフライトコントローラを搭載。

---

## デモ

```
python3 -m http.server 8080
# → ブラウザで http://localhost:8080 を開く
```

ビルド不要。ES Modules + Three.js (CDN importmap) のみで動作する。

---

## 特徴

| 機能 | 説明 |
|---|---|
| **6DOF 物理シミュレーション** | ENU 座標系、RK4 積分、回転行列によるリジッドボディ力学 |
| **カスケード PID** | 位置ループ (外側) → 姿勢ループ (内側) の 2 段構成 |
| **線形 MPC** | ホバリング線形化 + 事前計算ゲインによるリアルタイム MPC |
| **外乱モデル** | 定常風・突風・乱流を個別に注入可能 |
| **3D 可視化** | Three.js レンダリング、軌跡表示、カメラ追従 |
| **ターゲットドラッグ** | MOVE ボタンで 3D 空間上のセットポイントをマウスドラッグ |
| **PID ゲイン管理** | スライダ + 数値入力、JSON エクスポート/インポート |

---

## ファイル構成

```
xdrone/
├── index.html                     # エントリポイント・全 UI 定義
├── CLAUDE.md                      # AI アシスタント向けプロジェクト情報
├── docs/
│   └── mpc.md                     # MPC 実装の詳細ドキュメント
└── src/
    ├── main.js                    # アプリ初期化・UI イベント・レンダーループ
    ├── physics/
    │   ├── QuadrotorModel.js      # 6DOF 運動力学モデル
    │   └── Integrator.js          # RK4 / Euler / Heun 積分器
    ├── control/
    │   ├── PIDController.js       # 汎用 PID (derivative-on-measurement、anti-windup)
    │   ├── FlightController.js    # カスケード PID フライトコントローラ
    │   ├── MPCController.js       # 線形 MPC フライトコントローラ
    │   └── MatrixMath.js          # MPC 用密行列演算ライブラリ
    ├── disturbance/
    │   └── DisturbanceModel.js    # 風・突風・乱流モデル
    ├── simulation/
    │   └── Simulator.js           # シミュレーションループ管理
    └── visualization/
        └── DroneVisualizer.js     # Three.js 3D レンダリング・カメラ制御
```

---

## 座標系

```
ワールドフレーム (ENU)          Three.js 内部
  x = 東 (East)                  x = 東
  y = 北 (North)         →       y = 上 (= physics z)
  z = 上 (Up)                    z = 南 (= -physics y)
```

状態ベクトル (12 次元):

```
[ x, y, z,  vx, vy, vz,  φ, θ, ψ,  p, q, r ]
  位置(m)    速度(m/s)   姿勢(rad)  角速度(rad/s)
```

---

## 物理パラメータ (DJI F450 クラス相当)

| パラメータ | 値 | 単位 |
|---|---|---|
| 質量 | 1.0 | kg |
| アーム長 | 0.225 | m |
| Ixx = Iyy | 0.0196 | kg·m² |
| Izz | 0.0264 | kg·m² |
| 推力係数 kT | 1.269×10⁻⁵ | N/(rad/s)² |
| トルク係数 kQ | 1.269×10⁻⁷ | N·m/(rad/s)² |
| ホバー角速度 | ≈ 440 | rad/s |

---

## コントローラ

### カスケード PID

```
セットポイント (x,y,z,ψ)
    │
    ▼
[位置ループ]  x,y → θ_des, φ_des   z → δT
    │
    ▼
[姿勢ループ]  φ,θ,ψ → τx, τy, τz
    │
    ▼
[制御割り当て]  [T, τx, τy, τz] → [ω1², ω2², ω3², ω4²]
```

### 線形 MPC

詳細は [docs/mpc.md](docs/mpc.md) を参照。

事前計算したゲイン行列 **K_mpc (4×12)** を用いて、オンライン計算をシンプルな行列ベクトル積 `u* = K_mpc · e` のみに抑える。

---

## デフォルト PID ゲイン

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

---

## 拡張ポイント

- **ウェイポイント飛行**: `controller.setSetpoint()` を呼ぶウェイポイントマネージャを追加
- **積分器の交換**: `Simulator` に渡す積分器を適応ステップ幅の手法に変更
- **制御則の交換**: `simulator.controller` に任意のコントローラを代入可能 (LQR, NMPC など)
- **ハードウェア移植**: `QuadrotorModel.derivative()` は純粋な数学計算のみのため C/Arduino に移植しやすい

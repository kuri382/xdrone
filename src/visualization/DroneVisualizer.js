/**
 * DroneVisualizer.js
 *
 * Three.js によるクアッドロータ 3D 可視化
 *
 * 座標変換 (物理ENU → Three.js):
 *   Three.js  x = 物理 x  (東)
 *   Three.js  y = 物理 z  (上)
 *   Three.js  z = -物理 y (南, Three.jsはz=手前)
 *
 * 回転行列変換: R_t = T * R_physics * T^T
 *   T = [[1,0,0],[0,0,1],[0,-1,0]]
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STATE_IDX }     from '../physics/QuadrotorModel.js';

/** ENU → Three.js 座標変換 */
function enu2three(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

/** 物理回転行列 → Three.js 回転行列 */
function physicsRotToThree(R) {
  // R_t = T * R * T^T  (T: ENU→Three.js 座標変換行列)
  // R_t[row][col]:
  //   [0,0]=R[0][0]  [0,1]=R[0][2]  [0,2]=-R[0][1]
  //   [1,0]=R[2][0]  [1,1]=R[2][2]  [1,2]=-R[2][1]
  //   [2,0]=-R[1][0] [2,1]=-R[1][2] [2,2]=R[1][1]
  const m = new THREE.Matrix4();
  m.set(
     R[0][0],  R[0][2], -R[0][1], 0,
     R[2][0],  R[2][2], -R[2][1], 0,
    -R[1][0], -R[1][2],  R[1][1], 0,
    0, 0, 0, 1
  );
  return m;
}

export class DroneVisualizer {
  /**
   * @param {HTMLElement} container
   * @param {import('../physics/QuadrotorModel.js').QuadrotorModel} model
   */
  constructor(container, model) {
    this.container = container;
    this.model     = model;
    this._propAngle = [0, 0, 0, 0]; // プロペラ回転角 [rad]

    this._autoFollow = false;
    this._camTween   = null; // { startPos, startLook, startUp, tPos, tLook, tUp, elapsed, duration }

    this._moveTargetMode = false;
    this._isDragging     = false;
    this._targetPlane    = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0); // y = setpoint.z
    this._raycaster      = new THREE.Raycaster();
    /** @type {((x:number,y:number,z:number)=>void)|null} */
    this.onTargetMove    = null;

    this._initRenderer();
    this._initScene();
    this._initDroneMesh();
    this._initTrajectory();
    this._initSetpointMarker();
    this._initHUD();
    this._initTargetDrag();

    window.addEventListener('resize', () => this._onResize());
  }

  // ─────────────────────────────────────────────────────────────────────────
  // 初期化
  // ─────────────────────────────────────────────────────────────────────────

  _initRenderer() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.setClearColor(0xe8eaee);
    this.container.appendChild(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 500);
    this.camera.position.set(5, 4, 5);
    this.camera.lookAt(0, 1, 0);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.minDistance = 0.5;
    this.controls.maxDistance = 100;
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0xe8eaee, 80, 250);

    // 環境光
    this.scene.add(new THREE.AmbientLight(0x8090a0, 0.9));

    // 指向性ライト (影付き)
    const dirLight = new THREE.DirectionalLight(0xffffff, 1.1);
    dirLight.position.set(10, 20, 10);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.set(2048, 2048);
    dirLight.shadow.camera.near = 0.1;
    dirLight.shadow.camera.far  = 100;
    dirLight.shadow.camera.left = dirLight.shadow.camera.bottom = -20;
    dirLight.shadow.camera.right = dirLight.shadow.camera.top  =  20;
    this.scene.add(dirLight);

    // 地面
    const groundGeo  = new THREE.PlaneGeometry(100, 100);
    const groundMat  = new THREE.MeshLambertMaterial({ color: 0xcdd1da });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    // グリッド
    const grid = new THREE.GridHelper(50, 50, 0xa0a5b5, 0xc4c8d4);
    grid.material.opacity = 0.7;
    grid.material.transparent = true;
    this.scene.add(grid);

    // 原点座標軸
    this.scene.add(new THREE.AxesHelper(2));
  }

  _initDroneMesh() {
    this.droneGroup = new THREE.Group();
    this.scene.add(this.droneGroup);

    const l = this.model.armLength;

    // ── ボディ ─────────────────────────────────────────────────────────
    const bodyGeo = new THREE.BoxGeometry(0.08, 0.04, 0.08);
    const bodyMat = new THREE.MeshLambertMaterial({ color: 0x0c1a3a });
    const body    = new THREE.Mesh(bodyGeo, bodyMat);
    body.castShadow = true;
    this.droneGroup.add(body);

    // ── アーム (+ コンフィグ) ──────────────────────────────────────────
    const armMat = new THREE.MeshLambertMaterial({ color: 0x162444 });
    const armPositions = [
      { x:  l, y: 0, z:  0 }, // 前 (+x)
      { x: -l, y: 0, z:  0 }, // 後 (-x)
      { x:  0, y: 0, z:  l }, // 右 (Three.js +z = 物理 -y)
      { x:  0, y: 0, z: -l }, // 左 (Three.js -z = 物理 +y)
    ];

    armPositions.forEach(({ x, y, z }) => {
      const len  = Math.sqrt(x * x + z * z);
      const armGeo = new THREE.CylinderGeometry(0.008, 0.008, len, 6);
      const arm    = new THREE.Mesh(armGeo, armMat);
      arm.position.set(x / 2, y, z / 2);
      // アームを中心から先端方向に向ける
      if (Math.abs(x) > 0) arm.rotation.z = Math.PI / 2;
      else                  arm.rotation.x = Math.PI / 2;
      arm.castShadow = true;
      this.droneGroup.add(arm);
    });

    // ── モータ・プロペラ ───────────────────────────────────────────────
    const motorColors = [0x2868a8, 0x1a3868, 0x2868a8, 0x1a3868];
    // Three.js 座標での各モータ位置: [前(+x,CCW), 左(-z,CW), 後(-x,CCW), 右(+z,CW)]
    const motorPos = [
      new THREE.Vector3( l, 0,  0),  // M1 前
      new THREE.Vector3( 0, 0, -l),  // M2 左 (Three.js -z = 物理 +y)
      new THREE.Vector3(-l, 0,  0),  // M3 後
      new THREE.Vector3( 0, 0,  l),  // M4 右 (Three.js +z = 物理 -y)
    ];

    this._propellers = [];

    motorPos.forEach((pos, i) => {
      // モータハウジング
      const mGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.03, 8);
      const mMat = new THREE.MeshLambertMaterial({ color: 0x0a1428 });
      const mMesh = new THREE.Mesh(mGeo, mMat);
      mMesh.position.copy(pos);
      this.droneGroup.add(mMesh);

      // プロペラ (薄い楕円形ディスク)
      const propGroup = new THREE.Group();
      propGroup.position.copy(pos);
      propGroup.position.y += 0.02;

      const propColor = motorColors[i];
      // 2枚ブレード
      for (let b = 0; b < 2; b++) {
        const bGeo = new THREE.BoxGeometry(l * 0.9, 0.003, 0.025);
        const bMat = new THREE.MeshLambertMaterial({
          color:       propColor,
          transparent: true,
          opacity:     0.8,
        });
        const blade = new THREE.Mesh(bGeo, bMat);
        blade.rotation.y = (b * Math.PI) / 2;
        propGroup.add(blade);
      }

      this.droneGroup.add(propGroup);
      this._propellers.push(propGroup);
    });

    // ── 前方インジケータ (赤LEDぽい) ──────────────────────────────────
    const ledGeo = new THREE.SphereGeometry(0.012, 6, 6);
    const ledMat = new THREE.MeshBasicMaterial({ color: 0x3db8c8 });
    const led    = new THREE.Mesh(ledGeo, ledMat);
    led.position.set(l, 0.025, 0);
    this.droneGroup.add(led);

    // ── 速度ベクトル矢印 ──────────────────────────────────────────────
    this._velArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      0.5, 0x3db8c8, 0.1, 0.05
    );
    this.scene.add(this._velArrow);
  }

  _initTrajectory() {
    const maxPts = 2000;
    const geo    = new THREE.BufferGeometry();
    const pos    = new Float32Array(maxPts * 3);
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setDrawRange(0, 0);

    const mat = new THREE.LineBasicMaterial({ color: 0x3db8c8, opacity: 0.6, transparent: true });
    this._trajLine   = new THREE.Line(geo, mat);
    this._trajPtsMax = maxPts;
    this._trajPts    = 0;
    this.scene.add(this._trajLine);
  }

  _initSetpointMarker() {
    const geo  = new THREE.SphereGeometry(0.08, 12, 12);
    const mat  = new THREE.MeshBasicMaterial({ color: 0xc87828, wireframe: true });
    this._spMarker = new THREE.Mesh(geo, mat);
    this.scene.add(this._spMarker);
  }

  _initHUD() {
    // Three.js シーン内の HUD は CSS overlayで実装 (main.js側)
  }

  // ─────────────────────────────────────────────────────────────────────────
  // 更新
  // ─────────────────────────────────────────────────────────────────────────

  /**
   * フレームを更新
   * @param {number[]} state      12次元状態ベクトル
   * @param {number[]} omega2     ローター角速度二乗
   * @param {{ x, y, z }} setpoint セットポイント
   * @param {number}   dt         フレーム時間 [s]
   */
  update(state, omega2, setpoint, dt) {
    const S = STATE_IDX;

    // ── ドローン位置 ────────────────────────────────────────────────────
    const pos = enu2three(state[S.X], state[S.Y], state[S.Z]);
    this.droneGroup.position.copy(pos);

    // ── ドローン姿勢 ────────────────────────────────────────────────────
    const R    = this.model.rotationMatrix(state[S.PHI], state[S.THETA], state[S.PSI]);
    const Rt   = physicsRotToThree(R);
    this.droneGroup.setRotationFromMatrix(Rt);

    // ── プロペラ回転 ────────────────────────────────────────────────────
    // M1, M3: CCW (+), M2, M4: CW (-)
    const dirs  = [1, -1, 1, -1];
    const scale = 0.005;
    omega2.forEach((w2, i) => {
      const omega = Math.sqrt(Math.max(0, w2));
      this._propAngle[i] += dirs[i] * omega * scale;
      this._propellers[i].rotation.y = this._propAngle[i];
    });

    // ── 速度ベクトル ────────────────────────────────────────────────────
    const vel = enu2three(state[S.VX], state[S.VY], state[S.VZ]);
    const speed = vel.length();
    if (speed > 0.05) {
      this._velArrow.position.copy(pos);
      this._velArrow.setDirection(vel.clone().normalize());
      this._velArrow.setLength(Math.min(speed * 0.3, 2), 0.1, 0.05);
      this._velArrow.visible = true;
    } else {
      this._velArrow.visible = false;
    }

    // ── セットポイントマーカー ──────────────────────────────────────────
    this._spMarker.position.copy(enu2three(setpoint.x, setpoint.y, setpoint.z));
    // ターゲット平面をセットポイント高さに合わせる (Three.js y = 物理 z)
    this._targetPlane.constant = -setpoint.z;

    // ── カメラトゥイーン ────────────────────────────────────────────────
    if (this._camTween) {
      const t = this._camTween;
      t.elapsed = Math.min(t.duration, t.elapsed + dt);
      const p    = t.elapsed / t.duration;
      const ease = 1 - Math.pow(1 - p, 3); // cubic ease-out
      this.camera.position.lerpVectors(t.startPos,  t.tPos,  ease);
      this.controls.target.lerpVectors(t.startLook, t.tLook, ease);
      this.camera.up.lerpVectors(t.startUp, t.tUp, ease).normalize();
      if (p >= 1) this._camTween = null;
    }

    // ── 自動追尾 ─────────────────────────────────────────────────────
    if (this._autoFollow && !this._camTween) {
      this.controls.target.lerp(pos, 0.04);
    }

    this.controls.update();
  }

  // ─────────────────────────────────────────────────────────────────────────
  // カメラ制御
  // ─────────────────────────────────────────────────────────────────────────

  /**
   * カメラトゥイーン開始 (内部ヘルパー)
   */
  _startTween(tPos, tLook, tUp, duration = 0.8) {
    this._camTween = {
      startPos:  this.camera.position.clone(),
      startLook: this.controls.target.clone(),
      startUp:   this.camera.up.clone(),
      tPos, tLook, tUp,
      elapsed: 0, duration,
    };
  }

  /**
   * プリセットビューに切り替え
   * @param {'xy'|'xz'|'yz'} preset
   */
  setCameraView(preset) {
    this._autoFollow = false;
    const D = 14;
    const views = {
      // TOP: 物理 XY 平面を上から俯瞰 (Three.js Y 軸上方から)
      xy: { pos: new THREE.Vector3(0.001, 18, 0.001), look: new THREE.Vector3(0, 0, 0), up: new THREE.Vector3(1, 0, 0) },
      // FRONT: 物理 XZ 平面 (Three.js +Z 方向から見る)
      xz: { pos: new THREE.Vector3(0, 2, D),           look: new THREE.Vector3(0, 1, 0), up: new THREE.Vector3(0, 1, 0) },
      // SIDE: 物理 YZ 平面 (Three.js +X 方向から見る)
      yz: { pos: new THREE.Vector3(D, 2, 0),            look: new THREE.Vector3(0, 1, 0), up: new THREE.Vector3(0, 1, 0) },
    };
    const v = views[preset];
    if (!v) return;
    this._startTween(v.pos, v.look, v.up);
  }

  /**
   * ドローンを画角に収める (FIT)
   * @param {number[]} state 12次元状態ベクトル
   */
  fitView(state) {
    this._autoFollow = false;
    const dronePos = enu2three(state[0], state[1], state[2]);
    const dist     = Math.max(6, dronePos.length() + 5);
    const dir      = new THREE.Vector3(1, 0.8, 1).normalize();
    const newPos   = dronePos.clone().addScaledVector(dir, dist);
    this._startTween(newPos, dronePos.clone(), new THREE.Vector3(0, 1, 0));
  }

  /**
   * 自動追尾トグル
   * @param {boolean} enabled
   */
  setAutoFollow(enabled) {
    this._autoFollow = enabled;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // ターゲットドラッグ
  // ─────────────────────────────────────────────────────────────────────────

  _initTargetDrag() {
    const canvas = this.renderer.domElement;

    canvas.addEventListener('mousedown', (e) => {
      if (!this._moveTargetMode || e.button !== 0) return;
      e.stopPropagation();
      this._isDragging = true;
      this.controls.enabled = false;
      this.container.classList.add('dragging');
      this._updateTargetFromMouse(e);
    });

    window.addEventListener('mousemove', (e) => {
      if (!this._isDragging) return;
      this._updateTargetFromMouse(e);
    });

    window.addEventListener('mouseup', () => {
      if (!this._isDragging) return;
      this._isDragging = false;
      this.controls.enabled = true;
      this.container.classList.remove('dragging');
    });
  }

  _updateTargetFromMouse(e) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const nx   = ((e.clientX - rect.left) / rect.width)  *  2 - 1;
    const ny   = ((e.clientY - rect.top)  / rect.height) * -2 + 1;

    this._raycaster.setFromCamera(new THREE.Vector2(nx, ny), this.camera);
    const hit = new THREE.Vector3();
    if (!this._raycaster.ray.intersectPlane(this._targetPlane, hit)) return;

    // Three.js (x, y, z) → 物理 ENU (x=x, y=-z, z=y)
    if (this.onTargetMove) this.onTargetMove(hit.x, -hit.z, hit.y);
  }

  /**
   * ターゲットドラッグモードをトグル
   * @param {boolean} enabled
   */
  setMoveTargetMode(enabled) {
    this._moveTargetMode = enabled;
    this.renderer.domElement.style.cursor = enabled ? 'crosshair' : '';
    if (!enabled) {
      this._isDragging = false;
      this.controls.enabled = true;
      this.container.classList.remove('dragging');
    }
  }

  /**
   * トラジェクトリ点を追加
   * @param {number} px @param {number} py @param {number} pz  (物理ENU座標)
   */
  addTrajectoryPoint(px, py, pz) {
    const attr = this._trajLine.geometry.attributes.position;
    const data = attr.array;
    const tp   = enu2three(px, py, pz);

    if (this._trajPts < this._trajPtsMax) {
      data[this._trajPts * 3]     = tp.x;
      data[this._trajPts * 3 + 1] = tp.y;
      data[this._trajPts * 3 + 2] = tp.z;
      this._trajPts++;
    } else {
      // 古い点をシフト
      for (let i = 0; i < (this._trajPtsMax - 1) * 3; i++) {
        data[i] = data[i + 3];
      }
      data[(this._trajPtsMax - 1) * 3]     = tp.x;
      data[(this._trajPtsMax - 1) * 3 + 1] = tp.y;
      data[(this._trajPtsMax - 1) * 3 + 2] = tp.z;
    }

    attr.needsUpdate = true;
    this._trajLine.geometry.setDrawRange(0, this._trajPts);
  }

  /**
   * トラジェクトリをクリア
   */
  clearTrajectory() {
    this._trajPts = 0;
    this._trajLine.geometry.setDrawRange(0, 0);
  }

  /**
   * レンダリング
   */
  render() {
    this.renderer.render(this.scene, this.camera);
  }

  _onResize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }
}

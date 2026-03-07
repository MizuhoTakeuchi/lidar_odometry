# lidar_odometry パッケージ - アルゴリズム・アーキテクチャドキュメント

## 1. 概要

`lidar_odometry` は ROS 2 ノードとして実装されたスタンドアロンの LiDAR オドメトリパッケージである。LIO-SAM の scan-to-map マッチングアルゴリズムをベースに、IMU に依存しない純粋な LiDAR オドメトリとして再構成されている。エッジ/平面特徴量の抽出、ローカルマップへのスキャンマッチング、Levenberg-Marquardt 最適化、退化検出、共分散推定を一つのノードに統合している。

## 2. ファイル構成

```
lidar_odometry/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params.yaml              # パラメータ定義
├── include/lidar_odometry/
│   └── lidar_odometry_node.hpp  # ノードクラス定義・点群型定義
├── launch/
│   └── lidar_odometry.launch.py # launchファイル
└── src/
    └── lidar_odometry_node.cpp  # 全アルゴリズム実装 (~1240行)
```

## 3. 依存関係

| カテゴリ | ライブラリ | 用途 |
|----------|-----------|------|
| ROS 2 | rclcpp, sensor_msgs, geometry_msgs, nav_msgs, tf2, tf2_ros, tf2_eigen, tf2_geometry_msgs, pcl_conversions | ROS通信・座標変換 |
| 点群処理 | PCL (KdTree, VoxelGrid, transforms) | 近傍探索・ダウンサンプリング |
| 線形代数 | Eigen3 | 姿勢表現・行列演算 |
| 行列演算 | OpenCV | SVD・固有値分解・線形ソルバ |
| 並列化 | OpenMP | 対応点探索の並列化 |

## 4. ROS インターフェース

### 4.1 Subscribe

| トピック | 型 | 説明 |
|---------|---|------|
| `~/input/cloud` | `sensor_msgs/PointCloud2` | 入力LiDAR点群 (BestEffort QoS) |

### 4.2 Publish

| トピック | 型 | 説明 |
|---------|---|------|
| `~/output/relative_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 前フレームとの相対姿勢 + 共分散 |
| `~/output/odometry` | `nav_msgs/Odometry` | 累積オドメトリ姿勢 + 共分散 |
| `~/debug/local_map` | `sensor_msgs/PointCloud2` | ローカルマップ点群 (デバッグ用) |
| `~/debug/corners` | `sensor_msgs/PointCloud2` | エッジ特徴点群 (デバッグ用) |
| `~/debug/surfaces` | `sensor_msgs/PointCloud2` | 平面特徴点群 (デバッグ用) |

### 4.3 TF Broadcast

| 親フレーム | 子フレーム | 条件 |
|-----------|-----------|------|
| `odom_frame` (default: `odom`) | `lidar_frame` (default: `lidar_link`) | `publish_tf: true` 時 |

## 5. 処理パイプライン

コールバック `cloudCallback()` 内で以下の10ステップを逐次実行する。

```
PointCloud2 受信
  │
  ▼
(1) convertPointCloud     ── センサ形式変換 + NaN除去
  │
  ▼
(2) projectToRangeImage   ── 距離画像への投影
  │
  ▼
(3) extractCloudFromRangeImage ── 有効点の抽出
  │
  ▼
(4) extractFeatures       ── エッジ/平面特徴量抽出
  │    ├── calculateSmoothness
  │    ├── markOccludedPoints
  │    └── extractFeaturePoints
  │
  ▼
(5) computeInitialGuess   ── 等速度モデルによる初期推定
  │
  ▼
(6) buildLocalMap         ── キーフレームからローカルマップ構築
  │
  ▼
(7) scanToMapOptimization ── LM法によるスキャン-マップ最適化
  │    ├── cornerOptimization  (エッジ対応点探索)
  │    ├── surfOptimization    (平面対応点探索)
  │    ├── combineOptimizationCoeffs
  │    └── lmOptimization      (LM更新ステップ)
  │
  ▼
(8) computeCovariance     ── ヤコビアンからの共分散推定
  │
  ▼
(9) publishResults        ── 相対姿勢・累積オドメトリ・TF配信
  │
  ▼
(10) updateKeyframes      ── キーフレーム登録・速度モデル更新
```

## 6. アルゴリズム詳細

### 6.1 点群変換 (Step 1)

対応するセンサタイプに応じた点群フォーマット変換を行う。

- **Velodyne**: `VelodynePointXYZIRT` をそのまま読み込み。ring フィールドが無い場合は仰角から計算
- **Ouster**: `OusterPointXYZIRT` (タイムスタンプが `uint32_t t` ナノ秒) を Velodyne 形式に変換
- **Livox**: 非回転型LiDARとして列インデックスを順番に割り当て

### 6.2 距離画像投影 (Step 2)

3D点群を `n_scan × horizon_scan` の2D距離画像 (`range_mat_`) に投影する。

- **行** (row): ring チャネル番号 (= スキャンライン)
- **列** (column): 水平角度 `atan2(x, y)` を等角度分解能でインデックス化
- 範囲フィルタ: `lidar_min_range` ～ `lidar_max_range`
- ダウンサンプリング: `downsample_rate` に基づくリング間引き

### 6.3 特徴量抽出 (Step 4)

LIO-SAM/LOAM の特徴抽出アルゴリズムに基づく。

#### 6.3.1 平滑度 (Curvature) 計算

各点 `i` について前後5点の距離差の二乗を計算:

```
c(i) = [Σ(j=-5..5, j≠0) range(i+j) - 10 × range(i)]²
```

#### 6.3.2 遮蔽点の除外

- **遮蔽判定**: 隣接点間の距離差が 0.3m 以上 → 手前側の5点をマーク
- **平行レイ判定**: 前後点との距離差がともに range の 2% 超 → マーク

#### 6.3.3 特徴点選択

各スキャンラインを6セグメントに分割し、セグメントごとに:

- **エッジ特徴**: 曲率降順で最大20点を選択 (曲率 > `edge_threshold`)
- **平面特徴**: 曲率昇順で選択 (曲率 < `surf_threshold`)、Voxel Grid でダウンサンプリング
- 選択後の非最大抑制: 隣接5点をピック済みとしてマーク

### 6.4 初期推定 (Step 5)

- **1フレーム目**: 単位姿勢 (原点)
- **2フレーム目**: 前フレーム姿勢をそのまま使用 (ゼロ速度モデル)
- **3フレーム目以降**: 等速度モデル
  ```
  delta = prev_prev_pose⁻¹ × prev_pose
  initial_guess = prev_pose × delta
  ```

### 6.5 ローカルマップ構築 (Step 6)

- キーフレームの `deque` から全エッジ/平面点群をワールド座標に変換して結合
- Voxel Grid で corner/surface マップをそれぞれダウンサンプリング
- KdTree を構築して対応点探索に使用

### 6.6 Scan-to-Map 最適化 (Step 7)

#### 6.6.1 エッジ対応 (`cornerOptimization`)

1. 現在のスキャンのエッジ点をマップ座標に変換
2. KdTree で最近傍5点を検索
3. 5点の共分散行列の固有値分解で**線状性を判定** (λ₁ > 3λ₂)
4. 主成分方向の直線に対する点-直線距離を残差とする
5. 距離に基づく重み `s = 1 - 0.9|d|` でロバスト化

#### 6.6.2 平面対応 (`surfOptimization`)

1. 現在のスキャンの平面点をマップ座標に変換
2. KdTree で最近傍5点を検索
3. 5点による最小二乗平面フィッティング (`Ax + By + Cz + 1 = 0`)
4. 平面の妥当性を検証 (全5点が平面から0.2m以内)
5. 点-平面距離を残差とする
6. 距離に基づく重み `s = 1 - 0.9|d| / √(√r)` でロバスト化 (r は原点からの距離)

#### 6.6.3 LM最適化 (`lmOptimization`)

6自由度 `[roll, pitch, yaw, x, y, z]` を反復的に最適化する。

1. ヤコビアン行列 `A` (N×6) と残差ベクトル `B` (N×1) を構築
2. 正規方程式 `AᵀA × ΔX = AᵀB` を QR分解で解く
3. **退化検出** (初回イテレーションのみ): `AᵀA` の固有値分解で `degeneracy_threshold` 未満の固有値に対応する方向の更新を抑制 (射影行列 `P` で制限)
4. 収束判定: 回転変化 < 0.05° かつ並進変化 < 0.05cm
5. 最大 `max_iterations` 回 (デフォルト30回) まで反復

座標変換は LIO-SAM/LOAM のレガシー慣例 (lidar → camera 座標系変換) を内部で使用。

### 6.7 共分散推定 (Step 8)

収束後のヤコビアンと残差から共分散行列を推定する。

1. 収束姿勢でヤコビアン `A` と残差 `B` を再計算
2. Fisher 情報行列: `H = AᵀA`
3. 残差分散: `σ² = ||B||² / max(N - 6, 1)`
4. 共分散: `Σ = σ² × (AᵀA)⁻¹` (SVD による擬似逆行列)
5. **退化方向の膨張**: 退化が検出された場合、該当する固有ベクトル方向に `degeneracy_covariance` を加算
   ```
   Σ += degeneracy_covariance × v × vᵀ
   ```
6. LIO-SAM 内部順序 `[roll, pitch, yaw, x, y, z]` → ROS 標準順序 `[x, y, z, roll, pitch, yaw]` に並び替え

### 6.8 キーフレーム管理 (Step 10)

以下の条件でキーフレームを追加:

- 初回フレーム
- 前回のキーフレームからの移動距離 ≥ `keyframe_dist_threshold` (default: 1.0m)
- 前回のキーフレームからの回転角 ≥ `keyframe_angle_threshold` (default: 0.2rad ≈ 11.5°)

キーフレームの最大保持数は `max_local_map_keyframes` (default: 50) で、FIFO方式で古いものから削除される。

## 7. パラメータ一覧

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `sensor` | `"ouster"` | センサタイプ (`velodyne`, `ouster`, `livox`) |
| `n_scan` | 64 | スキャンライン数 |
| `horizon_scan` | 512 | 水平方向の分解能 |
| `downsample_rate` | 1 | リングダウンサンプリングレート |
| `lidar_min_range` | 1.0 | 最小距離フィルタ [m] |
| `lidar_max_range` | 1000.0 | 最大距離フィルタ [m] |
| `edge_threshold` | 1.0 | エッジ特徴の曲率閾値 |
| `surf_threshold` | 0.1 | 平面特徴の曲率閾値 |
| `edge_feature_min_valid_num` | 10 | 最適化に必要な最小エッジ特徴数 |
| `surf_feature_min_valid_num` | 100 | 最適化に必要な最小平面特徴数 |
| `corner_leaf_size` | 0.2 | エッジ点群のVoxelサイズ [m] |
| `surf_leaf_size` | 0.4 | 平面点群のVoxelサイズ [m] |
| `max_iterations` | 30 | LM最適化の最大反復回数 |
| `degeneracy_threshold` | 100.0 | 退化検出の固有値閾値 |
| `degeneracy_covariance` | 1.0e+4 | 退化方向の共分散膨張値 |
| `keyframe_dist_threshold` | 1.0 | キーフレーム追加の距離閾値 [m] |
| `keyframe_angle_threshold` | 0.2 | キーフレーム追加の角度閾値 [rad] |
| `max_local_map_keyframes` | 50 | ローカルマップの最大キーフレーム数 |
| `num_threads` | 4 | OpenMPスレッド数 |
| `lidar_frame` | `"lidar_link"` | LiDARフレームID |
| `odom_frame` | `"odom"` | オドメトリフレームID |
| `publish_tf` | `true` | TFブロードキャストの有効化 |

## 8. アーキテクチャ上の特徴

### シングルノード設計
LIO-SAM では imageProjection, featureExtraction, mapOptimization の3ノードに分離されていたが、本パッケージではすべてを1ノード・1コールバック内で逐次処理する。ノード間通信のオーバーヘッドを排除し、レイテンシを最小化している。

### IMU非依存
LIO-SAM が IMU プリインテグレーションで初期推定・デスキュー・姿勢制約を行うのに対し、本パッケージは等速度モデルのみで初期推定を行い、デスキューを省略している。これにより IMU センサが不要になるが、急激な運動時の推定精度は低下しうる。

### 共分散推定付き出力
最適化のヤコビアンから6×6共分散行列を推定し、退化検出と連動した共分散膨張機能を備える。これによりダウンストリームの EKF や因子グラフに対して信頼度情報を提供可能。

### OpenMP 並列化
対応点探索 (`cornerOptimization`, `surfOptimization`) と点群変換 (`transformPointCloud`) で OpenMP による並列化を行い、リアルタイム性能を確保する。

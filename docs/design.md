# LiDAR Odometry 設計ドキュメント

## 概要

Velodyne 3D LiDARの点群データを用いて、NDT（Normal Distributions Transform）スキャンマッチングによるオドメトリ推定を行うROS2ノード。

## システム構成

```
                    +-------------------------+
 /velodyne_points   |   LidarOdometryNode     |  /lidar_odometry/odom
 PointCloud2 ------>|                         |------> PoseWithCovarianceStamped
                    |  1. 距離フィルタ          |
                    |  2. Voxelダウンサンプリング |  /lidar_odometry/trans_pose
                    |  3. NDTマッチング         |------> PoseWithCovarianceStamped
                    |  4. 共分散推定            |
                    +-------------------------+
                             |
                    ndt_omp_ros2 (ライブラリ)
```

## 入出力

### 入力

| トピック | 型 | 説明 |
|---|---|---|
| `/velodyne_points` (変更可) | `sensor_msgs/msg/PointCloud2` | 前処理済みの3D点群。ring情報は使用しない |

### 出力

| トピック | 型 | 説明 |
|---|---|---|
| `/lidar_odometry/odom` | `geometry_msgs/msg/PoseWithCovarianceStamped` | ワールド座標系での累積姿勢 |
| `/lidar_odometry/trans_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1時刻前からの相対位置変換 |

## アルゴリズム

### 処理フロー

```
PointCloud2受信
    │
    ▼
[1] pcl::PointXYZに変換（ring情報不使用）
    │
    ▼
[2] 距離フィルタ
    min_scan_range ≤ √(x²+y²+z²) ≤ max_scan_range の点のみ保持
    │
    ▼
[3] VoxelGridダウンサンプリング（voxel_leaf_size単位）
    │
    ▼
[4] 初フレーム？ ─Yes→ prev_cloudに保存、単位行列をPublish、終了
    │No
    ▼
[5] NDTアライメント
    ・Target: 前フレーム点群 (prev_cloud)
    ・Source: 現フレーム点群
    ・初期推定: 前回のフレーム間変換 (定速モデル)
    │
    ▼
[6] 収束判定
    ・hasConverged() == false → 警告ログ、prev_cloud更新のみ
    │true
    ▼
[7] 姿勢更新
    delta = getFinalTransformation()
    cumulative_pose = cumulative_pose × delta
    │
    ▼
[8] 共分散推定（後述）
    │
    ▼
[9] Publish
    ・odom: cumulative_pose + 共分散
    ・trans_pose: delta + 共分散
    │
    ▼
[10] 状態更新
     prev_cloud = 現フレーム点群
     prev_delta = delta
```

### NDTスキャンマッチング

NDT（Normal Distributions Transform）は、ターゲット点群をボクセルグリッドに分割し、各ボクセル内の点群分布を正規分布で近似する。ソース点群をターゲットの正規分布に対してフィッティングすることで、6自由度の剛体変換を推定する。

本パッケージでは`ndt_omp_ros2`ライブラリを使用し、OpenMPによるマルチスレッド並列化で高速な処理を実現している。

**フレーム間マッチング方式を採用:**
- 現フレームを前フレームに対してマッチング（地図管理不要）
- 定速モデルによる初期推定で収束を高速化

#### NDTパラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `ndt_resolution` | 1.0 | ボクセル解像度 [m]。小さいほど精密だが計算コスト増 |
| `ndt_step_size` | 0.1 | Newton法ライン探索の最大ステップ幅 |
| `ndt_max_iterations` | 30 | 最大反復数 |
| `ndt_epsilon` | 0.01 | 収束判定閾値（変換差分がこの値以下で収束） |
| `ndt_num_threads` | 4 | OpenMPスレッド数 |
| `ndt_search_method` | "DIRECT7" | 近傍探索方式。KDTREE/DIRECT26/DIRECT7/DIRECT1 |

近傍探索方式の比較:

| 方式 | 速度 | 精度 | 用途 |
|---|---|---|---|
| KDTREE | 遅い | 高い | 精度重視 |
| DIRECT26 | 中間 | 中間 | バランス型 |
| DIRECT7 | 速い | 十分 | 一般用途（推奨） |
| DIRECT1 | 最速 | やや低い | リアルタイム重視 |

### 共分散推定

NDTは変換の共分散を直接出力しないため、アライメント品質指標からスケーリング方式で6x6対角共分散行列を推定する。

#### 使用する品質指標

| 指標 | 取得API | 意味 |
|---|---|---|
| Transformation Probability | `getTransformationProbability()` | アライメント確率。高い=良好 |
| Final Iterations | `getFinalNumIteration()` | 収束までの反復数。多い=収束困難 |
| Mean Correspondence Distance | `getLastMeanCorrespondenceDistance()` | 平均対応点距離。大きい=適合度低 |

#### 計算式

```
prob_scale  = score_scale_factor / probability      （確率が低いほど不確か）
iter_scale  = 1 + (iterations / max_iterations) × 2  （反復数が多いほど不確か）
dist_scale  = 1 + mean_distance × 2                  （距離が大きいほど不確か）

total_scale = clamp(prob_scale × iter_scale × dist_scale, 0.1, 100.0)

covariance = diag(
    trans_var × scale,  // x
    trans_var × scale,  // y
    trans_var × scale,  // z
    rot_var × scale,    // roll
    rot_var × scale,    // pitch
    rot_var × scale     // yaw
)
```

#### 共分散パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `base_translation_variance` | 0.01 | 基本並進分散 [m²] |
| `base_rotation_variance` | 0.001 | 基本回転分散 [rad²] |
| `score_scale_factor` | 1.0 | 確率スケーリング係数 |

## ファイル構成

```
lidar_odometry/
├── CMakeLists.txt                              # ビルド設定
├── package.xml                                 # パッケージ依存定義
├── include/lidar_odometry/
│   └── lidar_odometry_node.hpp                 # クラス宣言
├── src/
│   ├── main.cpp                                # エントリポイント
│   ├── lidar_odometry_node.cpp                 # ノード実装
│   └── ndt_omp_instantiation.cpp               # NDTテンプレートインスタンス化
├── launch/
│   └── lidar_odometry.launch.py                # Launch定義
├── config/
│   └── params.yaml                             # パラメータ設定ファイル
└── docs/
    └── design.md                               # 本ドキュメント
```

## 依存関係

| パッケージ | 用途 |
|---|---|
| `rclcpp` | ROS2 C++クライアントライブラリ |
| `sensor_msgs` | PointCloud2メッセージ型 |
| `geometry_msgs` | PoseWithCovarianceStampedメッセージ型 |
| `tf2`, `tf2_ros`, `tf2_geometry_msgs` | 座標変換 |
| `pcl_conversions` | ROS⇔PCL変換 |
| `ndt_omp_ros2` | NDTアルゴリズム（ヘッダのみ使用） |
| PCL | 点群処理（VoxelGrid等） |
| OpenMP | NDTのマルチスレッド並列化 |

## 使用方法

### ビルド

```bash
cd ~/lidar_ws
colcon build --packages-select lidar_odometry
```

### 起動

```bash
source install/setup.bash
ros2 launch lidar_odometry lidar_odometry.launch.py
```

### パラメータ変更

`config/params.yaml`を編集するか、launch時に上書き:

```bash
ros2 launch lidar_odometry lidar_odometry.launch.py \
  --ros-args -p ndt_resolution:=2.0 -p ndt_num_threads:=8
```

## パラメータチューニングガイド

### 精度を上げたい場合
- `ndt_resolution`を小さく（0.5〜1.0）
- `voxel_leaf_size`を小さく（0.2〜0.3）
- `ndt_search_method`を`KDTREE`に
- `ndt_max_iterations`を増やす（50〜100）

### 処理速度を上げたい場合
- `ndt_resolution`を大きく（2.0〜5.0）
- `voxel_leaf_size`を大きく（1.0〜2.0）
- `ndt_search_method`を`DIRECT1`に
- `ndt_num_threads`を増やす
- `max_scan_range`を小さくして入力点数を減らす

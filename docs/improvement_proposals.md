# LiDAR オドメトリ 推定精度向上 改良案

## 現状の分析

### 現在のアーキテクチャ

| 項目 | 現在の実装 |
|------|-----------|
| スキャンマッチング | NDT (ndt_omp_ros2) フレーム間マッチング |
| 動作モデル | 等速度モデル（前フレームの delta を初期値に使用） |
| 点群前処理 | 距離フィルタ + Voxel Grid ダウンサンプリング |
| 共分散推定 | NDT スコアベースの対角共分散スケーリング |
| 座標変換 | LiDAR→車両座標の外因性パラメータ（共役変換） |
| 出力 | 累積姿勢 + フレーム間差分（PoseWithCovarianceStamped） |

### 精度上の主要課題

1. **フレーム間マッチングによるドリフト蓄積** — 各フレーム間の微小誤差が累積し、長距離走行で大きな位置ずれが発生
2. **等速度モデルの限界** — 加減速・旋回時に初期推定が実際の動きから乖離し、NDT の収束品質が低下
3. **環境依存の劣化検出なし** — 長い廊下・広い平面等、幾何特徴が乏しい環境で NDT が退化しても検出・対策されない
4. **点群前処理が最小限** — 地面点・動的物体がマッチングに含まれ、精度を低下させている
5. **共分散が等方的** — 実際の不確実性は方向によって異なるが、全軸同一スケーリングで表現している

---

## 改良案一覧

| 優先度 | 改良案 | 期待効果 | 実装難度 | 影響範囲 |
|--------|--------|---------|---------|---------|
| **S** | 1. ローカルマップマッチング | ドリフト大幅削減 | 中 | 大 |
| **S** | 2. IMU 融合による動作予測 | 収束安定性・旋回精度向上 | 中 | 中 |
| **A** | 3. 地面点除去 | マッチング品質向上 | 低 | 小 |
| **A** | 4. 幾何的退化検出 | 廊下・平面環境での信頼性確保 | 中 | 中 |
| **A** | 5. キーフレーム戦略 | ドリフト抑制・計算効率化 | 低 | 中 |
| **B** | 6. 動的物体除去 | 動的環境でのロバスト性向上 | 中 | 小 |
| **B** | 7. 多段解像度 NDT | 大変位への対応力・収束速度向上 | 低 | 小 |
| **B** | 8. Hessian ベース共分散推定 | より正確な不確実性表現 | 中 | 小 |
| **C** | 9. ループクロージャ | 長距離のドリフト補正 | 高 | 大 |
| **C** | 10. スライディングウィンドウ最適化 | 局所的な軌跡最適化 | 高 | 大 |
| **C** | 11. 適応的パラメータ調整 | 環境に応じた自律的な精度最適化 | 中 | 中 |

---

## 改良案詳細

### 1. ローカルマップマッチング（優先度: S）

#### 現状の問題

現在はフレーム間（frame-to-frame）マッチングを行っている（`lidar_odometry_node.cpp:134`）。前フレーム1枚のみをターゲットとするため、点群密度が低く、NDT のボクセル内点数が不足してノイズに弱い。

```cpp
// 現在の実装
ndt_.setInputTarget(prev_cloud_);  // 前フレーム1枚のみ
ndt_.setInputSource(filtered_cloud);
```

#### 改良内容

直近 N フレーム（例: 10〜20フレーム）の点群を累積したローカルマップを NDT のターゲットとして使用する（scan-to-map マッチング）。

**設計:**

```
ローカルマップ管理:
  - std::deque<PointCloud::Ptr> local_map_frames_  (最大 N フレーム)
  - PointCloud::Ptr local_map_  (統合済みマップ)

更新タイミング:
  - 新フレーム受信時、推定姿勢で変換して local_map_frames_ に追加
  - N フレーム超過時、古いフレームを削除
  - 統合マップを再生成（または差分更新）してダウンサンプリング

マッチング:
  ndt_.setInputTarget(local_map_);      // ローカルマップ
  ndt_.setInputSource(filtered_cloud);  // 現在フレーム
```

**追加パラメータ:**

```yaml
local_map_size: 15           # 保持フレーム数
local_map_voxel_size: 0.5    # マップのダウンサンプリング解像度 [m]
```

#### 期待効果

- NDT ボクセル内の点数増加により、正規分布の推定精度が向上
- 単一フレームのノイズ・欠損に対するロバスト性向上
- 位置ドリフト率が概ね 30〜50% 改善（文献値）

#### 注意点

- マップ更新の計算コスト管理（毎フレーム全再構成は避け、差分更新を推奨）
- マップサイズが大きすぎると自車の移動による矛盾が生じるため、距離・フレーム数の上限を設ける
- ローカルマップのボクセルサイズは NDT 解像度より小さくする必要がある

---

### 2. IMU 融合による動作予測（優先度: S）

#### 現状の問題

等速度モデル（`prev_delta_`）は定速直線走行でのみ有効。加減速・旋回時に初期推定の誤差が大きくなり、NDT が局所解に陥るリスクがある。

```cpp
// 現在の実装: 等速度仮定
ndt_.align(*aligned, prev_delta_);
```

#### 改良内容

IMU（加速度・角速度）を購読し、NDT の初期推定に使用する。

**設計:**

```
IMU 統合:
  - sensor_msgs/msg/Imu を購読
  - フレーム間の角速度を積分して回転を予測
  - 加速度を二重積分して並進を予測（短期間のため積分誤差は許容範囲）

初期推定 = IMU 予測回転 × 等速度モデルの並進
  → 回転は IMU の方が信頼性が高い
  → 並進は等速度モデルの方が安定的な場合が多い

ハイブリッド戦略:
  rotation_init = imu_predicted_rotation
  translation_init = prev_delta_.translation()  // 等速度モデルを維持
  initial_guess = compose(rotation_init, translation_init)
```

**追加パラメータ:**

```yaml
imu_topic: "/imu/data"
use_imu_prediction: true
imu_rotation_weight: 0.8    # IMU回転の信頼重み (0.0=等速度, 1.0=IMU完全信頼)
```

#### 期待効果

- 旋回時の NDT 収束安定性が大幅に向上
- 急ブレーキ・急加速時の追従性改善
- NDT の反復回数削減による処理速度向上

#### 注意点

- IMU のバイアス補正が必要（静止時キャリブレーションまたはオンライン推定）
- IMU と LiDAR のタイムスタンプ同期が重要（補間処理を推奨）
- IMU の外因性キャリブレーション（LiDAR-IMU 間の相対姿勢）が必要

---

### 3. 地面点除去（優先度: A）

#### 現状の問題

地面点がスキャンマッチングに含まれている。平坦な地面は幾何的特徴量が乏しく、NDT の水平方向（x, y）の拘束力が弱くなる。また、路面の傾斜変化が擬似的な動きとして検出される。

#### 改良内容

点群前処理パイプラインに地面点除去ステップを追加する。

**方法 A: 高さベースのシンプルな除去**

```
ground_height_threshold: -1.5  # [m] LiDAR座標系での地面高さ閾値
→ p.z < threshold の点を除去
```

- 実装が最も単純
- 平坦な地面では十分な精度
- 傾斜地では不正確

**方法 B: RANSAC 平面フィッティング（推奨）**

```
処理フロー:
  1. 入力点群に対して RANSAC で平面モデルを推定
  2. 推定平面からの距離が閾値以下の点を地面点として除去
  3. 残りの点を NDT に入力

パラメータ:
  ground_removal_method: "ransac"
  ransac_distance_threshold: 0.15   # [m]
  ransac_max_iterations: 100
```

- 傾斜地でも動作
- PCL の `SACSegmentation` で容易に実装可能

**方法 C: リングベース地面推定（Velodyne 等マルチリング LiDAR 用）**

```
各リングの最低点を地面候補とし、隣接リング間の勾配で地面/非地面を判定
→ PointXYZIR（リング情報付き）の入力が必要
```

- 最も高精度だが、現在 PointXYZ のみ使用しているため型変更が必要

#### 処理パイプラインへの組み込み

```
距離フィルタ → 地面点除去（新規） → Voxel ダウンサンプリング → NDT
```

#### 期待効果

- 特に平坦な環境での水平方向マッチング精度向上
- 点群数削減による NDT 高速化（地面点は全体の 20〜40% を占める）
- 路面傾斜変化による擬似的な z 方向・pitch ドリフトの軽減

---

### 4. 幾何的退化検出（優先度: A）

#### 現状の問題

長い廊下、広い平面、トンネルなど、特定方向の幾何的特徴が不足する環境では NDT が退化（degeneracy）し、特定方向の推定が信頼できなくなる。現在この状況を検出する仕組みがない。

#### 改良内容

NDT の Hessian 行列の固有値解析により、マッチングの退化方向を検出する。

**設計:**

```
退化検出アルゴリズム:
  1. NDT 収束後の Hessian 行列 H (6×6) を取得
  2. H の固有値分解: H = V Λ V^T
  3. 最小固有値 λ_min を閾値と比較

  if λ_min < degeneracy_threshold:
    → 退化を検出
    → 対応する固有ベクトル方向の共分散を大幅に増加
    → ログに退化方向を出力

退化時の対策:
  - 退化方向の共分散を 10〜100 倍に増加
  - 退化方向の推定値を IMU 予測値で代替（IMU 融合実装時）
  - 退化スコアをメッセージに含めて下流ノードに通知
```

**追加パラメータ:**

```yaml
enable_degeneracy_detection: true
degeneracy_eigenvalue_threshold: 10.0   # Hessian最小固有値の閾値
degeneracy_covariance_scale: 50.0       # 退化時の共分散スケール倍率
```

#### 期待効果

- 退化環境でのフィルタ発散防止（EKF/UKF と組み合わせる場合に特に重要）
- 信頼できない方向の推定値に過度に依存するのを防止
- 下流の統合フィルタ（GNSS 融合等）に正確な不確実性情報を提供

#### 参考

- Zhang & Singh, "Laser Visual Inertial Odometry and Mapping with High Robustness and Low Drift" (LOAM degeneracy handling)
- 固有値比率 `λ_max / λ_min` が大きい場合も退化の兆候

---

### 5. キーフレーム戦略（優先度: A）

#### 現状の問題

毎フレーム直前のフレームとマッチングしているため、ほぼ静止している場合でも不要な計算を行い、微小なノイズがドリフトとして蓄積する。

#### 改良内容

一定以上の移動・回転があった場合のみマッチングを実行し、キーフレームとして管理する。

**設計:**

```
キーフレーム判定:
  delta_translation = ||prev_delta_.translation()||
  delta_rotation = angle(prev_delta_.rotation())

  if delta_translation > keyframe_translation_threshold ||
     delta_rotation > keyframe_rotation_threshold:
    → 現フレームを新しいキーフレームとして登録
    → NDTマッチングを実行

  else:
    → 等速度モデルの予測値をそのまま出力（NDTスキップ）
    → 共分散を予測に応じて増加
```

**追加パラメータ:**

```yaml
use_keyframe: true
keyframe_translation_threshold: 0.5   # [m]
keyframe_rotation_threshold: 0.1      # [rad] (~5.7°)
```

#### 期待効果

- 静止・微動時の不要なドリフト蓄積を防止
- 計算負荷の削減（不要な NDT 実行をスキップ）
- ローカルマップ（改良案1）との相乗効果

---

### 6. 動的物体除去（優先度: B）

#### 現状の問題

歩行者・車両等の動的物体の点群がマッチングに含まれると、静止環境の仮定が崩れてマッチング結果が歪む。

#### 改良内容

フレーム間の対応点残差に基づく動的物体検出・除去を行う。

**方法 A: 残差ベース除去（推奨、外部依存なし）**

```
処理フロー:
  1. NDT マッチングを1回実行
  2. 推定変換で source 点群を変換
  3. 各点と target の最近傍点との距離を計算
  4. 距離が閾値を超える点を動的物体候補として除去
  5. 残りの点で再度 NDT マッチング

パラメータ:
  dynamic_removal_enabled: true
  dynamic_distance_threshold: 1.0   # [m]
```

- 2回マッチングが必要だが、2回目は収束が速い
- 追加の外部依存なし

**方法 B: 占有格子比較（より高精度だが計算コスト大）**

```
前フレームと現フレームの占有格子を比較し、
消失/出現したボクセルを動的領域として検出
```

#### 期待効果

- 交通量の多い道路・歩行者が多い環境でのロバスト性向上
- 特にフレーム間マッチングでの効果が大きい（動的物体の影響を直接受けるため）

---

### 7. 多段解像度 NDT（優先度: B）

#### 現状の問題

NDT 解像度が固定（2.0m）のため、大きな変位に対する収束範囲が限られる。また、細かい解像度のみでは局所解に陥りやすい。

#### 改良内容

粗い解像度から細かい解像度へ段階的にマッチングを行う Coarse-to-Fine 戦略。

**設計:**

```
解像度リスト: [4.0, 2.0, 1.0]  # [m] 粗→細

for resolution in resolutions:
  ndt.setResolution(resolution)
  ndt.setInputTarget(target)
  ndt.setInputSource(source)
  ndt.align(aligned, current_guess)
  current_guess = ndt.getFinalTransformation()

final_result = current_guess
```

**追加パラメータ:**

```yaml
ndt_resolutions: [4.0, 2.0, 1.0]   # 多段解像度リスト [m]
```

#### 期待効果

- 大変位（低フレームレート・急旋回）時の収束能力向上
- 粗い段階で大域的な位置合わせ → 細かい段階で精密な位置合わせ
- 局所解に陥るリスクの低減

---

### 8. Hessian ベース共分散推定（優先度: B）

#### 現状の問題

現在の共分散推定（`estimateCovariance()`）は NDT スコアに基づくヒューリスティックなスケーリングであり、方向別の不確実性を正しく表現できていない。全軸に同じスケールが適用され、非対角成分は常に 0。

#### 改良内容

NDT のコスト関数の Hessian 行列の逆行列から共分散を推定する。

**設計:**

```
理論:
  NDT のコスト関数 f(x) の Hessian H(x*) を最適解 x* で評価
  共分散 ≈ H(x*)^{-1}  (Fisher 情報行列の逆行列に相当)

実装:
  1. NDT 収束後に Hessian 行列 (6×6) を取得
     - ndt_omp_ros2 が getHessian() を提供しているか確認
     - 未提供の場合は数値微分で近似:
       H[i][j] ≈ (f(x+ei+ej) - f(x+ei) - f(x+ej) + f(x)) / (eps^2)
  2. H の逆行列を計算: Σ = H^{-1}
  3. 正定値性を保証（固有値が負の場合は小さな正の値でクランプ）

出力:
  完全な 6×6 共分散行列（非対角成分を含む）
```

#### 期待効果

- 方向別の不確実性を正確に表現（例: 廊下では進行方向の不確実性が高い）
- 非対角共分散により軸間の相関を捕捉
- 下流のセンサ融合フィルタの性能向上

---

### 9. ループクロージャ（優先度: C）

#### 現状の問題

累積ドリフトを補正する手段がない。長時間走行では開始地点に戻っても位置が一致しない。

#### 改良内容

過去に訪れた場所を検出し、蓄積されたドリフトを補正する。

**設計概要:**

```
ループ検出:
  1. キーフレームごとに特徴記述子（Scan Context 等）を保存
  2. 現在のスキャンと過去のキーフレームの類似度を比較
  3. 類似度が閾値を超えた場合、NDT で詳細な位置合わせを実施
  4. 位置合わせが成功した場合、ループ拘束を追加

ドリフト補正:
  - ポーズグラフ最適化（g2o / GTSAM）で全軌跡を修正
  - オドメトリ拘束 + ループ拘束の同時最適化
```

**必要な追加依存:**

```
Scan Context: ループ検出用特徴量
g2o または GTSAM: グラフ最適化ライブラリ
```

#### 期待効果

- 長距離走行でのグローバルな整合性確保
- ドリフトの累積を根本的に解消

#### 注意点

- 実装規模が大きく、本パッケージの範囲を超える可能性がある
- 独立した SLAM パッケージとして切り出すことを推奨
- リアルタイム性の確保にはポーズグラフの効率的な管理が必要

---

### 10. スライディングウィンドウ最適化（優先度: C）

#### 現状の問題

現在は各フレームの推定が独立しており、過去のフレームの結果を遡って修正しない。連続するフレーム間の一貫性は保証されていない。

#### 改良内容

直近 M フレームの姿勢を同時に最適化する。

**設計概要:**

```
因子グラフ:
  - ノード: 各フレームの姿勢 (SE3)
  - 因子:
    - NDT マッチングによるフレーム間拘束
    - IMU プリインテグレーション（IMU融合時）
    - ループ拘束（ループクロージャ実装時）

最適化:
  - ウィンドウサイズ: 直近 M フレーム（例: 20）
  - 最古のフレームを固定（マージナライゼーション）
  - 新フレーム追加時に増分最適化

ライブラリ: GTSAM (iSAM2)
```

#### 期待効果

- フレーム間の一貫性向上
- IMU・ループ拘束との統合が自然
- 短期的なドリフトの抑制

---

### 11. 適応的パラメータ調整（優先度: C）

#### 現状の問題

NDT パラメータ（解像度・反復回数等）が固定であり、環境の変化に対応できない。高速走行時と低速走行時、特徴量が豊富な環境と乏しい環境では最適なパラメータが異なる。

#### 改良内容

NDT の収束品質に基づいてパラメータを動的に調整する。

**設計:**

```
適応ルール:
  - 収束スコアが低い場合:
    → 解像度を粗くする（収束範囲拡大）
    → 反復回数を増やす
    → ダウンサンプリングを控えめにする（点数増加）

  - 収束が速い（反復回数少ない）場合:
    → 解像度を細かくする（精度向上）
    → 反復回数を減らす（処理速度向上）
    → ダウンサンプリングを強める（計算負荷軽減）

  - フレーム間変位が大きい場合:
    → 解像度を粗くする
    → 等速度モデルの信頼度を下げる
```

#### 期待効果

- 多様な走行条件への自律的な適応
- 計算資源の効率的な利用

---

## 実装ロードマップ

### Phase 1: 基盤改良（即効性が高い改善）

```
目標: 既存アーキテクチャの枠内で精度を向上させる
期間目安: 小規模

実施項目:
  ✦ 3. 地面点除去（RANSAC）
  ✦ 5. キーフレーム戦略
  ✦ 7. 多段解像度 NDT

依存関係: なし（独立して実装可能）
```

### Phase 2: コア機能強化

```
目標: マッチング品質と信頼性を根本的に向上させる

実施項目:
  ✦ 1. ローカルマップマッチング
  ✦ 4. 幾何的退化検出
  ✦ 8. Hessian ベース共分散推定

依存関係: 1 → 5（キーフレーム戦略と相性が良い）
```

### Phase 3: センサ融合

```
目標: 外部センサ情報を統合して予測精度を強化する

実施項目:
  ✦ 2. IMU 融合による動作予測
  ✦ 6. 動的物体除去

依存関係: 2 は 4（退化検出）と組み合わせると効果が高い
```

### Phase 4: グローバル最適化（SLAM 化）

```
目標: 累積ドリフトの根本的な解消
注記: 本パッケージの範囲を超える可能性あり。別パッケージとしての実装を推奨。

実施項目:
  ✦ 9. ループクロージャ
  ✦ 10. スライディングウィンドウ最適化
  ✦ 11. 適応的パラメータ調整

依存関係: 9 → 10（グラフ最適化基盤を共有）
```

---

## Phase 1 の詳細設計

以下に、即座に実装可能な Phase 1 の各改良案についてコード変更の概要を示す。

### 3. 地面点除去の実装指針

**変更ファイル:**
- `include/lidar_odometry/lidar_odometry_node.hpp` — メソッド・パラメータ追加
- `src/lidar_odometry_node.cpp` — `filterPointCloud()` にステップ追加
- `config/params.yaml` — パラメータ追加

**追加メソッド:**

```cpp
PointCloud::Ptr removeGroundPoints(const PointCloud::Ptr & cloud) const;
```

**処理内容（RANSAC 方式）:**

```cpp
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

PointCloud::Ptr LidarOdometryNode::removeGroundPoints(const PointCloud::Ptr & cloud) const
{
  pcl::SACSegmentation<PointT> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ground_ransac_threshold_);  // e.g., 0.15m
  seg.setMaxIterations(ground_ransac_iterations_);      // e.g., 100
  seg.setInputCloud(cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.segment(*inliers, *coefficients);

  // 法線ベクトルが概ね鉛直方向(z軸)に近い場合のみ地面とみなす
  if (inliers->indices.empty()) return cloud;

  Eigen::Vector3f normal(coefficients->values[0],
                          coefficients->values[1],
                          coefficients->values[2]);
  float cos_angle = std::abs(normal.dot(Eigen::Vector3f::UnitZ()));
  if (cos_angle < 0.8f) return cloud;  // 概ね水平でなければ除去しない

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // 地面以外を抽出

  auto non_ground = std::make_shared<PointCloud>();
  extract.filter(*non_ground);
  return non_ground;
}
```

**パイプライン変更:**

```cpp
// filterPointCloud() 内
auto range_filtered = ... ;          // 既存の距離フィルタ
auto ground_removed = removeGroundPoints(range_filtered);  // 新規追加
auto downsampled = voxelDownsample(ground_removed);        // 既存のダウンサンプリング
```

### 5. キーフレーム戦略の実装指針

**追加メンバ変数:**

```cpp
PointCloud::Ptr keyframe_cloud_;
Eigen::Matrix4f keyframe_pose_;
double keyframe_translation_threshold_;
double keyframe_rotation_threshold_;
```

**判定ロジック（`pointCloudCallback()` 内）:**

```cpp
// NDT 結果取得後
Eigen::Matrix4f delta_from_keyframe = keyframe_pose_.inverse() * cumulative_pose_;
double trans_diff = delta_from_keyframe.block<3,1>(0,3).norm();
Eigen::Matrix3f rot_diff = delta_from_keyframe.block<3,3>(0,0);
double angle_diff = Eigen::AngleAxisf(rot_diff).angle();

if (trans_diff > keyframe_translation_threshold_ ||
    angle_diff > keyframe_rotation_threshold_) {
  // 新キーフレームとして登録
  keyframe_cloud_ = filtered_cloud;
  keyframe_pose_ = cumulative_pose_;
}

// NDT のターゲットはキーフレーム
ndt_.setInputTarget(keyframe_cloud_);
```

### 7. 多段解像度 NDT の実装指針

**変更箇所:** `pointCloudCallback()` のマッチング部分

```cpp
std::vector<double> resolutions = {4.0, 2.0, 1.0};
Eigen::Matrix4f current_guess = prev_delta_;

for (double res : resolutions) {
  ndt_.setResolution(static_cast<float>(res));
  ndt_.setInputTarget(prev_cloud_);
  ndt_.setInputSource(filtered_cloud);

  auto aligned = std::make_shared<PointCloud>();
  ndt_.align(*aligned, current_guess);

  if (ndt_.hasConverged()) {
    current_guess = ndt_.getFinalTransformation();
  }
}

Eigen::Matrix4f delta = current_guess;
```

**注意:** 解像度変更のたびに `setInputTarget()` の再呼び出しが必要（NDT 内部のボクセルグリッドが再構築される）。ターゲット側のボクセル構築コストを考慮し、解像度段数は 2〜3 段に抑える。

---

## パラメータチューニングガイドライン

### 現在のパラメータに対する推奨変更

| パラメータ | 現在値 | 推奨値 | 理由 |
|-----------|--------|--------|------|
| `ndt_resolution` | 2.0 | 1.0〜2.0 | 精度重視なら 1.0、速度重視なら 2.0 |
| `ndt_max_iterations` | 30 | 50 | 収束マージンの確保 |
| `voxel_leaf_size` | 0.25 | 0.2 | やや密にして NDT の品質向上 |
| `max_scan_range` | 100.0 | 80.0 | 遠方点はノイズが多く精度低下要因 |
| `ndt_search_method` | DIRECT7 | DIRECT7 | 速度と精度のバランスが最良（現状維持） |

### 環境別の推奨設定

**屋外・広域環境:**

```yaml
ndt_resolution: 2.0
voxel_leaf_size: 0.3
max_scan_range: 100.0
```

**屋内・狭い環境:**

```yaml
ndt_resolution: 0.5
voxel_leaf_size: 0.1
max_scan_range: 30.0
```

**高速走行（> 30 km/h）:**

```yaml
ndt_resolution: 2.0
ndt_max_iterations: 50
voxel_leaf_size: 0.3    # 処理速度確保
```

---

## 評価方法

### 精度評価指標

| 指標 | 説明 | 目標値 |
|------|------|--------|
| APE (Absolute Pose Error) | 真値軌跡との絶対的な位置誤差 | RMSE < 1% of travel distance |
| RPE (Relative Pose Error) | フレーム間の相対姿勢誤差 | < 0.5% (translation), < 0.1 deg/m (rotation) |
| ドリフト率 | 走行距離に対する累積誤差の比率 | < 1% |
| 処理時間 | 1フレームあたりの処理時間 | < 100ms (10Hz LiDAR) |

### 評価ツール

```
evo: Python ベースの軌跡評価ツール
  pip install evo
  evo_ape tum ground_truth.txt estimated.txt --plot
  evo_rpe tum ground_truth.txt estimated.txt --delta 1 --delta_unit m
```

### 評価データセット

| データセット | 特徴 | 用途 |
|-------------|------|------|
| KITTI Odometry | 屋外・都市部走行 | 標準ベンチマーク |
| MulRan | 都市部・多様な環境 | ロバスト性評価 |
| Newer College | 屋内外混在 | 退化環境の評価 |

---

## まとめ

現在の実装はフレーム間 NDT マッチングによるシンプルで安定した設計であるが、推定精度向上のために以下の改良が効果的である:

1. **最も効果が大きい改良**: ローカルマップマッチング（改良案1）と IMU 融合（改良案2）の組み合わせにより、ドリフト率と収束安定性の両方を大幅に改善できる
2. **コストパフォーマンスが高い改良**: 地面点除去（改良案3）とキーフレーム戦略（改良案5）は実装が比較的容易で即効性がある
3. **根本的な解決**: ループクロージャ（改良案9）は累積ドリフトを根本的に解消するが、SLAM システムへの発展を意味するため、パッケージ設計の再検討が必要

Phase 1 から段階的に実装し、各段階で定量的な評価を行いながら進めることを推奨する。

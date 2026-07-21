# AM_M5_SEN0628_x3_0pm50_CorridorAvoid

M5StickC Plus + Arduino Mega + SEN0628(8x8 ToF)x3 + BMA400 構成の、通路追従・障害物回避スキッドステアロボット。

## 概要

- 前方/左右(0°, +50°, -50°)に取り付けた3基のSEN0628(8x8マトリクスToF)で周囲を計測し、
  M5StickC Plus側で回避・通路中央維持・コーナー旋回を判断する。
- 駆動はArduino Megaが担当し、M5からUART経由で受け取った左右デューティ指令を
  L298N経由でモーターに出力する(500HzソフトウェアPWM、キック/ランプ制御付き)。
- M5側はWi-Fi SoftAPで点群・姿勢・加速度をUDP配信し、PC側の`tools/point_cloud_viewer.py`で
  リアルタイムに可視化(点群マップ・推定軌跡・ステータス表示)できる。

## ハードウェア構成

### 主要部品

| 部品 | 役割 |
|---|---|
| M5StickC Plus | メイン制御(判断・IMU・センサー統合・Wi-Fi配信) |
| Arduino Mega | モーター駆動専用(L298N制御、500Hz PWM) |
| SEN0628 (DFRobot 8x8 ToF) x3 | 前方/左右の距離計測(TCA9548A経由) |
| TCA9548A | I2Cマルチプレクサ(センサー3基+BMA400をチャンネル分離) |
| BMA400 (Seeed Grove, ±16g) | 補助加速度計(TCA CH0)。M5内蔵IMUとブレンドして点群ビューアーの位置推定に使用 |
| L298N | モータードライバ |

`robot-parts-all.3mf` / `robot-parts-all.stl` に3Dプリント用の車体パーツ一式が入っている。

### 配線

**TCA9548A チャンネル割当**(実機確認済み):

| チャンネル | 接続先 |
|---|---|
| CH0 | BMA400 (Grove, I2C 0x15) - コネクタは車体後ろ向き |
| CH1 | SEN0628 front (0°, 中央) |
| CH2 | SEN0628 left (+50°, ひだり) |
| CH3 | SEN0628 right (-50°, 右) |

**M5StickC Plus側I2Cバス**(2系統を`Wire.begin()`の切り替えで使い分け):

| バス | ピン | 用途 |
|---|---|---|
| 内蔵(body) | SDA=21, SCL=22 | M5内蔵IMU(MPU6886) |
| 外部(external) | SDA=32, SCL=33 | TCA9548A(SEN0628 x3 + BMA400) |

**M5 ⇔ Mega UART**(115200bps):

```
M5 GPIO26 TX  -> AM D19 RX1
AM D18 TX1    -> level shifter/divider -> M5 GPIO36 RX
GND common
```

**Mega ⇔ L298N**:

```
L298N IN1 -> AM D22 / PA0
L298N IN2 -> AM D23 / PA1
L298N IN3 -> AM D24 / PA2
L298N IN4 -> AM D25 / PA3
```

## ソフトウェア構成

| ファイル | 対象 | 内容 |
|---|---|---|
| `M5_AM_SEN0628_3xSEN0628_v15/M5_AM_SEN0628_3xSEN0628_v15.ino` | M5StickC Plus | センサー統合・判断ロジック・モーター指令送信・UDP点群配信(v19時点) |
| `AM_HighTorqueAntiStall_500HzPWM_NoJumper_v4_for_x3/AM_HighTorqueAntiStall_500HzPWM_NoJumper_v4_for_x3.ino` | Arduino Mega | モーター駆動専用(500Hz PWM、キック/ランプ、UARTでM5から受信)(v2時点) |
| `tools/point_cloud_viewer.py` | PC(Python) | UDP点群・IMU/加速度・推定軌跡・キャリブレーション進行状況のリアルタイム可視化ツール |
| `PARAMETERS.md` | - | 全チューニング定数の意味・調整効果の詳細解説 |

ファイル名の`_v15`/`_v4`はスケッチ内の初出バージョンを指すが、実際の版数は各ファイル冒頭の
changelogコメント(現時点でM5側はv19、Mega側はv2)を参照すること。

### 通信プロトコル(UDP点群配信)

- SoftAP: SSID `M5_POINT_CLOUD` / パスワード `m5pointcloud` / ポート `4210`
- フレームは`PCLD`マジック+CRC16-CCITT付きバイナリ。`version`フィールドで後方互換:
  - v1: 距離・姿勢の基本情報のみ
  - v2: + M5内蔵IMU加速度、センサー個別ヨー/タイムスタンプ
  - v3: + BMA400(TCA CH0)加速度。M5内蔵IMUと単純平均でブレンドして位置推定に利用
  - v4: + キャリブレーション(BtnB)の進行状況(`trimMilli`/`calRound`/`calPass`/`calDriftCdeg`)。
    `driveState`に`DS_CALIBRATING(3)`を追加し、キャリブ中はPC側ビューアーの
    「CALIBRATION」パネルにラウンド/パス/ドリフト/トリムをリアルタイム表示する
- 詳細なフィールド定義は`tools/point_cloud_viewer.py`の`PointCloudFrame`/`decode_frame()`、
  および送信側は`.ino`の`PointCloudFrame`構造体/`streamPointCloud()`を参照。

## セットアップ

### 必要なArduinoライブラリ

- M5StickCPlus
- DFRobot_MatrixLidar (SEN0628用)
- Seeed_BMA400 (`https://github.com/Seeed-Studio/Grove_3Axis_Digital_Accelerometer_BMA400`)
- 標準搭載: Wire, Preferences, WiFi, WiFiUdp

### 書き込み手順

1. `M5_AM_SEN0628_3xSEN0628_v15/M5_AM_SEN0628_3xSEN0628_v15.ino` をM5StickC Plusへ書き込み。
2. `AM_HighTorqueAntiStall_500HzPWM_NoJumper_v4_for_x3/AM_HighTorqueAntiStall_500HzPWM_NoJumper_v4_for_x3.ino` をArduino Megaへ書き込み。
3. 起動時、M5のLCDにセンサー初期化状況(SEN0628 3基、BMA400)が順に表示される。
   `not found` / `NG` 表示が出た場合は配線・TCAチャンネルを確認する。

### PC側ツールの実行

```
python3 tools/point_cloud_viewer.py
python3 tools/point_cloud_viewer.py --self-test   # ユニットテスト(プロトコルデコード・デッドレコニング)
```

PCをM5の SoftAP(`M5_POINT_CLOUD`)に接続してから起動する。追加の依存パッケージは無し(標準ライブラリのみ、tkinter必須)。

## 操作方法

- **BtnA**: IMUゼロ点校正 → 走行開始
- **BtnB**(停止中のみ): 直進キャリブレーション(v18)。短いパス(約3.3秒)を3回走って
  中央値ドリフトから左右モーター差(motorTrim)を算出しNVSへ保存。`CAL_OK_DEG`に収まるまで
  最大5ラウンド自動継続し、ラウンド間は「Reposition robot / B:continue A:stop」表示で
  ロボットを開始位置へ戻すよう案内する(後方無センサのため自動バックはしない)
- 走行中はIMU方位維持+距離センサーによる回避ピボット/バック/通路中央維持を自動判断
- LCDに状態(FORWARD/PIVOT/BACK)、理由、距離、トリム値などをリアルタイム表示

## 既知の課題・未検証事項

- **BMA400の軸対応**: コネクタが車体後ろ向きという情報とSeeed製品ページの座標系図から
  X=車体右/Y=車体前方と結論づけたが、実機での重力方向テスト(静止時Z≒+1000mg等)による
  裏取りはまだ行っていない。
- **M5側ファームウェア(BMA400関連コード)は実機コンパイル・書き込み未検証**
  (開発環境にArduino向けビルドツールチェーンが無いため、構文レビューのみ)。
- **点群ビューアーのデッドレコニング**: ピボット(超信地旋回)中は並進をゼロ扱いする近似のため、
  実際のわずかな並進(タイヤの遊び等)がピボット回数分蓄積し、地図がズレる可能性がある。
  スキャンマッチング(ICP)等での絶対補正は未実装。
- **v16で変更したキャリブレーションゲイン**(`CAL_TRIM_GAIN`, `AUTO_TRIM_RATE`)は
  静的コードレビューに基づく変更で、実走行での再調整前提(詳細はPARAMETERS.md参照)。
- **v18の複数パス平均・自動ラウンド継続キャリブレーションは実機での収束確認がまだ**。
  1ラウンドの合計走行距離は旧来の単発10秒とほぼ同じになるよう設計したが、
  実際のコース長で問題なく収まるかは要確認。
- **v19のキャリブレーション情報UDP配信・点群ビューアーのCALIBRATIONパネルも実機未検証**
  (Pythonデコード側はセルフテスト済みだが、M5側からの実送信・表示は未確認)。
- センサー高さ・車体の静止傾き(前方1〜2度上向き)による遠方誤判定の可能性は理論上残っている
  (実測での数値補正は未実装)。

## 詳細パラメータ

各定数の意味・調整時の挙動変化・相互依存関係は [PARAMETERS.md](PARAMETERS.md) を参照。

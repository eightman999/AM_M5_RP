/*
  M5StickC Plus + Arduino Mega + ToF/SEN0628
  SEN0628 corridor follow v8: PIVOT-TURN architecture
  v9: + motor L/R calibration (trim)
  v10: sensor suite unified to 3x SEN0628 8x8, angles 0/+50/-50deg

  現行運用変更:
    - SEN0628の距離閾値は、テスト時の値へ一律40mmを加えた固定値を採用。
    - 左右モータ差はIMUヨー補正と保存済みmotorTrimで補正。
      左タイヤが強い実測に合わせ、左モーターへ固定96%係数を適用。
    - 側方距離はセンサー位置からタイヤ外端までの50mmを差し引き、
      タイヤ外端基準のクリアランスとして回避判定する。
    - 通常はIMU方位を維持して前進し、障害物検知時だけLCD赤判定による
      ピボット回避、前方極近時だけバックを行う。
    - BtnBは停止中の最大速度10秒直進テスト専用。IMUヨードリフトから
      motorTrimを算出し、NVSへ保存する。

  v15: 接触保険(IMU衝撃BACK)を削除、LCD表示抜け修正、キャリブ即中断修正

  v15 changes:
    - 接触保険(IMU加速度によるimpactValid BACKトリガ)を完全削除。
      BACKは前方センサ実測の極近(tofRealClose)のみで発動する。
    - 加速度由来の診断値(lastImpactG/accelWarn/連続カウント)は
      TEST_MODE時のみ計算・表示するように変更(実走行には一切影響しない)。
    - LCD表示に「BACK中なのにRUN表示のまま」という抜けがあった原因を特定:
      FORCE_BACK_MS(240ms) < LCD_UPDATE_PERIOD_MS(250ms) のため、
      BACK状態が丸ごと1回も描画されずに終わることがあった。
      -> LCD_UPDATE_PERIOD_MSを80msに短縮し、さらに状態(dstate)が
      変化した瞬間は更新間隔を無視して即座に再描画するように変更。
    - キャリブ走行(個体差テスト)がすぐ中断される問題を修正:
      CAL_ABORT_FRONT_MMがTEST_MODEの極小閾値(60mm)に連動していたため、
      直進テスト自体が数cmの近接ですぐアボートしていた。
      TEST_MODEから切り離し、キャリブ専用の固定値(150mm)に変更。

  v14: 距離センサが何も検知していないのにBACKする問題を修正

  v14 changes:
    - BACKのトリガは「前方センサ実測の極近」か「IMU衝撃検知(impactValid)」の
      いずれかだが、後者は距離センサと完全に無関係(加速度のみ)。
      IMPACT_G_THRESH=0.80という単発サンプルの閾値超えだけで発動していたため、
      走行中の振動・モータのキック・机上の小さな段差等の単発ノイズでも
      「センサは何も見ていないのにBACK」が起こり得た。
      -> 加速度が閾値を連続IMPACT_CONSECUTIVE_REQUIRED回以上続けて超えた場合のみ
      impactLikeをtrueにするよう変更(単発スパイクの除外)。
      IMPACT_G_THRESHも0.80->1.00に引き上げ、二重に安全側へ。

  v13: RUN表示中に後進して見えるバグを修正

  v13 changes:
    - ピボット旋回終了→FORWARD(RUN表示)へ切り替わる瞬間、smoothCmdLeft/Right
      がピボット時の値(片輪が負=逆転)を引きずったまま前進用のゆるいslew
      (CMD_SLEW_PER_SEND=42/回)に渡っていたため、LCDが既に"RUN"を表示していても
      実際の指令は数フレーム(~100〜160ms)片輪が負のまま=後進方向に見える、
      という状態表示と実指令の不整合があった。
      -> ピボット終了時にsmoothCmdLeft/Rightを0にリセットしてから
      FORWARDのランプを開始するように修正。

  v12: TEST MODE (卓上ベンチテスト用の閾値切替)

  v11: threshold retune for small-scale course + faster/visible init

  v11 changes:
    - 実コースの壁間隔は片側10〜50cm程度しかないと判明。
      v10までの閾値(BODY_*_MARGIN他)は倍以上大きい前提だったため、
      常に「危険」判定になり曲がりすぎ/止まりすぎていた。
      -> FRONT/SIDE/SEN/CORRIDOR系の閾値を全て実測スケールに合わせて縮小。
      -> BODY_*_MARGIN/EXTRA_SAFETY方式(加算式)をやめ、直接値で明示。
    - チャンネル配置を実機確認: CH0=BM(未使用/予約), CH1=front(中央),
      CH2=left(ひだり), CH3=right(右)。既存の SEN_CH={1,2,3} 割当は
      この実機配置と一致していたため変更なし(確認のみ)。
    - 起動シーケンスを高速化: 起動前待機 500->150ms、
      センサ初期化後の待機 60->20ms、各センサの初期化進捗をLCDに表示
      (今何番目のセンサを初期化中か分かるようにし、フリーズと区別できるように)。

  v10 changes (sensor hardware change):
    - VL53L0X(単点ToF x3)を完全撤去。前方/左右とも SEN0628 8x8 に統一。
    - マウント角度: idx0=front(0deg), idx1=left(+50deg), idx2=right(-50deg)
      (角度は body forward axis からの水平角。上向きチルトは廃止)
    - 各センサの中心4セル平均距離(centerMm)を、マウント角度で
      前方成分(*cos)・側方成分(*sin)に幾何分解して使用する。
        idx0(0deg):  forward = center,           side = 0
        idx1(+50):   forward = center*cos50,     side  = center*sin50 (左壁距離)
        idx2(-50):   forward = center*cos50,     side  = center*sin50 (右壁距離)
      前方の"real close"判定は idx0 の最小値(minMm)ベースで従来より保守的に判定。
    - 通路モードの leftClear/rightClear は idx1/idx2 の側方成分から算出。
      (旧: 単点ToFとSEN中央の融合 -> 新: 単一8x8の幾何投影)
    - POLL_SEQは前方(idx0)を安全上重要なので多めにポーリング。

  v9 changes:
    - モータ個体差トリム(motorTrim)を追加。M5側で左右指令を
      (1-trim)/(1+trim) 倍してから送信。trim>0 = 右を強め左を弱める。
    - 直進キャリブモードは現行運用変更で最大速度10秒へ更新。
    - 走行中(FWD直進安定時のみ)恒常的なヘディング補正量からトリムを微学習。
    - トリムはNVSに保存し、再起動後も保持。

  v8 changes (turns didn't work / BACK spam / stall):
    - 差動アーク(150:245等)ではこの車体は曲がらない(スキッドステア+高負荷)。
      -> 旋回は全て「正転逆転の超信地旋回」に統一。IMUの旋回角で終了判定。
    - 前進は常に高デューティ固定(215)。低デューティ指令を廃止しストール排除。
    - BACKのトリガから SEN0628 を完全撤去。前方ToF実測の極近 or 衝突検知のみ。
      (BACK頻発の残存原因は上向き8x8が通路壁を拾い続けることだったため)
    - 通路中央維持は「目標ヨーの微調整」+「片側極近時の小角度ピボット」で実現。
    - 状態機械: FORWARD / PIVOT / BACK の3状態。

  Goal:
    - smoother motor behavior
    - more sensitive obstacle avoidance
    - account for robot body size / clearance
    - no distance estimation
    - when a wall/obstacle appears in front, turn left early
    - if front is very close, stop
    - BtnA: IMU zero -> start
    - BtnB: 最大速度10秒の直進補正テスト(停止中のみ)

  Sensor layout (v10/v11, 実機確認済み):
    TCA CH0: (BM, 未使用/予約 - このコードでは触らない)
    TCA CH1: SEN0628 front  (0 deg, 中央)
    TCA CH2: SEN0628 left   (+50 deg, ひだり)
    TCA CH3: SEN0628 right  (-50 deg, 右)

  UART:
    M5 GPIO26 TX -> AM D19 RX1
    AM D18 TX1 -> level shifter/divider -> M5 GPIO36 RX
    GND common
*/

#include <M5StickCPlus.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "DFRobot_MatrixLidar.h"

// ===============================
// Pin / bus
// ===============================
#define EXT_I2C_SDA 32
#define EXT_I2C_SCL 33

#define BODY_I2C_SDA 21
#define BODY_I2C_SCL 22
// If builtin IMU is dead, try:
// #define BODY_I2C_SDA 0
// #define BODY_I2C_SCL 26

#define AM_UART_RX 36
#define AM_UART_TX 26

HardwareSerial AMSerial(2);
const uint32_t AM_UART_BAUD = 115200;

// ===============================
// TCA / sensors
// ===============================
const uint8_t TCA_ADDR = 0x70;

// v10: 3x SEN0628 8x8, idx0=front(0deg) idx1=left(+50deg) idx2=right(-50deg)
const uint8_t SEN_COUNT = 3;
const uint8_t SEN_CH[SEN_COUNT] = {1, 2, 3};
const float SEN_ANGLE_DEG[SEN_COUNT] = {0.0f, 50.0f, -50.0f};
const uint8_t SEN0628_ADDR = 0x33;

DFRobot_MatrixLidar_I2C senFront(SEN0628_ADDR);
DFRobot_MatrixLidar_I2C senLeft(SEN0628_ADDR);
DFRobot_MatrixLidar_I2C senRight(SEN0628_ADDR);
DFRobot_MatrixLidar_I2C *senDev[SEN_COUNT] = {&senFront, &senLeft, &senRight};
bool senOK[SEN_COUNT] = {false, false, false};

// マウント角度から前方/側方への幾何投影係数(setup()で算出)。
float senCosAbs[SEN_COUNT] = {1.0f, 1.0f, 1.0f};
float senSinAbs[SEN_COUNT] = {0.0f, 0.0f, 0.0f};

struct SenCache {
  bool ok = false;
  int minMm = -1;
  int centerMm = -1;
  int validCount = 0;
  int nearCountDanger = 0;
  int nearCountWarn = 0;
  float captureYawDeg = 0.0f;
  unsigned long updatedMs = 0;
  uint16_t raw[64];
};

SenCache cachedSen[SEN_COUNT];

// v10: 前方(idx0)を安全上重要なので多めにポーリング。左右(idx1,2)は交互。
const uint8_t POLL_SEQ[] = {0, 1, 0, 2, 0, 1, 0, 2};
const uint8_t POLL_SEQ_LEN = sizeof(POLL_SEQ) / sizeof(POLL_SEQ[0]);
uint8_t pollSeqIndex = 0;

// ===============================
// AM telemetry / command
// ===============================
struct AMTelemetry {
  bool online = false;
  unsigned long lastRxMs = 0;
  unsigned long amMillis = 0;
  int actualLeft = 0;
  int actualRight = 0;
};

AMTelemetry am;

char amLineBuf[96];
uint8_t amLineLen = 0;

int targetCmdLeft = 0;
int targetCmdRight = 0;
float smoothCmdLeft = 0;
float smoothCmdRight = 0;

const unsigned long AM_COMMAND_PERIOD_MS = 16;
const unsigned long AM_TIMEOUT_MS = 300;
unsigned long lastAMCommandMs = 0;

// 加速度診断表示は通常運用では無効。
const bool TEST_MODE = false;

// ===============================
// Robot footprint / safety bubble
// ===============================
// この値は「車体が思ったよりデカい問題」を吸収するための当たり判定。
// 大きくすると早めに避ける。小さくすると攻める。
// v11: 実コースの壁間隔が片側10〜50cm(=100〜500mm)しかないと判明したため、
// v10までの値(220/170/120)は最大でも「常に危険」判定になり過剰旋回・過剰停止していた。
// 実測スケールに合わせて縮小。
const int BODY_FRONT_MARGIN_MM = 40;    // センサより前にある/旋回時に当たりそうな前方余裕
const int BODY_SIDE_MARGIN_MM  = 25;    // 半幅+ケーブル/タイヤ/旋回時の腹
const int EXTRA_SAFETY_MM      = 20;    // 不確実性ぶん
const int SENSOR_TO_TIRE_OUTER_MM = 50; // 左右センサー位置からタイヤ外端までの張り出し

// ===============================
// Drive tuning (v8: pivot-turn architecture)
// ===============================
// 前進は常に高デューティ。低デューティ差動は使わない(曲がらない+ストールするため)。
const int FWD_CMD = 215;
const int FWD_SLOW_CMD = 195;      // 前方が近い時の減速。これでも高duty維持。
const int MIN_MOVING_CMD = 150;    // AM側 MIN_EFFECTIVE_CMD(148) の直上
const int MAX_MOVING_CMD = 252;

// 超信地旋回(正転逆転)。左右とも同じ大きさで逆向きに回す。
const int PIVOT_CMD = 205;

// 旋回の終了判定(IMU旋回角ベース)
const float PIVOT_MIN_DEG        = 20.0f;   // 前方回避: 小刻みに左右交互でかわす最低角
const float PIVOT_MAX_DEG        = 45.0f;   // Uターン化を防ぐ絶対上限
const float ALIGN_PIVOT_DEG      = 18.0f;   // 片側極近時の姿勢直し小旋回
const unsigned long PIVOT_TIMEOUT_MS       = 1300;
const unsigned long ALIGN_PIVOT_TIMEOUT_MS = 650;
const unsigned long PIVOT_COOLDOWN_MS      = 300;  // 前方回避の再判定間隔
const unsigned long SIDE_ALIGN_DELAY_MS    = 1000; // 回避直後の逆向き姿勢直しを抑止
const float COURSE_RETURN_DONE_DEG = 3.0f;         // 旋回前方位へ戻ったとみなす誤差
const unsigned long COURSE_RETURN_TIMEOUT_MS = 2200;

// テスト時に確認した閾値へ安全余裕40mmを加え、正式な運用値とする。
const int FRONT_PIVOT_MM      = 90;   // テスト値50 + 40mm
const int FRONT_CLEAR_EXIT_MM = 110;  // テスト値70 + 40mm
const int FRONT_SLOW_MM       = 120;  // テスト値80 + 40mm

// 片側がこの距離未満(SEN側方投影クリアランス)なら反対へ小旋回
const int SIDE_PIVOT_MM = 65;         // テスト値25 + 40mm

// ===============================
// Motor L/R calibration (v9)
// ===============================
// motorTrim > 0: 右モータを増幅 / 左を減衰 (直進で右へ流れる=右が弱い個体の補正)
// motorTrim < 0: その逆
// 送信直前に L*0.96*(1-trim), R*(1+trim) を適用する。
// 注意: AM側 MIN_EFFECTIVE_CMD(148) の底上げがあるため、
//       指令が~175未満だとトリムの効きは薄れる(v8以降は前進215固定なので実用上問題なし)。
const int   CAL_DRIVE_CMD = MAX_MOVING_CMD;  // 最大速度で左右差を計測
const unsigned long CAL_DRIVE_MS = 10000;    // 10秒間直進
const int   CAL_ABORT_FRONT_MM = 170;  // 補正テスト中の前方安全停止も従来値+20mm
const float CAL_TRIM_GAIN = 0.0035f;         // ヨードリフト1degあたりのトリム修正量
const float TRIM_LIMIT = 0.25f;
const float CAL_OK_DEG = 4.0f;               // この範囲に収まれば合格表示
const float LEFT_MOTOR_SCALE = 0.92f;        // 左が強いため左モーターを固定で4%減速

const bool  AUTO_TRIM_ENABLE = true;         // 走行中の微学習
const float AUTO_TRIM_RATE = 0.00004f;       // 補正量1あたり/周期の学習率
const unsigned long TRIM_SAVE_PERIOD_MS = 20000;

float motorTrim = 0.0f;
float savedTrim = 0.0f;
Preferences drivePrefs;
unsigned long lastTrimSaveMs = 0;

// Very close recovery: back first, then pivot away.
const int BACK_L_CMD = -215;
const int BACK_R_CMD = -215;
const unsigned long FORCE_BACK_MS = 400;   // 後方は無センサなので短めに制限

// v10: 水平マウントになったため上向きチルト補正は廃止。
// センサ取付面の凹み等がある場合の微調整用に残す(通常0でよい)。
const int SEN_MOUNT_OFFSET_MM = 0;

// If SEN valid points are sparse, do not trust it as "safe".
const int SEN_LOW_VALID_CAUTION_SCORE = 18;

// Avoid false reverse immediately after start.
// Strong startup torque and normal acceleration can look like an impact.
const unsigned long BACK_ARM_DELAY_MS = 1200;

// v8: BACKのトリガは「前方ToF実測の極近」か「衝突検知」のみ。
// SEN0628はBACK判定に一切使わない(上向き8x8が通路壁を拾い続けて
// BACKが頻発する問題の最終対策)。
const unsigned long BACK_COOLDOWN_MS = 2000; // バック完了後、再バックまでの最短間隔
const unsigned long KICK_IGNORE_MS = 450;    // 発進キック直後は衝撃検知を無効化

// Corridor / both-side-wall mode.
// v8: 差動では曲がらないため、中央寄せは「目標ヨーの微調整」で行う。
// 左右差が大きいほど目標ヨーを開いた側へゆっくり振る。
// v11: 壁間隔100〜500mmの実測スケールに合わせて範囲・不感帯を縮小。
// テスト時に確認した閾値へ安全余裕40mmを加えた運用値。
const int CORRIDOR_MIN_WALL_MM = 55;   // テスト値15 + 40mm
const int CORRIDOR_MAX_WALL_MM = 160;  // テスト値120 + 40mm
const int CORRIDOR_DETECT_SCORE = 25;
const int CORRIDOR_DIFF_DEADBAND_MM = 50; // テスト値10 + 40mm
const float CORRIDOR_YAW_STEP_DEG = 0.35f;   // 1制御周期あたりの目標ヨー移動量(最大)
const float CORRIDOR_YAW_CLAMP_DEG = 22.0f;  // 現在ヨーからの目標乖離上限

// v11: BODY_*_MARGIN/EXTRA_SAFETYの加算式のままだと調整の見通しが悪いため、
// 実測スケール(壁10〜50cm)向けの絶対値として明示。
// マージン定数は「ここから何mm余裕を持たせたか」の内訳表示用に残す。
const int FRONT_STOP_MM = 60;   // テスト値20 + 40mm: very close -> stop/back
const int FRONT_HARD_MM = 75;   // テスト値35 + 40mm
const int FRONT_WARN_MM = 100;  // テスト値60 + 40mm

const int SIDE_DANGER_MM = 60; // テスト値20 + 40mm
const int SIDE_WARN_MM   = 80; // テスト値40 + 40mm

const int SEN_DANGER_MM  = 60; // テスト値20 + 40mm
const int SEN_WARN_MM    = 80; // テスト値40 + 40mm

const int SEN_DANGER_NEAR_COUNT = 3;   // v7: 1->3 ノイズ1セルで即ブロック判定しない
const int SEN_WARN_NEAR_COUNT = 3;
const int OBSTACLE_RED_SCORE = 70;      // LCD赤表示と実際の回避開始を共通化

const unsigned long SENSOR_STALE_MS = 620;

// IMU correction
const float HEADING_KP = 1.15f;
const int HEADING_CORR_LIMIT = 22;     // v8: 16->22 中央寄せをヨー制御で行うため拡大
const float IMPACT_G_THRESH = 1.00f;   // v14: 0.80->1.00 単発ノイズへの耐性を上げる
const int IMPACT_CONSECUTIVE_REQUIRED = 3;  // v14: この回数連続で閾値超えしないとimpactLikeにしない
const float ACCEL_WARN_G = 0.32f;
const unsigned long AVOID_HOLD_MS = 280;

// Command smoothing on M5 side.
// Smaller = smoother; larger = more responsive.
const float CMD_SLEW_PER_SEND = 42.0f;

// UI/debug
const bool DEBUG_SERIAL_OUTPUT = false;
const unsigned long DEBUG_SERIAL_PERIOD_MS = 500;
const unsigned long LCD_UPDATE_PERIOD_MS = 80;  // v15: 250->80 (BACK等の短い状態を描画漏れさせないため)
const bool POINT_CLOUD_STREAM_ENABLE = true;
const unsigned long POINT_CLOUD_STREAM_PERIOD_MS = 50; // 20Hz: PC側加速度積分用
const char POINT_CLOUD_WIFI_SSID[] = "M5_POINT_CLOUD";
const char POINT_CLOUD_WIFI_PASSWORD[] = "m5pointcloud";
const uint16_t POINT_CLOUD_UDP_PORT = 4210;

unsigned long lastDebugSerialMs = 0;
unsigned long lastLCDUpdateMs = 0;
unsigned long lastPointCloudStreamMs = 0;
WiFiUDP pointCloudUdp;

// ===============================
// State
// ===============================
bool imuZeroed = false;
bool driveEnabled = false;
bool emergencyPaused = false;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

float yawDeg = 0.0f;
float targetYawDeg = 0.0f;
unsigned long lastImuMs = 0;
float imuAccelXG = 0.0f;
float imuAccelYG = 0.0f;
float imuAccelZG = 0.0f;

float lastImpactG = 0.0f;
bool impactLike = false;
int impactConsecutive = 0;  // v14: 単発ノイズ除外用の連続超過カウント
bool accelWarn = false;

unsigned long lastAvoidMs = 0;
bool justExitedAvoid = false;

unsigned long forceBackUntilMs = 0;
unsigned long autoStartMs = 0;
unsigned long lastBackEndMs = 0;        // バック完了時刻(クールダウン用)
unsigned long lastCmdSignChangeMs = 0;  // 直近の発進/反転指令時刻(キック揺れ無視用)

// v8 state machine
enum DriveState : uint8_t { DS_FORWARD = 0, DS_PIVOT, DS_BACK };
DriveState dstate = DS_FORWARD;
DriveState lastDrawnDstate = DS_FORWARD;  // v15: 状態変化時の強制再描画用

float pivotStartYaw = 0.0f;
unsigned long pivotStartMs = 0;
int pivotDir = 1;                  // +1 = left, -1 = right
int nextFrontPivotDir = -1;        // 前方回避は右から開始し、右/左を交互にする
bool pivotNeedsFrontClear = true;  // true=前方回避(前方が開くまで), false=姿勢直し小旋回
float pivotReturnYawDeg = 0.0f;    // 旋回前の直進基準方位
bool courseReturnActive = false;
unsigned long courseReturnUntilMs = 0;
unsigned long lastPivotEndMs = 0;
int backRecoverDir = 1;            // バック後にどちらへピボットするか

int lastSenLeftScore = 0;
int lastSenRightScore = 0;
int lastSenFrontScore = 0;
bool lastForceBack = false;

bool lastCorridorMode = false;
int lastLeftClearanceMm = -1;
int lastRightClearanceMm = -1;
int lastWallDiffMm = 0;



char driveReason[32] = "BOOT";

int lastFrontScore = 0;
int lastLeftScore = 0;
int lastRightScore = 0;
int lastTurnBias = 0;

// ===============================
// Utility
// ===============================
float normDeg180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void setReason(const char *s) {
  strncpy(driveReason, s, sizeof(driveReason) - 1);
  driveReason[sizeof(driveReason) - 1] = '\0';
}

bool fresh(unsigned long t, unsigned long maxAgeMs = SENSOR_STALE_MS) {
  return t > 0 && (millis() - t) <= maxAgeMs;
}

int obstacleScore(int mm, int warnMm, int dangerMm) {
  if (mm < 0) return 0;
  if (mm <= dangerMm) return 100;
  if (mm >= warnMm) return 0;

  long num = (long)(warnMm - mm) * 100L;
  long den = (long)(warnMm - dangerMm);
  if (den <= 0) return 0;

  return constrain((int)(num / den), 0, 100);
}

int max3(int a, int b, int c) {
  return max(a, max(b, c));
}

int slewToward(float cur, int tgt, float step) {
  float d = (float)tgt - cur;
  if (d > step) d = step;
  if (d < -step) d = -step;
  return (int)(cur + d);
}

// ===============================
// I2C switching
// ===============================
void useExternalI2C() {
  Wire.end();
  delayMicroseconds(200);
  Wire.begin(EXT_I2C_SDA, EXT_I2C_SCL);
  Wire.setClock(400000);
  delayMicroseconds(200);
}

void useBodyI2C() {
  Wire.end();
  delayMicroseconds(200);
  Wire.begin(BODY_I2C_SDA, BODY_I2C_SCL);
  Wire.setClock(400000);
  delayMicroseconds(200);
}

bool i2cExists(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool tcaSelect(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

void tcaDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ===============================
// AM UART
// ===============================
void sendAMMotorRaw(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // v7: 停止からの発進・回転方向の反転はAM側でキックが走り、
  // その加速度が「衝突」に見えるため、時刻を記録して衝撃判定から除外する。
  bool signEventL = (left != 0) && (targetCmdLeft == 0 || (targetCmdLeft > 0) != (left > 0));
  bool signEventR = (right != 0) && (targetCmdRight == 0 || (targetCmdRight > 0) != (right > 0));
  if (signEventL || signEventR) {
    lastCmdSignChangeMs = millis();
  }

  // v9: 個体差トリム。送信直前に左右へ逆向きに適用。
  int sendL = (int)lroundf(left  * LEFT_MOTOR_SCALE * (1.0f - motorTrim));
  int sendR = (int)lroundf(right * (1.0f + motorTrim));
  sendL = constrain(sendL, -255, 255);
  sendR = constrain(sendR, -255, 255);

  targetCmdLeft = sendL;
  targetCmdRight = sendR;

  AMSerial.printf("M,%d,%d\n", sendL, sendR);
  lastAMCommandMs = millis();
}

void sendAMStop() {
  targetCmdLeft = 0;
  targetCmdRight = 0;
  smoothCmdLeft = 0;
  smoothCmdRight = 0;
  AMSerial.print("S\n");
  lastAMCommandMs = millis();
}

void parseAMLine(char *line) {
  if (line[0] != 'A' || line[1] != ',') return;

  long vals[7] = {0};
  uint8_t idx = 0;
  char *p = line + 2;

  while (idx < 7 && p != nullptr && *p != '\0') {
    vals[idx++] = atol(p);
    p = strchr(p, ',');
    if (p) p++;
  }

  if (idx >= 7) {
    am.amMillis = vals[0];
    am.actualLeft = vals[5];
    am.actualRight = vals[6];
    am.lastRxMs = millis();
    am.online = true;
  }
}

void readAMTelemetry() {
  while (AMSerial.available()) {
    char c = (char)AMSerial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      amLineBuf[amLineLen] = '\0';
      if (amLineLen > 0) parseAMLine(amLineBuf);
      amLineLen = 0;
    } else {
      if (amLineLen < sizeof(amLineBuf) - 1) {
        amLineBuf[amLineLen++] = c;
      } else {
        amLineLen = 0;
      }
    }
  }

  if (millis() - am.lastRxMs > AM_TIMEOUT_MS) {
    am.online = false;
  }
}

// ===============================
// Sensor init/read
// ===============================
bool validMatrixMm(uint16_t mm) {
  return mm >= 20 && mm <= 3900;
}

void initSEN0628() {
  useExternalI2C();

  for (int i = 0; i < SEN_COUNT; i++) {
    senOK[i] = false;

    // v11: 起動中に何番目のセンサを初期化しているかLCDに表示。
    // (フリーズしているように見える対策。実際の所要時間自体はセンサ側次第)
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 6);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.println("INIT SEN");
    M5.Lcd.printf("%d/%d CH%d\n", i + 1, SEN_COUNT, SEN_CH[i]);

    // v10: マウント角度から前方/側方への投影係数を先に確定させる。
    float rad = radians(fabs(SEN_ANGLE_DEG[i]));
    senCosAbs[i] = cosf(rad);
    senSinAbs[i] = sinf(rad);

    if (!tcaSelect(SEN_CH[i])) {
      tcaDisableAll();
      continue;
    }

    if (!i2cExists(SEN0628_ADDR)) {
      Serial.printf("SEN%d CH%d not found\n", i, SEN_CH[i]);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.println("not found");
      tcaDisableAll();
      continue;
    }

    if (senDev[i]->begin() == 0 && senDev[i]->setRangingMode(eMatrix_8X8) == 0) {
      senOK[i] = true;
      Serial.printf("SEN%d CH%d OK (angle %.0fdeg)\n", i, SEN_CH[i], SEN_ANGLE_DEG[i]);
      M5.Lcd.setTextColor(GREEN, BLACK);
      M5.Lcd.println("OK");
    } else {
      Serial.printf("SEN%d CH%d begin/mode NG\n", i, SEN_CH[i]);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.println("begin NG");
    }

    tcaDisableAll();
    delay(20);  // v11: 60->20 起動短縮(TCA/センサの切替待ちとして最低限は残す)
  }
}

void readOneSEN(uint8_t idx) {
  if (idx >= SEN_COUNT) return;

  SenCache &out = cachedSen[idx];
  out.ok = false;
  out.minMm = -1;
  out.centerMm = -1;
  out.validCount = 0;
  out.nearCountDanger = 0;
  out.nearCountWarn = 0;

  if (!senOK[idx]) {
    out.updatedMs = millis();
    return;
  }

  useExternalI2C();

  if (!tcaSelect(SEN_CH[idx])) {
    out.updatedMs = millis();
    return;
  }

  senDev[idx]->getAllData(out.raw);
  tcaDisableAll();

  int minVal = 99999;
  long centerSum = 0;
  int centerCount = 0;
  const uint8_t centerIdx[4] = {27, 28, 35, 36};

  for (int k = 0; k < 64; k++) {
    uint16_t v = out.raw[k];
    if (!validMatrixMm(v)) continue;

    out.validCount++;

    if ((int)v < minVal) minVal = v;
    if ((int)v < SEN_DANGER_MM) out.nearCountDanger++;
    if ((int)v < SEN_WARN_MM) out.nearCountWarn++;
  }

  for (uint8_t k = 0; k < 4; k++) {
    uint16_t v = out.raw[centerIdx[k]];
    if (validMatrixMm(v)) {
      centerSum += v;
      centerCount++;
    }
  }

  if (out.validCount > 0) {
    out.ok = true;
    out.minMm = minVal;
  }

  if (centerCount > 0) {
    out.centerMm = centerSum / centerCount;
  }

  out.updatedMs = millis();
  out.captureYawDeg = yawDeg;
}

void pollOneSensor() {
  uint8_t item = POLL_SEQ[pollSeqIndex];
  readOneSEN(item);

  pollSeqIndex++;
  if (pollSeqIndex >= POLL_SEQ_LEN) pollSeqIndex = 0;
}

// ===============================
// IMU
// ===============================
void readBuiltinIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  useBodyI2C();
  M5.IMU.getAccelData(&ax, &ay, &az);
  M5.IMU.getGyroData(&gx, &gy, &gz);
}

void zeroIMUAndStart() {
  sendAMStop();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.println("IMU ZERO");
  M5.Lcd.println("Keep still");

  useBodyI2C();
  M5.IMU.Init();
  delay(30);

  const int samples = 80;
  float sax = 0, say = 0, saz = 0;
  float sgx = 0, sgy = 0, sgz = 0;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    M5.IMU.getAccelData(&ax, &ay, &az);
    M5.IMU.getGyroData(&gx, &gy, &gz);

    sax += ax;
    say += ay;
    saz += az;

    sgx += gx;
    sgy += gy;
    sgz += gz;

    delay(8);
  }

  accelOffsetX = sax / samples;
  accelOffsetY = say / samples;
  accelOffsetZ = saz / samples;

  gyroOffsetX = sgx / samples;
  gyroOffsetY = sgy / samples;
  gyroOffsetZ = sgz / samples;

  yawDeg = 0.0f;
  targetYawDeg = 0.0f;
  imuAccelXG = 0.0f;
  imuAccelYG = 0.0f;
  imuAccelZG = 0.0f;
  smoothCmdLeft = 0;
  smoothCmdRight = 0;
  lastImuMs = millis();

  imuZeroed = true;
  driveEnabled = true;
  emergencyPaused = false;
  forceBackUntilMs = 0;
  lastForceBack = false;
  dstate = DS_FORWARD;
  lastPivotEndMs = 0;
  lastBackEndMs = 0;
  autoStartMs = millis();
  lastAvoidMs = millis();
  justExitedAvoid = false;
  pivotDir = 1;
  nextFrontPivotDir = -1;
  pivotReturnYawDeg = 0.0f;
  courseReturnActive = false;
  courseReturnUntilMs = 0;

  setReason("START");

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.setTextSize(4);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.println("START");
  delay(140);
}

void updateIMU(float ax, float ay, float az, float gx, float gy, float gz) {
  unsigned long now = millis();

  if (imuZeroed) {
    ax -= accelOffsetX;
    ay -= accelOffsetY;
    az -= accelOffsetZ;

    gx -= gyroOffsetX;
    gy -= gyroOffsetY;
    gz -= gyroOffsetZ;
  }

  // Wi-Fiテレメトリ用。静止ゼロ補正後の機体座標加速度[g]を保持する。
  imuAccelXG = ax;
  imuAccelYG = ay;
  imuAccelZG = az;

  if (lastImuMs == 0) {
    lastImuMs = now;
  }

  float dt = (now - lastImuMs) / 1000.0f;
  lastImuMs = now;

  if (imuZeroed && dt > 0 && dt < 0.2f) {
    yawDeg += gz * dt;
    yawDeg = normDeg180(yawDeg);
  }

  // v15: 接触保険(IMU加速度によるBACKトリガ)は削除。
  // 加速度診断(ax,ay由来のlastImpactG/impactLike/accelWarn)はTEST_MODE時のみ
  // 計算する(実走行の判断には一切使わない。表示・調整用の参考値として残すのみ)。
  if (TEST_MODE) {
    lastImpactG = sqrt(ax * ax + ay * ay);

    // v7: 発進キック/反転直後の加速度は衝突ではないので無視する。
    bool kickWindow = (now - lastCmdSignChangeMs) < KICK_IGNORE_MS;

    // v14: 単発の振動スパイクを除外するため、閾値超えが
    // 連続IMPACT_CONSECUTIVE_REQUIRED回続いた場合のみimpactLikeにする。
    // (v15時点ではimpactLikeはBACKトリガに使われず、表示のみに使用)
    bool overThreshNow = !kickWindow && (lastImpactG >= IMPACT_G_THRESH);
    if (overThreshNow) {
      if (impactConsecutive < 1000) impactConsecutive++;
    } else {
      impactConsecutive = 0;
    }

    impactLike = impactConsecutive >= IMPACT_CONSECUTIVE_REQUIRED;
    accelWarn = lastImpactG >= ACCEL_WARN_G;
  } else {
    lastImpactG = 0.0f;
    impactConsecutive = 0;
    impactLike = false;
    accelWarn = false;
  }
}

// ===============================
// Motor trim calibration (v9)
// ===============================
void saveTrim() {
  drivePrefs.putFloat("trim", motorTrim);
  savedTrim = motorTrim;
}

// 停止中にBtnBを押すと実行。最大速度で10秒直進し、IMUヨードリフトから
// 左右モータのトリムを算出してNVSへ保存する。
// 現在のトリムを適用した状態で走るので、繰り返すほど収束する。
void runStraightCalibration() {
  sendAMStop();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.println("CAL STRAIGHT");
  M5.Lcd.println("MAX / 10 sec");
  M5.Lcd.println("Clear path!");
  M5.Lcd.println("Keep still...");

  // IMUオフセット取り直し(走行開始はしない)
  useBodyI2C();
  M5.IMU.Init();
  delay(30);

  const int samples = 80;
  float sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    M5.IMU.getAccelData(&ax, &ay, &az);
    M5.IMU.getGyroData(&gx, &gy, &gz);
    sax += ax; say += ay; saz += az;
    sgx += gx; sgy += gy; sgz += gz;
    delay(8);
  }
  accelOffsetX = sax / samples; accelOffsetY = say / samples; accelOffsetZ = saz / samples;
  gyroOffsetX = sgx / samples; gyroOffsetY = sgy / samples; gyroOffsetZ = sgz / samples;

  yawDeg = 0.0f;
  targetYawDeg = 0.0f;
  lastImuMs = millis();
  imuZeroed = true;
  smoothCmdLeft = 0;
  smoothCmdRight = 0;

  unsigned long t0 = millis();
  unsigned long lastTofMs = 0;
  bool aborted = false;

  while (millis() - t0 < CAL_DRIVE_MS) {
    M5.update();
    if (M5.BtnA.wasPressed()) { aborted = true; break; }

    float ax, ay, az, gx, gy, gz;
    readBuiltinIMU(ax, ay, az, gx, gy, gz);
    updateIMU(ax, ay, az, gx, gy, gz);

    if (millis() - lastTofMs > 90) {
      readOneSEN(0);
      lastTofMs = millis();
      int fm = senEffectiveMin(0);
      if (fm >= 0 && fm < CAL_ABORT_FRONT_MM) { aborted = true; break; }
    }

    if (millis() - lastAMCommandMs >= AM_COMMAND_PERIOD_MS) {
      sendAMMotorRaw(CAL_DRIVE_CMD, CAL_DRIVE_CMD);  // トリムはsendAMMotorRaw内で適用
    }
    delay(2);
  }

  unsigned long ranMs = millis() - t0;
  sendAMStop();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextSize(2);

  if (ranMs < 1000) {
    // 有効なデータが取れないまま中断
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.println("CAL ABORT");
    M5.Lcd.println("No space?");
  } else {
    // 走行時間で正規化(中断時も部分データで計算)
    float driftNorm = yawDeg * ((float)CAL_DRIVE_MS / (float)ranMs);

    // drift>0 = 左へ流れた = 左が弱い/右が強い -> 左を強める = trimを負方向へ
    motorTrim = constrain(motorTrim - driftNorm * CAL_TRIM_GAIN, -TRIM_LIMIT, TRIM_LIMIT);
    saveTrim();

    bool good = fabs(driftNorm) < CAL_OK_DEG;
    M5.Lcd.setTextColor(good ? GREEN : YELLOW, BLACK);
    M5.Lcd.println(good ? "CAL OK" : "CAL: RUN AGAIN");
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("Drift %+.1f deg\n", driftNorm);
    M5.Lcd.printf("Trim  %+.3f\n", motorTrim);
    if (aborted) M5.Lcd.println("(cut short)");
  }

  delay(2400);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("PIVOT v15");
  M5.Lcd.println("A:start B:test");
  M5.Lcd.printf("Trim %+.2f\n", motorTrim);
}

// ===============================
// Drive logic
// ===============================
int headingCorrection() {
  float err = normDeg180(targetYawDeg - yawDeg);
  int corr = (int)(err * HEADING_KP);
  return constrain(corr, -HEADING_CORR_LIMIT, HEADING_CORR_LIMIT);
}

void sendDriveCommandSmooth(int desiredLeft, int desiredRight, bool useHeading) {
  if (useHeading) {
    int corr = headingCorrection();
    desiredLeft -= corr;
    desiredRight += corr;
  }

  // v15: accelWarnはTEST_MODE時のみtrueになりうる(実走行では常にfalseで無効)。
  if (accelWarn && useHeading) {
    desiredLeft = min(desiredLeft, FWD_CMD);
    desiredRight = min(desiredRight, FWD_CMD);
  }

  desiredLeft = constrain(desiredLeft, -255, 255);
  desiredRight = constrain(desiredRight, -255, 255);

  // 前進中の小さすぎる正方向指令だけ底上げ。
  // 旋回用の逆転/低速は潰さない。
  if (desiredLeft > 0 && desiredLeft < MIN_MOVING_CMD) desiredLeft = MIN_MOVING_CMD;
  if (desiredRight > 0 && desiredRight < MIN_MOVING_CMD) desiredRight = MIN_MOVING_CMD;

  if (millis() - lastAMCommandMs < AM_COMMAND_PERIOD_MS) return;

  bool aggressive = (desiredLeft < 0 || desiredRight < 0 || abs(desiredRight - desiredLeft) > 120);
  float step = aggressive ? 44.0f : CMD_SLEW_PER_SEND;

  int newL = slewToward(smoothCmdLeft, desiredLeft, step);
  int newR = slewToward(smoothCmdRight, desiredRight, step);

  smoothCmdLeft = newL;
  smoothCmdRight = newR;

  sendAMMotorRaw(newL, newR);
}


int senEffectiveMin(uint8_t idx) {
  if (idx >= SEN_COUNT) return -1;
  if (!fresh(cachedSen[idx].updatedMs) || !cachedSen[idx].ok) return -1;
  if (cachedSen[idx].minMm < 0) return -1;

  int eff = cachedSen[idx].minMm - SEN_MOUNT_OFFSET_MM;
  if (eff < 0) eff = 0;
  return eff;
}

int senEffectiveCenter(uint8_t idx) {
  if (idx >= SEN_COUNT) return -1;
  if (!fresh(cachedSen[idx].updatedMs) || !cachedSen[idx].ok) return -1;
  if (cachedSen[idx].centerMm < 0) return -1;

  int eff = cachedSen[idx].centerMm - SEN_MOUNT_OFFSET_MM;
  if (eff < 0) eff = 0;
  return eff;
}

// v10: マウント角度による幾何投影。
// idx0(0deg)は forward=raw, side=0。idx1/2(±50deg)は forward=raw*cos, side=raw*sin。
int senForwardMm(uint8_t idx) {
  int c = senEffectiveCenter(idx);
  if (c < 0 || idx >= SEN_COUNT) return -1;
  return (int)(c * senCosAbs[idx]);
}

int senForwardMinMm(uint8_t idx) {
  int m = senEffectiveMin(idx);
  if (m < 0 || idx >= SEN_COUNT) return -1;
  return (int)(m * senCosAbs[idx]);
}

int senSidewaysMm(uint8_t idx) {
  int c = senEffectiveCenter(idx);
  if (c < 0 || idx >= SEN_COUNT) return -1;
  return (int)(c * senSinAbs[idx]);
}

int senSidewaysMinMm(uint8_t idx) {
  int m = senEffectiveMin(idx);
  if (m < 0 || idx >= SEN_COUNT) return -1;
  return (int)(m * senSinAbs[idx]);
}

int senSideScore(uint8_t idx) {
  if (idx >= SEN_COUNT) return 0;
  if (!fresh(cachedSen[idx].updatedMs)) return 0;

  int score = 0;

  if (cachedSen[idx].ok) {
    int effMin = senEffectiveMin(idx);
    int effCenter = senEffectiveCenter(idx);

    score = max(score, obstacleScore(effMin, SEN_WARN_MM, SEN_DANGER_MM));
    score = max(score, obstacleScore(effCenter, SEN_WARN_MM + 120, SEN_DANGER_MM + 40));

    // Use 8x8 surface information aggressively.
    if (cachedSen[idx].nearCountDanger >= SEN_DANGER_NEAR_COUNT) {
      score = max(score, 88 + min(12, cachedSen[idx].nearCountDanger * 3));
    }
    if (cachedSen[idx].nearCountWarn >= SEN_WARN_NEAR_COUNT) {
      score = max(score, 55 + min(35, cachedSen[idx].nearCountWarn * 5));
    }

    // Sparse valid points are not proof of safety (低反射率/端の欠測など)。
    if (cachedSen[idx].validCount > 0 && cachedSen[idx].validCount <= 5) {
      score = max(score, SEN_LOW_VALID_CAUTION_SCORE);
    }
  }

  return constrain(score, 0, 100);
}

int senFrontLikeScore() {
  // 前方判定へ左右斜めセンサや前方センサの視野端を混ぜると、広い通路でも
  // 側壁を前方障害物と誤認する。前方専用センサ(idx0)の中央4列だけを見る。
  if (!fresh(cachedSen[0].updatedMs) || !cachedSen[0].ok) return 0;

  int score = obstacleScore(senEffectiveCenter(0), FRONT_WARN_MM, FRONT_HARD_MM);
  int dangerCount = 0;
  int warnCount = 0;
  int validCount = 0;

  for (int row = 0; row < 8; row++) {
    for (int col = 2; col <= 5; col++) {
      uint16_t raw = cachedSen[0].raw[row * 8 + col];
      if (!validMatrixMm(raw)) continue;
      int mm = max(0, (int)raw - SEN_MOUNT_OFFSET_MM);
      validCount++;
      if (mm < SEN_DANGER_MM) dangerCount++;
      if (mm < SEN_WARN_MM) warnCount++;
    }
  }

  if (dangerCount >= SEN_DANGER_NEAR_COUNT) {
    score = max(score, 88 + min(12, dangerCount * 3));
  }
  if (warnCount >= SEN_WARN_NEAR_COUNT) {
    score = max(score, 55 + min(35, warnCount * 5));
  }
  if (validCount > 0 && validCount <= 2) {
    score = max(score, SEN_LOW_VALID_CAUTION_SCORE);
  }
  return constrain(score, 0, 100);
}

// v8: ピボット開始ヘルパ。
// needsFrontClear=true : 前方回避(最低PIVOT_MIN_DEG回り、前方が開くまで)
// needsFrontClear=false: 姿勢直し(ALIGN_PIVOT_DEGの小旋回)
void startPivot(int dir, bool needsFrontClear) {
  dstate = DS_PIVOT;
  pivotDir = (dir < 0) ? -1 : 1;
  pivotNeedsFrontClear = needsFrontClear;
  pivotReturnYawDeg = targetYawDeg;
  pivotStartYaw = yawDeg;
  pivotStartMs = millis();
}

// v8: バック開始ヘルパ。バック後は開いている側へ回避ピボットする。
void startForcedBack(int leftClear, int rightClear) {
  forceBackUntilMs = millis() + FORCE_BACK_MS;
  dstate = DS_BACK;
  lastForceBack = true;

  if (leftClear >= 0 && rightClear >= 0) {
    backRecoverDir = (rightClear > leftClear) ? -1 : 1;
  } else if (rightClear >= 0) {
    backRecoverDir = -1;
  } else {
    backRecoverDir = 1;
  }
}


int minPositive2(int a, int b) {
  if (a >= 0 && b >= 0) return min(a, b);
  if (a >= 0) return a;
  if (b >= 0) return b;
  return -1;
}

// v10: 単一8x8センサの幾何投影から側方クリアランスを算出。
// idx1=left(+50deg), idx2=right(-50deg)。
// 中心セル平均の投影を基本とし、8x8のどこかが極端に近い(生の最小値の投影)場合は
// そちらを優先採用して小さな出っ張りにも反応できるようにする。
int sideClearanceMm(uint8_t idx) {
  int centerSide = senSidewaysMm(idx);
  int minSide = senSidewaysMinMm(idx);

  int baseClear = centerSide;
  if (minSide >= 0 && minSide < 420) {
    baseClear = minPositive2(baseClear, minSide);
  }
  // センサー面ではなくタイヤ外端基準の実クリアランスへ変換する。
  if (baseClear >= 0) baseClear = max(0, baseClear - SENSOR_TO_TIRE_OUTER_MM);
  return baseClear;
}

bool clearanceLooksLikeWall(int clearanceMm, int sideScore) {
  if (clearanceMm >= CORRIDOR_MIN_WALL_MM && clearanceMm <= CORRIDOR_MAX_WALL_MM) return true;
  if (sideScore >= CORRIDOR_DETECT_SCORE) return true;
  return false;
}

void computeAndDrive() {
  unsigned long now = millis();

  if (!imuZeroed || !driveEnabled || emergencyPaused) {
    setReason(emergencyPaused ? "E_STOP" : "WAIT_A");
    lastCorridorMode = false;
    if (now - lastAMCommandMs >= 80) sendAMStop();
    return;
  }

  // ---- sensing (v10: 3x SEN0628 8x8, idx0=front idx1=left idx2=right) ----
  int front = senEffectiveCenter(0);      // 前方距離(代表値。回避判定・減速判定に使用)
  int frontMin = senEffectiveMin(0);      // 前方の生最小値(BACK判定用。より保守的)

  int senLeftScore = senSideScore(1);
  int senRightScore = senSideScore(2);
  int senFrontScore = senFrontLikeScore();

  lastSenLeftScore = senLeftScore;
  lastSenRightScore = senRightScore;
  lastSenFrontScore = senFrontScore;

  int frontScore = max(
    obstacleScore(front, FRONT_WARN_MM, FRONT_HARD_MM),
    senFrontScore
  );

  int leftScore = max(
    obstacleScore(senSidewaysMm(1), SIDE_WARN_MM, SIDE_DANGER_MM),
    senLeftScore
  );

  int rightScore = max(
    obstacleScore(senSidewaysMm(2), SIDE_WARN_MM, SIDE_DANGER_MM),
    senRightScore
  );

  int leftClear = sideClearanceMm(1);
  int rightClear = sideClearanceMm(2);

  lastLeftClearanceMm = leftClear;
  lastRightClearanceMm = rightClear;
  lastWallDiffMm = (leftClear >= 0 && rightClear >= 0) ? (leftClear - rightClear) : 0;

  bool leftWallSeen = clearanceLooksLikeWall(leftClear, leftScore);
  bool rightWallSeen = clearanceLooksLikeWall(rightClear, rightScore);
  bool corridorMode = leftWallSeen && rightWallSeen;

  lastCorridorMode = corridorMode;
  lastFrontScore = frontScore;
  lastLeftScore = leftScore;
  lastRightScore = rightScore;

  // =========================================================
  // v8 state machine: FORWARD -> (PIVOT | BACK) -> FORWARD
  // 旋回は全て正転逆転の超信地旋回。差動アークは使わない。
  // =========================================================

  // ---- DS_BACK: 短時間バック -> 開いた側へ回避ピボット ----
  if (dstate == DS_BACK) {
    if (now < forceBackUntilMs) {
      setReason("BACK");
      lastTurnBias = 0;
      sendDriveCommandSmooth(BACK_L_CMD, BACK_R_CMD, false);
      return;
    }
    lastBackEndMs = now;
    lastForceBack = false;
    startPivot(backRecoverDir, true);
    // fallthrough to DS_PIVOT
  }

  // ---- DS_PIVOT: IMU旋回角で終了判定 ----
  if (dstate == DS_PIVOT) {
    float turned = fabs(normDeg180(yawDeg - pivotStartYaw));
    float signedCourseDeviation = normDeg180(yawDeg - pivotReturnYawDeg);
    bool courseLimitReached = fabs(signedCourseDeviation) >= PIVOT_MAX_DEG &&
                              signedCourseDeviation * pivotDir >= 0.0f;
    bool frontClearNow = (front < 0) || (front > FRONT_CLEAR_EXIT_MM);

    bool done;
    if (pivotNeedsFrontClear) {
      done = (turned >= PIVOT_MIN_DEG && frontClearNow) ||
             (turned >= PIVOT_MAX_DEG) ||
             courseLimitReached ||
             (now - pivotStartMs > PIVOT_TIMEOUT_MS);
    } else {
      done = (turned >= ALIGN_PIVOT_DEG) ||
             (turned >= PIVOT_MAX_DEG) ||
             courseLimitReached ||
             (now - pivotStartMs > ALIGN_PIVOT_TIMEOUT_MS);
    }

    if (!done) {
      if (pivotDir > 0) {
        setReason(pivotNeedsFrontClear ? "PIVOT_L" : "ALIGN_L");
        lastTurnBias = 500;
        sendDriveCommandSmooth(-PIVOT_CMD, PIVOT_CMD, false);
      } else {
        setReason(pivotNeedsFrontClear ? "PIVOT_R" : "ALIGN_R");
        lastTurnBias = -500;
        sendDriveCommandSmooth(PIVOT_CMD, -PIVOT_CMD, false);
      }
      lastAvoidMs = now;
      justExitedAvoid = true;
      return;
    }

    // 旋回時の向きを新しい直進方位にはしない。旋回前の進行方位を維持し、
    // 前進しながらIMU補正で緩やかに復帰することでUターン化を防ぐ。
    targetYawDeg = pivotReturnYawDeg;
    courseReturnActive = true;
    courseReturnUntilMs = now + COURSE_RETURN_TIMEOUT_MS;
    dstate = DS_FORWARD;
    lastPivotEndMs = now;

    // v13: ピボット中は片輪が負(逆転)。そのままFORWARDのゆるいslewに渡すと、
    // LCDが"RUN"を表示している間も数フレーム片輪が負のまま(=後進して見える)になる。
    // RUN表示に切り替わる瞬間にゼロへ戻し、正方向へまっさらからランプさせる。
    smoothCmdLeft = 0;
    smoothCmdRight = 0;
  }

  // ---- DS_FORWARD ----

  bool frontRed = frontScore >= OBSTACLE_RED_SCORE;
  bool leftRed = leftScore >= OBSTACLE_RED_SCORE;
  bool rightRed = rightScore >= OBSTACLE_RED_SCORE;

  // (1) BACKトリガ: 前方センサ実測の極近のみ。
  // v15: 接触保険(IMU加速度によるBACKトリガ)は削除。距離センサが
  // 何も検知していないのにBACKする経路を完全になくすため。
  bool backArmed = (now - autoStartMs) > BACK_ARM_DELAY_MS;
  bool tofRealClose = (frontMin >= 0 && frontMin < FRONT_STOP_MM);
  bool backCooldownOk = tofRealClose || (now - lastBackEndMs > BACK_COOLDOWN_MS);

  if (backArmed && backCooldownOk && tofRealClose) {
    startForcedBack(leftClear, rightClear);
    setReason("BACK");
    lastTurnBias = 0;
    sendDriveCommandSmooth(BACK_L_CMD, BACK_R_CMD, false);
    return;
  }

  bool pivotCooldownOk = (now - lastPivotEndMs > PIVOT_COOLDOWN_MS);
  bool sideAlignDelayOk = (now - lastPivotEndMs > SIDE_ALIGN_DELAY_MS);

  // (2) 前方回避ピボット: 代表距離が近い、またはLCD前方表示が赤なら旋回する。
  //     通常は右/左を交互に選び、回避後の方位ずれが一方向へ累積しないようにする。
  //     ただし片側だけが赤なら、安全を優先して必ず障害物と反対側へ旋回する。
  bool frontNeedsPivot = (front >= 0 && front < FRONT_PIVOT_MM) || frontRed;
  if (pivotCooldownOk && frontNeedsPivot) {
    int dir;
    if (leftRed && !rightRed) {
      dir = -1;                                        // 左危険 -> 右へ
    } else if (rightRed && !leftRed) {
      dir = 1;                                         // 右危険 -> 左へ
    } else {
      dir = nextFrontPivotDir;
      nextFrontPivotDir = -nextFrontPivotDir;
    }
    startPivot(dir, true);
    lastAvoidMs = now;
    justExitedAvoid = true;
    if (dir > 0) {
      setReason("PIVOT_L");
      lastTurnBias = 500;
      sendDriveCommandSmooth(-PIVOT_CMD, PIVOT_CMD, false);
    } else {
      setReason("PIVOT_R");
      lastTurnBias = -500;
      sendDriveCommandSmooth(PIVOT_CMD, -PIVOT_CMD, false);
    }
    return;
  }

  // (3) 側方極近またはLCD赤: 障害物と反対側へ姿勢直しピボット。
  //     両側が赤い四つ角では、広い側へ強めの回避旋回を行う。
  bool leftTooClose = (leftClear >= 0 && leftClear < SIDE_PIVOT_MM) || leftRed;
  bool rightTooClose = (rightClear >= 0 && rightClear < SIDE_PIVOT_MM) || rightRed;
  // 赤が継続しても旋回直後は必ず前進時間を確保し、連続ピボットで
  // 元来た方向へ向くのを防ぐ。初回の赤は通常どおり即座に処理される。
  if (sideAlignDelayOk && (leftTooClose || rightTooClose)) {
    bool bothBlocked = leftTooClose && rightTooClose;
    int dir;
    if (bothBlocked) {
      if (leftClear >= 0 && rightClear >= 0 && leftClear != rightClear) {
        dir = (rightClear > leftClear) ? -1 : 1; // 広い側へ旋回
      } else if (leftScore != rightScore) {
        dir = (leftScore > rightScore) ? -1 : 1; // 危険度が低い側へ旋回
      } else {
        dir = pivotDir;                         // 同値なら直前の旋回方向を維持
      }
    } else {
      dir = leftTooClose ? -1 : 1;              // 左が近い -> 右へ
    }

    startPivot(dir, bothBlocked);
    lastAvoidMs = now;
    justExitedAvoid = true;
    if (dir > 0) {
      setReason(bothBlocked ? "ESCAPE_L" : "ALIGN_L");
      lastTurnBias = bothBlocked ? 500 : 300;
      sendDriveCommandSmooth(-PIVOT_CMD, PIVOT_CMD, false);
    } else {
      setReason(bothBlocked ? "ESCAPE_R" : "ALIGN_R");
      lastTurnBias = bothBlocked ? -500 : -300;
      sendDriveCommandSmooth(PIVOT_CMD, -PIVOT_CMD, false);
    }
    return;
  }

  // (4) 通路中央維持: 目標ヨーを開いた側へゆっくり振る。
  //     実際の舵はヘディング補正(±HEADING_CORR_LIMIT)が担う。
  float courseReturnError = normDeg180(targetYawDeg - yawDeg);
  bool courseReturnTimedOut = (long)(now - courseReturnUntilMs) >= 0;
  if (courseReturnActive &&
      (fabs(courseReturnError) <= COURSE_RETURN_DONE_DEG || courseReturnTimedOut)) {
    if (courseReturnTimedOut) targetYawDeg = yawDeg;
    courseReturnActive = false;
    justExitedAvoid = false;
  }

  if (corridorMode && !courseReturnActive && lastWallDiffMm != 0) {
    int diff = lastWallDiffMm;
    if (abs(diff) > CORRIDOR_DIFF_DEADBAND_MM) {
      float step = constrain(diff * 0.004f, -CORRIDOR_YAW_STEP_DEG, CORRIDOR_YAW_STEP_DEG);
      targetYawDeg = normDeg180(targetYawDeg + step);

      // 目標が現在ヨーから離れすぎないよう制限
      float dev = normDeg180(targetYawDeg - yawDeg);
      if (dev > CORRIDOR_YAW_CLAMP_DEG) targetYawDeg = normDeg180(yawDeg + CORRIDOR_YAW_CLAMP_DEG);
      if (dev < -CORRIDOR_YAW_CLAMP_DEG) targetYawDeg = normDeg180(yawDeg - CORRIDOR_YAW_CLAMP_DEG);
    }
  }

  // (5) 前進: 常に高デューティ。前方が近ければ少しだけ減速。
  int base = FWD_CMD;
  if ((front >= 0 && front < FRONT_SLOW_MM) || frontScore >= 35 ||
      max(leftScore, rightScore) >= 55) {
    base = FWD_SLOW_CMD;
  }

  lastTurnBias = 0;

  if (corridorMode) {
    setReason("CORRIDOR");
  } else if (max(leftScore, rightScore) > 35 || frontScore > 18) {
    setReason("FWD_CARE");
    lastAvoidMs = now;
    justExitedAvoid = true;
  } else {
    setReason("FWD");
    if (!courseReturnActive && justExitedAvoid && now - lastAvoidMs > AVOID_HOLD_MS) {
      targetYawDeg = yawDeg;
      justExitedAvoid = false;
    }

    // v9: 直進安定時のみ、恒常的なヘディング補正量からトリムを微学習。
    // corr>0が続く = 常に左へ舵を切っている = 右へ流れている = 右が弱い -> trimを正へ。
    // CORRIDOR中は中央寄せの意図的な舵が混ざるため学習しない。
    if (AUTO_TRIM_ENABLE && !justExitedAvoid) {
      int corr = headingCorrection();
      motorTrim = constrain(motorTrim + corr * AUTO_TRIM_RATE, -TRIM_LIMIT, TRIM_LIMIT);
    }
  }

  sendDriveCommandSmooth(base, base, true);
}

// ===============================
// UI (v7: graphical, sprite-based, no flicker)
// ===============================
// 画面 240x135 (rotation 1)
//   左側: 上から見たロボット + 近接バー(前/左/右, 緑->黄->赤) + 旋回矢印
//   右側: 状態(RUN/BACK/STOP)大表示, AMリンク, CORRインジケータ,
//         通路左右差バー, モータ出力バー(L/R, 緑=前進/赤=後進)
TFT_eSprite lcdSpr = TFT_eSprite(&M5.Lcd);
bool lcdSprOK = false;

uint16_t scoreColor(int score) {
  if (score >= OBSTACLE_RED_SCORE) return M5.Lcd.color565(255, 60, 40); // red
  if (score >= 35) return M5.Lcd.color565(255, 200, 0);   // yellow
  return M5.Lcd.color565(40, 220, 80);                    // green
}

void drawStatus() {
  unsigned long now = millis();

  // v15: dstateが変化した瞬間は間引かず即描画する。
  // (旧: LCD_UPDATE_PERIOD_MSより短い状態(例: BACK)が一度も描画されないまま
  //  終わってしまい、表示が古い状態のまま止まって見える問題があった)
  bool stateChanged = (dstate != lastDrawnDstate);

  if (!stateChanged && (now - lastLCDUpdateMs < LCD_UPDATE_PERIOD_MS)) return;
  lastLCDUpdateMs = now;
  lastDrawnDstate = dstate;

  if (!lcdSprOK) return;

  const uint16_t COL_GRAY  = M5.Lcd.color565(80, 80, 80);
  const uint16_t COL_BODY  = M5.Lcd.color565(90, 110, 220);
  const uint16_t COL_NOSE  = M5.Lcd.color565(200, 210, 255);

  lcdSpr.fillSprite(TFT_BLACK);

  // ---- robot body (top-down, nose up) ----
  const int rx = 62, ry = 64, rw = 26, rh = 44;
  lcdSpr.fillRoundRect(rx, ry, rw, rh, 4, COL_BODY);
  lcdSpr.fillRect(rx + 4, ry - 4, rw - 8, 5, COL_NOSE);

  // ---- front proximity (fills downward toward robot) ----
  {
    const int sx = 24, sy = 4, sw = 102, sh = 52;
    lcdSpr.drawRect(sx, sy, sw, sh, COL_GRAY);
    int f = constrain(lastFrontScore, 0, 100);
    int fh = (f * (sh - 2)) / 100;
    if (fh > 0) lcdSpr.fillRect(sx + 1, sy + sh - 1 - fh, sw - 2, fh, scoreColor(f));
  }

  // ---- left proximity (fills rightward toward robot) ----
  {
    const int sx = 2, sy = 62, sw = 54, sh = 48;
    lcdSpr.drawRect(sx, sy, sw, sh, COL_GRAY);
    int s = constrain(lastLeftScore, 0, 100);
    int fw2 = (s * (sw - 2)) / 100;
    if (fw2 > 0) lcdSpr.fillRect(sx + sw - 1 - fw2, sy + 1, fw2, sh - 2, scoreColor(s));
  }

  // ---- right proximity (fills leftward toward robot) ----
  {
    const int sx = 94, sy = 62, sw = 54, sh = 48;
    lcdSpr.drawRect(sx, sy, sw, sh, COL_GRAY);
    int s = constrain(lastRightScore, 0, 100);
    int fw2 = (s * (sw - 2)) / 100;
    if (fw2 > 0) lcdSpr.fillRect(sx + 1, sy + 1, fw2, sh - 2, scoreColor(s));
  }

  // ---- turn arrow (below robot): points in turn direction ----
  {
    int tb = lastTurnBias;
    int mag = abs(tb);
    // 通常制御は<=100, 回避コード(444/666/777/888/999)は最大長で表示
    int len = (mag > 100) ? 50 : (mag * 50) / 100;
    int ax = rx + rw / 2, ay = 124;

    bool reversing = (smoothCmdLeft < -5 && smoothCmdRight < -5);
    if (reversing) {
      // 後進中: 下向き矢印(赤)
      lcdSpr.fillTriangle(ax, ay + 8, ax - 10, ay - 4, ax + 10, ay - 4, TFT_RED);
    } else if (tb > 0 && len > 6) {         // left turn
      lcdSpr.fillRect(ax - len + 8, ay - 3, len - 8, 6, TFT_CYAN);
      lcdSpr.fillTriangle(ax - len, ay, ax - len + 10, ay - 8, ax - len + 10, ay + 8, TFT_CYAN);
    } else if (tb < 0 && len > 6) {         // right turn
      lcdSpr.fillRect(ax, ay - 3, len - 8, 6, TFT_CYAN);
      lcdSpr.fillTriangle(ax + len, ay, ax + len - 10, ay - 8, ax + len - 10, ay + 8, TFT_CYAN);
    } else {
      // straight: 上向き小矢印
      lcdSpr.fillTriangle(ax, ay - 6, ax - 6, ay + 4, ax + 6, ay + 4, COL_GRAY);
    }
  }

  // ---- impact warning "!" (top-left of field) ----
  if (TEST_MODE && lastImpactG >= ACCEL_WARN_G) {
    lcdSpr.setTextSize(2);
    lcdSpr.setTextColor(TFT_RED, TFT_BLACK);
    lcdSpr.setCursor(4, 4);
    lcdSpr.print("!");
  }

  // ---- right panel ----
  const int px = 154;
  lcdSpr.drawFastVLine(151, 0, 135, COL_GRAY);

  // state (big)
  const char *st;
  uint16_t stc;
  if (emergencyPaused)            { st = "STOP"; stc = TFT_RED; }
  else if (!driveEnabled)         { st = "WAIT"; stc = TFT_YELLOW; }
  else if (dstate == DS_BACK)     { st = "BACK"; stc = TFT_ORANGE; }
  else if (dstate == DS_PIVOT)    { st = "PIVT"; stc = TFT_CYAN; }
  else                            { st = "RUN";  stc = TFT_GREEN; }
  lcdSpr.setTextSize(3);
  lcdSpr.setTextColor(stc, TFT_BLACK);
  lcdSpr.setCursor(px + 2, 4);
  lcdSpr.print(st);

  // AM link dot
  lcdSpr.fillCircle(232, 14, 6, am.online ? TFT_GREEN : TFT_RED);

  // corridor indicator
  lcdSpr.setTextSize(1);
  if (lastCorridorMode) {
    lcdSpr.fillRoundRect(px + 2, 32, 44, 14, 3, TFT_DARKGREEN);
    lcdSpr.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  } else {
    lcdSpr.drawRoundRect(px + 2, 32, 44, 14, 3, COL_GRAY);
    lcdSpr.setTextColor(COL_GRAY, TFT_BLACK);
  }
  lcdSpr.setCursor(px + 9, 35);
  lcdSpr.print("CORR");

  // v9: trim常時表示
  lcdSpr.setTextColor(TFT_WHITE, TFT_BLACK);
  lcdSpr.setCursor(px + 50, 35);
  lcdSpr.printf("T%+.2f", motorTrim);

  // corridor L-R diff bar (center = centered; marker to the wall-closer side)
  {
    const int bx = px + 2, by = 50, bw = 82, bh = 10;
    lcdSpr.drawRect(bx, by, bw, bh, COL_GRAY);
    lcdSpr.drawFastVLine(bx + bw / 2, by, bh, TFT_WHITE);
    if (lastCorridorMode) {
      // diff>0: 左が広い(=右壁寄り) -> マーカーを右に出す
      int off = constrain(lastWallDiffMm, -250, 250);
      off = off * (bw / 2 - 3) / 250;
      lcdSpr.fillRect(bx + bw / 2 + off - 2, by + 1, 5, bh - 2, TFT_CYAN);
    }
  }

  // motor output bars: up = forward(green), down = reverse(red)
  {
    const int mbY = 66, mbH = 52;
    const int mid = mbY + mbH / 2;
    const int xs[2] = {px + 8, px + 34};
    const int vals[2] = {(int)smoothCmdLeft, (int)smoothCmdRight};
    const char *lb[2] = {"L", "R"};

    for (int i = 0; i < 2; i++) {
      lcdSpr.drawRect(xs[i], mbY, 16, mbH, COL_GRAY);
      lcdSpr.drawFastHLine(xs[i], mid, 16, TFT_WHITE);

      int v = constrain(vals[i], -255, 255);
      int h = (abs(v) * (mbH / 2 - 1)) / 255;
      if (v > 0 && h > 0) lcdSpr.fillRect(xs[i] + 1, mid - h, 14, h, TFT_GREEN);
      if (v < 0 && h > 0) lcdSpr.fillRect(xs[i] + 1, mid + 1, 14, h, TFT_RED);

      lcdSpr.setTextColor(TFT_WHITE, TFT_BLACK);
      lcdSpr.setCursor(xs[i] + 5, mbY + mbH + 3);
      lcdSpr.print(lb[i]);
    }

    // yaw bar next to motors: heading error visual
    const int yx = px + 62;
    lcdSpr.drawRect(yx, mbY, 16, mbH, COL_GRAY);
    lcdSpr.drawFastHLine(yx, mid, 16, TFT_WHITE);
    float err = normDeg180(targetYawDeg - yawDeg);  // + = 左へ戻したい
    int eh = constrain((int)(fabs(err) * (mbH / 2 - 1) / 45.0f), 0, mbH / 2 - 1);
    uint16_t ec = (fabs(err) > 20) ? TFT_ORANGE : TFT_CYAN;
    if (err > 0 && eh > 0) lcdSpr.fillRect(yx + 1, mid - eh, 14, eh, ec);
    if (err < 0 && eh > 0) lcdSpr.fillRect(yx + 1, mid + 1, 14, eh, ec);
    lcdSpr.setTextColor(TFT_WHITE, TFT_BLACK);
    lcdSpr.setCursor(yx + 2, mbY + mbH + 3);
    lcdSpr.print("Yw");
  }

  // reason (small, bottom of panel)
  lcdSpr.setTextSize(1);
  lcdSpr.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcdSpr.setCursor(px + 2, 127);
  lcdSpr.print(driveReason);

  // v12: TEST_MODE中は常時表示(本走行との混同防止)
  if (TEST_MODE) {
    lcdSpr.setTextColor(TFT_RED, TFT_BLACK);
    lcdSpr.setCursor(px + 90, 4);
    lcdSpr.print("TEST");
  }

  lcdSpr.pushSprite(0, 0);
}

void debugSerial() {
  if (!DEBUG_SERIAL_OUTPUT) return;

  unsigned long now = millis();
  if (now - lastDebugSerialMs < DEBUG_SERIAL_PERIOD_MS) return;
  lastDebugSerialMs = now;

  Serial.printf("run=%d reason=%s cmd=%d,%d act=%d,%d score=%d,%d,%d turn=%d yaw=%.1f trim=%+.3f Sf=%d Sl=%d Sr=%d\n",
                driveEnabled,
                driveReason,
                targetCmdLeft,
                targetCmdRight,
                am.actualLeft,
                am.actualRight,
                lastFrontScore,
                lastLeftScore,
                lastRightScore,
                lastTurnBias,
                yawDeg,
                motorTrim,
                cachedSen[0].minMm, cachedSen[1].minMm, cachedSen[2].minMm);
}

struct __attribute__((packed)) PointCloudFrame {
  char magic[4];                 // "PCLD"
  uint8_t version;               // protocol version = 2
  uint8_t driveState;
  uint8_t flags;                 // bit0=drive, bit1=IMU, bit3=AM, bit4=Wi-Fi client
  uint8_t reserved;
  uint32_t timestampMs;
  int16_t yawCdeg;
  int16_t targetYawCdeg;
  int16_t gapTargetYawCdeg;
  int16_t leftClearanceMm;
  int16_t rightClearanceMm;
  int16_t motorLeft;
  int16_t motorRight;
  int16_t accelXmilliG;          // 静止オフセット補正済み機体座標加速度
  int16_t accelYmilliG;
  int16_t accelZmilliG;
  int16_t sensorYawCdeg[SEN_COUNT]; // 各点群を測距した瞬間のヨー
  uint32_t sensorTimestampMs[SEN_COUNT]; // 同一点群の再登録防止用
  char reason[16];
  uint16_t distances[SEN_COUNT * 64];
  uint16_t crc16;
};

static_assert(sizeof(PointCloudFrame) == 452, "PointCloudFrame size mismatch");

uint16_t pointCloudCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void streamPointCloud() {
  if (!POINT_CLOUD_STREAM_ENABLE) return;

  unsigned long now = millis();
  if (now - lastPointCloudStreamMs < POINT_CLOUD_STREAM_PERIOD_MS) return;
  lastPointCloudStreamMs = now;

  PointCloudFrame frame = {};
  memcpy(frame.magic, "PCLD", 4);
  frame.version = 2;
  frame.driveState = (uint8_t)dstate;
  frame.flags = (driveEnabled ? 0x01 : 0x00) |
                (imuZeroed ? 0x02 : 0x00) |
                (am.online ? 0x08 : 0x00) |
                (WiFi.softAPgetStationNum() > 0 ? 0x10 : 0x00);
  frame.timestampMs = now;
  frame.yawCdeg = (int16_t)lroundf(yawDeg * 100.0f);
  frame.targetYawCdeg = (int16_t)lroundf(targetYawDeg * 100.0f);
  frame.gapTargetYawCdeg = (int16_t)lroundf(targetYawDeg * 100.0f);
  frame.leftClearanceMm = (int16_t)constrain(lastLeftClearanceMm, -1, 32767);
  frame.rightClearanceMm = (int16_t)constrain(lastRightClearanceMm, -1, 32767);
  frame.motorLeft = (int16_t)targetCmdLeft;
  frame.motorRight = (int16_t)targetCmdRight;
  frame.accelXmilliG = (int16_t)constrain((int)lroundf(imuAccelXG * 1000.0f), -32767, 32767);
  frame.accelYmilliG = (int16_t)constrain((int)lroundf(imuAccelYG * 1000.0f), -32767, 32767);
  frame.accelZmilliG = (int16_t)constrain((int)lroundf(imuAccelZG * 1000.0f), -32767, 32767);
  for (uint8_t sensor = 0; sensor < SEN_COUNT; sensor++) {
    frame.sensorYawCdeg[sensor] = (int16_t)lroundf(cachedSen[sensor].captureYawDeg * 100.0f);
    frame.sensorTimestampMs[sensor] = cachedSen[sensor].updatedMs;
  }
  strncpy(frame.reason, driveReason, sizeof(frame.reason) - 1);

  for (uint8_t sensor = 0; sensor < SEN_COUNT; sensor++) {
    bool sensorFresh = fresh(cachedSen[sensor].updatedMs) && cachedSen[sensor].ok;
    for (uint8_t point = 0; point < 64; point++) {
      frame.distances[sensor * 64 + point] = sensorFresh
        ? cachedSen[sensor].raw[point]
        : 0xFFFF;
    }
  }

  frame.crc16 = pointCloudCrc16((const uint8_t *)&frame, sizeof(frame) - sizeof(frame.crc16));
  pointCloudUdp.beginPacket(IPAddress(192, 168, 4, 255), POINT_CLOUD_UDP_PORT);
  pointCloudUdp.write((const uint8_t *)&frame, sizeof(frame));
  pointCloudUdp.endPacket();
}

// ===============================
// setup / loop
// ===============================
void setup() {
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);

  // v7: sprite for flicker-free graphical status
  lcdSpr.setColorDepth(8);
  lcdSprOK = (lcdSpr.createSprite(240, 135) != nullptr);

  Serial.begin(115200);
  AMSerial.begin(AM_UART_BAUD, SERIAL_8N1, AM_UART_RX, AM_UART_TX);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(POINT_CLOUD_WIFI_SSID, POINT_CLOUD_WIFI_PASSWORD);
  pointCloudUdp.begin(POINT_CLOUD_UDP_PORT);

  // v9: トリムをNVSから復元
  drivePrefs.begin("drive", false);
  motorTrim = constrain(drivePrefs.getFloat("trim", 0.0f), -TRIM_LIMIT, TRIM_LIMIT);
  savedTrim = motorTrim;

  delay(150);  // v11: 500->150 起動短縮(電源安定待ちとして最低限は残す)

  useBodyI2C();
  M5.IMU.Init();
  delay(80);

  useExternalI2C();
  if (!i2cExists(TCA_ADDR)) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.println("TCA NOT");
    M5.Lcd.println("FOUND");
    while (true) delay(1000);
  }

  initSEN0628();

  sendAMStop();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("PIVOT v15");
  M5.Lcd.println("A:start B:test");
  M5.Lcd.println("B:MAX 10s trim");
  M5.Lcd.printf("Trim %+.2f\n", motorTrim);
}

void loop() {
  M5.update();
  readAMTelemetry();

  if (M5.BtnA.wasPressed()) {
    zeroIMUAndStart();
  }

  // BtnBの割り当ては、停止中の最大速度10秒トリムテストのみ。
  if (!driveEnabled && M5.BtnB.wasPressed()) {
    runStraightCalibration();
  }

  // v9: 自動学習分の定期保存(変化があった時のみ書き込み)
  if (millis() - lastTrimSaveMs > TRIM_SAVE_PERIOD_MS) {
    lastTrimSaveMs = millis();
    if (fabs(motorTrim - savedTrim) > 0.01f) saveTrim();
  }

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  readBuiltinIMU(ax, ay, az, gx, gy, gz);
  updateIMU(ax, ay, az, gx, gy, gz);

  pollOneSensor();

  computeAndDrive();
  drawStatus();
  debugSerial();
  streamPointCloud();

  delay(0);
}

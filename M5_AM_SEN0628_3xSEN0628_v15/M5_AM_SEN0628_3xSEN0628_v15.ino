/*
  M5StickC Plus + Arduino Mega + ToF/SEN0628
  SEN0628 corridor follow v8: PIVOT-TURN architecture
  v9: + motor L/R calibration (trim)
  v10: sensor suite unified to 3x SEN0628 8x8, angles 0/+50/-50deg

  現行運用変更:
    - SEN0628の距離閾値は、テスト時の値へ一律40mmを加えた固定値を採用。
    - 左右モータ差はIMUヨー補正とNVSに保存されたmotorTrimだけで補正する。
    - 側方距離はセンサー位置からタイヤ外端までの50mmを差し引き、
      タイヤ外端基準のクリアランスとして回避判定する。
    - 通常はIMU方位を維持して前進し、障害物検知時だけLCD赤判定による
      ピボット回避、前方極近時だけバックを行う。
    - BtnBは停止中の最大速度10秒直進テスト専用。IMUヨードリフトから
      motorTrimを算出し、NVSへ保存する。

  v43: コース中央に置かれた障害物の回避方向を固定。

  v43 changes:
    - 前方赤、L字形状なし、左右スコア差15以内、左右クリアランス差50mm以内を
      CENTER_OBSTACLEと判定。
    - CLOCKWISE=trueなら右、falseなら左へ、既存STEER_MAX_DEG(35度)で
      前進しながら固定回避する。
    - 左右同値時にlastSteerDirへ依存して回避方向が変わる挙動を解消。
    - 中央障害物は90度コーナーTURN/コーナーカーブバックへ入れず、
      通過後はRETURNで回避前の直進方位へ戻す。

  v42: 半透明プラ板の弱い反射と、幅約1mの広いコースに合わせて再調整。

  v42 changes:
    - FWD_CMD 140->155、FWD_SLOW_CMD 120->135。最低指令110は維持。
    - FRONT_SLOW_MM 200->320、FRONT_HARD/WARN 120/180->180/280。
    - SIDE_DANGER/WARN 120/160->200/320。
    - SEN_DANGER/WARN 100/160->160/280。
    - 有効点が少ない弱反射時の警戒スコアを40->50へ引き上げ。
    - 【注意】半透明板はToF光を透過・乱反射するため、実機で欠測状況を確認すること。

  v41: 四角判定時のバック過多で90度旋回後に引っ掛かる問題を調整。

  v41 changes:
    - CORNER_BACK_MSを3000->2000msへ短縮。
    - 前方極近時の通常直線バックFORCE_BACK_MS=3000msは維持。
    - 四角検知時の最終目標ヨーをバック前に固定。カーブバックで回った角度を含めて
      合計90度とし、バック終了角からさらに90度回す過旋回を修正。

  v40: 前方極近時の通常バックも実走可能な3秒へ延長。

  v40 changes:
    - FORCE_BACK_MSを400->3000msへ変更。
    - 通常直線バック、コーナーカーブバックともに3000ms。
    - 【注意】後方センサーなしで3秒後退するため、後方空間の確保が必要。

  v39: コーナー脱出時のカーブバックを最低3秒へ延長。

  v39 changes:
    - CORNER_BACK_MSを700->3000msへ変更。
    - 【v40で変更】前方極近時の通常直線バック時間。
    - 【注意】後方センサーなしで3秒後退するため、後方空間の確保が必要。

  v38: コーナー脱出時のカーブバック距離を延長。

  v38 changes:
    - ALL_RED/四角脱出時のカーブバックを400ms->700msへ延長。
    - 【v40で変更】前方極近時の通常直線バック時間。

  v37: 実コース幅が約1000mmと判明したため通路認識範囲を更新。

  v37 changes:
    - 車幅約300mmの場合、中央時のタイヤ外端〜壁は片側約350mm。
      CORRIDOR_MAX_WALL_MMを260->420へ拡大し、中央走行を通路として認識可能にした。
    - 幾何四角判定の外周側壁探索上限も380->600mmへ拡大。
    - 四角の前方連続壁400mm条件は維持。幅1000mmのコース壁に対し十分小さく、
      最大幅300mmの障害物を除外する境界としても有効。

  v36: 後壁へ密着した最大幅300mmの障害物をL字コーナーと誤認しないよう改善。

  v36 changes:
    - 前方壁候補をy方向50mm区画へ分割し、点の最小〜最大幅ではなく
      「欠落なく連続して埋まった横幅」で評価する。
    - 四角の前方壁には400mm以上の連続幅を要求。
      最大300mmの障害物前面は除外し、障害物の左右に見える後壁も
      中央の欠落によって別々の壁片として扱う。
    - 合成点群で幅約700mmのコース壁は距離70〜300mmの全域で
      400〜600mm連続、幅300mm以下の障害物は400mm未満になることを確認。

  v35: 8x8 ToF x3の点群をXY座標へ変換し、四角を幾何形状で判定。

  v35 changes:
    - 各セルを x=L*cos(theta), y=L*sin(theta) で車体座標へ変換。
    - 前方70〜320mmでxが約25mm幅に揃い、横幅140mm以上に連続する点群を
      「前方を横切る壁」と判定。
    - CLOCKWISE=trueでは左側、falseでは右側をコーナー外周壁とし、
      yが約25mm幅に揃い、前後長120mm以上に連続する点群を「側壁」と判定。
    - 前方壁と側壁の推定交点が側壁範囲内にある場合だけL字形状とする。
    - L字形状+前方赤+異なる2フレーム+200ms継続の全条件で四角確定。
    - L字にならない細い障害物・単独壁・単発点は通常回避のまま。

  v34: 四角の瞬間誤判定防止と、通常障害物回避後の直進方位復帰を追加。

  v34 changes:
    - 前方赤が200ms継続し、異なる前方測距フレームで2回連続した場合だけ
      四角確定とし、90度旋回へ入る。
      単発/短時間の前方赤は通常の連続ステア回避として処理する。
    - 通常回避へ入る直前のtargetYawDegを保存し、障害物消失後280ms保持してから
      保存方位へ戻す。誤差3度以内または2000msで復帰完了。
    - 方位復帰中は通路中央補正と競合させず、LCD reasonをRETURNとする。
    - 90度コーナー旋回後は旋回後ヨーを新しい直進基準とし、旧方位へ戻さない。
    - ALL_REDバックも前方赤200ms継続後にだけ発動する。

  v33: 周回方向を固定設定し、四角検知時の旋回をIMU基準90度へ変更。

  v33 changes:
    - CLOCKWISEパラメータを追加。true=時計回り(右90度)、false=反時計回り(左90度)。
    - 前方赤を四角到達としてDS_CORNER_TURNへ入り、開始ヨーから設定方向へ
      90度変化するまで片輪タンクターンを継続する。
    - 旋回終了許容差5度、タイムアウト3000ms。終了後500msは再トリガを抑制。
    - ALL_RED時は設定方向へカーブバックした後、同じ90度旋回へ接続する。
    - DS_ESCAPE_TURNをDS_CORNER_TURNへ置換するがenum値1は維持。
    - 【要実機確認】CLOCKWISE=trueで右90度、falseで左90度となり、
      LCD reasonがCORNER_R/CORNER_LからRUNへ戻ること。

  v32: 四角/コーナーで「壁検知→旋回→三方向すべて赤」となった際、
       直線バックと停止だけでは車首が壁から外れず脱出できない問題へ対応。

  v32 changes:
    - ALL_RED時は広い側へカーブバックし、続けて同方向へ800msの
      強制片輪タンクターンを行ってから通常判定へ戻る。
    - 未使用だったDS_PIVOTの値1をDS_ESCAPE_TURNとして再利用し、
      DS_BACK=2/DS_CALIBRATING=3の値は維持する。
    - 前方極近だけの通常BACKは従来どおり直線バック。
    - 後方センサーがないため後退時間は400msから延長しない。
    - 【要実機確認】ALL_RED時にBACK_ARC_L/R→ESCAPE_L/Rと遷移すること。

  v31: v30の全セル角度統合後、縦視野端が床を障害物として拾う実走結果へ対応。

  v31 changes:
    - 障害物判定に使う縦方向を中央4行(row 2〜5)へ限定。
      上下各2行(row 0,1,6,7)は床・車体・天井反射を避けるため走行判断から除外。
    - minMm/validCount/nearCount、前方全列、左右端列、192セル角度統合の
      全経路で同じ有効行範囲を使用する。
    - raw 8x8取得とUDP点群配信は全行を維持し、診断表示からは床反射を消さない。
    - 水平方向は全8列・3基統合のまま維持する。
    - 【要実機確認】床だけの状態で赤判定にならず、中央4行へ映る壁・障害物には
      従来どおり反応すること。

  v30: 8x8 ToFの視野エッジで障害物への反応が弱い実走結果へ対応。

  v30 changes:
    - 前方SEN0628の回避判定を中央6列(1〜6)から全8列(0〜7)へ拡張。
      これまで意図的に除外していた左右端列も前方回避へ使う。
    - 3基それぞれの左右端2列(0,1,6,7)を専用集計し、2セル以上が
      SEN_WARN_MM未満なら最低65点、SEN_DANGER_MM未満なら最低90点とする。
    - 端列専用判定を各センサーのスコアへ合成し、中央値へ潰すと消えていた
      細い障害物・センサー視野の継ぎ目を変針判断へ反映する。
    - 点群ビューアーと同じ水平角モデル(60度/8列、列中心7.5度刻み)を走行制御へ導入。
      3基x64点を前方(-32〜+32度)、左(+18〜+80度)、右(-80〜-18度)の
      重なり付きセクターへ統合し、センサー間の約10度重複も回避判断へ使う。
    - 走行方式、速度、motorTrim、NVS、ALL_REDバック条件は変更していない。
    - 【要実機確認】LCD/点群で端に映る障害物に対して該当方向が赤へ変わり、
      接触前に反対側へ変針すること。

  v29: motorTrimの通常走行時適用をキャリブレーションと統一。

  v29 changes:
    - v28で追加した低速時の適用trim±0.12制限を撤回。
    - 通常走行は起動時にNVSから読み込んだmotorTrimを全量そのまま適用する。
    - AUTO_TRIM_ENABLEをfalseにし、走行中の微学習・NVS自動更新を停止。
      motorTrimの更新元はBtnBキャリブレーションと明示的NVSリセットだけに限定する。
    - 片輪停止/強旋回時のtrim無効化は、左右の旋回力を揃えるため維持。

  v28: 実車寸法と実走結果を反映。STL外形は約268x286mmで実測最大幅約300mmと整合。
       中央時の左右斜め測距約350mmを50度で側方投影すると約268mm、
       センサーからタイヤ外端まで50mmを引いた実クリアランスは約218mmとなる。

  v28 changes:
    - OBSTACLE_RED_SCOREを70->60(3/5)へ変更し、回避開始を早めた。
    - CORRIDOR_MAX_WALL_MMを160->260へ変更。中央時約218mmのタイヤ外端
      クリアランスを通路壁として認識できるようにした。
    - TURN_CMD=210を追加し、前方赤時の片輪停止旋回をFWD_SLOW_CMD(120)より強化。
    - 片輪停止旋回にはmotorTrimを適用しない。左右どちらへ旋回する場合も、
      駆動側の片輪へ同じTURN_CMDが届くようにした。
    - 【v29で撤回】低速実走指令での適用trim±0.12制限。
    - 【要実機確認】通常走行時の左弱体化、左右両方向の旋回力、中央時の
      corridorMode認識を確認すること。

  v27: 実走で前・左・右がすべて赤の状態でも避けきれなかったため、
       三方向すべて赤なら前進タンクターンをやめ、即バックへ切り替える。

  v27 changes:
    - frontRed && leftRed && rightRedをALL_REDとして、起動後待機時間に関係なくBACKトリガへ追加。
      直前のバックからクールダウン中も前進へ戻さず、ALL_RED_WAITで停止する。
    - 前方最小距離FRONT_STOP_MM未満の従来BACK条件は維持。
    - 前だけ赤、または前+片側赤の場合は従来どおり空いている側へのタンクターンを維持。
    - 【要実機確認】ALL_RED時にLCD reasonがBACKへ変わり、後退すること。

  v26: キャリブレーション成功後の実走フィードバックを反映。軽量化した車体に対して
       速度下限が高く、前方センサーの回避反映距離も短かったため、キャリブ系は維持したまま
       実走速度と障害物判定距離だけを調整。

  v26 changes:
    - FWD_CMD 155->140、FWD_SLOW_CMD 145->120、MIN_MOVING_CMD 140->110。
      Mega側MIN_EFFECTIVE_CMDも132->100へ下げ、軽量化後の低速域を使えるようにした。
    - FRONT_SLOW_MM 120->200、FRONT_HARD_MM 75->120、FRONT_WARN_MM 100->180。
      前方障害物を従来より遠い位置から減速・回避へ反映する。
    - SEN_DANGER_MM 60->100、SEN_WARN_MM 80->160。8x8個別セルで捉えた
      細い障害物も、接触直前ではなく余裕のある距離からスコアへ反映する。
    - 3基のうち1基でも未初期化・読み取り失敗・SENSOR_STALE_MS超過なら
      「安全(スコア0)」として走らず、reason:SENSOR_WAITで停止するフェイルセーフを追加。
    - センサーポーリング順、キャリブレーション、motorTrimは変更していない。
    - 【要実機確認】最低出力100で連続走行できるか、約120〜180mmからの前方回避で
      タイヤ外端が引っかからなくなったかを確認すること。

  v24: キャリブ結果(LCD Drift表示)と実機の見た目の変針量が大きく食い違うという報告
       (表示-2.9degに対し実際は右へ5〜60度)への対応。yawDeg積分がdt>=0.2sの回を
       丸ごと捨てていたバグを修正、各パスの生ドリフト値もLCD/Serialに表示するよう追加。

  v24 changes:
    - updateIMU(): 旧実装は dt>=0.2s の回をヨー積分から完全に除外していた
      (streamPointCloud()等でループが一瞬詰まりdtが伸びると、その間の実回転が
      yawDegに一切反映されない)。dtを捨てず、上限0.5sでクランプしてから必ず積分するよう修正。
    - runStraightCalibration(): ラウンド確定時に中央値だけでなく、各パスの生ドリフト値を
      Serial(常時)とLCD(小文字表示)に出力するようにした。中央値に隠れた外れ値パスの
      有無を次回テストで直接確認できる。
    - 【要実機再確認】(1)がユーザー報告の食い違いの原因だったかは未確認。次回のBtnB
      キャリブレでLCD/Serialの各パス値と実際の変針量を突き合わせて検証すること。

  v23: ユーザーフィードバックによる調整4件。(1) v21のWiFi STA接続を撤回しSoftAP方式に戻した
       (学校WiFi等、認証情報を書き込むのが望ましくない環境を考慮)。(2) 走行速度をさらに減速
       (FWD_CMD 180->165, FWD_SLOW_CMD 165->155)。(3) キャリブレーション中の前方ToF
       スペースチェック(CAL_ABORT_FRONT_MM)を廃止し、パスは常にCAL_PASS_DRIVE_MSいっぱいまで走る。
       (4) モーター個体差を最大2倍と仮定し、TRIM_LIMITを2倍を補正しきれる値(0.25->0.34)へ
       引き上げ、そこまで自動収束できるようCAL_MAX_ROUNDSも拡大(5->12)。

  v23 changes:
    - setup()を元のWiFi.mode(WIFI_AP)+WiFi.softAP()のみに戻した。wifi_secrets.h/
      wifi_secrets.h.exampleは削除、.gitignoreのwifi_secrets.h行も削除。
    - streamPointCloud()のbeginPacket先を固定192.168.4.255に戻し、flagsのWiFiクライアント
      判定もWiFi.softAPgetStationNum()ベースに戻した(wifiLinkUp()/pointCloudBroadcastIp/
      wifiStationConnected/STATION_WIFI_WIFI_STA_CONNECT_TIMEOUT_MSは削除)。
    - runOneCalPass()からreadOneSEN(0)呼び出しとCAL_ABORT_FRONT_MMによる早期中断を削除。
      戻り値CAL_PASS_NO_SPACEは(ranMs不足という理論上の保険としてのみ)コード上残存。
    - TRIM_LIMIT: 実効左右比(1+trim)/(1-trim)が2倍に達するよう逆算しT=1/3、余裕を見て0.34へ
      (実機で旧上限0.25に張り付いたままdrift -4.7degが解消しないことを確認済み)。
      CAL_MAX_ROUNDSも5->12に拡大し、0からこの上限まで自動収束できる余地を確保。
    - 【未検証】(2)(3)(4)とも実機での効果確認はまだ行っていない。

  v22: v20で超信地旋回を全廃止した後、「前方に壁が来た時に連続ステアだけでは曲がりきれない」
       懸念(ユーザー指摘)に対応。frontRed時のみ片輪を止める急旋回(タンクターン)を追加。

  v22 changes:
    - frontScoreがOBSTACLE_RED_SCORE以上(frontRed)の場合、通常の連続ステア(2)より優先して
      片輪を0、もう片輪をFWD_SLOW_CMDにするタンクターンを実行する(reason: TURN_L/TURN_R、
      両側危険ならESCAPE_L/ESCAPE_R)。逆転はしないため「常に前進」の方針とは矛盾しない。
    - HEADING_CORR_LIMIT頭打ちの継続ステア(v20)は側方の危険度が高いだけの場合(frontは
      無事)には引き続き使う。前方が実際に危険域に入った場合のみタンクターンへ切り替わる。
    - 【未検証】実機での効き確認はまだ行っていない。

  (v21でWiFi STA接続方式を試したが、学校WiFi等の環境考慮のため撤回しSoftAP方式に戻した)

  v20: 超信地旋回(PIVOT)を全廃止。常に前進しながら、危険度・クリアランス差に応じて
       目標ヨーを連続的に「広い側/安全な側」へバイアスする方式(差動ステアのみ)に変更。

  v20 changes:
    - DS_PIVOT状態・startPivot()・PIVOT_ALIGN_PIVOT_COURSE_RETURN
      CORNER_CONTINUE_WINDOW_MS等のピボット専用ロジック/定数/変数を全削除。
    - 前方回避・側方姿勢直しの2種類のピボットトリガーを、1つの連続ステア計算に統合
      (STEER_MAX_DEG/STEER_URGENCY_MIN)。危険度(0-100スコア)に比例してtargetYawDegを
      その場でバイアスし、実際の差動はheadingCorrection()(±HEADING_CORR_LIMIT)がそのまま担う。
    - DS_BACK(前方極近の緊急バック)は維持。バック後はピボットせずそのままDS_FORWARDへ戻り、
      連続ステアに委ねる。
    - 差動ステアは超信地旋回よりも実効的な旋回半径がずっと大きいため、反応・回避の余裕を
      確保する目的でFWD_CMD(215->180)/FWD_SLOW_CMD(195->165)を減速。
    - 【未検証】この変更は静的コードレビューのみで実機での回避挙動確認はまだ行っていない。
      特に急な障害物・直角コーナーでの回避可否は要実地確認(旧ピボット方式より
      その場での方向転換能力が大幅に落ちているため、狭いコースでは壁に接触するリスクがある)。

  v19: TCA CH0/BMA400と並行して、キャリブレーション(BtnB)の進行状況を
       UDP点群フレーム経由でPC側点群ビューアーへリアルタイム配信するように変更。

  v19 changes:
    - dstateにDS_CALIBRATING(3)を追加。runStraightCalibration()/runOneCalPass()実行中は
      calStreamRound/calStreamPass/calStreamDriftDegを更新しつつstreamPointCloud()を呼ぶ。
    - PointCloudFrameにtrimMilli/calRound/calPass/calDriftCdegを追加(protocol version 4)。
    - PC側point_cloud_viewer.pyはCALIBRATIONパネルでラウンド/パス/ドリフト/トリムを表示する。

  v18: 手動キャリブレ(BtnB直進テスト)を強化。1回10秒の単発測定から、
       短いパスを複数回走って中央値を取る方式+収束するまでの自動ラウンド継続に変更。

  v18 changes:
    - CAL_DRIVE_MS(10秒)を CAL_PASSES_PER_ROUND(3)分割し、1パス約3.3秒に短縮。
      1ラウンドの合計走行距離は旧来の単発10秒とほぼ同じに保ちつつ、3回分のdriftNormを
      中央値で集約するため単発測定のノイズ・外れ値に強くなった。
    - CAL_OK_DEG未満に収束するまで、最大CAL_MAX_ROUNDS(5)ラウンドまで自動継続。
      ラウンド間はBtnBの再操作なしで自動進行はしない(後方無センサのため、
      ロボットを開始位置へ戻す必要がある。LCDでBtnB:継続/BtnA:中止を案内する)。
    - パス中にCAL_ABORT_FRONT_MM(前方近接=コース不足)で止まった場合と、
      BtnA押下による明示的な中止を区別。前者はそのラウンドを打ち切って
      それまでに取れた有効パスだけで中央値を計算(0件なら「CAL FAILED」表示)、
      後者はキャリブレ全体を即座に中止する(ユーザーの明示操作を尊重)。
    - 【未検証】このロジックは静的コードレビューのみで実機での収束確認はまだ行っていない。

  v17: TCA CH0(予約チャンネル)にBMA400(Grove)を追加。点群ビューアーの
       デッドレコニング用に、M5内蔵IMUと並行して加速度をUDP配信する。

  v17 changes:
    - 【実機仕様】BMA400モジュールはコネクタが車体後ろ向きに取付(2026-07-22ユーザー確認)。
      Seeed公式データシート図の座標系(Z=チップ面から手前/垂直上, Y=コネクタと反対方向,
      X=Y×Zの右手系)と照合すると、水平・チップ面上向き取付の前提で
      Y軸=車体前方、X軸=車体右方向に一致し、符号反転や軸入替は不要と判断した
      (Web上のSeeed製品ページの軸図から導出。実機での重力方向テストは未実施)。
    - initBMA400()/readBMA400()を追加。TCA CH0経由、50ms間隔でポーリング。
    - PointCloudFrameにbmaAccelX/Y/ZmilliGを追加(protocol version 3)。
      M5内蔵IMUのaccelXmilliG等と同じmilli-g単位で、両方をPC側で比較・ブレンドできるようにした。
    - BMA400未接続/未初期化でも既存動作に影響しないようフォールバック
      (bmaOK=falseなら該当フィールドは0、flagsのBMA400ビットも立てない)。

  v16: 周回コーナー旋回対応、kick瞬間反転バグ修正(Mega側)、BACK_COOLDOWN無効化バグ修正、
       前方回避方向選択のクリアランス化、corridorMode優先度見直し、各種閾値調整

  v16 changes:
    - 【実測データ】SEN0628の実視野角はDFRobot公称60°(3.5m測距、20-200mm域で
      ±11-12mm精度、最大60Hz)。0/+50/-50degの3基構成だと隣接センサ間で約10度の
      重なりがあり、-80度〜+80度の範囲に角度的な死角はないことを確認した
      (Web検索で確認。実機のFOVそのものは未実測)。
    - 【実測データ】ロボット車体は前方が1〜2度高くなる姿勢で静止する(センサー自体は
      車体に対して水平マウント)。この傾きぶん、全センサーのビームは実際の水平面より
      わずかに上を向く。近距離(数百mm以内)では影響は小さいが、長い直線区間の遠方で
      低い壁を飛び越えて「前方クリア」と誤判定する可能性は理論上ある。センサー高さ・
      壁高さの実測値がないため数値的な補正は入れていない(要注意点として記録のみ)。
    - コーナー(90度)旋回に対応。前方回避ピボットが上限角/タイムアウトで終了(=前方が
      まだ開いていない)した場合、courseReturnActiveで元の(=今避けた壁の)方位に
      戻ろうとするのをやめ、新しいヨーを直進基準として確定するよう変更。
      同時に、同方向への継続旋回(PIVOT_MAX_DEG_CORNER=70度)を追加。
    - 前方回避ピボットの方向選択に、側方クリアランス(leftClear/rightClear)の差が
      大きい場合はクリアランスが広い側を優先する分岐を追加(従来はred/not-redの
      2値と交互選択のみだった)。
    - corridorModeに連続ミス回数によるヒステリシスを追加(瞬間的な障害物検知での
      フリッカーを防止)。また、壁を再検知できたらcourseReturnActiveより
      corridorModeの壁基準中央維持を優先するよう変更(ヨードリフトの影響緩和)。
    - Mega側 AM_HighTorqueAntiStall_..._v4_for_x3.ino: kick機構(発進補助)が
      符号反転のたびに瞬間フルパワー反転を起こすバグを修正(v2参照)。
    - M5側 BACK_COOLDOWN_MSが論理式の冗長ORで無効化されていたバグを修正。
    - POLL_SEQを前方偏重(4:2:2)から均等(1:1:1)へ。側方センサ(通路追従・
      コーナー方向判断の主データ)の鮮度を優先。
    - 前方危険判定の中央列範囲を4列->6列に拡張(中央から外れた細い障害物対策)。
    - 低validCount時の警戒スコアを18->40に引き上げ(減速閾値に届くように)。
    - SIDE_ALIGN_DELAY_MSを1000->400msに短縮(前方回避クールダウン300msとの
      非対称で側方姿勢直しが後回しになっていたのを緩和)。
    - 左右モーター補正をmotorTrimに一本化。
      AUTO_TRIM_RATEを5倍に増加(収束が遅すぎたため)。
    - 前進速度を2値切替から frontScore/側方scoreに応じた連続減速に変更。
    - CORRIDOR_DIFF_DEADBAND_MMを50->30mmに縮小(中央維持精度向上)。
    - 未使用関数 senForwardMm/senForwardMinMm を削除。
    - 【未検証】上記は静的なコードレビューに基づく修正であり、実機での動作確認は
      行っていない。特にCAL_TRIM_GAIN/AUTO_TRIM_RATE/PIVOT_MAX_DEG_CORNER/
      各種閾値の具体的な数値は実走行での再調整が前提。

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

  Sensor layout (v10/v11, 実機確認済み。v17でCH0にBMA400を追加):
    TCA CH0: BMA400 (Grove, I2C 0x15) - コネクタは車体後ろ向き
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
#include <BMA400.h>  // v17: TCA CH0のBMA400(Grove)。グローバルインスタンス bma400 はライブラリ側で定義済み

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
// v16: このコースは常に両側壁があり、側方センサは通路追従(壁基準の中央維持・
// コーナー旋回方向判断)の主要データでもあるため、前方偏重(4:2:2)から均等(1:1:1)に
// 変更。前方の安全マージンはFRONT_STOP_MM等の即時トリガで別途担保されており、
// ポーリング比率を下げても即時反応性は変わらない。
// (実センサのI2C読み取り所要時間は未計測。安全マージンに余裕が無いと分かれば
//  前方寄りの比率に戻すこと)
const uint8_t POLL_SEQ[] = {0, 1, 2, 0, 1, 2};
const uint8_t POLL_SEQ_LEN = sizeof(POLL_SEQ) / sizeof(POLL_SEQ[0]);
uint8_t pollSeqIndex = 0;

// ===============================
// BMA400 (Grove, TCA CH0) - v17
// ===============================
// コネクタが車体後ろ向きに実装されている(2026-07-22ユーザー確認)。Seeed製品ページの
// センサー座標系シルク図(Z=チップ面から手前=垂直上、Y=コネクタと反対方向、
// X=Y×Zの右手系)と照合すると、水平・チップ面上向き取付の前提でX軸=車体右、
// Y軸=車体前方に一致する。よってgetAcceleration()の生値(mg)はそのまま
// 「右方向/前方向」として使える(符号反転・軸入替は不要と判断)。
// ただし実機での重力方向テスト(静止時Z≒+1000mg等)による裏取りはまだ行っていない。
const uint8_t BMA400_TCA_CH = 0;
const unsigned long BMA400_READ_PERIOD_MS = 50;  // 点群配信(50ms)と同程度で十分

bool bmaOK = false;
float bmaAccelXG = 0.0f;  // 車体右方向, g単位(getAcceleration()のmgを1000で除算)
float bmaAccelYG = 0.0f;  // 車体前方向, g単位
float bmaAccelZG = 0.0f;
unsigned long bmaUpdatedMs = 0;
unsigned long lastBmaReadMs = 0;

void initBMA400() {
  useExternalI2C();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("INIT BMA400");

  if (!tcaSelect(BMA400_TCA_CH)) {
    tcaDisableAll();
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.println("TCA NG");
    delay(300);
    return;
  }

  if (!i2cExists(BMA400_ADDRESS)) {
    Serial.println("BMA400 not found on CH0");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.println("not found");
    tcaDisableAll();
    delay(300);
    return;
  }

  bma400.initialize();
  bmaOK = bma400.isConnection();
  tcaDisableAll();

  M5.Lcd.setTextColor(bmaOK ? GREEN : RED, BLACK);
  M5.Lcd.println(bmaOK ? "OK" : "init NG");
  delay(300);
}

void readBMA400() {
  if (!bmaOK) return;

  unsigned long now = millis();
  if (now - lastBmaReadMs < BMA400_READ_PERIOD_MS) return;
  lastBmaReadMs = now;

  useExternalI2C();
  if (!tcaSelect(BMA400_TCA_CH)) {
    tcaDisableAll();
    return;
  }

  float x = 0, y = 0, z = 0;
  bma400.getAcceleration(&x, &y, &z);  // mg
  tcaDisableAll();

  bmaAccelXG = x / 1000.0f;
  bmaAccelYG = y / 1000.0f;
  bmaAccelZG = z / 1000.0f;
  bmaUpdatedMs = now;
}

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
// Drive tuning (v20: 常時前進+連続ステア方式。pivot-turnは廃止)
// ===============================
// 前進は常に高デューティ。低デューティ差動は使わない(曲がらない+ストールするため)。
// v20: 超信地旋回を廃止し差動ステアのみで避けるようになったため、旧値(215/195)より
// 全体的に減速した。差動ステアはheadingCorrection()の±HEADING_CORR_LIMITでしか
// 曲がれず、旧来の超信地旋回より旋回半径がずっと大きいため、反応・回避の余裕を
// 確保する目的。v23: ユーザー指摘によりさらに減速(180/165->165/155)。
// v26: 車体軽量化後も速すぎてタイヤ外端が引っかかる実走結果を受けて再減速。
// MIN_MOVING_CMDはAM側MIN_EFFECTIVE_CMD(100)を下回らない110とする。
const int FWD_CMD = 165;
const int FWD_SLOW_CMD = 135;
const int MIN_MOVING_CMD = 110;
const int MAX_MOVING_CMD = 252;
const int TURN_CMD = 210;       // v28: 前方赤時の片輪停止旋回専用。通常減速値より強くする
const bool CLOCKWISE = true;    // v33: true=右回り(時計回り)、false=左回り(反時計回り)
const float CORNER_TURN_DEG = 90.0f;
const float CORNER_TURN_DONE_DEG = 5.0f;
const unsigned long CORNER_TURN_TIMEOUT_MS = 3000;
const unsigned long CORNER_TURN_COOLDOWN_MS = 500;
const unsigned long CORNER_CONFIRM_MS = 200;
const uint8_t CORNER_CONFIRM_FRAMES = 2;
const int CORNER_FRONT_X_MIN_MM = 70;
const int CORNER_FRONT_X_MAX_MM = 320;
const int CORNER_FRONT_BIN_MM = 25;
const int CORNER_FRONT_MIN_SPAN_MM = 140;
const int CORNER_FRONT_MIN_POINTS = 8;
const int CORNER_FRONT_Y_BIN_MM = 50;
const int CORNER_FRONT_MIN_CONTIGUOUS_MM = 400;
const int CORNER_SIDE_Y_MIN_MM = 80;
const int CORNER_SIDE_Y_MAX_MM = 600;
const int CORNER_SIDE_BIN_MM = 25;
const int CORNER_SIDE_MIN_SPAN_MM = 50;
const int CORNER_SIDE_MIN_POINTS = 6;
const int CORNER_INTERSECTION_TOL_MM = 50;
const int CENTER_OBS_SCORE_DELTA_MAX = 15;
const int CENTER_OBS_CLEARANCE_DELTA_MAX_MM = 50;
const int VEHICLE_WIDTH_MM = 300;  // v44: 規定回避側が車幅未満なら反対側へ逃がす判定基準
const float COURSE_RETURN_DONE_DEG = 3.0f;
const unsigned long COURSE_RETURN_TIMEOUT_MS = 2000;

// v20: 超信地旋回(PIVOT)を廃止。常に前進しながら、危険度・クリアランス差に応じて
// 目標ヨー(targetYawDeg)を連続的に「広い側/安全な側」へバイアスする方式に一本化。
// 実際の差動量は既存のheadingCorrection()(±HEADING_CORR_LIMIT)がそのまま担う。
const float STEER_MAX_DEG = 35.0f;      // 最大緊急度でのヨーバイアス(HEADING_CORR_LIMITを飽和させる値)
const int   STEER_URGENCY_MIN = 15;     // v25: 小さな左右差へ少し早く反応する

// テスト時に確認した閾値へ安全余裕40mmを加え、正式な運用値とする。
const int FRONT_SLOW_MM       = 320;  // v42: 増速と半透明板対策で200->320

// v16: 連続ステアの方向選択で、leftClear/rightClearの差がこれ以上あれば
// 「クリアランスが広い側」を優先する(赤フラグだけでは判断できない際どい局面向け)。
const int CLEARANCE_DIR_BIAS_MM = 20;

// ===============================
// Motor L/R calibration (v9)
// ===============================
// motorTrim > 0: 右モータを増幅 / 左を減衰 (直進で右へ流れる=右が弱い個体の補正)
// motorTrim < 0: その逆
// 送信直前に L*(1-trim), R*(1+trim) を適用する。
// 注意: AM側 MIN_EFFECTIVE_CMD(100)の底上げがあるため、強いtrimで片側の送信値が
//       100未満になると、その側はMega側で100へ底上げされる。
const int   CAL_DRIVE_CMD = MAX_MOVING_CMD;  // 最大速度で左右差を計測
const unsigned long CAL_DRIVE_MS = 10000;    // 1ラウンドの合計走行時間(=旧来の単発10秒と同じ基準)
// v23: ユーザー指摘によりキャリブ中の前方ToFスペースチェックを廃止したため、
// CAL_ABORT_FRONT_MMは未使用になった(コース長は十分確保されている前提で運用する)。

// v18: 複数パス平均+自動ラウンド継続。
const uint8_t CAL_PASSES_PER_ROUND = 3;  // 1ラウンドをこの回数に分割し、中央値で外れ値を除去
const unsigned long CAL_PASS_DRIVE_MS = CAL_DRIVE_MS / CAL_PASSES_PER_ROUND;  // 1パスあたりの走行時間
const unsigned long CAL_PASS_MIN_VALID_MS = CAL_PASS_DRIVE_MS / 2;  // これ未満で中断したパスは無効
// v23: モーター個体差は最大2倍(片側が反対側の2倍出力)を想定して設計。
// CAL_MAX_ROUNDSは、trim=0からTRIM_LIMIT(下記)まで自動収束しきれるよう
// 5->12へ拡大(1ラウンドあたりの所要時間は変わらないため、単純に収束の粘りが増す)。
const uint8_t CAL_MAX_ROUNDS = 12;  // 収束しない場合の自動打ち切り上限
// v16: 0.0035->0.005 初回キャリブの収束を早める(要実走行での再調整)。
const float CAL_TRIM_GAIN = 0.005f;          // ヨードリフト1degあたりのトリム修正量
// v23: 送信直前の適用式は sendL=left*(1-trim), sendR=right*(1+trim) であり、
// 左右の実効比は (1+trim)/(1-trim)。モーター個体差の最大値を2倍と仮定すると、
// これを完全補正するのに必要なtrimは (1+T)/(1-T)=2 を解いて T=1/3≒0.333。
// 実測ノイズ・丸め誤差ぶんの余裕を見て0.34とした(この時点で比2.03倍まで補正可能)。
// 【要実機再確認】ユーザー実機で3回連続+0.250(旧上限)に張り付いたままdrift -4.7degが
// 解消しないことを確認済み。0.34でも足りない場合は
// モーター/ギア/配線側の物理的な非対称を疑うこと。
const float TRIM_LIMIT = 0.34f;
const float CAL_OK_DEG = 4.0f;               // この範囲に収まれば合格表示
// 左右差はNVSへ保存されたmotorTrimだけで補正する。

const bool  AUTO_TRIM_ENABLE = false;        // v29: NVS保存済みキャリブ値だけを使い、走行中は学習しない
// v16: 0.00004->0.00020 収束が遅すぎたため5倍に。振動的になるようなら下げること。
const float AUTO_TRIM_RATE = 0.00020f;       // 補正量1あたり/周期の学習率
const unsigned long TRIM_SAVE_PERIOD_MS = 20000;

float motorTrim = 0.0f;
float savedTrim = 0.0f;

// v19: キャリブレ進行状況をUDP点群フレームへ載せるための状態(streamPointCloud()が参照)。
// キャリブレ中以外は0のまま。
uint8_t calStreamRound = 0;
uint8_t calStreamPass = 0;
float calStreamDriftDeg = 0.0f;
Preferences drivePrefs;
unsigned long lastTrimSaveMs = 0;
bool nvsResetComboLatched = false;

// Very close recovery: back first, then pivot away.
const int BACK_L_CMD = -215;
const int BACK_R_CMD = -215;
const unsigned long FORCE_BACK_MS = 3000;  // v46: 前方極近時は最低3秒後退
const unsigned long CORNER_BACK_MS = 1600; // v41: 四角旋回後の引っ掛かり防止で3秒->2秒
const unsigned long BACK_CLEAR_CONFIRM_MS = 300; // v46: バック後、前方赤解除の連続確認時間
const int BACK_ARC_SLOW_CMD = -80;          // Mega側の最低値により実効約-100

// v10: 水平マウントになったため上向きチルト補正は廃止。
// センサ取付面の凹み等がある場合の微調整用に残す(通常0でよい)。
const int SEN_MOUNT_OFFSET_MM = 0;

// If SEN valid points are sparse, do not trust it as "safe".
// v16: 18->40 (減速閾値35を超える値に引き上げ。低反射率/端の欠測で
// validCountが少ない=「見えていないだけで安全とは限らない」状況を、
// 従来は事実上無視していたため、最低限の減速はかかるようにする)
const int SEN_LOW_VALID_CAUTION_SCORE = 50; // v42: 弱反射・少数有効点をより強く警戒

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
const int CORRIDOR_MAX_WALL_MM = 420;  // v37: 幅1m・車幅30cmの中央時クリアランス約350mmを含める
const int CORRIDOR_DETECT_SCORE = 25;
const int CORRIDOR_DIFF_DEADBAND_MM = 30; // v16: 50->30 中央維持精度を上げるため縮小(要実走行確認)
const float CORRIDOR_YAW_STEP_DEG = 0.35f;   // 1制御周期あたりの目標ヨー移動量(最大)
const float CORRIDOR_YAW_CLAMP_DEG = 22.0f;  // 現在ヨーからの目標乖離上限
// v16: 障害物の瞬間検知でcorridorModeがフリッカーするのを防ぐヒステリシス。
// 連続でこの回数だけ壁未検知が続いた場合のみ実際にcorridorModeを解除する。
const int CORRIDOR_MISS_STREAK_LIMIT = 3;

// v11: BODY_*_MARGIN/EXTRA_SAFETYの加算式のままだと調整の見通しが悪いため、
// 実測スケール(壁10〜50cm)向けの絶対値として明示。
// マージン定数は「ここから何mm余裕を持たせたか」の内訳表示用に残す。
const int FRONT_STOP_MM = 60;   // テスト値20 + 40mm: very close -> stop/back
const int FRONT_HARD_MM = 180;  // v42: 120->180
const int FRONT_WARN_MM = 280;  // v42: 180->280

// v23: ユーザー指摘により左右ToF閾値を2倍の距離へ(60/80mm -> 120/160mm)。
// より遠い距離から側方の危険/警戒スコアが立ち上がるようになり、回避の反応が早まる。
const int SIDE_DANGER_MM = 160; // v45: 常時赤対策で200->160
const int SIDE_WARN_MM   = 250; // v45: 幅1mコース中央の通常壁を赤にしないよう320->250

const int SEN_DANGER_MM  = 160; // v42: 100->160
const int SEN_WARN_MM    = 280; // v42: 160->280
const int SEN_SIDE_DANGER_MM = 130; // v45: 左右8x8点群専用。前方の半透明板感度は維持
const int SEN_SIDE_WARN_MM   = 230;

const int SEN_DANGER_NEAR_COUNT = 3;   // v7: 1->3 ノイズ1セルで即ブロック判定しない
const int SEN_WARN_NEAR_COUNT = 3;
const int SEN_EDGE_NEAR_COUNT = 2;     // v30: 視野端は面積が狭いため2セルで反応
const int SEN_ACTIVE_ROW_FIRST = 2;    // v31: 上下端の床/車体反射を除き中央4行だけ使う
const int SEN_ACTIVE_ROW_LAST = 5;
const int SEN_FRONT_ROW_FIRST = 1;     // v48: 前方だけ床側row 4〜5を除外
const int SEN_FRONT_ROW_LAST = 3;
const float SEN_HORIZONTAL_FOV_DEG = 60.0f;
const float SEN_PIXEL_STEP_DEG = SEN_HORIZONTAL_FOV_DEG / 8.0f;
const float SEN_FIRST_COL_OFFSET_DEG = SEN_HORIZONTAL_FOV_DEG / 2.0f - SEN_PIXEL_STEP_DEG / 2.0f;
const int OBSTACLE_RED_SCORE = 60;      // v28: 70->60(3/5)。LCD赤表示と回避開始を共通化

const unsigned long SENSOR_STALE_MS = 620;

// IMU correction
const float HEADING_KP = 1.30f;        // v25: 1.15->1.30 左右ステアの追従を鋭敏化
const int HEADING_CORR_LIMIT = 28;     // v25: 22->28 最大左右差動を拡大
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
float imuGyroXDps = 0.0f;
float imuGyroYDps = 0.0f;
float imuGyroZDps = 0.0f;
bool builtinImuDataOK = false;
int builtinImuInitResult = -999;

float lastImpactG = 0.0f;
bool impactLike = false;
int impactConsecutive = 0;  // v14: 単発ノイズ除外用の連続超過カウント
bool accelWarn = false;

unsigned long lastAvoidMs = 0;
bool justExitedAvoid = false;

unsigned long forceBackUntilMs = 0;
unsigned long backClearSinceMs = 0;
unsigned long autoStartMs = 0;
unsigned long lastBackEndMs = 0;        // バック完了時刻(クールダウン用)
unsigned long lastCmdSignChangeMs = 0;  // 直近の発進/反転指令時刻(キック揺れ無視用)

// v8 state machine
enum DriveState : uint8_t { DS_FORWARD = 0, DS_CORNER_TURN, DS_BACK, DS_CALIBRATING };

// v18: runOneCalPass()の戻り値。Arduinoの自動プロトタイプ生成はinclude直後の固定位置に
// 全関数プロトタイプを挿入するため、独自enum/structを戻り値/引数型に使うと(ファイル内の
// どこで定義しても)その挿入位置より後になり「型が見つからない」エラーになる。
// 組み込み型(uint8_t)+定数にすることでこの問題を回避する。
const uint8_t CAL_PASS_OK = 0;
const uint8_t CAL_PASS_USER_ABORT = 1;
const uint8_t CAL_PASS_NO_SPACE = 2;
DriveState dstate = DS_FORWARD;
DriveState lastDrawnDstate = DS_FORWARD;  // v15: 状態変化時の強制再描画用

int backRecoverDir = 1;            // バック後、直後の連続ステアがどちらへ効きやすいかの参考値(未使用でも安全)
bool cornerEscapeActive = false;
float cornerTurnTargetYawDeg = 0.0f;
bool cornerTurnTargetPrepared = false;
unsigned long cornerTurnStartMs = 0;
unsigned long cornerTurnCooldownUntilMs = 0;
unsigned long frontRedSinceMs = 0;
unsigned long lastCornerFrontSampleMs = 0;
uint8_t frontRedConfirmFrames = 0;
float avoidStraightYawDeg = 0.0f;
bool avoidStraightCaptured = false;
bool courseReturnActive = false;
unsigned long courseReturnStartMs = 0;

// v20: 超信地旋回廃止に伴う連続ステア用の状態。
// 危険度・クリアランスから方向を判断できない(スコア同値等)場合のtie-break用に、
// 直前に実際にステアした方向を覚えておく。
int lastSteerDir = 1;               // +1 = left, -1 = right

// v16: corridorModeのフリッカー防止用ヒステリシスカウンタ
int corridorMissStreak = 0;

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
  // v29: 通常走行もキャリブレーションと同じNVS保存値を全量適用する。
  // 片輪停止/強旋回だけは左右方向で旋回力が変わらないようtrimを適用しない。
  float appliedTrim = motorTrim;
  bool oneWheelTurn = (left == 0) != (right == 0);
  bool hardTurn = oneWheelTurn || abs(left - right) > 120;
  if (hardTurn) {
    appliedTrim = 0.0f;
  }

  int sendL = (int)lroundf(left  * (1.0f - appliedTrim));
  int sendR = (int)lroundf(right * (1.0f + appliedTrim));
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
  // v47: getAllData()が全セルを更新しなかった場合に、前フレームの近距離値が
  // raw[]へ残って赤判定を継続しないよう、読取前に64セルを無効値(0)へクリアする。
  // validMatrixMm()は20mm未満を無効扱いするため、未更新セルは集計に入らない。
  memset(out.raw, 0, sizeof(out.raw));
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

  uint16_t frameRaw[64];
  memset(frameRaw, 0, sizeof(frameRaw));
  uint8_t readResult = senDev[idx]->getAllData(frameRaw);
  tcaDisableAll();
  if (readResult != 0) {
    out.updatedMs = millis();
    return;
  }
  memcpy(out.raw, frameRaw, sizeof(out.raw));

  int minVal = 99999;
  long centerSum = 0;
  int centerCount = 0;
  const uint8_t frontCenterIdx[4] = {19, 20, 27, 28}; // row 2〜3
  const uint8_t sideCenterIdx[4] = {27, 28, 35, 36};  // row 3〜4
  const uint8_t *centerIdx = (idx == 0) ? frontCenterIdx : sideCenterIdx;

  for (int k = 0; k < 64; k++) {
    uint16_t v = out.raw[k];
    if (!validMatrixMm(v)) continue;

    int row = k / 8;
    int firstRow = (idx == 0) ? SEN_FRONT_ROW_FIRST : SEN_ACTIVE_ROW_FIRST;
    int lastRow = (idx == 0) ? SEN_FRONT_ROW_LAST : SEN_ACTIVE_ROW_LAST;
    if (row < firstRow || row > lastRow) continue;

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
  // M5StickCPlusライブラリのMPU6886はWire1(GPIO21/22)を直接使用する。
  // 外部センサー用Wireの切替は内蔵IMU読取には不要。
  M5.IMU.getAccelData(&ax, &ay, &az);
  M5.IMU.getGyroData(&gx, &gy, &gz);
  if (fabs(ax) > 0.0001f || fabs(ay) > 0.0001f || fabs(az) > 0.0001f ||
      fabs(gx) > 0.0001f || fabs(gy) > 0.0001f || fabs(gz) > 0.0001f) {
    builtinImuDataOK = true;
  }
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
  backClearSinceMs = 0;
  lastForceBack = false;
  dstate = DS_FORWARD;
  cornerEscapeActive = false;
  cornerTurnStartMs = 0;
  cornerTurnTargetPrepared = false;
  cornerTurnCooldownUntilMs = 0;
  frontRedSinceMs = 0;
  lastCornerFrontSampleMs = 0;
  frontRedConfirmFrames = 0;
  avoidStraightYawDeg = 0.0f;
  avoidStraightCaptured = false;
  courseReturnActive = false;
  courseReturnStartMs = 0;
  lastBackEndMs = 0;
  autoStartMs = millis();
  lastAvoidMs = millis();
  justExitedAvoid = false;
  lastSteerDir = 1;
  corridorMissStreak = 0;

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
  imuGyroXDps = gx;
  imuGyroYDps = gy;
  imuGyroZDps = gz;

  if (lastImuMs == 0) {
    lastImuMs = now;
  }

  float dt = (now - lastImuMs) / 1000.0f;
  lastImuMs = now;

  // v24: 旧実装は dt>=0.2s の回はヨー積分を丸ごとスキップしていた(その間の実回転を
  // 完全に失う)。streamPointCloud()等でループが一瞬詰まりdt>=0.2sになると、
  // その間ロボットが実際に曲がっていてもyawDegに反映されず、キャリブ結果の
  // ドリフトが実際の変針量より大幅に小さく出る不具合があった(ユーザー実機で
  // 報告: LCD表示-2.9degに対し実際は右へ5〜60度変針)。
  // dtを捨てず、異常値対策として上限0.5sでクランプしてから必ず積分する。
  if (imuZeroed && dt > 0) {
    float safeDt = min(dt, 0.5f);
    yawDeg += gz * safeDt;
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

void resetDriveNVS() {
  sendAMStop();
  driveEnabled = false;
  drivePrefs.clear();
  motorTrim = 0.0f;
  savedTrim = 0.0f;
  lastTrimSaveMs = millis();

  Serial.println("NVS drive reset: trim=0");
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.println("NVS RESET");
  M5.Lcd.println("Trim +0.000");
  M5.Lcd.println("Release A+B");
}

// v18: 1パス(CAL_PASS_DRIVE_MS)だけ直進走行し、CAL_DRIVE_MS基準に正規化した
// driftNormを返す。BtnA押下(ユーザー中止)と前方近接(コース不足)を区別して返す。
// v23: ユーザー指摘によりキャリブ中の前方ToFスペースチェックを無効化。
// コース長は十分確保されている前提で、パスは常にCAL_PASS_DRIVE_MSいっぱいまで走る
// (CAL_ABORT_FRONT_MMによる早期中断はしない)。
uint8_t runOneCalPass(float &driftNormOut) {
  yawDeg = 0.0f;
  targetYawDeg = 0.0f;
  smoothCmdLeft = 0;
  smoothCmdRight = 0;

  unsigned long t0 = millis();
  bool userAborted = false;

  while (millis() - t0 < CAL_PASS_DRIVE_MS) {
    M5.update();
    if (M5.BtnA.wasPressed()) { userAborted = true; break; }

    float ax, ay, az, gx, gy, gz;
    readBuiltinIMU(ax, ay, az, gx, gy, gz);
    updateIMU(ax, ay, az, gx, gy, gz);

    if (millis() - lastAMCommandMs >= AM_COMMAND_PERIOD_MS) {
      sendAMMotorRaw(CAL_DRIVE_CMD, CAL_DRIVE_CMD);  // トリムはsendAMMotorRaw内で適用
    }
    streamPointCloud();  // v19: キャリブ中もPC点群ビューアーへ進行状況を配信(内部で間引き済み)
    delay(2);
  }

  unsigned long ranMs = millis() - t0;
  sendAMStop();

  if (userAborted) return CAL_PASS_USER_ABORT;
  if (ranMs < CAL_PASS_MIN_VALID_MS) return CAL_PASS_NO_SPACE;

  // CAL_DRIVE_MS(1ラウンド全体の基準時間)換算に正規化。パスを分割してもCAL_TRIM_GAINの
  // 意味(ドリフト1degあたりの補正量)が変わらないようにするため。
  driftNormOut = yawDeg * ((float)CAL_DRIVE_MS / (float)ranMs);
  return CAL_PASS_OK;
}

// 小規模配列(CAL_PASSES_PER_ROUND件)の中央値。外れ値1件を自動的に無視できる。
float medianOfDrifts(float *values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    float key = values[i];
    int j = (int)i - 1;
    while (j >= 0 && values[j] > key) { values[j + 1] = values[j]; j--; }
    values[j + 1] = key;
  }
  if (count % 2 == 1) return values[count / 2];
  return (values[count / 2 - 1] + values[count / 2]) * 0.5f;
}

// 停止中にBtnBを押すと実行。最大速度で短い直進パスをCAL_PASSES_PER_ROUND回走り、
// 中央値ドリフトからtrimを更新。CAL_OK_DEGに収まるまで最大CAL_MAX_ROUNDSラウンド、
// ラウンド間はロボットの手動リポジション(後方無センサのため自動バックはしない)を挟んで続ける。
void runStraightCalibration() {
  sendAMStop();

  dstate = DS_CALIBRATING;
  calStreamRound = 0;
  calStreamPass = 0;
  calStreamDriftDeg = 0.0f;
  setReason("CAL_ZERO");
  streamPointCloud();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.println("CAL STRAIGHT");
  M5.Lcd.println("MAX / auto-round");
  M5.Lcd.println("Clear path!");
  M5.Lcd.println("Keep still...");

  // IMUオフセット取り直し(ラウンド全体で1回。走行開始はしない)
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
  lastImuMs = millis();
  imuZeroed = true;

  bool cancelled = false;

  for (uint8_t round = 1; round <= CAL_MAX_ROUNDS && !cancelled; round++) {
    float drifts[CAL_PASSES_PER_ROUND];
    uint8_t validCount = 0;
    bool noSpaceHit = false;

    calStreamRound = round;

    for (uint8_t pass = 0; pass < CAL_PASSES_PER_ROUND; pass++) {
      calStreamPass = pass + 1;
      char reasonBuf[16];
      snprintf(reasonBuf, sizeof(reasonBuf), "CAL R%d P%d/%d", round, pass + 1, CAL_PASSES_PER_ROUND);
      setReason(reasonBuf);

      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 6);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("ROUND %d PASS %d/%d\n", round, pass + 1, CAL_PASSES_PER_ROUND);

      float driftNorm = 0.0f;
      uint8_t result = runOneCalPass(driftNorm);

      if (result == CAL_PASS_USER_ABORT) { cancelled = true; break; }
      if (result == CAL_PASS_NO_SPACE) { noSpaceHit = true; break; }

      drifts[validCount++] = driftNorm;
      delay(300);  // パス間の短い間(急停止直後の余韻を切る程度)
    }

    calStreamPass = 0;

    if (cancelled) {
      setReason("CAL_CANCEL");
      streamPointCloud();
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 6);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.println("CAL CANCELLED");
      delay(1500);
      break;
    }

    if (validCount == 0) {
      setReason("CAL_FAILED");
      streamPointCloud();
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 6);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.println("CAL FAILED");
      M5.Lcd.println("No space?");
      delay(1500);
      break;
    }

    // v24: 中央値だけでなく各パスの生値も残す(外れ値の有無を判別できるようにするため)。
    Serial.print("CAL round "); Serial.print(round); Serial.print(" raw drifts:");
    for (uint8_t i = 0; i < validCount; i++) { Serial.print(" "); Serial.print(drifts[i], 1); }
    Serial.println();

    float aggregated = medianOfDrifts(drifts, validCount);
    calStreamDriftDeg = aggregated;

    // drift>0 = 左へ流れた = 左が弱い/右が強い -> 左を強める = trimを負方向へ
    motorTrim = constrain(motorTrim - aggregated * CAL_TRIM_GAIN, -TRIM_LIMIT, TRIM_LIMIT);
    saveTrim();

    bool good = fabs(aggregated) < CAL_OK_DEG;
    setReason(good ? "CAL_OK" : "CAL_RETRY");
    streamPointCloud();

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 6);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(good ? GREEN : YELLOW, BLACK);
    M5.Lcd.printf("R%d: %s\n", round, good ? "CAL OK" : "RETRY");
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("Drift %+.1f deg (%d/%d)\n", aggregated, validCount, CAL_PASSES_PER_ROUND);
    M5.Lcd.printf("Trim  %+.3f\n", motorTrim);
    if (noSpaceHit) M5.Lcd.println("(space ran out)");

    // v24: 各パスの生ドリフトを小さめ文字で並べて表示(中央値に隠れた外れ値を確認するため)。
    M5.Lcd.setTextSize(1);
    for (uint8_t i = 0; i < validCount; i++) {
      M5.Lcd.printf("P%d:%+.1f ", i + 1, drifts[i]);
    }
    M5.Lcd.println();
    M5.Lcd.setTextSize(2);

    if (good) { delay(2400); break; }

    if (round >= CAL_MAX_ROUNDS) {
      setReason("CAL_NOCONV");
      streamPointCloud();
      delay(600);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.println("NOT CONVERGED");
      delay(2000);
      break;
    }

    // 次ラウンドの前にロボットを開始位置へ戻してもらう。後方センサが無いため
    // 自動バックでの復帰はせず、必ずユーザー操作を待つ。
    setReason("CAL_WAIT");
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.println("Reposition robot");
    M5.Lcd.println("B:continue A:stop");
    while (true) {
      M5.update();
      if (M5.BtnA.wasPressed()) { cancelled = true; break; }
      if (M5.BtnB.wasPressed()) break;
      streamPointCloud();
      delay(20);
    }
    if (cancelled) {
      setReason("CAL_CANCEL");
      streamPointCloud();
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 6);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.println("CAL CANCELLED");
      delay(1500);
    }
  }

  dstate = DS_FORWARD;
  cornerEscapeActive = false;
  cornerTurnStartMs = 0;
  cornerTurnTargetPrepared = false;
  cornerTurnCooldownUntilMs = 0;
  frontRedSinceMs = 0;
  lastCornerFrontSampleMs = 0;
  frontRedConfirmFrames = 0;
  avoidStraightCaptured = false;
  courseReturnActive = false;
  courseReturnStartMs = 0;
  calStreamRound = 0;
  calStreamPass = 0;

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("A: START");
  M5.Lcd.println("B: CAL");
  M5.Lcd.println("A+B: NVS RESET");
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
// (前方(idx0)自体はforward=centerそのままなのでsenEffectiveCenter(0)を直接使えばよく、
//  かつてここにあったsenForwardMm/senForwardMinMmはどこからも呼ばれていなかったため
//  v16で削除した)
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

    score = max(score, obstacleScore(effMin, SEN_SIDE_WARN_MM, SEN_SIDE_DANGER_MM));
    score = max(score, obstacleScore(effCenter, SEN_SIDE_WARN_MM + 120, SEN_SIDE_DANGER_MM + 40));

    // v30: 視野の左右端2列は中央集計に埋もれやすいため専用に数える。
    // 3基の視野が重なる継ぎ目や細い障害物を、少ないセル数でも回避へ反映する。
    int dangerCount = 0;
    int warnCount = 0;
    int edgeDangerCount = 0;
    int edgeWarnCount = 0;
    for (int row = SEN_ACTIVE_ROW_FIRST; row <= SEN_ACTIVE_ROW_LAST; row++) {
      for (int col = 0; col < 8; col++) {
        uint16_t raw = cachedSen[idx].raw[row * 8 + col];
        if (!validMatrixMm(raw)) continue;
        int mm = max(0, (int)raw - SEN_MOUNT_OFFSET_MM);
        if (mm < SEN_SIDE_DANGER_MM) dangerCount++;
        if (mm < SEN_SIDE_WARN_MM) warnCount++;
        if (col == 0 || col == 1 || col == 6 || col == 7) {
          if (mm < SEN_SIDE_DANGER_MM) edgeDangerCount++;
          if (mm < SEN_SIDE_WARN_MM) edgeWarnCount++;
        }
      }
    }
    if (dangerCount >= SEN_DANGER_NEAR_COUNT) {
      score = max(score, 88 + min(12, dangerCount * 3));
    }
    if (warnCount >= SEN_WARN_NEAR_COUNT) {
      score = max(score, 55 + min(35, warnCount * 5));
    }
    if (edgeDangerCount >= SEN_EDGE_NEAR_COUNT) score = max(score, 90);
    if (edgeWarnCount >= SEN_EDGE_NEAR_COUNT) score = max(score, 65);

    // Sparse valid points are not proof of safety (低反射率/端の欠測など)。
    if (cachedSen[idx].validCount > 0 && cachedSen[idx].validCount <= 5) {
      score = max(score, SEN_LOW_VALID_CAUTION_SCORE);
    }
  }

  return constrain(score, 0, 100);
}

int senFrontLikeScore() {
  // 前方判定へ左右斜めセンサや前方センサの視野端を混ぜると、広い通路でも
  // 側壁を前方障害物と誤認する。前方専用センサ(idx0)の中央寄り列だけを見る。
  // v16では中央6列(1-6)だけを使い両端列を除外していたが、実走で視野の継ぎ目にある
  // 障害物への反応が弱かった。v30から全8列を使い、端列は専用カウントでも補強する。
  if (!fresh(cachedSen[0].updatedMs) || !cachedSen[0].ok) return 0;

  int score = obstacleScore(senEffectiveCenter(0), FRONT_WARN_MM, FRONT_HARD_MM);
  int dangerCount = 0;
  int warnCount = 0;
  int validCount = 0;
  int edgeDangerCount = 0;
  int edgeWarnCount = 0;

  for (int row = SEN_FRONT_ROW_FIRST; row <= SEN_FRONT_ROW_LAST; row++) {
    for (int col = 0; col < 8; col++) {
      uint16_t raw = cachedSen[0].raw[row * 8 + col];
      if (!validMatrixMm(raw)) continue;
      int mm = max(0, (int)raw - SEN_MOUNT_OFFSET_MM);
      validCount++;
      if (mm < SEN_DANGER_MM) dangerCount++;
      if (mm < SEN_WARN_MM) warnCount++;
      if (col == 0 || col == 1 || col == 6 || col == 7) {
        if (mm < SEN_DANGER_MM) edgeDangerCount++;
        if (mm < SEN_WARN_MM) edgeWarnCount++;
      }
    }
  }

  if (dangerCount >= SEN_DANGER_NEAR_COUNT) {
    score = max(score, 88 + min(12, dangerCount * 3));
  }
  if (warnCount >= SEN_WARN_NEAR_COUNT) {
    score = max(score, 55 + min(35, warnCount * 5));
  }
  if (edgeDangerCount >= SEN_EDGE_NEAR_COUNT) score = max(score, 90);
  if (edgeWarnCount >= SEN_EDGE_NEAR_COUNT) score = max(score, 65);
  if (validCount > 0 && validCount <= 2) {
    score = max(score, SEN_LOW_VALID_CAUTION_SCORE);
  }
  return constrain(score, 0, 100);
}

// v30: 3基x8x8の各セルを水平角へ展開し、重なり付き角度セクターで統合する。
// 点群ビューアーと同じ「sensorAngle + 26.25 - col*7.5」モデルを使う。
int fusedAngularSectorScore(float minAngleDeg, float maxAngleDeg,
                            int warnMm, int dangerMm,
                            int firstRow, int lastRow) {
  int dangerCount = 0;
  int warnCount = 0;
  int validCount = 0;

  for (uint8_t sensor = 0; sensor < SEN_COUNT; sensor++) {
    if (!fresh(cachedSen[sensor].updatedMs) || !cachedSen[sensor].ok) continue;

    for (int row = firstRow; row <= lastRow; row++) {
      for (int col = 0; col < 8; col++) {
        float rayDeg = SEN_ANGLE_DEG[sensor] +
                       SEN_FIRST_COL_OFFSET_DEG -
                       col * SEN_PIXEL_STEP_DEG;
        if (rayDeg < minAngleDeg || rayDeg > maxAngleDeg) continue;

        uint16_t raw = cachedSen[sensor].raw[row * 8 + col];
        if (!validMatrixMm(raw)) continue;

        int mm = max(0, (int)raw - SEN_MOUNT_OFFSET_MM);
        validCount++;
        if (mm < dangerMm) dangerCount++;
        if (mm < warnMm) warnCount++;
      }
    }
  }

  int score = 0;
  if (dangerCount >= SEN_EDGE_NEAR_COUNT) score = max(score, 90);
  if (warnCount >= SEN_EDGE_NEAR_COUNT) score = max(score, 65);
  if (validCount > 0 && validCount <= 4) score = max(score, SEN_LOW_VALID_CAUTION_SCORE);
  return constrain(score, 0, 100);
}

// v35: 全センサー点を車体固定XY座標へ変換し、前方壁+外周側壁のL字を検出する。
// x=前方、y=左。CLOCKWISE時は左壁、反時計回り時は右壁がコーナー外周になる。
bool geometricCornerDetected() {
  const int FRONT_BIN_COUNT = 12;
  const int SIDE_BIN_COUNT = 24;

  int frontCount[FRONT_BIN_COUNT] = {0};
  int frontMinY[FRONT_BIN_COUNT];
  int frontMaxY[FRONT_BIN_COUNT];
  uint16_t frontYMask[FRONT_BIN_COUNT] = {0};
  int sideCount[SIDE_BIN_COUNT] = {0};
  int sideMinX[SIDE_BIN_COUNT];
  int sideMaxX[SIDE_BIN_COUNT];

  for (int i = 0; i < FRONT_BIN_COUNT; i++) {
    frontMinY[i] = 99999;
    frontMaxY[i] = -99999;
  }
  for (int i = 0; i < SIDE_BIN_COUNT; i++) {
    sideMinX[i] = 99999;
    sideMaxX[i] = -99999;
  }

  for (uint8_t sensor = 0; sensor < SEN_COUNT; sensor++) {
    if (!fresh(cachedSen[sensor].updatedMs) || !cachedSen[sensor].ok) continue;

    for (int row = SEN_ACTIVE_ROW_FIRST; row <= SEN_ACTIVE_ROW_LAST; row++) {
      for (int col = 0; col < 8; col++) {
        uint16_t raw = cachedSen[sensor].raw[row * 8 + col];
        if (!validMatrixMm(raw)) continue;

        float rayDeg = SEN_ANGLE_DEG[sensor] +
                       SEN_FIRST_COL_OFFSET_DEG -
                       col * SEN_PIXEL_STEP_DEG;
        float rayRad = radians(rayDeg);
        int x = (int)lroundf(raw * cosf(rayRad));
        int y = (int)lroundf(raw * sinf(rayRad));

        if (x >= CORNER_FRONT_X_MIN_MM && x <= CORNER_FRONT_X_MAX_MM) {
          int bin = (x - CORNER_FRONT_X_MIN_MM) / CORNER_FRONT_BIN_MM;
          if (bin >= 0 && bin < FRONT_BIN_COUNT) {
            frontCount[bin]++;
            frontMinY[bin] = min(frontMinY[bin], y);
            frontMaxY[bin] = max(frontMaxY[bin], y);
            int yBin = (y + 400) / CORNER_FRONT_Y_BIN_MM;
            if (yBin >= 0 && yBin < 16) {
              frontYMask[bin] |= (uint16_t)(1U << yBin);
            }
          }
        }

        // 周回方向と反対側が四角の外周壁になる。
        int outerY = CLOCKWISE ? y : -y;
        if (outerY >= CORNER_SIDE_Y_MIN_MM && outerY <= CORNER_SIDE_Y_MAX_MM &&
            x >= 0 && x <= 450) {
          int bin = (outerY - CORNER_SIDE_Y_MIN_MM) / CORNER_SIDE_BIN_MM;
          if (bin >= 0 && bin < SIDE_BIN_COUNT) {
            sideCount[bin]++;
            sideMinX[bin] = min(sideMinX[bin], x);
            sideMaxX[bin] = max(sideMaxX[bin], x);
          }
        }
      }
    }
  }

  int bestFrontBin = -1;
  int bestFrontSpan = 0;
  for (int i = 0; i < FRONT_BIN_COUNT - 1; i++) {
    int count = frontCount[i] + frontCount[i + 1];
    int minY = min(frontMinY[i], frontMinY[i + 1]);
    int maxY = max(frontMaxY[i], frontMaxY[i + 1]);
    int span = maxY - minY;
    uint16_t mask = frontYMask[i] | frontYMask[i + 1];
    int longestRun = 0;
    int currentRun = 0;
    for (int bit = 0; bit < 16; bit++) {
      if (mask & (uint16_t)(1U << bit)) {
        currentRun++;
        longestRun = max(longestRun, currentRun);
      } else {
        currentRun = 0;
      }
    }
    int contiguousMm = longestRun * CORNER_FRONT_Y_BIN_MM;
    if (count >= CORNER_FRONT_MIN_POINTS &&
        span >= CORNER_FRONT_MIN_SPAN_MM &&
        contiguousMm >= CORNER_FRONT_MIN_CONTIGUOUS_MM &&
        span > bestFrontSpan) {
      bestFrontBin = i;
      bestFrontSpan = span;
    }
  }
  if (bestFrontBin < 0) return false;

  int frontWallX = CORNER_FRONT_X_MIN_MM +
                   (bestFrontBin + 1) * CORNER_FRONT_BIN_MM;

  for (int i = 0; i < SIDE_BIN_COUNT - 1; i++) {
    int count = sideCount[i] + sideCount[i + 1];
    int minX = min(sideMinX[i], sideMinX[i + 1]);
    int maxX = max(sideMaxX[i], sideMaxX[i + 1]);
    int span = maxX - minX;
    if (count < CORNER_SIDE_MIN_POINTS ||
        span < CORNER_SIDE_MIN_SPAN_MM) continue;

    if (frontWallX >= minX - CORNER_INTERSECTION_TOL_MM &&
        frontWallX <= maxX + CORNER_INTERSECTION_TOL_MM) {
      return true;
    }
  }

  return false;
}

// v20: バック開始ヘルパ。バック後は開いている側へ連続ステアで戻れるよう、
// backRecoverDirにヒントを残す(超信地旋回は廃止済み)。
void startForcedBack(int leftClear, int rightClear, bool cornerEscape) {
  forceBackUntilMs = millis() + (cornerEscape ? CORNER_BACK_MS : FORCE_BACK_MS);
  backClearSinceMs = 0;
  dstate = DS_BACK;
  lastForceBack = true;
  cornerEscapeActive = cornerEscape;

  if (leftClear >= 0 && rightClear >= 0) {
    backRecoverDir = (rightClear > leftClear) ? -1 : 1;
  } else if (rightClear >= 0) {
    backRecoverDir = -1;
  } else {
    backRecoverDir = 1;
  }

  // v33: コーナー脱出時の方向はクリアランス推測より周回設定を優先する。
  if (cornerEscape) {
    backRecoverDir = CLOCKWISE ? -1 : 1;
    cornerTurnTargetYawDeg = normDeg180(yawDeg + backRecoverDir * CORNER_TURN_DEG);
    cornerTurnTargetPrepared = true;
  }
}

void startCornerTurn(bool usePreparedTarget) {
  int turnDir = CLOCKWISE ? -1 : 1;  // +1=左(CCW)、-1=右(CW)
  if (!usePreparedTarget || !cornerTurnTargetPrepared) {
    cornerTurnTargetYawDeg = normDeg180(yawDeg + turnDir * CORNER_TURN_DEG);
  }
  cornerTurnTargetPrepared = false;
  cornerTurnStartMs = millis();
  dstate = DS_CORNER_TURN;
  targetYawDeg = yawDeg;
  frontRedSinceMs = 0;
  frontRedConfirmFrames = 0;
  avoidStraightCaptured = false;
  courseReturnActive = false;
  smoothCmdLeft = 0;
  smoothCmdRight = 0;
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

  // v26: センサー欠測を「障害物なし」と解釈して走り続けない。
  // 起動直後は3基すべての初回測距が揃うまで待ち、走行中も1基でも古い/無効なら停止する。
  bool allSensorsReady = true;
  for (uint8_t i = 0; i < SEN_COUNT; i++) {
    if (!senOK[i] || !fresh(cachedSen[i].updatedMs) || !cachedSen[i].ok) {
      allSensorsReady = false;
      break;
    }
  }
  if (!allSensorsReady) {
    setReason("SENSOR_WAIT");
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

  // v30: センサー単位の集計に加え、全192セルを実角度で3方向へ再集計する。
  // セクターを重ねることで、隣接センサー間の約10度重複と視野端を捨てない。
  int fusedFrontScore = fusedAngularSectorScore(
    -32.0f, 32.0f, SEN_WARN_MM, SEN_DANGER_MM,
    SEN_FRONT_ROW_FIRST, SEN_FRONT_ROW_LAST);
  int fusedLeftScore = fusedAngularSectorScore(
    18.0f, 80.0f, SEN_SIDE_WARN_MM, SEN_SIDE_DANGER_MM,
    SEN_ACTIVE_ROW_FIRST, SEN_ACTIVE_ROW_LAST);
  int fusedRightScore = fusedAngularSectorScore(
    -80.0f, -18.0f, SEN_SIDE_WARN_MM, SEN_SIDE_DANGER_MM,
    SEN_ACTIVE_ROW_FIRST, SEN_ACTIVE_ROW_LAST);

  senFrontScore = max(senFrontScore, fusedFrontScore);
  senLeftScore = max(senLeftScore, fusedLeftScore);
  senRightScore = max(senRightScore, fusedRightScore);

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
  bool corridorModeRaw = leftWallSeen && rightWallSeen;

  // v16: ランダム障害物が一瞬側方ビームに映るだけでcorridorModeが瞬間的に外れる
  // (フリッカー)のを防ぐ。連続CORRIDOR_MISS_STREAK_LIMIT回未検知が続いた場合のみ
  // 実際に解除する(直前がcorridorModeだった場合に限りスティッキーに維持)。
  if (corridorModeRaw) {
    corridorMissStreak = 0;
  } else if (corridorMissStreak < 250) {
    corridorMissStreak++;
  }
  bool corridorMode = corridorModeRaw ||
                       (lastCorridorMode && corridorMissStreak < CORRIDOR_MISS_STREAK_LIMIT);

  lastCorridorMode = corridorMode;
  lastFrontScore = frontScore;
  lastLeftScore = leftScore;
  lastRightScore = rightScore;
  bool currentFrontRed = frontScore >= OBSTACLE_RED_SCORE;

  // =========================================================
  // v20 state machine: FORWARD -> BACK -> FORWARD
  // 超信地旋回(PIVOT)は廃止。回避は常に前進しながらの連続ステアで行う。
  // =========================================================

  // ---- DS_BACK: 通常は直線、ALL_RED脱出時は広い側へカーブバック ----
  if (dstate == DS_BACK) {
    if (now < forceBackUntilMs) {
      lastTurnBias = 0;
      if (cornerEscapeActive && backRecoverDir > 0) {
        setReason("BACK_ARC_L");
        sendDriveCommandSmooth(BACK_L_CMD, BACK_ARC_SLOW_CMD, false);
      } else if (cornerEscapeActive) {
        setReason("BACK_ARC_R");
        sendDriveCommandSmooth(BACK_ARC_SLOW_CMD, BACK_R_CMD, false);
      } else {
        setReason("BACK");
        sendDriveCommandSmooth(BACK_L_CMD, BACK_R_CMD, false);
      }
      return;
    }

    // v46: 通常バックの時間切れだけを理由に前進へ戻さない。
    // 前方が赤い間は停止し、赤解除が連続して確認できてからDS_FORWARDへ戻す。
    // 後方センサーがないため、赤が残る状態でバックを無期限延長することもしない。
    if (!cornerEscapeActive) {
      if (currentFrontRed) {
        backClearSinceMs = 0;
        setReason("BACK_HOLD");
        lastTurnBias = 0;
        sendAMStop();
        return;
      }
      if (backClearSinceMs == 0) backClearSinceMs = now;
      if (now - backClearSinceMs < BACK_CLEAR_CONFIRM_MS) {
        setReason("BACK_CLEAR");
        lastTurnBias = 0;
        sendAMStop();
        return;
      }
    }

    lastBackEndMs = now;
    lastForceBack = false;
    lastSteerDir = backRecoverDir;
    lastAvoidMs = now;
    justExitedAvoid = true;

    // v13由来: バック直後にFORWARDのゆるいslewへそのまま渡すと片輪が負のまま
    // 数フレーム残る(=後進して見える)ため、ゼロから前進をランプさせ直す。
    smoothCmdLeft = 0;
    smoothCmdRight = 0;

    if (cornerEscapeActive) {
      startCornerTurn(true);
    } else {
      dstate = DS_FORWARD;
    }
  }

  // ---- DS_CORNER_TURN: 周回設定方向へIMU基準90度旋回 ----
  if (dstate == DS_CORNER_TURN) {
    float remainingDeg = fabs(normDeg180(cornerTurnTargetYawDeg - yawDeg));
    bool turnDone = remainingDeg <= CORNER_TURN_DONE_DEG;
    bool turnTimedOut = now - cornerTurnStartMs >= CORNER_TURN_TIMEOUT_MS;

    if (!turnDone && !turnTimedOut) {
      if (CLOCKWISE) {
        setReason("CORNER_R");
        lastTurnBias = -900;
        sendDriveCommandSmooth(TURN_CMD, 0, false);
      } else {
        setReason("CORNER_L");
        lastTurnBias = 900;
        sendDriveCommandSmooth(0, TURN_CMD, false);
      }
      return;
    }

    dstate = DS_FORWARD;
    cornerEscapeActive = false;
    targetYawDeg = yawDeg;
    avoidStraightYawDeg = yawDeg;
    avoidStraightCaptured = false;
    courseReturnActive = false;
    cornerTurnCooldownUntilMs = now + CORNER_TURN_COOLDOWN_MS;
    smoothCmdLeft = 0;
    smoothCmdRight = 0;
    sendAMStop();
    return;
  }

  // ---- DS_FORWARD ----

  bool frontRed = currentFrontRed;
  bool leftRed = leftScore >= OBSTACLE_RED_SCORE;
  bool rightRed = rightScore >= OBSTACLE_RED_SCORE;
  bool allRed = frontRed && leftRed && rightRed;
  bool cornerGeometry = frontRed && geometricCornerDetected();
  bool centerScoresBalanced = abs(leftScore - rightScore) <= CENTER_OBS_SCORE_DELTA_MAX;
  bool centerClearanceBalanced =
    (leftClear < 0 || rightClear < 0) ||
    abs(leftClear - rightClear) <= CENTER_OBS_CLEARANCE_DELTA_MAX_MM;
  bool centerObstacle = frontRed &&
                        !cornerGeometry &&
                        centerScoresBalanced &&
                        centerClearanceBalanced;

  if (cornerGeometry) {
    if (frontRedSinceMs == 0) frontRedSinceMs = now;
  } else {
    frontRedSinceMs = 0;
    frontRedConfirmFrames = 0;
  }
  if (cachedSen[0].updatedMs != lastCornerFrontSampleMs) {
    lastCornerFrontSampleMs = cachedSen[0].updatedMs;
    if (cornerGeometry) {
      if (frontRedConfirmFrames < 250) frontRedConfirmFrames++;
    } else {
      frontRedConfirmFrames = 0;
    }
  }
  bool cornerConfirmed = cornerGeometry &&
                         frontRedSinceMs != 0 &&
                         frontRedConfirmFrames >= CORNER_CONFIRM_FRAMES &&
                         now - frontRedSinceMs >= CORNER_CONFIRM_MS;
  bool allRedConfirmed = allRed && cornerConfirmed;

  // (1) BACKトリガ: 前方センサ実測の極近、または前・左・右の三方向すべて赤。
  // v15: 接触保険(IMU加速度によるBACKトリガ)は削除。距離センサが
  // 何も検知していないのにBACKする経路を完全になくすため。
  // v27: ALL_REDでは空いている旋回方向がないため、接触距離まで待たず即バックする。
  bool backArmed = (now - autoStartMs) > BACK_ARM_DELAY_MS;
  bool tofRealClose = (frontMin >= 0 && frontMin < FRONT_STOP_MM);
  // v16: 旧実装は `tofRealClose || (...)` になっており、外側のifが既にtofRealCloseを
  // 要求しているため冗長ORでbackCooldownOkが常にtrueになり、クールダウンが無効化されていた。
  // 本来の意図(コメント通り「再バックまでの最短間隔」)通りに機能するよう修正。
  bool backCooldownOk = (lastBackEndMs == 0) || (now - lastBackEndMs > BACK_COOLDOWN_MS);

  if (allRedConfirmed) {
    if (backCooldownOk) {
      startForcedBack(leftClear, rightClear, true);
      lastTurnBias = 0;
      if (backRecoverDir > 0) {
        setReason("BACK_ARC_L");
        sendDriveCommandSmooth(BACK_L_CMD, BACK_ARC_SLOW_CMD, false);
      } else {
        setReason("BACK_ARC_R");
        sendDriveCommandSmooth(BACK_ARC_SLOW_CMD, BACK_R_CMD, false);
      }
    } else {
      setReason("ALL_RED_WAIT");
      lastTurnBias = 0;
      sendAMStop();
    }
    return;
  }

  if (backArmed && backCooldownOk && tofRealClose) {
    startForcedBack(leftClear, rightClear, false);
    setReason("BACK");
    lastTurnBias = 0;
    sendDriveCommandSmooth(BACK_L_CMD, BACK_R_CMD, false);
    return;
  }

  // (2) v20: 連続ステア回避。常に前進しながら、危険度(スコア)に応じて目標ヨーを
  //     「広い側/安全な側」へ連続的にバイアスする(超信地旋回は行わない)。
  //     方向選択の優先順位(v16の考え方を継承):
  //       1. 片側だけ赤なら、安全を優先して必ず障害物と反対側へ。
  //       2. クリアランス差(leftClear/rightClear)が広い側があればそちらへ。
  //       3. スコア差があれば危険度が低い側へ。
  //       4. 判断材料がなければ直前にステアした方向を維持(tie-break)。
  //     強さは前方/側方の最大危険度スコアに比例し、STEER_URGENCY_MIN未満では0。
  int steerDir = 0;  // +1 = left, -1 = right, 0 = 判断材料なし
  if (centerObstacle) {
    int preferredDir = CLOCKWISE ? -1 : 1;
    int preferredClear = (preferredDir > 0) ? leftClear : rightClear;
    int oppositeClear = (preferredDir > 0) ? rightClear : leftClear;
    bool preferredTooNarrow = preferredClear >= 0 &&
                              preferredClear < VEHICLE_WIDTH_MM;
    bool oppositeCanEscape = oppositeClear >= VEHICLE_WIDTH_MM;
    steerDir = (preferredTooNarrow && oppositeCanEscape)
                 ? -preferredDir
                 : preferredDir;
  } else if (leftRed && !rightRed) {
    steerDir = -1;                                        // 左危険 -> 右へ
  } else if (rightRed && !leftRed) {
    steerDir = 1;                                         // 右危険 -> 左へ
  } else if (leftClear >= 0 && rightClear >= 0 &&
             abs(leftClear - rightClear) > CLEARANCE_DIR_BIAS_MM) {
    steerDir = (rightClear > leftClear) ? -1 : 1;          // クリアランスが広い側へ
  } else if (leftScore != rightScore) {
    steerDir = (leftScore < rightScore) ? -1 : 1;          // 危険度が低い側へ
  } else {
    steerDir = lastSteerDir;                               // 同値なら直前の方向を維持
  }

  int avoidUrgency = max(frontScore, max(leftScore, rightScore));
  bool bothRed = leftRed && rightRed;

  // (2a) v33: 前方赤を四角到達として、周回設定方向へIMU基準90度旋回する。
  if (cornerConfirmed) {
    lastAvoidMs = now;
    justExitedAvoid = true;

    if (now < cornerTurnCooldownUntilMs) {
      setReason("CORNER_WAIT");
      lastTurnBias = 0;
      sendAMStop();
      return;
    }

    startCornerTurn(false);
    if (CLOCKWISE) {
      setReason("CORNER_R");
      lastTurnBias = -900;
      sendDriveCommandSmooth(TURN_CMD, 0, false);
    } else {
      setReason("CORNER_L");
      lastTurnBias = 900;
      sendDriveCommandSmooth(0, TURN_CMD, false);
    }
    return;
  }

  if (avoidUrgency > STEER_URGENCY_MIN) {
    // 通常回避へ入る直前の直進基準を一度だけ保存する。
    // 方位復帰中に再度障害物を検知した場合は、元の保存方位を維持する。
    if (!avoidStraightCaptured) {
      if (!courseReturnActive) avoidStraightYawDeg = targetYawDeg;
      avoidStraightCaptured = true;
    }
    courseReturnActive = false;

    float scale = constrain((float)(avoidUrgency - STEER_URGENCY_MIN) / (100 - STEER_URGENCY_MIN), 0.0f, 1.0f);
    targetYawDeg = normDeg180(yawDeg + steerDir * STEER_MAX_DEG * scale);
    lastSteerDir = steerDir;
    lastAvoidMs = now;
    justExitedAvoid = true;

    if (steerDir > 0) {
      setReason(centerObstacle ? "CENTER_L" : (bothRed ? "ESCAPE_L" : "AVOID_L"));
      lastTurnBias = bothRed ? 500 : 300;
    } else {
      setReason(centerObstacle ? "CENTER_R" : (bothRed ? "ESCAPE_R" : "AVOID_R"));
      lastTurnBias = bothRed ? -500 : -300;
    }
  } else if (justExitedAvoid && now - lastAvoidMs > AVOID_HOLD_MS) {
    // 回避後は現在方位を採用せず、回避開始前の直進基準へ戻す。
    if (avoidStraightCaptured) {
      targetYawDeg = avoidStraightYawDeg;
      courseReturnActive = true;
      courseReturnStartMs = now;
      avoidStraightCaptured = false;
    }
    justExitedAvoid = false;
  }

  if (courseReturnActive) {
    targetYawDeg = avoidStraightYawDeg;
    float returnErrDeg = fabs(normDeg180(avoidStraightYawDeg - yawDeg));
    if (returnErrDeg <= COURSE_RETURN_DONE_DEG ||
        now - courseReturnStartMs >= COURSE_RETURN_TIMEOUT_MS) {
      courseReturnActive = false;
    }
  }

  // (3) 通路中央維持: 目標ヨーを開いた側へゆっくり振る。
  //     実際の舵はヘディング補正(±HEADING_CORR_LIMIT)が担う。
  //     回避バイアス中(avoidUrgency > STEER_URGENCY_MIN)は競合させない。
  if (corridorMode && !courseReturnActive &&
      lastWallDiffMm != 0 && avoidUrgency <= STEER_URGENCY_MIN) {
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

  // (4) 前進: v16 frontScore/側方scoreに応じて連続的に減速する(旧: 2値切替)。
  //     FWD_SLOW_CMDを下限とし、それより下げない(低デューティはストールするため、
  //     v8以降の「低デューティ廃止」方針は維持する)。速度自体は回避中も同じ式で減速する。
  int speedScore = max(frontScore, max(leftScore, rightScore) - 20);
  if (front >= 0 && front < FRONT_SLOW_MM) speedScore = max(speedScore, 40);
  speedScore = constrain(speedScore, 0, 100);
  int base = FWD_CMD - (int)((long)(FWD_CMD - FWD_SLOW_CMD) * speedScore / 100);
  base = constrain(base, FWD_SLOW_CMD, FWD_CMD);

  // reason/lastTurnBiasは(2)で既に回避中の値がセットされている場合、上書きしない。
  if (avoidUrgency <= STEER_URGENCY_MIN) {
    lastTurnBias = 0;

    if (courseReturnActive) {
      setReason("RETURN");
    } else if (corridorMode) {
      setReason("CORRIDOR");
    } else if (max(leftScore, rightScore) > 35 || frontScore > 18) {
      setReason("FWD_CARE");
      lastAvoidMs = now;
      justExitedAvoid = true;
    } else {
      setReason("FWD");

      // v9: 直進安定時のみ、恒常的なヘディング補正量からトリムを微学習。
      // corr>0が続く = 常に左へ舵を切っている = 右へ流れている = 右が弱い -> trimを正へ。
      // CORRIDOR中は中央寄せの意図的な舵が混ざるため学習しない。
      if (AUTO_TRIM_ENABLE && !justExitedAvoid) {
        int corr = headingCorrection();
        motorTrim = constrain(motorTrim + corr * AUTO_TRIM_RATE, -TRIM_LIMIT, TRIM_LIMIT);
      }
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
  else if (dstate == DS_CORNER_TURN) { st = "TURN"; stc = TFT_ORANGE; }
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
  uint8_t version;               // protocol version = 5 (補正後ジャイロXYZ追加)
  uint8_t driveState;            // DS_CALIBRATING(3)を含む
  uint8_t flags;                 // bit0=drive, bit1=IMU zero済, bit3=AM, bit4=Wi-Fi, bit5=BMA, bit6=内蔵IMU data
  uint8_t reserved;
  uint32_t timestampMs;
  int16_t yawCdeg;
  int16_t targetYawCdeg;
  int16_t gapTargetYawCdeg;
  int16_t leftClearanceMm;
  int16_t rightClearanceMm;
  int16_t motorLeft;
  int16_t motorRight;
  int16_t accelXmilliG;          // 静止オフセット補正済み機体座標加速度(M5内蔵IMU)
  int16_t accelYmilliG;
  int16_t accelZmilliG;
  int16_t bmaAccelXmilliG;       // v17: BMA400(TCA CH0)。既に車体right/forward座標系
  int16_t bmaAccelYmilliG;
  int16_t bmaAccelZmilliG;
  int16_t trimMilli;             // v19: motorTrim*1000。キャリブ中に限らず常時送信
  uint8_t calRound;              // v19: キャリブ中のラウンド番号(非キャリブ中は0)
  uint8_t calPass;               // v19: キャリブ中のパス番号(非キャリブ中は0)
  int16_t calDriftCdeg;          // v19: 直近に確定したラウンドの中央値ドリフト*100
  int16_t gyroXmilliDps;         // v25: 静止オフセット補正後の内蔵IMUジャイロ[0.001deg/s]
  int16_t gyroYmilliDps;
  int16_t gyroZmilliDps;
  int16_t sensorYawCdeg[SEN_COUNT]; // 各点群を測距した瞬間のヨー
  uint32_t sensorTimestampMs[SEN_COUNT]; // 同一点群の再登録防止用
  char reason[16];
  uint16_t distances[SEN_COUNT * 64];
  uint16_t crc16;
};

static_assert(sizeof(PointCloudFrame) == 470, "PointCloudFrame size mismatch");

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
  frame.version = 5;
  frame.driveState = (uint8_t)dstate;
  bool bmaFresh = bmaOK && fresh(bmaUpdatedMs, BMA400_READ_PERIOD_MS * 4);
  frame.flags = (driveEnabled ? 0x01 : 0x00) |
                (imuZeroed ? 0x02 : 0x00) |
                (am.online ? 0x08 : 0x00) |
                (WiFi.softAPgetStationNum() > 0 ? 0x10 : 0x00) |
                (bmaFresh ? 0x20 : 0x00) |
                (builtinImuDataOK ? 0x40 : 0x00);
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
  frame.bmaAccelXmilliG = bmaFresh ? (int16_t)constrain((int)lroundf(bmaAccelXG * 1000.0f), -32767, 32767) : 0;
  frame.bmaAccelYmilliG = bmaFresh ? (int16_t)constrain((int)lroundf(bmaAccelYG * 1000.0f), -32767, 32767) : 0;
  frame.bmaAccelZmilliG = bmaFresh ? (int16_t)constrain((int)lroundf(bmaAccelZG * 1000.0f), -32767, 32767) : 0;
  frame.trimMilli = (int16_t)constrain((int)lroundf(motorTrim * 1000.0f), -32767, 32767);
  frame.calRound = calStreamRound;
  frame.calPass = calStreamPass;
  frame.calDriftCdeg = (int16_t)constrain((int)lroundf(calStreamDriftDeg * 100.0f), -32767, 32767);
  frame.gyroXmilliDps = (int16_t)constrain((int)lroundf(imuGyroXDps * 1000.0f), -32767, 32767);
  frame.gyroYmilliDps = (int16_t)constrain((int)lroundf(imuGyroYDps * 1000.0f), -32767, 32767);
  frame.gyroZmilliDps = (int16_t)constrain((int)lroundf(imuGyroZDps * 1000.0f), -32767, 32767);
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
  initBMA400();

  // 起動直後より電源とI2Cが安定した時点でMPU6886を再初期化する。
  // Init()失敗時は従来コードでは全軸0のままでも検出できなかった。
  builtinImuInitResult = M5.IMU.Init();
  builtinImuDataOK = false;
  Serial.printf("MPU6886 late init: %s (%d)\n",
                builtinImuInitResult == 0 ? "OK" : "FAILED", builtinImuInitResult);

  sendAMStop();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 6);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("A: START");
  M5.Lcd.println("B: CAL");
  M5.Lcd.println("A+B: NVS RESET");
  M5.Lcd.printf("Trim %+.2f\n", motorTrim);
}

void loop() {
  M5.update();
  readAMTelemetry();

  // BtnA+BtnB同時押し: drive名前空間のNVSを初期化する。
  // 押しっぱなしで繰り返しclear()しないよう、両ボタンを離すまでラッチする。
  bool nvsResetCombo = M5.BtnA.isPressed() && M5.BtnB.isPressed();
  if (nvsResetCombo) {
    if (!nvsResetComboLatched) {
      nvsResetComboLatched = true;
      resetDriveNVS();
    }
    return;
  }
  if (nvsResetComboLatched) {
    if (!M5.BtnA.isPressed() && !M5.BtnB.isPressed()) {
      nvsResetComboLatched = false;
    }
    return;
  }

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
  readBMA400();

  computeAndDrive();
  drawStatus();
  debugSerial();
  streamPointCloud();

  delay(0);
}

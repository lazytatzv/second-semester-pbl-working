# 2自由度×4脚 蜘蛛型ロボット

Arduino Uno + PCA9685 サーボドライバで動く 4 脚クロール歩容ロボット。
実機の寸法・サーボ負荷・課題仕様は [`docs/robot_spec.md`](docs/robot_spec.md) を参照。
足の軌道生成と重心シフトの数理モデルは [`docs/gait_kinematics.md`](docs/gait_kinematics.md) を参照。

## 1. 書き込みとシリアル接続

```bash
pio run -t upload              # ビルドして Arduino Uno に書き込む
pio device monitor             # シリアルモニタを開く (115200 bps, Ctrl+C で終了)
```

> **Arduino IDE のシリアルモニタを使う場合**
> - ボーレートを **115200 baud** に設定する
> - 改行コードを **「改行のみ (LF)」または「CRおよびLF」** にする（「改行なし」だとコマンドが実行されません）
> - サーボ電源（5〜6V）は USB とは別系統で給電すること

## 2. シリアルコマンド一覧

1 行ずつ入力して Enter で送信します（大文字・小文字は区別しません）。

| コマンド | 動作 |
|---|---|
| `check` | サーボの動作確認（FL → FR → RL → RR の順に 1 本ずつ上げて前後に振る） |
| `stand` / `stop` | 歩行を止めて待機姿勢に戻る |
| `start` | 現在の `speed` / `turn` で歩行開始 |
| `fwd` | 前進（`speed = 1.0`, `turn = 0.0`） |
| `back` | 後退（`speed = -1.0`, `turn = 0.0`） |
| `left` | 左旋回（現在の `speed` のまま `turn = -0.8`） |
| `right` | 右旋回（現在の `speed` のまま `turn = +0.8`） |
| `speed <-1..1>` | 前後速度を細かく指定（例: `speed 0.5`, `speed -0.4`） |
| `turn <-1..1>` | 旋回量を細かく指定（例: `turn -0.3` で緩やかに左、`-1`〜`+1`） |
| `status` | 現在の状態を表示（`WALK/STANDBY`, `speed`, `turn`, `phase`） |
| `help` / `?` | コマンド一覧を表示 |

### 初回チェックと調整

1. `check` を実行し、表示される脚名（`Leg FL` など）と同じ脚が「上がる → 前に振る → 後ろに振る → 戻る」の順に動くか確認する。
2. 逆向きに動く軸があれば [`include/config.h`](include/config.h) の `LEGS` 表の `yawDir` / `kneeDir`（`+1` / `-1`）を反転する。
3. `stand` で立たせたとき角度がずれている脚は、`LEGS` 表の `yawTrim` / `kneeTrim` [deg] で微調整する。

## 3. シミュレータ (`sim/`)

| ツール | 実行方法 | 内容 |
|---|---|---|
| ブラウザ版シミュレータ | `sim/robot_sim.html` をブラウザで開く | スライダーで歩容パラメータを調整し、俯瞰・正面・軌跡をリアルタイム表示 |
| 安定余裕の解析 | `python3 sim/run_analysis.py` | 3 脚支持率と安定余裕（最小・平均 [mm]）を計算 |
| 機体座標の軌道図 | `python3 sim/gait_sim.py` | `sim/gait.png`, `sim/gait.gif` を出力 |
| ワールド座標の歩行図 | `python3 sim/world_sim.py` | `sim/world.png`, `sim/world.gif` を出力 |

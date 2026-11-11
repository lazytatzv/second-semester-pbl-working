# クロール歩容と足軌道の数理モデル (Gait Kinematics)

Arduino Uno + PCA9685 による「2自由度 × 4脚 蜘蛛型ロボット」の歩行軌道生成および連続重心スライド制御の数理モデルです。

---

## 1. 座標系と機体パラメータ

機体幾何中心を原点 $O$、前方を $+x$、左方を $+y$、上方を $+z$ とします。

- **脚配置**: $i \in \{0, 1, 2, 3\} = \{\text{FL(前左)}, \text{FR(前右)}, \text{RL(後左)}, \text{RR(後右)}\}$
  - 前後符号: $x_i = +1 \; (i \in \{0, 1\}), \quad x_i = -1 \; (i \in \{2, 3\})$
  - 左右符号: $y_i = +1 \; (i \in \{0, 2\}), \quad y_i = -1 \; (i \in \{1, 3\})$
- **寸法**:
  - 付け根（Yaw）位置: $\mathbf{p}_{\text{base}, i} = [x_i B, \; y_i B, \; 0]^T \quad (B = 75\text{ mm})$
  - 太もも長: $L_1 = 60\text{ mm}$、スネ長: $L_2 = 70\text{ mm}$

---

## 2. 歩行位相と脚順（対角クロール）

歩行全体の時刻 $t$ に対する正規化位相 $\phi(t) \in [0, 1)$（周期 $T = 2.5\text{ s}$）：
$$\phi(t) = \left( \frac{t}{T} \right) \pmod{1}$$

対角クロール歩容（$\text{FL} \to \text{RR} \to \text{FR} \to \text{RL}$）の各脚オフセット $\Delta_i$：
$$\Delta_{\text{FL}} = 0.00, \quad \Delta_{\text{RR}} = 0.75, \quad \Delta_{\text{FR}} = 0.50, \quad \Delta_{\text{RL}} = 0.25$$

脚 $i$ の局所位相 $u_i(\phi) = (\phi + \Delta_i) \pmod{1}$ に対し、遊脚比率を $\beta = 0.20$（20%遊脚 / 80%支持脚）とします。

---

## 3. 単脚軌道生成（遊脚・支持脚）

各脚の運動は、正規化された角度軌道 $\tilde{\theta}_{\text{yaw}}, \tilde{\theta}_{\text{knee}} \in [-1, 1]$ で生成されます。

### ① 遊脚期（$0 \le u_i < \beta$）: 空中で足を前へ運ぶ
正規化スイング時間 $s = \frac{u_i}{\beta} \in [0, 1]$：
- **付け根（Yaw）**: 後方から前方へ滑らかに振り出し
  $$\tilde{\theta}_{\text{yaw}, i}(s) = -\cos(\pi s)$$
- **膝（Knee）**: サイン波で鉛直上方に山なりリフト
  $$\tilde{\theta}_{\text{knee}, i}(s) = \sin(\pi s)$$

### ② 支持脚期（$\beta \le u_i < 1$）: 地面を蹴って前進
正規化スタンス時間 $\tau = \frac{u_i - \beta}{1 - \beta} \in [0, 1]$：
- **付け根（Yaw）**: 前方から後方へ等速で地面を蹴る
  $$\tilde{\theta}_{\text{yaw}, i}(\tau) = 1 - 2\tau$$
- **膝（Knee）**: 接地を維持（リフト量ゼロ）
  $$\tilde{\theta}_{\text{knee}, i}(\tau) = 0$$

---

## 4. 足先高さと運動学（リフト計算）

スネの立ち姿勢角を $\theta_{\text{stance}} = 25^\circ$、最大足上げ角を $\Delta\theta_{\text{lift}} = 48^\circ$ とすると、瞬時スネ角 $\theta_{\text{shank}, i}$ は：
$$\theta_{\text{shank}, i} = \theta_{\text{stance}} + \Delta\theta_{\text{lift}} \cdot \tilde{\theta}_{\text{knee}, i}$$

足先の地面からの持ち上がり高さ $h_i$ は：
$$h_i = L_2 \left( \cos\theta_{\text{stance}} - \cos\theta_{\text{shank}, i} \right)$$

最高点（$\tilde{\theta}_{\text{knee}} = 1$、$\theta_{\text{shank}} = 73^\circ$）における最大足上げ高さ：
$$h_{\max} = 70 \cdot (\cos 25^\circ - \cos 73^\circ) \approx 70 \cdot (0.906 - 0.292) \approx 43.0\text{ mm}$$

---

## 5. 連続重心スライド制御（転倒防止）

クロール歩容では、足を上げた瞬間に重心が支持三角形の境界線上（安定余裕 $\approx 0\text{ mm}$）に来るため転倒します。
これを防ぐため、支持脚のヨー軸を用いて**車体（重心）をごっそり対角支持脚側へスライド**させます。

### 連続サイン波重心変位
歩行サイクル $\phi$ に応じて滑らかに重心を旋回させます：
- 前後シフト（周期 $0.5$、周波数 $2\omega$）:
  $$X_{\text{shift}}(\phi) = -\cos\left(4\pi (\phi - 0.10)\right)$$
- 左右シフト（周期 $1.0$、周波数 $\omega$）:
  $$Y_{\text{shift}}(\phi) = \sqrt{2}\cos\left(2\pi (\phi - 0.475)\right)$$

### ヨー軸補正角への射影
各脚 $i$ の前向きヨー角補正量 $\delta\theta_{\text{yaw}, i}$：
$$\delta\theta_{\text{yaw}, i}(\phi) = - K_x X_{\text{shift}}(\phi) + (x_i \cdot y_i) K_y Y_{\text{shift}}(\phi)$$
- $K_x = 4.0^\circ$（前後ゲイン：前進キックとの干渉を防ぐため控えめ）
- $K_y = 7.0^\circ$（左右ゲイン：転倒を防ぐため主軸として付与）

> **効果**: 4歩すべてで完全に均等（まんべんなく）なスライドになり、安定余裕は最小 $8.25\text{ mm}$ / 平均 $26.31\text{ mm}$ に拡大します。

---

## 6. 最終的なサーボ指令値の合成式

前後速度指令 $V \in [-1, 1]$、旋回指令 $T \in [-1, 1]$、歩幅振幅 $\Theta_{\text{amp}} = 24.0^\circ$ に対し、各サーボへの最終指示角度は以下の通りです：

$$\theta_{\text{yaw}, i} = 90^\circ + d_{\text{yaw}, i} \cdot \left[ \Theta_{\text{amp}} \cdot (V + y_i T) \cdot \tilde{\theta}_{\text{yaw}, i} + \delta\theta_{\text{yaw}, i}(\phi) \right] + \text{trim}_{\text{yaw}, i}$$

$$\theta_{\text{knee}, i} = 90^\circ + d_{\text{knee}, i} \cdot \left[ \theta_{\text{stance}} + \Delta\theta_{\text{lift}} \cdot \tilde{\theta}_{\text{knee}, i} \right] + \text{trim}_{\text{knee}, i}$$

- $d_{\text{yaw}, i}, d_{\text{knee}, i} \in \{+1, -1\}$: 回転方向の符号反転
- $\text{trim}_i$: 90°中立位置からの微調整オフセット

---

## 7. 逆運動学（IK）を採用しなかった設計判断

本機では、足先座標 $(x, y, z)$ から関節角度を逆算する「逆運動学（Inverse Kinematics: IK）」ではなく、関節角を直接パラメータ生成する「順軌道生成（Forward Parametric Trajectory）」を採用しています。その工学的理由は以下の4点です：

1. **自由度の幾何学的拘束（2自由度脚の劣駆動特性）**:
   3次元空間の足先位置 $(x, y, z)$ を任意に決めるには最低3自由度が必要ですが、本機は「付け根Yaw ＋ 膝Knee」の2自由度です。
   膝の回転によって高さを変えると水平リーチ $R = L_1 + L_2 \sin\theta_{\text{shank}}$ も同時に変化するため、$z$ と $R$ を独立に制御できません。任意の空間座標を指定しても数学的に解が存在しないため、IKを解く必然性が低い構成です。

2. **Arduino Uno（ATmega328P）の演算リソースと50Hz周期保証**:
   ATmega328P は 16MHz、RAM 2KB、ハードウェアFPU非搭載の8ビットマイコンです。
   4脚×2関節の逆三角関数（`atan2`, `acos`, `sqrt`）を毎回解くとCPU負荷が高まり、20ms（50Hz）の厳密な制御ループ周期を維持できなくなるリスクがあります。
   一方、三角関数を用いた順軌道生成であれば極めて軽量に計算が完了し、制御周期を確実に死守できます。

3. **特異点（Singularity）の回避とサーボ保護**:
   IKでは可動域限界付近でヤコビアンの逆行列が発散し、サーボ指令角度が急激に跳躍してギア欠けやストールを引き起こす危険があります。
   関節空間（Joint Space）で直接滑らかな連続波形（サイン・コサイン）を生成することで、物理可動範囲 $[0^\circ, 180^\circ]$ を絶対に逸脱しない安全性が数理的に保証されます。

4. **実機調整（キャリブレーション）の容易性**:
   3Dプリントの寸法公差やサーボホーンの差し込み誤差に対し、関節空間制御であれば `yawTrim` や `kneeTrim`、`YAW_AMP`、`KNEE_LIFT` がサーボ角度と1対1で直感的に対応しており、実機チューニングが極めて容易です。


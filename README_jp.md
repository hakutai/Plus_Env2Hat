# 持ち歩き温湿度計（Pocket environment meter）

[English](README.md) | 日本語

<img src="Plus_Env2Hat_jp.jpg" width="320px">

４つの機能をＡボタン（Homeボタン・Ｍ５ボタン）で切り替え、自動でディープスリープに入るため電源オフ操作は不要、起動はＡボタン。

暑さ指数（ＷＢＧＴ）が３１以上と危険な環境になるとビープと液晶をフラッシュさせ注意喚起する。（が時間が経つとディープスリープに落ちます）

３０分間隔で起動しその後ディープスリープに、この時に３時間毎のログ取得と暑さ指数の警報も行います。

外部電源接続時は起動中の場合はそのまま動作し続けます。

# ４つの機能
## メイン画面ー温度／湿度計

温度・湿度および暑さ指数（ＷＢＧＴ）と不快指数の表示、暑さ指数は三時間前の測定値から上昇中の場合は矢印を表示。

暑さ指数（WBGT）が３１以上になるとビープ音と液晶が明滅します。

Bボタンで設定画面になります。

### 背景と文字の色
暑さ指数 | 背景色
---------|---------
28未満 | ダークシアン
28以上～31未満 | 黄色
31以上 | 赤

### 暑さ指数文字色
暑さ指数 | 文字色
---------|---------
25未満 | シアン
25以上～28未満 | 緑黄
28以上～31未満 | 黄色
31以上 | 赤

### 不快指数文字色
不快指数 | 文字色
---------|---------
76未満 | シアン
76以上～80未満 | 緑
80以上～85未満 | 緑黄
85以上～90未満 | 黄
90以上 | 赤地に灰文字

### 設定

メイン画面からBボタンを押し機能・日時などの設定をする。

操作はBボタンで項目選択、Aボタンで値変更。
「RETURN」もしくはスリープ時間が経つとメイン画面へ戻ります。

#### 設定項目
項目 | 内容 |規定値
----|----|----
Scrn Direction（液晶の向き） | 液晶の表示向きの変更で使いやすい方向に。利き手やケーブル接続時に変更してください。 | LEFT
Scrn Brightness（液晶の明るさ） | 見やすい明るさで、７から１５ | 9
Sleep Time（スリープ時間） | ディープスリープまでの時間、10秒から10秒間隔で50秒まで。 | 20秒
Resume（レジューム） | ディープスリープ復帰時に元の機能へ戻る設定 | OFF
DT（日時） | 年は2021から2029まで設定可、2月は29日の設定も可能ですついては自己責任で。 | 
Demomode（デモモード） | ５秒間隔で４機能が自動で切り替わる。 | OFF

## 気圧・標高計

気圧と標高の他に温度／湿度と電圧／電流を表示。

電圧はバッテリー電圧を表示し、電流はUSB接続中はUSB電流でバッテリー使用中ばバッテリー放電電流をマイナス表示、＋は充電中／ーは放電中。

### 標高校正（キャリブレーション）

指定標高から基準気圧を算出。

Bボタンを押し続けると、標準気圧（1013.25hPa）、0m、100m、200mと100mごとに増え求める標高にする、行きすぎたらBボタンを一度離し最初からやり直す。

現在地の標高に近い値を設定してください。
校正中に表示標高は100mところ99mとかに高度が上がると差が広がっていきます、標高から基準気圧を算出しそれから標高を求め直して表示しているため誤差がでます。

## 方位磁石

コンパス図柄が北の方向、数字で向いている方向、その他に時計・標高・温度を表示。

登山・ハイキング時にこの画面でレジューム表示するのが良いかと。

### 方位校正

Ｂボタンを押しながらM5StickCPlusを八の字（昔の携帯やスマホでの校正方法）動作をしてください、一通り振り回したらＢボタンを離す。

## 履歴（ログ）

３時間間隔で気圧・温度・湿度・暑さ指数の記録を表示、気圧前の上（^）下（v）矢印は２４時間前と5hPa以上の差がある場合に表示し天気の変動を予測に。

# 更新履歴
  2021/03/04 1.00&nbsp;&nbsp;&nbsp;    履歴にＷBGT指数の表示<br>
  2021/03/05 1.01&nbsp;&nbsp;&nbsp;    外部電源中にディープスリープから復帰した場合は自動でディープスリープへ<br>
  2021/03/22 1.02&nbsp;&nbsp;&nbsp;    外部電源充電中に指定電圧（BATTERMAX）以下時に確認用にLEDを点灯<br>
  2021/03/29 1.02.01 充電をbattery(GetBatCurrent())の正負で判断へ変更<br>
  2021/04/05 1.03&nbsp;&nbsp;&nbsp;    ディープスリープを正時と３０分に起動するよう変更<br>


# 今後

山歩きに利用できるように、心拍センサユニット（HeartRate-Unit）を使い心拍・血中酸素濃度を測定する機能。


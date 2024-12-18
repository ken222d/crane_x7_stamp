# crane_x7_stamp
ロボット設計製作論3 P班

## 動作
指定されている座標にハンコを押す. 

## セットアップ, ビルド

ROS 2及びCRANE-X7のセットアップ, ビルドの方法に関しては[こちら](https://github.com/cit22ros2/crane_x7_simple_examples)を確認してください.

## このパッケージの使い方

### リポジトリのクローン方法

```
git clone https://github.com/ken222d/crane_x7_stamp.git
cd crane_x7_stamp
```

### 実行

for real machine
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py
```

for gazebo
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py use_sim_time:='true'
```

## 注意
- 動作が大きくなることがあるので, 使用時は周囲に十分注意してください.
- マニピュレータ動作終了後, ```Ctrl+C```でプログラムを終了してください.

## やり残したこと
- RealSenseを用いて色検出を行いハンコを押す
- キーボードで入力した座標にハンコを押す
- 無駄な移動経路の削減
- YOLOでハンコを検出し掴む

## ライセンス
このパッケージはApache License 2.0のもとに, [crane_x7_simple_examples](https://github.com/cit22ros2/crane_x7_simple_examples)の一部を改変して利用しています.
このパッケージはApache License 2.0に基づき公開されています.

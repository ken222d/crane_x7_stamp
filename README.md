# crane_x7_stamp
ロボット設計製作論3 P班 Ishizeki 
- マニピュレータを利用してハンコを押します. 

# 環境
- Ubuntu 22.04
- ROS 2 Humble
- OpenCV 4.5.4

# セットアップ, ビルド

ROS 2及びCRANE-X7のセットアップ, ビルドの方法に関しては[https://github.com/cit22ros2/crane_x7_simple_examples](https://github.com/cit22ros2/crane_x7_simple_examples)の以下をを確認してください.
- このパッケージを使う前に
  - ROS 2及びCRANE-X7セットアップ
- このパッケージの使い方
  - インストール
  - ビルド

# このパッケージの使い方

## 実行

### 推奨実行手順
1. open_close_handでハンドを開き, ハンコを掴ませる.  
2. press_the_stampもしくはpress_the_stamp_tfでハンコを押す.

### camera_picking(うまくいかない)
物体の色を認識して, その位置にハンコを押します. 
- 実機用
```
ros2 launch crane_x7_stamp camera_picking.launch.py
```
- Gazebo用
```
ros2 launch crane_x7_stamp camera_picking.launch.py use_sim_time:='true'
```
### open_close_hand
ハンコを掴ませるためにハンドを5秒間開きます.
- 実機用
```
ros2 run crane_x7_stamp open_close_hand
```
- Gazebo用
```
ros2 run crane_x7_stamp open_close_hand use_sim_time:='true'
```

### press_the_stamp
ノード内で指定した座標にハンコを押します.
- 実機用
```
ros2 launch crane_x7_stamp press_the_stamp.launch.py
```
- Gazebo用
```
ros2 launch crane_x7_stamp press_the_stamp.launch.py use_sim_time:='true'
```

### press_the_stamp_tf
[![YouTubeの動画](https://img.youtube.com/vi/GrYh_InDjL0/0.jpg)](https://youtu.be/GrYh_InDjL0)  
上の動画は, ハンコを掴む動作はしない. 
指定した位置にあるハンコを掴みに行き, その後, 他のノード(fixed_position_publisher)から流れてきた座標の情報をもとにハンコを押します.
- 実機用
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py
```

- Gazebo用
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py use_sim_time:='true'
```

# 注意
- 動作が大きくなることがあるので, 使用時は周囲に十分注意してください.
- マニピュレータ動作終了後, ```Ctrl+C```でプログラムを終了してください.
- 色をうまく認識できないので, 色検出関連のファイルを改善する必要がある.

# 引き継ぎ
## このパッケージに変更を加えたい場合
- 新たな実行ファイルを追加する場合, [CMakeLists.txt](https://github.com/ken222d/crane_x7_stamp/blob/main/CMakeLists.txt)を編集してください.
## srcファイルの説明
- color_detection.cpp
  - 色を認識するノードです.
  - pick_and_move_tf.cppに物体座標を送信するために使用しています. 
  - 青色と赤色, 緑色のHSV比が書いてあります.
- pick_and_move_tf.cpp
  - color_detection.cppから送られてきた座標にハンコを押す動作をします. 
- fixed_position_publisher.cpp
  - 座標を送信します.
  - press_the_stamp_tf.cppに座標を送信するために使用しています.
- open_close_hand
  - ハンドを5秒間開き, その後閉じます.
- press_the_stamp.cpp
  - このノード(プログラム)内で指定した座標にハンコを押します.
- press_the_stamp_tf.cpp
  - 指定した座標においたハンコを掴み, 指定した座標にハンコを押します.
## やり残したこと
- RealSenseを用いて色検出を行いハンコを押す.
- キーボードで入力した座標にハンコを押す.
- 無駄な移動経路の削減.
- YOLOでハンコを検出し掴む.

## mainブランチとの違い
 - press_the_stamp_tf.cppの動きにハンコを掴みに行く動作がある点. 
 - 色検出して, ハンコを押す動作をするプログラムがある点. 

# ライセンス
- このパッケージはApache License 2.0のもとに, [crane_x7_simple_examples](https://github.com/cit22ros2/crane_x7_simple_examples)の一部を改変して利用しています.
  - 以下のファイルを改変し, 新規ファイルを作成しました.
    - color_detection.cppとpick_and_move_tf.cppを改変
    - color_detection.cppをもとに, fixed_position_publisher.cpp
    - pick_and_move.cppをもとに, press_the_stamp.cppおよびopen_close_hand
    - pick_and_move_tf.cppをもとに, press_the_stamp_tf.cpp
    - camera_picking.launch.pyをもとに, press_the_stamp_tf.launch.py
    - pick_and_move.launch.pyをもとに, press_the_stamp.launch.py
  - また, その他のファイルについても, 微小な変更を行いました.
- このパッケージはApache License 2.0に基づき公開されています.  
- ライセンスの全文は[LICENSE](https://github.com/ken222d/crane_x7_stamp/blob/main/LICENSE)から確認できます.

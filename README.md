# crane_x7_stamp
ロボット設計製作論3 P班

# 動作
指定された座標にハンコを押す. 

# セットアップ, ビルド

ROS 2及びCRANE-X7のセットアップ, ビルドの方法に関しては[こちら](https://github.com/cit22ros2/crane_x7_simple_examples)を確認してください.

# このパッケージの使い方

## リポジトリのクローン方法

```
git clone https://github.com/ken222d/crane_x7_stamp.git
cd crane_x7_stamp
```

## 実行

### press_the_stamp
ノード内で指定した座標にハンコを押します.
- for real machine
```
ros2 run crane_x7_stamp press_the_stamp
```
- for Gazebo
```
ros2 run crane_x7_stamp press_the_stamp use_sim_time:='true'
```

### press_the_stamp_tf
トピックから流れてきた座標の情報をもとにハンコを押します.
- for real machine
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py
```

- for Gazebo
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py use_sim_time:='true'
```

### open_close_hand
ハンコを掴ませるためにハンドを5秒間開きます.
- for real machine
```
ros2 run crane_x7_stamp open_close_hand
```
- for Gazebo
```
ros2 run crane_x7_stamp open_close_hand use_sim_time:='true'
```

# 注意
- 動作が大きくなることがあるので, 使用時は周囲に十分注意してください.
- マニピュレータ動作終了後, ```Ctrl+C```でプログラムを終了してください.
- 新たな実行ファイルを追加する場合, [CMakeLists.txt](https://github.com/ken222d/crane_x7_stamp/blob/main/CMakeLists.txt)を編集してください.

# ライセンス
このパッケージはApache License 2.0のもとに, [crane_x7_simple_examples](https://github.com/cit22ros2/crane_x7_simple_examples)の一部を改変して利用しています.  
このパッケージはApache License 2.0に基づき公開されています.  
ライセンスの全文は[LICENSE](https://github.com/ken222d/crane_x7_stamp/blob/main/LICENSE)から確認できます.

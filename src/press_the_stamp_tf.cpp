// Copyright 2024 Keitaro Nakamura
// SPDX-FileCopyrightText: 2024 Haruto Yamamoto, Akira Matsumoto
// SPDX-License-Identifier: Apache 2.0
// このファイルは元々Keitaro NakamuraとRyotaro Karikomiによって作成され、その後Haruto YamamotoとAkira Matsumoyoによって変更されました。

// 設計方針
//        構成
//            void move_specific_joint
//                特定の関節を現在の角度からn°動かす
//            void stamping 
//                変数の定義
//                init_pose
//                xyz座標を移動するfor文
//                一時停止
//                void move_specific_jointで第2関節を-5°動かす
//                一時停止
//                void move_specific_jointで第2関節を5°動かす
//                init_pose
//                rclcpp::shutdown();
//
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <iostream> // std::cout を使うために必要

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// ノード定義
class PickAndPlaceTf : public rclcpp::Node
{
public:
  // コンストラクタ
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_move_tf_node")
  {
    using namespace std::placeholders;

    // アームの設定
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.6);  // アームの最大速度スケーリング
    move_group_arm_->setMaxAccelerationScalingFactor(0.6);  // アームの最大加速度スケーリング

    // グリッパーの設定
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);  // グリッパーの最大速度スケーリング
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);  // グリッパーの最大加速度スケーリング

    // ホームポジションに設定
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();

    // TFバッファとリスナーの初期化
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // タイマーの設定
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));  // タイマーをセットし定期的にon_timerを呼び出す
  }

private:
  // タイマー処理
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform("base_link", "target_0", tf2::TimePointZero);  // "base_link"から"target_0"への変換を取得
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform base_link to target: %s", ex.what());  // 変換失敗時にログ出力
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const std::chrono::nanoseconds FILTERING_TIME = 2s;  // フィルタリング時間
    const std::chrono::nanoseconds STOP_TIME_THRESHOLD = 3s;  // 停止時間しきい値
    const double DISTANCE_THRESHOLD = 0.01;  // 距離しきい値
    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);  // tf変換
    const auto TF_ELAPSED_TIME = now.nanoseconds() - tf.stamp_.time_since_epoch().count();  // 時間差
    const auto TF_STOP_TIME = now.nanoseconds() - tf_past_.stamp_.time_since_epoch().count();  // 停止時間
    const double TARGET_Z_MIN_LIMIT = 0.04;  // Z軸最小限界

    // フィルタリングと停止判定
    if (TF_ELAPSED_TIME < FILTERING_TIME.count()) {
      double tf_diff = (tf_past_.getOrigin() - tf.getOrigin()).length();  // 位置差
      if (tf_diff < DISTANCE_THRESHOLD) {
        if (TF_STOP_TIME > STOP_TIME_THRESHOLD.count()) {
          if (tf.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
            tf.getOrigin().setZ(TARGET_Z_MIN_LIMIT);  // Z位置制限
          }
          stamping(tf.getOrigin());  // スタンピング（動作）開始
        }
      } else {
        tf_past_ = tf;  // 過去の変換を更新
      }
    }
  }

  // ホームポジションの設定
  void init_pose()
  {
    std::vector<double> joint_values = {
      angles::from_degrees(0.0),
      angles::from_degrees(85),
      angles::from_degrees(0.0),
      angles::from_degrees(-158),
      angles::from_degrees(0.0),
      angles::from_degrees(-50),
      angles::from_degrees(90)
    };
    move_group_arm_->setJointValueTarget(joint_values);  // ジョイント値を設定
    move_group_arm_->move();  // アームを移動
  }

  // 特定の関節を指定して動かす
  void move_specific_joint_step(int joint_index, double relative_angle_deg, int num_steps = 10)
  {
    std::vector<double> current_joint_values = move_group_arm_->getCurrentJointValues();
    if (joint_index >= 0 && joint_index < current_joint_values.size()) {
      double current_angle_deg = angles::to_degrees(current_joint_values[joint_index]);
      double angle_difference = relative_angle_deg;

      for (int i = 1; i <= num_steps; ++i) {
        current_joint_values[joint_index] = angles::from_degrees(current_angle_deg + (angle_difference * i / num_steps));
        move_group_arm_->setJointValueTarget(current_joint_values);  // ジョイント目標を設定
        move_group_arm_->move();  // アームを移動
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // 100msごとに動かす
      }
    } else {
      std::cerr << "Invalid joint index: " << joint_index << std::endl;  // 無効な関節インデックス
    }
  }

  // スタンピング動作
  void stamping(tf2::Vector3 target_position)
  {
    const double GRIPPER_DEFAULT = 0.0;  // デフォルトのグリッパー角度
    const double GRIPPER_OPEN = angles::from_degrees(60.0);  // グリッパーオープン角度
    const double GRIPPER_CLOSE = angles::from_degrees(15.0);  // グリッパークローズ角度
    const int move_steps = 4;  // 移動ステップ数
    const double before_press_z = 0.05;  // プレス前のZ位置

    control_gripper(GRIPPER_DEFAULT);  // グリッパーをデフォルトに設定
    init_pose();  // ホームポジションに移動

    geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;  // 現在のアームのポーズを取得

    // 中間位置を設定して移動
    for (int i = 1; i <= move_steps; ++i) {
      geometry_msgs::msg::Pose intermediate_pose;
      intermediate_pose.position.x = current_pose.position.x + (target_position.x() - current_pose.position.x) * i / move_steps;
      intermediate_pose.position.y = current_pose.position.y + (target_position.y() - current_pose.position.y) * i / move_steps;
      intermediate_pose.position.z = 0.4 - (-0.05 + (0.05 * i));  // Z軸位置を調整

      control_arm(intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z, 90, 0, 90);  // アームを移動
      std::cout << "Move steps loop iteration: " << i << "/" << move_steps << std::endl;  // ステップの表示
    }

    // 目標位置に向かって最終的な調整
    current_pose.position.x = target_position.x();
    current_pose.position.y = target_position.y();

    double calculated_arm_length = std::sqrt((target_position.x() * target_position.x()) + (target_position.y() * target_position.y()));  // アームの長さ計算
    double point_z = ((-0.12 * calculated_arm_length) + 0.07);  // Z位置計算

    // Z軸調整
    for (int i = 3; i >= 1; i--) {
      geometry_msgs::msg::Pose intermediate_pose;
      intermediate_pose.position.x = current_pose.position.x;
      intermediate_pose.position.y = current_pose.position.y;
      intermediate_pose.position.z = ((0.1 - point_z) * i);  // Z軸の最終調整

      control_arm(intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z, 90, 0, 90);  // アームを移動
      std::cout << "Movement Control Loop: " << i << std::endl;
    }

    double theta = angles::to_degrees(std::asin(before_press_z / calculated_arm_length));  // プレス角度を計算

    // ジョイントを動かして微調整
    for (int i = 0; i < 5; ++i) {
      move_specific_joint_step(1, -theta / 5);  // ジョイントの角度調整
      std::cout << "Step " << (i + 1) << ": Joint moved by" << theta / 5 << "°" << std::endl;
    }

    control_arm(0.3, 0, 0.3, 90, 0, 90);  // アームの最終位置に移動
    init_pose();  // ホームポジションに戻る
    rclcpp::shutdown();  // ノード終了
  }

  // グリッパー制御
  void control_gripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();  // 現在のグリッパーの関節値を取得
    joint_values[0] = angle;  // グリッパーの角度を設定
    move_group_gripper_->setJointValueTarget(joint_values);  // ジョイント値ターゲットを設定
    move_group_gripper_->move();  // グリッパーを移動
  }

  // アーム制御
  void control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));  // 回転角度を設定
    target_pose.orientation = tf2::toMsg(q);  // クォータニオンに変換
    move_group_arm_->setPoseTarget(target_pose);  // 目標位置を設定
    move_group_arm_->move();  // アームを移動
  }

  // メンバ変数
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char **argv)
{
  // rclcppの初期化
  rclcpp::init(argc, argv);

  // ノードオプションの設定
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // ノードの作成
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // マルチスレッドエグゼキュータの設定
  rclcpp::executors::MultiThreadedExecutor exec;

  // PickAndPlaceTfノードの作成
  auto pick_and_move_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_arm_node,
    move_group_gripper_node);

  // ノードをエグゼキュータに追加
  exec.add_node(pick_and_move_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);

  // エグゼキュータの実行
  exec.spin();

  // rclcppのシャットダウン
  rclcpp::shutdown();

  return 0;
}


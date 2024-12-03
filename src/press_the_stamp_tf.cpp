// Copyright 2024 Keitaro Nakamura
// SPDX-FileCopyrightText: 2024 Haruto Yamamoto
// SPDX-License-Identifier: Apache 2.0
// このファイルは元々Keitaro NakamuraとRyotaro Karikomiによって作成され、その後Haruto YamamotoとAkira Matsumoyoによって変更されました。
// 変更内容：
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
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

class StampAndPlaceTf : public rclcpp::Node
{
public:
  StampAndPlaceTf(rclcpp::Node::SharedPtr move_group_arm_node)
  : Node("stamp_and_place_tf_node")
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.7);
    move_group_arm_->setMaxAccelerationScalingFactor(0.7);
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&StampAndPlaceTf::on_timer, this));
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", "target_0",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform base_link to target: %s", ex.what());
      return;
    }

    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);

    // ハンコを押す位置に移動して実行
    stamping(tf.getOrigin().x(), tf.getOrigin().y());
  }

  void stamping(double x, double y)
  {
    const double Z_HEIGHT = 0.2;  // ハンコを押す高さ
    const double Z_PRESS = 0.05; // ハンコを押すための高さ

    // 初期姿勢に移動
    move_to_position(x, y, Z_HEIGHT);

    // ハンコを押す
    move_to_position(x, y, Z_PRESS);

    // 元の高さに戻る
    move_to_position(x, y, Z_HEIGHT);

    // 初期姿勢に戻る
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();
    rclcpp::shutdown();
  }

  void move_to_position(double x, double y, double z)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(90), 0, angles::from_degrees(90)); // アームの向きを調整
    target_pose.orientation = tf2::toMsg(q);
    move_group_arm_->setPoseTarget(target_pose);
    move_group_arm_->move();
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto stamp_and_place_tf_node = std::make_shared<StampAndPlaceTf>(move_group_arm_node);
  exec.add_node(stamp_and_place_tf_node);
  exec.add_node(move_group_arm_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}


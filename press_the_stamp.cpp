// SPDX-FileCopyrightText: 2023 Keitaro Nakamura,Ryotaro karikomi
// SPDX-FileCopyrightText: 2024 Haruto Yamamoto
// SPDX-License-Identifier: Apache 2.0
# このファイルは元々Keitaro NakamuraとRyotaro Karikomiによって作成され、その後Haruto Yamamotoによって変更されました。
# 変更内容：アームの開閉をコメントアウト, その他移動を大幅削除, Z座標のみを上下させる動作を追加
#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("press_the_stamp");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  //アームとグリッパーの速度と加速度を設定
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();

  // SRDFに定義されている"home"の姿勢にする
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // 可動範囲を制限する
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "arm_constraints";

  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(45);
  joint_constraint.tolerance_below = angles::from_degrees(45);
  joint_constraint.weight = 1.0;
  constraints.joint_constraints.push_back(joint_constraint);

  joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(45);
  joint_constraint.tolerance_below = angles::from_degrees(45);
  joint_constraint.weight = 0.8;
  constraints.joint_constraints.push_back(joint_constraint);

  move_group_arm.setPathConstraints(constraints);

//アームを開く
//  gripper_joint_values[0] = angles::from_degrees(60);
//  move_group_gripper.setJointValueTarget(gripper_joint_values);
//  move_group_gripper.move();
//  return 0;

//アームを閉じる
//  gripper_joint_values[0] = angles::from_degrees(15);
//  move_group_gripper.setJointValueTarget(gripper_joint_values);
//  move_group_gripper.move();
//  return 0;

  //ハンコを押す動作
  float arm_updown[6] = {0.25, 0.17, 0.12, 0.05, 0.14, 0.25};
  for (int i = 0; i<= 5; i+=1){
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.0;
  target_pose.position.z = arm_updown[i];
  q.setRPY(angles::from_degrees(90), angles::from_degrees(0), angles::from_degrees(90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();
}
 
//なんとなく真ん中へ
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;

  target_pose.position.x = 0.0;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.5;
  q.setRPY(angles::from_degrees(90), angles::from_degrees(0), angles::from_degrees(90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();



  // 可動範囲の制限を解除
  move_group_arm.clearPathConstraints();

  //homeの姿勢に戻る
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}

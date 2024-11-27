// Copyright 2023 Keitaro Nakamura
// Copyright 2024 Akira Matsumoto
// SPDX-License-Identifier: Apache 2.0
// Reference:
// https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
// このファイルはKeitaro Nakamuraによって作成されたcolor_detection.cppをコピーし, 一部を改変したものです.
// 変更内容: カメラによる撮影部分の削除

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class FixedPositionPublisher : public rclcpp::Node
{
public:
  FixedPositionPublisher()
  : Node("fixed_position_publisher")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_fixed_transform()
  {
    geometry_msgs::msg::TransformStamped t;
    t.child_frame_id = "target_0";  

    // 固定座標を設定
    t.transform.translation.x = 0.5;
    t.transform.translation.y = 0.4;
    t.transform.translation.z = 0.2;

    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedPositionPublisher>());
  rclcpp::shutdown();
  return 0;
}


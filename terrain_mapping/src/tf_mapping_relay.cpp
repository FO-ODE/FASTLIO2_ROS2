#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"

class TfMappingRelay : public rclcpp::Node
{
public:
  TfMappingRelay()
  : Node("tf_mapping_relay")
  {
    publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("mapping", rclcpp::QoS(100));
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "body_marker",
      rclcpp::QoS(1).transient_local().reliable());

    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf",
      rclcpp::QoS(100),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        publisher_->publish(*msg);
        update_body_in_lidar(*msg);
      });

    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        publish_lidar_square_marker();
      });
  }

private:
  void publish_lidar_square_marker()
  {
    if (!has_body_in_lidar_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "lidar";
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "terrain_mapping";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = latest_body_in_lidar_.transform.translation.x;
    marker.pose.position.y = latest_body_in_lidar_.transform.translation.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;

    marker_publisher_->publish(marker);
  }

  void update_body_in_lidar(const tf2_msgs::msg::TFMessage & msg)
  {
    for (const auto & transform : msg.transforms) {
      if (transform.header.frame_id == "lidar" && transform.child_frame_id == "body") {
        latest_body_in_lidar_ = transform;
        has_body_in_lidar_ = true;
        return;
      }

      if (transform.header.frame_id == "body" && transform.child_frame_id == "lidar") {
        tf2::Transform tf;
        tf2::Quaternion rotation(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w);
        tf.setOrigin(tf2::Vector3(
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z));
        tf.setRotation(rotation);

        tf = tf.inverse();

        latest_body_in_lidar_ = transform;
        latest_body_in_lidar_.header.frame_id = "lidar";
        latest_body_in_lidar_.child_frame_id = "body";
        latest_body_in_lidar_.transform.translation.x = tf.getOrigin().x();
        latest_body_in_lidar_.transform.translation.y = tf.getOrigin().y();
        latest_body_in_lidar_.transform.translation.z = tf.getOrigin().z();
        latest_body_in_lidar_.transform.rotation.x = tf.getRotation().x();
        latest_body_in_lidar_.transform.rotation.y = tf.getRotation().y();
        latest_body_in_lidar_.transform.rotation.z = tf.getRotation().z();
        latest_body_in_lidar_.transform.rotation.w = tf.getRotation().w();
        has_body_in_lidar_ = true;
        return;
      }
    }
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  geometry_msgs::msg::TransformStamped latest_body_in_lidar_;
  bool has_body_in_lidar_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfMappingRelay>());
  rclcpp::shutdown();
  return 0;
}
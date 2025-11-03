#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>

class OdomTFBroadcaster : public rclcpp::Node {
public:
  OdomTFBroadcaster() : Node("odom_tf_broadcaster") {
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // publish_static_transform();

    // Subscribe to /odom topic
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/laser_odometry", 10, std::bind(&OdomTFBroadcaster::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Odom TF Broadcaster started. Subscribing to /laser_odometry...");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;

    // Set header
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id;  // e.g., "odom"
    t.child_frame_id = msg->child_frame_id;    // e.g., "base_link"

    // Set translation
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    // Set rotation
    t.transform.rotation = msg->pose.pose.orientation;

    // Broadcast transform
    tf_broadcaster_->sendTransform(t);
  }

  // void publish_static_transform()
  // {
  //   geometry_msgs::msg::TransformStamped t;

  //   t.header.stamp = this->get_clock()->now();
  //   t.header.frame_id = "/odom";
  //   t.child_frame_id = "/world";

  //   t.transform.translation.x = 0.0;
  //   t.transform.translation.y = 0.0;
  //   t.transform.translation.z = 0.0;

  //   // Example: No rotation (identity quaternion)
  //   t.transform.rotation.x = 0.0;
  //   t.transform.rotation.y = 0.0;
  //   t.transform.rotation.z = 0.0;
  //   t.transform.rotation.w = 1.0;

  //   static_broadcaster_->sendTransform(t);
  //   RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s", t.header.frame_id.c_str(), t.child_frame_id.c_str());
  // }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
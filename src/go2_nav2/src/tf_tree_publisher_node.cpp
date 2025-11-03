#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>

class TfTreePublisherNode : public rclcpp::Node {
public:
  TfTreePublisherNode() : Node("tf_tree_publisher_node") {
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to /laser_odometry topic
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/laser_odometry", 10, std::bind(&TfTreePublisherNode::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Tf Tree Publisher Node started. Subscribing to /laser_odometry...");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Ensure the input message is in the correct frame
    if (msg->header.frame_id != "odom" || msg->child_frame_id != "base_link") {
      RCLCPP_WARN(this->get_logger(), "Expected /laser_odometry with frame_id 'odom' and child_frame_id 'base_link', got '%s' and '%s'",
                  msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
      return;
    }

    // Extract pose from /laser_odometry (base_link relative to odom)
    geometry_msgs::msg::Pose base_link_pose = msg->pose.pose;

    // Create odom -> base_footprint transform
    geometry_msgs::msg::TransformStamped odom_to_footprint;
    odom_to_footprint.header.stamp = msg->header.stamp;
    odom_to_footprint.header.frame_id = "odom";
    odom_to_footprint.child_frame_id = "base_footprint";

    // Set position: same x, y as base_link, z = 0
    odom_to_footprint.transform.translation.x = base_link_pose.position.x;
    odom_to_footprint.transform.translation.y = base_link_pose.position.y;
    odom_to_footprint.transform.translation.z = 0.0;

    // Extract yaw from base_link orientation
    tf2::Quaternion q_orig(
        base_link_pose.orientation.x,
        base_link_pose.orientation.y,
        base_link_pose.orientation.z,
        base_link_pose.orientation.w);
    tf2::Matrix3x3 m(q_orig);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set orientation: yaw only (roll = 0, pitch = 0)
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yaw);
    odom_to_footprint.transform.rotation = tf2::toMsg(q_yaw);

    // Broadcast odom -> base_footprint transform
    tf_broadcaster_->sendTransform(odom_to_footprint);

    // Create base_footprint -> base_link transform
    geometry_msgs::msg::TransformStamped footprint_to_base_link;
    footprint_to_base_link.header.stamp = msg->header.stamp;
    footprint_to_base_link.header.frame_id = "base_footprint";
    footprint_to_base_link.child_frame_id = "base_link";

    // Compute base_footprint -> base_link: inverse(odom -> base_footprint) * (odom -> base_link)
    tf2::Transform tf_odom_to_footprint, tf_odom_to_base_link, tf_footprint_to_base_link;
    tf2::fromMsg(odom_to_footprint.transform, tf_odom_to_footprint);

    geometry_msgs::msg::Transform odom_to_base_link_transform;
    odom_to_base_link_transform.translation.x = base_link_pose.position.x;
    odom_to_base_link_transform.translation.y = base_link_pose.position.y;
    odom_to_base_link_transform.translation.z = base_link_pose.position.z;
    odom_to_base_link_transform.rotation = base_link_pose.orientation;
    tf2::fromMsg(odom_to_base_link_transform, tf_odom_to_base_link);

    tf_footprint_to_base_link = tf_odom_to_footprint.inverse() * tf_odom_to_base_link;
    footprint_to_base_link.transform = tf2::toMsg(tf_footprint_to_base_link);

    // Broadcast base_footprint -> base_link transform
    tf_broadcaster_->sendTransform(footprint_to_base_link);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfTreePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>

class TfTreePublisherNode : public rclcpp::Node {
public:
  TfTreePublisherNode() : Node("tf_tree_publisher_node") {
    // Initialize TF broadcaster and listener
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to /laser_odometry topic
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/laser_odometry", 10, std::bind(&TfTreePublisherNode::odom_callback, this, std::placeholders::_1));

    // Create publisher for /odom topic
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    RCLCPP_INFO(this->get_logger(), "Tf Tree Publisher Node started. Subscribing to /laser_odometry...");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Republish /laser_odometry to /odom
    nav_msgs::msg::Odometry odom_msg = *msg; // Copy the incoming message
    odom_msg.header.frame_id = "odom"; // Ensure the frame_id is set appropriately
    odom_msg.child_frame_id = "base_link"; // Ensure the child_frame_id is set appropriately
    odom_pub_->publish(odom_msg);

    try {
      // Look up map -> odom transform
      geometry_msgs::msg::TransformStamped map_to_odom = tf_buffer_->lookupTransform(
          "map", "odom", msg->header.stamp, rclcpp::Duration::from_seconds(1.0));

      // Extract pose from /laser_odometry (base_link relative to map)
      geometry_msgs::msg::Pose base_link_pose = msg->pose.pose;

      // Create map -> base_footprint transform
      geometry_msgs::msg::TransformStamped map_to_footprint;
      map_to_footprint.header.stamp = msg->header.stamp;
      map_to_footprint.header.frame_id = "map";
      map_to_footprint.child_frame_id = "base_footprint";

      // Set position: same x, y as base_link, z = 0
      map_to_footprint.transform.translation.x = base_link_pose.position.x;
      map_to_footprint.transform.translation.y = base_link_pose.position.y;
      map_to_footprint.transform.translation.z = 0.0;

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
      map_to_footprint.transform.rotation = tf2::toMsg(q_yaw);

      // Compute odom -> base_footprint transform
      geometry_msgs::msg::TransformStamped odom_to_footprint;
      odom_to_footprint.header.stamp = msg->header.stamp;
      odom_to_footprint.header.frame_id = "odom";
      odom_to_footprint.child_frame_id = "base_footprint";

      // Convert transforms to tf2::Transform
      tf2::Transform tf_map_to_odom, tf_map_to_footprint, tf_odom_to_footprint;
      tf2::fromMsg(map_to_odom.transform, tf_map_to_odom);
      tf2::fromMsg(map_to_footprint.transform, tf_map_to_footprint);

      // Compute odom -> base_footprint: inverse(map -> odom) * (map -> base_footprint)
      tf_odom_to_footprint = tf_map_to_odom.inverse() * tf_map_to_footprint;
      odom_to_footprint.transform = tf2::toMsg(tf_odom_to_footprint);

      // Broadcast odom -> base_footprint transform
      tf_broadcaster_->sendTransform(odom_to_footprint);

      // Create base_footprint -> base_link transform
      geometry_msgs::msg::TransformStamped footprint_to_base_link;
      footprint_to_base_link.header.stamp = msg->header.stamp;
      footprint_to_base_link.header.frame_id = "base_footprint";
      footprint_to_base_link.child_frame_id = "base_link";

      // Compute base_footprint -> base_link: inverse(map -> base_footprint) * (map -> base_link)
      tf2::Transform tf_map_to_base_link;
      geometry_msgs::msg::Transform map_to_base_link_transform;
      map_to_base_link_transform.translation.x = base_link_pose.position.x;
      map_to_base_link_transform.translation.y = base_link_pose.position.y;
      map_to_base_link_transform.translation.z = base_link_pose.position.z;
      map_to_base_link_transform.rotation = base_link_pose.orientation;
      tf2::fromMsg(map_to_base_link_transform, tf_map_to_base_link);

      tf2::Transform tf_footprint_to_base_link = tf_map_to_footprint.inverse() * tf_map_to_base_link;
      footprint_to_base_link.transform = tf2::toMsg(tf_footprint_to_base_link);

      // Broadcast base_footprint -> base_link transform
      tf_broadcaster_->sendTransform(footprint_to_base_link);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
      return;
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfTreePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
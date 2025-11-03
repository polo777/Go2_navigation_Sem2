#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>

class FootprintTransformPublisher : public rclcpp::Node {
public:
  FootprintTransformPublisher() : Node("footprint_transform_publisher") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FootprintTransformPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    geometry_msgs::msg::TransformStamped transform;
    try {
      // Get odom -> base_link transform
      transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);

      // Publish odom -> base_footprint (x, y, yaw from base_link; z, roll, pitch from map)
      geometry_msgs::msg::TransformStamped footprint_transform;
      footprint_transform.header.stamp = this->get_clock()->now();
      footprint_transform.header.frame_id = "odom";
      footprint_transform.child_frame_id = "base_footprint";
      footprint_transform.transform.translation.x = transform.transform.translation.x;
      footprint_transform.transform.translation.y = transform.transform.translation.y;
      footprint_transform.transform.translation.z = 0.0; // Map frame z
      tf2::Quaternion quat(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w);
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      tf2::Quaternion yaw_only;
      yaw_only.setRPY(0.0, 0.0, yaw); // Map frame roll, pitch
      footprint_transform.transform.rotation.x = yaw_only.x();
      footprint_transform.transform.rotation.y = yaw_only.y();
      footprint_transform.transform.rotation.z = yaw_only.z();
      footprint_transform.transform.rotation.w = yaw_only.w();
      tf_broadcaster_->sendTransform(footprint_transform);

      // Publish base_footprint -> base_link (dynamic z, roll, pitch offset)
      geometry_msgs::msg::TransformStamped footprint_to_base_link;
      footprint_to_base_link.header.stamp = this->get_clock()->now();
      footprint_to_base_link.header.frame_id = "base_footprint";
      footprint_to_base_link.child_frame_id = "base_link";
      footprint_to_base_link.transform.translation.x = 0.0; // Same x, y
      footprint_to_base_link.transform.translation.y = 0.0;
      footprint_to_base_link.transform.translation.z = transform.transform.translation.z; // Dynamic z
      tf2::Quaternion inverse_yaw;
      inverse_yaw.setRPY(roll, pitch, 0.0); // Keep roll, pitch; zero yaw
      footprint_to_base_link.transform.rotation.x = inverse_yaw.x();
      footprint_to_base_link.transform.rotation.y = inverse_yaw.y();
      footprint_to_base_link.transform.rotation.z = inverse_yaw.z();
      footprint_to_base_link.transform.rotation.w = inverse_yaw.w();
      tf_broadcaster_->sendTransform(footprint_to_base_link);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootprintTransformPublisher>());
  rclcpp::shutdown();
  return 0;
}
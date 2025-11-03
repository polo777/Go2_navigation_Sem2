#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

class TransformRemapper : public rclcpp::Node {
public:
  TransformRemapper() : Node("transform_remapper") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TransformRemapper::timer_callback, this));
  }

private:
  void timer_callback() {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      transform.header.frame_id = "map";
      transform.child_frame_id = "odom";
      transform.header.stamp = this->get_clock()->now();
      tf_broadcaster_->sendTransform(transform);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformRemapper>());
  rclcpp::shutdown();
  return 0;
}
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomTFBroadcaster : public rclcpp::Node {
public:
  OdomTFBroadcaster() : Node("odom_tf_broadcaster") {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->declare_parameter<std::string>("parent_frame", "warthog1/odom");
    this->declare_parameter<std::string>("child_frame", "warthog1/base_link");

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&OdomTFBroadcaster::odom_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Broadcasting transform from %s to %s",
                parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();

    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;

    // Set translation from odometry position
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;

    // Set rotation from odometry orientation
    transform.transform.rotation.x = msg->pose.pose.orientation.x;
    transform.transform.rotation.y = msg->pose.pose.orientation.y;
    transform.transform.rotation.z = msg->pose.pose.orientation.z;
    transform.transform.rotation.w = msg->pose.pose.orientation.w;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string child_frame_;
  std::string parent_frame_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OdomTFBroadcaster>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
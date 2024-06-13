#include <chrono>
#include <functional>
#include <memory>
#include <string>

extern "C" {
    #include "dwm_api.h"
}

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class UwbPublisher : public rclcpp::Node {
public:
  UwbPublisher() : Node("uwb_publisher") {
    dwm_init();
    mTagName = declare_parameter("tag_name", "robot_tag");
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(mTagName, 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&UwbPublisher::timer_callback, this));
    RCLCPP_INFO(get_logger(), "UWB node has been initialized");
  }

private:
  void timer_callback() {
    int rv = 0;
    dwm_status_t uwb_status;
    dwm_pos_t uwb_pos;
    geometry_msgs::msg::PoseStamped pose;

    rv = dwm_status_get(&uwb_status);
    if (rv == DWM_OK) {
        dwm_pos_get(&uwb_pos);
        pose.header.frame_id = "uwb_base";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = uwb_pos.x;
        pose.pose.position.y = uwb_pos.y;
        pose.pose.position.z = uwb_pos.z;
        //RCLCPP_INFO(get_logger(), "UWB pos: [%5d,%5d,%5d,%3u]", uwb_pos.x, uwb_pos.y, uwb_pos.z, uwb_pos.qf);
        publisher_->publish(pose);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string mTagName;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbPublisher>());
  rclcpp::shutdown();
  return 0;
}
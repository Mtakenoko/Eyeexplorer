#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"
#include "my_messages/srv/calc_two_floats.hpp"

class ReadEncoder : public rclcpp::Node{
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  rclcpp::Service<my_messages::srv::CalcTwoFloats>::SharedPtr srv_;
  void handleService_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<my_messages::srv::CalcTwoFloats::Request> request,
    const std::shared_ptr<my_messages::srv::CalcTwoFloats::Response> response
  );
public:
  explicit ReadEncoder();
};
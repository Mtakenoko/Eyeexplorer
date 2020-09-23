#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
#define TS01_AO_CH_NUM 12

class AO_Publisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

public:
    AO_Publisher();
};

AO_Publisher::AO_Publisher()
    : Node("ts01_AO_subscriber"), count_(0)
{
    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/ts01/analog/output", qos);

    timer_ = this->create_wall_timer(
        1s,
        [this]() {
            auto msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
            msg->data.resize(TS01_AO_CH_NUM);

            static int counter = 0;
            for (int i = 0; i < TS01_AO_CH_NUM; i++)
            {
                msg->data[i] = 0.0;
            }
            msg->data[counter] = 5.0;
            counter++;
            if (counter > TS01_AO_CH_NUM)
                counter = 0;

            RCLCPP_INFO(this->get_logger(), "msg->data[%d] is 5V", counter);
            publisher_->publish(*msg);
        });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AO_Publisher>());
    rclcpp::shutdown();
    return 0;
}
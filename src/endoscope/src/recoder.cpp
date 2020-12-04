#include <iostream>
#include <string>
#include <map>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

// ImageSubscriber.hpp
class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber();

private:
    void topic_callback_(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// ImageSubscriber.cpp
ImageSubscriber::ImageSubscriber()
    : Node("recoder")
{
    // Topic Name
    std::string topic_sub("endoscope_image");

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic_sub, qos, std::bind(&ImageSubscriber::topic_callback_, this, std::placeholders::_1));
}

void ImageSubscriber::topic_callback_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    if (std::atoi(msg_image->header.frame_id.c_str()) % 100 == 0)
    {
        // Subscribe
        RCLCPP_INFO(this->get_logger(), "Save image #%s", msg_image->header.frame_id.c_str());
        cv::Mat frame_image(msg_image->height, msg_image->width, CV_8UC3, const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
        cv::imwrite("/home/takeyama/workspace/ros2_eyeexplorer/src/endoscope/data/image" + msg_image->header.frame_id + ".jpg", frame_image); // JPEGフォーマットで保存
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
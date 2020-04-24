#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "../include/endoscope/Tracker.hpp"

// TrackingSubscriber.hpp
class TrackingSubscriber : public rclcpp::Node, public Tracker
{
public:
    TrackingSubscriber();

private:
    void topic_callback_(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// TrackingSubscriber.cpp
TrackingSubscriber::TrackingSubscriber()
    : Node("tracking_test"), Tracker(Tracker::ORB, Tracker::FLANNBASED)
{
    // Topic Name
    std::string topic_sub("endoscope_image");

    //Tracker Setting
    this->setThreshold_MatchRatio(0.9f);
    this->setThreshold_TrackNo(5000);
    this->setThreshold_RANSAC(15.);
    this->setFlagShowImage(true);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_sub, qos, std::bind(&TrackingSubscriber::topic_callback_, this, std::placeholders::_1));
}

void TrackingSubscriber::topic_callback_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    RCLCPP_INFO(this->get_logger(), "Received image #%s", msg_image->header.frame_id.c_str());

    // Subscribe
    cv::Mat frame_image(msg_image->height, msg_image->width, CV_8UC3, const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);

    // tracking
    this->setFrame_Id(msg_image->header.frame_id);
    this->process(frame_image);
    this->showMatchedImage();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingSubscriber>());
    rclcpp::shutdown();
    return 0;
}
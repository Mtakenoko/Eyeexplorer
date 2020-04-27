#include <iostream>
#include <string>
#include <map>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "../include/endoscope/Tracker.hpp"

// TrackingSubscriber.hpp
class TrackingSubscriber : public rclcpp::Node
{
public:
    TrackingSubscriber();

private:
    void topic_callback_(const sensor_msgs::msg::Image::SharedPtr msg);
    void setPubMsg(sensor_msgs::msg::PointCloud2 &msg_pointcloud2);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    // Tracker tracker;
    std::unique_ptr<Tracker> tracker_ = std::make_unique<Tracker>();
};

// TrackingSubscriber.cpp
TrackingSubscriber::TrackingSubscriber()
    : Node("tracking")
{
    // Topic Name
    std::string topic_sub("endoscope_image");
    std::string topic_pub("tracking_point");

    //Tracker Setting
    tracker_->setThreshold_MatchRatio(0.8f);
    tracker_->setThreshold_MatchRatio_other(0.0f);
    tracker_->setThreshold_TrackNo(1000);
    tracker_->setThreshold_RANSAC(10.);
    tracker_->setDetector(Tracker::DetectorType::ORB);
    tracker_->setMatcher(Tracker::MatcerType::FLANNBASED);
    tracker_->setFlagShowImage(true);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub, qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic_sub, qos, std::bind(&TrackingSubscriber::topic_callback_, this, std::placeholders::_1));
}

void TrackingSubscriber::topic_callback_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    RCLCPP_INFO(this->get_logger(), "Received image #%s", msg_image->header.frame_id.c_str());

    // Subscribe
    cv::Mat frame_image(msg_image->height, msg_image->width, CV_8UC3, const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);

    // tracking
    tracker_->setFrame_Id(msg_image->header.frame_id);
    tracker_->process(frame_image);
    tracker_->showMatchedImage();

    auto msg_pointcloud2 = std::make_unique<sensor_msgs::msg::PointCloud2>();
    TrackingSubscriber::setPubMsg(*msg_pointcloud2);
    publisher_->publish(std::move(msg_pointcloud2));
}

void TrackingSubscriber::setPubMsg(sensor_msgs::msg::PointCloud2 &msg_pointcloud2)
{
    std::vector<std::map<unsigned int, LastFrame>> find_frame;
    find_frame = tracker_->getFindTrack();

    msg_pointcloud2.header = std_msgs::msg::Header();
    msg_pointcloud2.header.stamp = rclcpp::Clock().now();
    msg_pointcloud2.header.frame_id = "track_point";

    msg_pointcloud2.is_bigendian = false;
    msg_pointcloud2.is_dense = false;

    msg_pointcloud2.height = 1;
    msg_pointcloud2.width = find_frame.size();

    sensor_msgs::msg::PointField::_offset_type offset = 0;
    msg_pointcloud2.fields.resize(4);

    msg_pointcloud2.fields[0].name = "track_no";
    msg_pointcloud2.fields[0].count = 1;
    msg_pointcloud2.fields[0].offset = offset;
    msg_pointcloud2.fields[0].datatype = sensor_msgs::msg::PointField::UINT32;
    offset += 4;

    msg_pointcloud2.fields[1].name = "frame_id";
    msg_pointcloud2.fields[1].count = 1;
    msg_pointcloud2.fields[1].offset = offset;
    msg_pointcloud2.fields[1].datatype = sensor_msgs::msg::PointField::INT32;
    offset += 4;

    msg_pointcloud2.fields[2].name = "u";
    msg_pointcloud2.fields[2].count = 1;
    msg_pointcloud2.fields[2].offset = offset;
    msg_pointcloud2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    offset += 4;

    msg_pointcloud2.fields[3].name = "v";
    msg_pointcloud2.fields[3].count = 1;
    msg_pointcloud2.fields[3].offset = offset;
    msg_pointcloud2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    offset += 4;

    msg_pointcloud2.point_step = offset;
    msg_pointcloud2.row_step = msg_pointcloud2.point_step * msg_pointcloud2.width;
    msg_pointcloud2.data.resize(msg_pointcloud2.row_step * msg_pointcloud2.height);

    auto floatdata = reinterpret_cast<float *>(msg_pointcloud2.data.data());
    for (uint32_t i = 0; i < msg_pointcloud2.width; i++)
    {
        auto itr = find_frame[i].begin();
        while (itr != find_frame[i].end())
        {
            floatdata[i * (msg_pointcloud2.point_step / sizeof(unsigned int)) + 0] = itr->first;
            floatdata[i * (msg_pointcloud2.point_step / sizeof(int)) + 1] = std::atoi(itr->second.frame_id.c_str());
            floatdata[i * (msg_pointcloud2.point_step / sizeof(float)) + 2] = itr->second.x2;
            floatdata[i * (msg_pointcloud2.point_step / sizeof(float)) + 3] = itr->second.y2;
            itr++;
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingSubscriber>());
    rclcpp::shutdown();
    return 0;
}
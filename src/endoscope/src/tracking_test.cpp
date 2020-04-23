#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <unordered_map>

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
    this->setThreshold_MatchRatio(0.7f);
    this->setThreshold_TrackNo(500);
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

    // 検索キーをトラック番号に変更
    // std::map<int, LastFrame> のintが検索キー(iterator使うときにこうしとくと便利)
    // last_frameではこのintにはidx2が入っていたが、ここではトラック番号に変更する
    std::vector<std::unordered_multimap<unsigned long int, LastFrame>> find_track;
    std::vector<std::unordered_multimap<int, LastFrame>> last_frame = this->getLastFrame();
    for (size_t i = 0; i < last_frame.size(); i++)
    {
        std::unordered_multimap<unsigned long int, LastFrame> track;
        auto itr = last_frame[i].begin();
        while (itr != last_frame[i].end())
        {
            track.insert(std::make_pair(itr->second.track_no, itr->second));
            itr++;
        }
        find_track.push_back(track);
    }

    // trackの番号順に読み出し
    // printf("track_no = %zu, find_track.size() = %zu\n", track_no, find_track.size());
    static unsigned long int min_track_no = 0;
    bool min_setting = true;
    for (unsigned long int i = min_track_no; i < this->track_no; i++)
    {
        auto itr_1 = find_track[0].find(i);

        for (size_t j = 0; j < find_track.size(); j++)
        {
            bool flag_first = false;

            if (j == 0)
            {
                // Frameが0の時に対応点の登録があれば
                if (itr_1 != find_track[0].end())
                    flag_first = true;
            }
            else
            {
                // Frameが1以上で、一個前のフレームに登録があれば
                if (itr_1 != find_track[j - 1].end())
                    flag_first = true;
            }

            // トラック番号iについて、j番目のフレーム内にありますか？
            auto itr = find_track[j].find(i);
            if (itr != find_track[j].end())
            {
                if (flag_first)
                {
                    // j-1番目のフレームにもトラック番号iが登場するなら
                    // printf("TrackNo.%zu, FrameNo:%d: (x1,y1)=(%f, %f)\n", itr_1->first, std::atoi(itr_1->second.frame_id.c_str()), itr->second.x1, itr->second.y1);
                    // printf("TrackNo.%zu, FrameNo:%d: (x2,y2)=(%f, %f)\n", itr->first, std::atoi(itr->second.frame_id.c_str()), itr->second.x2, itr->second.y2);
                }
                if (min_setting)
                {
                    min_setting = false;
                    min_track_no = itr->first;
                }
            }
            itr_1 = itr;
        }
        // printf("track_No.%zu is end\n", i);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingSubscriber>());
    rclcpp::shutdown();
    return 0;
}
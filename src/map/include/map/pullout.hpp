#ifndef PULLOUT_ENDOSCOPE2_HPP__
#define PULLOUT_ENDOSCOPE2_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/opencv.hpp>

#define DISTANCE_SAFETY 0.001
#define AO_PORT_BRAKE 1
#define AO_PORT_SOLENOID 2
#define TS01_AO_CH_NUM 12

class Eye_Shape
{
public:
    Eye_Shape()
    {
        Orientation = (cv::Mat_<float>(4, 1) << 0., 0., 0., 1.);
    };
    cv::Mat Orientation;
    cv::Point3d Position;
    cv::Point3d Scale;
};

class PullOut_Endoscope : public rclcpp::Node
{
public:
    PullOut_Endoscope();
    void calc_distance();
    void publish();
    Eye_Shape eye_shape;
    cv::Point3d Ptip;

private:
    void initialize();
    void input_Ptip_data(const geometry_msgs::msg::Transform::SharedPtr msg_tip);
    void input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_tip);
    void topic_Ptip_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_tip);
    void topic_eyeball_callback_(const visualization_msgs::msg::Marker::SharedPtr msg_tip);
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subscription_pointcloud_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_insertpoint_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    bool flag_SetPtip;
    bool flag_SetEyeball;
    bool flag_safety;
};

PullOut_Endoscope::PullOut_Endoscope()
    : Node("pullout"), flag_SetPtip(false), flag_SetEyeball(false), flag_safety(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/ts01/analog/output", 10);

    subscription_pointcloud_ = this->create_subscription<geometry_msgs::msg::Transform>(
        "/endoscope_transform", qos,
        std::bind(&PullOut_Endoscope::topic_Ptip_callback_, this, std::placeholders::_1));
    subscription_insertpoint_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/EyeBall", qos,
        std::bind(&PullOut_Endoscope::topic_eyeball_callback_, this, std::placeholders::_1));
}

void PullOut_Endoscope::topic_Ptip_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_tip)
{
    this->input_Ptip_data(msg_tip);
    this->calc_distance();
    this->publish();
}

void PullOut_Endoscope::topic_eyeball_callback_(const visualization_msgs::msg::Marker::SharedPtr msg_marker)
{
    this->input_eyeball_data(msg_marker);
}

void PullOut_Endoscope::input_Ptip_data(const geometry_msgs::msg::Transform::SharedPtr msg_tip)
{
    Ptip.x = msg_tip->translation.x;
    Ptip.y = msg_tip->translation.y;
    Ptip.z = msg_tip->translation.z;
    flag_SetPtip = true;
}

void PullOut_Endoscope::input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    eye_shape.Position.x = msg_eyeball->pose.position.x;
    eye_shape.Position.y = msg_eyeball->pose.position.y;
    eye_shape.Position.z = msg_eyeball->pose.position.z;
    eye_shape.Scale.x = msg_eyeball->scale.x;
    eye_shape.Scale.y = msg_eyeball->scale.y;
    eye_shape.Scale.z = msg_eyeball->scale.z;
    flag_SetEyeball = true;
}

void PullOut_Endoscope::calc_distance()
{
    if (!flag_SetPtip || !flag_SetEyeball)
        return;

    double dist;
    dist = std::fabs(std::sqrt((eye_shape.Position.x - Ptip.x) * (eye_shape.Position.x - Ptip.x) +
                               (eye_shape.Position.y - Ptip.y) * (eye_shape.Position.y - Ptip.y) +
                               (eye_shape.Position.z - Ptip.z) * (eye_shape.Position.z - Ptip.z)) -
                     (eye_shape.Scale.x / 2. + eye_shape.Scale.y / 2. + eye_shape.Scale.z / 2.) / 3.);
    RCLCPP_INFO(this->get_logger(), "Distance : %lf", dist);
    if (dist > DISTANCE_SAFETY)
    {
        RCLCPP_INFO(this->get_logger(), "Pull out endoscope!");
        flag_safety = true;
    }
}

void PullOut_Endoscope::publish()
{
    // Set message
    auto ts01_aout_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    ts01_aout_msg->data.resize(TS01_AO_CH_NUM);
    for (int i = 0; i < TS01_AO_CH_NUM; i++)
    {
        ts01_aout_msg->data[i] = 0.0;
    }
    if (flag_safety)
    {
        ts01_aout_msg->data[AO_PORT_BRAKE] = 5.0;    // ブレーキは引き続きON
        ts01_aout_msg->data[AO_PORT_SOLENOID] = 0.0; // 抜去
        flag_safety = false;
    }
    else
    {
        ts01_aout_msg->data[AO_PORT_BRAKE] = 5.0;    // ブレーキは引き続きON
        ts01_aout_msg->data[AO_PORT_SOLENOID] = 5.0; // 維持
    }

    // Publish
    publisher_->publish(*ts01_aout_msg);
}
#endif

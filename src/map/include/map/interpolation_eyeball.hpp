#ifndef INTERPOLATION_EYEBALL_HPP
#define INTERPOLATION_EYEBALL_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <math.h>

class Interpolation : public rclcpp::Node
{
public:
    Interpolation();
    ~Interpolation();
    void setQoS(const size_t &depth,
                const rmw_qos_reliability_policy_t &reliability_policy,
                const rmw_qos_history_policy_t &history_policy);
    void setCutParameter(const int &cut);
    void setThreshDistance(const double &thresh);

private:
    void topic_markerarray_callback_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_markerarray);
    void topic_eyeball_callback_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball);
    void input_markerarray_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_markerarray);
    void input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball);
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_markerarray_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_eyeball_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_markerarray_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::QoS qos;

private:
    bool checknearbypoint(double x, double y, double z);
    visualization_msgs::msg::MarkerArray marker_array;
    int num_cut;
    double thresh_distance;
    bool flag_set_markerarray;
    bool flag_set_eyeball;
};

Interpolation::Interpolation() : Node("interpolator"), qos(rmw_qos_profile_default.depth),
                                 num_cut(10), thresh_distance(0.003),
                                 flag_set_markerarray(false), flag_set_eyeball(false)
{
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupancy_grid/interpolated_marker", 10);
    subscription_markerarray_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/occupancy_grid/marker", qos, std::bind(&Interpolation::topic_markerarray_callback_, this, std::placeholders::_1));
    subscription_eyeball_ = this->create_subscription<visualization_msgs::msg::Marker>("/EyeBall", qos, std::bind(&Interpolation::topic_eyeball_callback_, this, std::placeholders::_1));
}

Interpolation::~Interpolation()
{
}

void Interpolation::setQoS(const size_t &depth, const rmw_qos_reliability_policy_t &reliability_policy, const rmw_qos_history_policy_t &history_policy)
{
    this->qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    this->qos.reliability(reliability_policy);
}

void Interpolation::setCutParameter(const int &cut)
{
    this->num_cut = cut;
}

void Interpolation::setThreshDistance(const double &thresh)
{
    this->thresh_distance = thresh;
}

void Interpolation::topic_markerarray_callback_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_markerarray)
{
    this->input_markerarray_data(msg_markerarray);
    flag_set_markerarray = true;
}

void Interpolation::topic_eyeball_callback_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    this->input_eyeball_data(msg_eyeball);
    flag_set_eyeball = true;
}

void Interpolation::input_markerarray_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_markerarray)
{
    marker_array = *msg_markerarray;
}

void Interpolation::input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    double r = msg_eyeball->scale.x / 2.0;
    double x_0 = msg_eyeball->pose.position.x;
    double y_0 = msg_eyeball->pose.position.y;
    double z_0 = msg_eyeball->pose.position.z;
    std::cout << "x, y, z, r : " << x_0 << ", " << y_0 << ", " << z_0 << ", " << r << std::endl;

    // 等間隔で点群を生成する
    // 球は極座標系で表す。（x=rsin(s)cos(t), y=rsin(s)sin(t), z=rcos(s)）
    // 0<=s<=pi, 0<=t<=2*pi
    auto markerarray_msgs_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
    markerarray_msgs_->markers = marker_array.markers;
    int id = (int)marker_array.markers.size();
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    double radius = marker_array.markers.begin()->scale.x;
    for (int i = 0; i < num_cut; i++)
    {
        double theta = M_PI * (double)i / (double)num_cut;
        for (int j = 0; j < 2 * num_cut; j++)
        {
            double phi = M_PI * (double)j / (double)num_cut;
            double x = r * std::sin(theta) * std::cos(phi) + x_0;
            double y = r * std::sin(theta) * std::sin(phi) + y_0;
            double z = r * std::cos(theta) + z_0;
            if (checknearbypoint(x, y, z))
            {
                // std::cout << "x, y, z: " << x << ", " << y << ", " << z << std::endl;
                visualization_msgs::msg::Marker marker_msg;

                marker_msg.header.frame_id = "world";
                marker_msg.header.stamp = clock->now();
                marker_msg.ns = "occupancy_grid";
                marker_msg.id = id++;

                // 形状
                marker_msg.type = visualization_msgs::msg::Marker::CUBE;
                marker_msg.action = visualization_msgs::msg::Marker::ADD;

                // 大きさ
                marker_msg.scale.x = radius;
                marker_msg.scale.y = radius;
                marker_msg.scale.z = radius;

                // 色
                marker_msg.color.a = 1.0;
                marker_msg.color.r = 0.0;
                marker_msg.color.g = 1.0;
                marker_msg.color.b = 1.0;

                // 位置・姿勢
                marker_msg.pose.position.x = x;
                marker_msg.pose.position.y = y;
                marker_msg.pose.position.z = z;
                marker_msg.pose.orientation.x = 0.0;
                marker_msg.pose.orientation.y = 0.0;
                marker_msg.pose.orientation.z = 0.0;
                marker_msg.pose.orientation.w = 1.0;
                markerarray_msgs_->markers.push_back(marker_msg);
            }
        }
    }

    publisher_->publish(*markerarray_msgs_);
}

bool Interpolation::checknearbypoint(double x, double y, double z)
{
    for (auto itr = marker_array.markers.begin(); itr != marker_array.markers.end(); itr++)
    {
        double distance = std::sqrt((itr->pose.position.x - x) * (itr->pose.position.x - x) +
                                    (itr->pose.position.y - y) * (itr->pose.position.y - y) +
                                    (itr->pose.position.z - z) * (itr->pose.position.z - z));
        if (distance < thresh_distance)
            return false;
    }
    return true;
}
#endif
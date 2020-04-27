#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "../include/endoscope/Tracker.hpp"

// TrackingSubscriber.hpp
class TrackingSubscriber : public rclcpp::Node, public std::enable_shared_from_this<TrackingSubscriber>
{
public:
    TrackingSubscriber();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_track,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm);

private:
    void setPubMsg(sensor_msgs::msg::Image msg_track, geometry_msgs::msg::Transform msg_arm);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    sensor_msgs::msg::PointCloud2 pointcloud2;
};

// TrackingSubscriber.cpp
TrackingSubscriber::TrackingSubscriber()
    : Node("tracking_subscriber")
//       ,sub_track(this, "endoscope_image"), sub_arm(this, "endoscope_transform"), sync(sub_track, sub_arm, 10)
{
    // Topic Name
    std::string topic_sub_track("endoscope_image");
    std::string topic_sub_arm("endoscope_transform");
    std::string topic_pub("tracking_endoscope");

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    RCLCPP_INFO(this->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos);

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_track_(this, topic_sub_track, qos.get_rmw_qos_profile());
    message_filters::Subscriber<geometry_msgs::msg::Transform> sub_arm_(this, topic_sub_arm, qos.get_rmw_qos_profile());
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync_(sub_track_, sub_arm_, 10);
    sync_.registerCallback(std::bind(&TrackingSubscriber::topic_callback_, this, std::placeholders::_1, std::placeholders::_2));
}

void TrackingSubscriber::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_track,
                                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm)
{
    printf("unko\n");
    // RCLCPP_INFO(this->get_logger(), "Received Message #%s", msg_track->header.frame_id.c_str());

    std::cout << "msg_track [" << msg_track->data[0] << "]" << std::endl;
    std::cout << "msg_track [" << msg_arm->translation.x << "]" << std::endl;

    // setPubMsg(*msg_track, *msg_arm);
    // publisher_->publish(pointcloud2);
}

/*void TrackingSubscriber::setPubMsg(sensor_msgs::msg::Image msg_track, geometry_msgs::msg::Transform msg_arm)
{
    std::vector<std::map<unsigned int, LastFrame>> find_frame;
    sensor_msgs::msg::Image pointcloud2;

    pointcloud2.header = std_msgs::msg::Header();
    pointcloud2.header.stamp = rclcpp::Clock().now();
    pointcloud2.header.frame_id = "track_point";

    pointcloud2.is_bigendian = false;
    pointcloud2.is_dense = false;

    pointcloud2.height = 1;
    pointcloud2.width = find_frame.size();

    sensor_msgs::msg::PointField::_offset_type offset = 0;
    pointcloud2.fields.resize(6);

    pointcloud2.fields[0].name = "track_no";
    pointcloud2.fields[0].count = 1;
    pointcloud2.fields[0].offset = offset;
    pointcloud2.fields[0].datatype = sensor_msgs::msg::PointField::UINT32;
    offset += 4;

    pointcloud2.fields[1].name = "frame_id";
    pointcloud2.fields[1].count = 1;
    pointcloud2.fields[1].offset = offset;
    pointcloud2.fields[1].datatype = sensor_msgs::msg::PointField::INT32;
    offset += 4;

    pointcloud2.fields[2].name = "u";
    pointcloud2.fields[2].count = 1;
    pointcloud2.fields[2].offset = offset;
    pointcloud2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    offset += 4;

    pointcloud2.fields[3].name = "v";
    pointcloud2.fields[3].count = 1;
    pointcloud2.fields[3].offset = offset;
    pointcloud2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    offset += 4;

    pointcloud2.fields[4].name = "Revo";
    pointcloud2.fields[4].count = 4;
    pointcloud2.fields[4].offset = offset;
    pointcloud2.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    offset += 4;

    pointcloud2.fields[5].name = "trans";
    pointcloud2.fields[5].count = 3;
    pointcloud2.fields[5].offset = offset;
    pointcloud2.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    offset += 4;

    pointcloud2.point_step = offset;
    pointcloud2.row_step = pointcloud2.point_step * pointcloud2.width;
    pointcloud2.data.resize(pointcloud2.row_step * pointcloud2.height);

    auto floatdata = reinterpret_cast<float *>(pointcloud2.data.data());
    for (uint32_t i = 0; i < pointcloud2.width; i++)
    {
        auto itr = find_frame[i].begin();
        while (itr != find_frame[i].end())
        {
            floatdata[i * (pointcloud2.point_step / sizeof(unsigned int)) + 0] = itr->first;
            floatdata[i * (pointcloud2.point_step / sizeof(int)) + 1] = std::atoi(itr->second.frame_id.c_str());
            floatdata[i * (pointcloud2.point_step / sizeof(float)) + 2] = itr->second.x2;
            floatdata[i * (pointcloud2.point_step / sizeof(float)) + 3] = itr->second.y2;
            floatdata[i * (pointcloud2.point_step / sizeof(float)) + 4] = itr->second.y2;
            floatdata[i * (pointcloud2.point_step / sizeof(float)) + 5] = itr->second.y2;

            itr++;
        }
    }
}*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingSubscriber>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("markerpoint_publisher");
    rclcpp::Logger node_logger = node->get_logger();

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Set quality of service profile based on command line options.
    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    std::string topic_pub_pointcloud("marker_point");
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());
    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        for (int i = 0; i < 10; i++)
        {
            pcl::PointXYZRGB pt = pcl::PointXYZRGB(0, 255, 255);
            if (i == 0)
            {
                pt.x = -0.2335;
                pt.y = -0.3685;
                pt.z = -0.0560;
            }
            else if (i == 1)
            {
                pt.x = -0.2335;
                pt.y = -0.3060;
                pt.z = -0.0560;
            }
            else if (i == 2)
            {
                pt.x = -0.2335;
                pt.y = -0.2430;
                pt.z = -0.0560;
            }
            else if (i == 3)
            {
                pt.x = -0.2335;
                pt.y = -0.1803;
                pt.z = -0.0560;
            }
            else if (i == 4)
            {
                pt.x = -0.2335;
                pt.y = -0.3060;
                pt.z = -0.1082;
            }
            else if (i == 5)
            {
                pt.x = -0.2335;
                pt.y = -0.2430;
                pt.z = -0.1082;
            }
            else if (i == 6)
            {
                pt.x = -0.2335;
                pt.y = -0.3685;
                pt.z = -0.1610;
            }
            else if (i == 7)
            {
                pt.x = -0.2335;
                pt.y = -0.3060;
                pt.z = -0.1610;
            }
            else if (i == 8)
            {
                pt.x = -0.2335;
                pt.y = -0.2430;
                pt.z = -0.1610;
            }
            else if (i == 9)
            {
                pt.x = -0.2335;
                pt.y = -0.1803;
                pt.z = -0.1610;
            }
            cloud_.push_back(pt);
        }
        auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msg_);
        pc2_msg_->header.frame_id = "world";
        pc2_msg_->header.stamp = rclcpp::Clock().now();
        pub_pointcloud->publish(*pc2_msg_);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
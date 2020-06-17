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
        for (int i = 0; i < 20; i++)
        {
            //　色
            pcl::PointXYZRGB pt;
            if (i < 10)
            {
                pt = pcl::PointXYZRGB(0, 255, 255);
            }
            else if (i < 20)
            {
                pt = pcl::PointXYZRGB(255, 0, 255);
            }

            // 位置情報
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
            else if (i == 10)
            {
                pt.x = -0.191;
                pt.y = 0.3415;
                pt.z = -0.210;
            }
            else if (i == 11)
            {
                pt.x = -0.191;
                pt.y = 0.404;
                pt.z = -0.210;
            }
            else if (i == 12)
            {
                pt.x = -0.191;
                pt.y = 0.467;
                pt.z = -0.210;
            }
            else if (i == 13)
            {
                pt.x = -0.191;
                pt.y = 0.5297;
                pt.z = -0.210;
            }
            else if (i == 14)
            {
                pt.x = -0.1388;
                pt.y = 0.404;
                pt.z = -0.210;
            }
            else if (i == 15)
            {
                pt.x = -0.1388;
                pt.y = 0.467;
                pt.z = -0.210;
            }
            else if (i == 16)
            {
                pt.x = -0.086;
                pt.y = 0.3415;
                pt.z = -0.210;
            }
            else if (i == 17)
            {
                pt.x = -0.086;
                pt.y = 0.404;
                pt.z = -0.210;
            }
            else if (i == 18)
            {
                pt.x = -0.086;
                pt.y = 0.467;
                pt.z = -0.210;
            }
            else if (i == 19)
            {
                pt.x = -0.086;
                pt.y = 0.5297;
                pt.z = -0.210;
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
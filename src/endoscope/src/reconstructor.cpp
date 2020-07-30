#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../include/endoscope/Reconstruction.hpp"
#include "../option/options_reconstructor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Ceres Solver Logging用　Initializer
    google::InitGoogleLogging(argv[0]);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    bool show_camera = false;
    size_t mode = 0;
    bool est_move = false;
    float thresh_knn_ratio = 0.7f;
    float thresh_ransac = 5.0;
    int cpu_core = 8;
    size_t num_scene = 4;
    size_t matching = Reconstruction::Matching::BruteForce;
    int extractor = Extractor::DetectorType::AKAZE;
    size_t publish = Reconstruction::Publish::FILTER_HOLD;
    // Configure demo parameters with command line options.
    if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy,
                               &show_camera, &mode, &est_move, &thresh_knn_ratio, &thresh_ransac,
                               &cpu_core, &num_scene, &matching, &extractor, &publish))
    {
        return 0;
    }

    // Reconstructorの設定
    auto reconstructor = Reconstruction();
    reconstructor.setThreshold_knn_ratio(thresh_knn_ratio);
    reconstructor.setThreshold_ransac(thresh_ransac);
    reconstructor.setFlagEstimationMovement(est_move);
    reconstructor.setFlagShowImage(show_camera);
    reconstructor.setCPUCoreforBundler(cpu_core);
    reconstructor.setSceneNum(num_scene);
    reconstructor.setPublishType(publish);
    reconstructor.setUseMode(mode);

    // node
    auto node = rclcpp::Node::make_shared("reconstructor"); //Set QoS to Publish

    // Topic Name
    std::string topic_sub_track("endoscope_image");
    std::string topic_sub_arm("endoscope_transform");
    std::string topic_pub("pointcloud");

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Pub/Subの設定
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    auto publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub, qos);
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_track_(node.get(), topic_sub_track);
    message_filters::Subscriber<geometry_msgs::msg::Transform> sub_arm_(node.get(), topic_sub_arm);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync_(sub_track_, sub_arm_, 1000);
    sync_.registerCallback(std::bind(&Reconstruction::topic_callback_, reconstructor, std::placeholders::_1, std::placeholders::_2, publisher_));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
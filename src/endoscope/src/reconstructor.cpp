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
    bool ceres_stdout = false;
    size_t mode = Reconstruction::UseMode::EYE;
    bool est_move = false;
    float thresh_knn_ratio = 0.7f;
    float thresh_ransac = 5.0;
    int cpu_core = 8;
    size_t num_scene = 4;
    size_t matching_method = Reconstruction::Matching::BruteForce;
    size_t extractor = Extractor::DetectorType::AKAZE;
    // Configure demo parameters with command line options.
    if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy,
                               &show_camera, &ceres_stdout, &mode, &est_move, &thresh_knn_ratio, &thresh_ransac,
                               &cpu_core, &num_scene, &matching_method, &extractor))
    {
        return 0;
    }

    // Reconstructorの設定
    auto reconstructor = Reconstruction();
    reconstructor.setThreshold_knn_ratio(thresh_knn_ratio);
    reconstructor.setThreshold_ransac(thresh_ransac);
    reconstructor.setFlagEstimationMovement(est_move);
    reconstructor.setFlagShowImage(show_camera);
    reconstructor.setFlagCeresstdout(ceres_stdout);
    reconstructor.setCPUCoreforBundler(cpu_core);
    reconstructor.setSceneNum(num_scene);
    reconstructor.setUseMode(mode);
    reconstructor.setMatchingMethod(matching_method);
    reconstructor.setExtractor(extractor);

    // node
    auto node = rclcpp::Node::make_shared("reconstructor"); //Set QoS to Publish

    // Subscriber Topic Name
    std::string topic_sub_track("endoscope_image");
    std::string topic_sub_arm("endoscope_transform");
    // Publisher Topic Name
    std::string topic_pub_normal("/pointcloud/normal");
    std::string topic_pub_normal_hold("/pointcloud/normal_hold");
    std::string topic_pub_BA("/pointcloud/BA");
    std::string topic_pub_BA_hold("/pointcloud/BA_hold");
    std::string topic_pub_filtered("/pointcloud/filtered");
    std::string topic_pub_filtered_hold("/pointcloud/filtered_hold");
    std::string topic_pub_est("/pointcloud/est");
    std::string topic_pub_est_hold("/pointcloud/est_hold");
    std::string topic_pub_matching_image("/matching_image");
    std::string topic_pub_nomatching_image("/nomatching_image");

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Pub/Subの設定
    auto publisher_normal_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_normal, qos);
    auto publisher_normal_hold_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_normal_hold, qos);
    auto publisher_BA_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_BA, qos);
    auto publisher_BA_hold_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_BA_hold, qos);
    auto publisher_filtered_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_filtered, qos);
    auto publisher_filtered_hold_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_filtered_hold, qos);
    auto publisher_est_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_est, qos);
    auto publisher_est_hold_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_est_hold, qos);
    auto publisher_matching_image_ = node->create_publisher<sensor_msgs::msg::Image>(topic_pub_matching_image, qos);
    auto publisher_nomatching_image_ = node->create_publisher<sensor_msgs::msg::Image>(topic_pub_nomatching_image, qos);

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_track_(node.get(), topic_sub_track);
    message_filters::Subscriber<geometry_msgs::msg::Transform> sub_arm_(node.get(), topic_sub_arm);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync_(sub_track_, sub_arm_, 1000);
    sync_.registerCallback(std::bind(&Reconstruction::topic_callback_, reconstructor, std::placeholders::_1, std::placeholders::_2,
                                     publisher_normal_, publisher_normal_hold_, publisher_BA_, publisher_BA_hold_,
                                     publisher_filtered_, publisher_filtered_hold_, publisher_est_, publisher_est_hold_,
                                     publisher_matching_image_, publisher_nomatching_image_));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
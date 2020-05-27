#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ktl.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../include/calibration/calibrate_arm.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Ceres Solver Logging用　Initializer
    google::InitGoogleLogging(argv[0]);

    // Topic Name
    std::string topic_sub_track("endoscope_image");
    std::string topic_sub_joint("joint_states");
    auto calib_param = Calib_Param();

    // node
    auto node = rclcpp::Node::make_shared("arm_param_calibrator"); //Set QoS to Publish

    // Set quality of service profile based on command line options.
    // コマンドラインでのQoSの設定（よくわからん）
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Subscribeの設定
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_track_(node.get(), topic_sub_track);
    message_filters::Subscriber<sensor_msgs::msg::JointState> sub_arm_(node.get(), topic_sub_joint);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::JointState> sync_(sub_track_, sub_arm_, 1000);
    sync_.registerCallback(std::bind(&Calib_Param::topic_callback_, calib_param, std::placeholders::_1, std::placeholders::_2));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

int main(int argc, char **argv)
{
    //Initialize
    rclcpp::init(argc, argv);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("octomap_test");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // 空間を作る
    std::shared_ptr<octomap::ColorOcTree> tree;
    tree = std::make_shared<octomap::ColorOcTree>(0.05);

    // レイを一つ飛ばして空間を削り出す
    double x0(0.0), y0(0.0), z0(0.0);
    double x1(0.001), y1(0.02), z1(0.005);
    octomap::point3d origin(x0, y0, z0); // 計測原点。カメラの三次元座標
    octomap::point3d end(x1, y1, z1);    // 計測した1点の三次元座標
    tree->insertRay(origin, end);        // レイを飛ばして空間を削り出す

    // 空間の更新
    tree->updateInnerOccupancy();

    // データの保存
    tree->writeBinary("/src/test/data/map.bt");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
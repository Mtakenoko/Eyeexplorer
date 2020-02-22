#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct CostFunctor
{
    template <typename T>
    bool operator()(const T *const x, T *residual) const
    {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

void ceres_test(char **argv)
{
    //////////////////////
    google::InitGoogleLogging(argv[0]);

    //最適化問題を解く変数と初期値の設定
    double initial_x = 5.0;
    double x = initial_x;

    //最適化問題を解く用のオブジェクトの生成
    ceres::Problem problem;

    //コスト関数の設定
    //AutoDiffCostFunctionを使うことで、自動的にヤコビ行列を設定できる
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);

    //最適化問題に残差項と変数を設定
    problem.AddResidualBlock(cost_function, NULL, &x);

    //最適化の実行
    ceres::Solver::Options options; //最適化のオプション設定用構造体
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false; //最適化の結果を標準出力に表示する。
    ceres::Solver::Summary summary;                      //最適化の結果を格納するよう構造体
    ceres::Solve(options, &problem, &summary);           //最適化の実行

    //結果の表示
    // std::cout << summary.BriefReport() << std::endl;
    std::cout << summary.FullReport() << std::endl;
    std::cout << "x:" << initial_x << "->" << x << std::endl;
}

int main(int argc, char **argv)
{

    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("pointcloud");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("ceres_test");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    ceres_test(argv);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
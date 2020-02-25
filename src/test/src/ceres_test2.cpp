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

#include "glog/logging.h"

#include "../include/test/bundle_aj.hpp"

int main(int argc, char **argv)
{

    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("ceres_test2");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //

    google::InitGoogleLogging(argv[0]);
    if (argc != 2)
    {
        std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
        return 1;
    }
    BALProblem bal_problem;
    if (!bal_problem.LoadFile(argv[1]))
    {
        std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
        return 1;
    }
    // const double *observations = bal_problem.observations();
    double *pre_mutable_camera = bal_problem.mutable_camera_for_observation(0);
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i)
    {
        static int count = 0;
        //最適化問題に残差項と変数を設定
        // ceres::CostFunction *cost_function =
        //     new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(new SnavelyReprojectionError(observations[2 * i + 0], observations[2 * i + 1]));
        ceres::CostFunction *cost_function =
            SnavelyReprojectionError::Create(bal_problem.observations()[2 * i + 0], bal_problem.observations()[2 * i + 1]);
        problem.AddResidualBlock(cost_function, NULL, bal_problem.mutable_camera_for_observation(i), bal_problem.mutable_point_for_observation(i));
        // bal_problem.observations()[2 * i + 0]は画像のx, yのことっぽい
        // bal_problem.mutable_camera_for_observation(i)はカメラの情報っぽい（R, t, f, k1, k2の9変数）
        // bal_problem.mutable_point_for_observation(i)は点の3次元座標っぽい（3変数）
        // つまりi番目の特徴点のときのカメラの位置座標および内部パラメータと元画像平面での座標、そして三次元復元した点の座標が必要
        // printf("observationsがあーだこーだ  = #%d [%lf %lf]\n", count, observations[2 * i + 0], observations[2 * i + 1]);
        // printf("mutablecameraがあーだこーだ = #%d [%lf %lf %lf %lf %lf %lf %lf %lf %lf]\n", count, bal_problem.mutable_camera_for_observation(i)[0], bal_problem.mutable_camera_for_observation(i)[1], bal_problem.mutable_camera_for_observation(i)[2], bal_problem.mutable_camera_for_observation(i)[3], bal_problem.mutable_camera_for_observation(i)[4], bal_problem.mutable_camera_for_observation(i)[5], bal_problem.mutable_camera_for_observation(i)[6], bal_problem.mutable_camera_for_observation(i)[7], bal_problem.mutable_camera_for_observation(i)[8]);
        // printf("mutablepointがあーだこーだ  = #%d [%lf %lf %lf]\n", count, bal_problem.mutable_point_for_observation(i)[0], bal_problem.mutable_point_for_observation(i)[1], bal_problem.mutable_point_for_observation(i)[2]);
        count++;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    // printf("mutablecameraがあーだこーだ  = [%lf %lf %lf %lf %lf %lf %lf %lf %lf]\n", pre_mutable_camera[0], pre_mutable_camera[1], pre_mutable_camera[2], pre_mutable_camera[3], pre_mutable_camera[4], pre_mutable_camera[5], pre_mutable_camera[6], pre_mutable_camera[7], pre_mutable_camera[8]);
    // printf("mutablecameraがあーだこーだ2 = [%lf %lf %lf %lf %lf %lf %lf %lf %lf]\n", bal_problem.mutable_camera_for_observation(0)[0], bal_problem.mutable_camera_for_observation(0)[1], bal_problem.mutable_camera_for_observation(0)[2], bal_problem.mutable_camera_for_observation(0)[3], bal_problem.mutable_camera_for_observation(0)[4], bal_problem.mutable_camera_for_observation(0)[5], bal_problem.mutable_camera_for_observation(0)[6], bal_problem.mutable_camera_for_observation(0)[7], bal_problem.mutable_camera_for_observation(0)[8]);
    // printf("mutablepointがあーだこーだ  = [%lf %lf %lf]\n", bal_problem.mutable_point_for_observation(0)[0], bal_problem.mutable_point_for_observation(0)[1], bal_problem.mutable_point_for_observation(0)[2]);

    //

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
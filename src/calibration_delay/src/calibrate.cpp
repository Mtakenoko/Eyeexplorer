#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <ceres/ceres.h>
#include "ceres/rotation.h"
#include "../include/calibration_delay/calibrate.hpp"
#include "../include/calibration_delay/cost_function.hpp"
#include "../include/calibration_delay/arm.hpp"

Calib_Param::Calib_Param()
    : handle_scene(0),
      scene_counter(0), use_scene_counter(0),
      flag_finish(false), flag_optimize(false), flag_start(false)
{
    std::cout << "Welcome to Calibration node!" << std::endl;

    // Aruco Marker Detectionのdictionary生成
    dictionary_name = cv::aruco::DICT_4X4_50;
    dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
    parameters = cv::aruco::DetectorParameters::create();
    parameters->adaptiveThreshWinSizeMin = 10;
    parameters->adaptiveThreshWinSizeMax = 53;
    parameters->adaptiveThreshWinSizeStep = 5;
    parameters->minMarkerPerimeterRate = 0.5;
    parameters->maxMarkerPerimeterRate = 4.0;
    parameters->polygonalApproxAccuracyRate = 0.05;
    parameters->minOtsuStdDev = 0.4;
    parameters->maxErroneousBitsInBorderRate = 1.;
    parameters->errorCorrectionRate = 1.;
}

void Calib_Param::topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    now_image = frame_image.clone();

    if (flag_start)
        Calib_Param::input_image_data(msg_image);

    if (flag_optimize)
        Calib_Param::optimization();
}

void Calib_Param::topic_callback_joint_(const sensor_msgs::msg::JointState::SharedPtr msg_joint)
{
    if (flag_start)
        Calib_Param::input_joint_data(msg_joint);

    if (flag_optimize)
        Calib_Param::optimization();
}
void Calib_Param::input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    // 取り扱うsceneの決定
    if (handle_scene >= 20)
    {
        handle_scene -= 20;
    }
    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    new_Scene[handle_scene].Image = frame_image.clone();
    scene_image = frame_image.clone();

    // マーカー位置検出
    int detected = Calib_Param::detect_marker(frame_image, &new_Scene[handle_scene].marker);

    if (detected == 1)
    {
        new_Scene[handle_scene].set_image_flag = false;
    }

    handle_scene++;
}

void Calib_Param::input_joint_data(const sensor_msgs::msg::JointState::SharedPtr msg_joint)
{
    for (int i = 0; i < SCENE_NUM; i++)
    {
        if (!new_Scene[i].set_joint_flag)
        {
            Calib_Param::setNewScene();
            return;
        }

        new_Scene[i].joint[new_Scene[i].joint_counter].theta_0 = msg_joint->position[0];
        new_Scene[i].joint[new_Scene[i].joint_counter].theta_1 = msg_joint->position[1];
        new_Scene[i].joint[new_Scene[i].joint_counter].theta_2 = msg_joint->position[3];
        new_Scene[i].joint[new_Scene[i].joint_counter].theta_3 = msg_joint->position[5];
        new_Scene[i].joint[new_Scene[i].joint_counter].theta_4 = msg_joint->position[6];
        new_Scene[i].joint_counter++;

        if (new_Scene[i].joint_counter == 1000)
            new_Scene[i].set_joint_flag = false;
    }
}

void Calib_Param::setNewScene()
{
    for (int i = 0; i < SCENE_NUM; i++)
    {
        if (!new_Scene[i].set_joint_flag && !new_Scene[i].set_image_flag)
        {
            scene.push_back(new_Scene[i]);
            scene_counter++;
            std::cout << "Scene#" << scene_counter << " was push_backed! " << std::endl;
            new_Scene[i].marker.clear();
            new_Scene[i].finish_flag = true;
        }
    }
}

void Calib_Param::optimization()
{
    if (scene_counter < 5)
    {
        std::cout << "More than 5 Scenes are needed for Calibration" << std::endl
                  << "Now: " << scene_counter << "scenes." << std::endl;
        flag_optimize = false;
        return;
    }
    std::cout << "Start Calibration" << std::endl
              << scene_counter << " scenes are used for this calibration" << std::endl
              << "Optimizing..." << std::endl;

    projectPoint();

    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;

    // 角度量
    double time_delay[1];
    time_delay[0] = 0.226;

    // コスト関数生成
    for (auto itr = scene.begin(); itr != scene.end(); itr++)
    {
        // 見つけたマーカーの数だけ処理する
        for (auto marker_itr = itr->marker.begin(); marker_itr != itr->marker.end(); marker_itr++)
        {
            ceres::CostFunction *cost_function = ProjectionErrorCostFuctor::Create((double)marker_itr->Point_Image.x, (double)marker_itr->Point_Image.y,
                                                                                   (double)marker_itr->Position.x, (double)marker_itr->Position.y, (double)marker_itr->Position.z,
                                                                                   itr->joint);
            problem.AddResidualBlock(cost_function, NULL, &time_delay[0]);
        }
    }
    // 制約設定
    problem.SetParameterLowerBound(&time_delay[0], 0, -M_PI / 10);
    problem.SetParameterUpperBound(&time_delay[0], 0, M_PI / 10);

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;
    // options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
    // options.max_num_line_search_step_size_iterations = 100;

    // options.use_explicit_schur_complement = true;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 結果を出力
    std::cout << "Optimizing Finished!!!" << std::endl;
    printf("after: [%lf]\n", time_delay[0]);
    // std::cout << summary.FullReport() << std::endl;
    offset_output = (double *)malloc(sizeof(double));
    offset_output[0] = time_delay[0];

    // 結果をファイルに保存
    Calib_Param::saveOffsetData();

    // 終了処理
    flag_optimize = false;
    flag_finish = true;
    Calib_Param::clear();
}

void Calib_Param::projectPoint()
{
    for (auto itr = scene.begin(); itr != scene.end(); itr++)
    {
        PassiveArm passivearm;
        passivearm.q[0] = itr->joint[0].theta_0;
        passivearm.q[1] = itr->joint[0].theta_1;
        passivearm.q[2] = itr->joint[0].theta_2;
        passivearm.q[3] = itr->joint[0].theta_3;
        passivearm.q[4] = itr->joint[0].theta_4;
        passivearm.forward_kinematics();

        Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                                   Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
        Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;     // 内視鏡姿勢行列
        Ktl::Vector<3> n = endoscope_pose.column(2);                     // 内視鏡の向き
        Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;

        // 見つけたマーカーの数だけ処理する
        for (auto marker_itr = itr->marker.begin(); marker_itr != itr->marker.end(); marker_itr++)
        {
            double point[3]; // マーカーの三次元点(ワールド座標系)
            point[0] = marker_itr->Position.x;
            point[1] = marker_itr->Position.y;
            point[2] = marker_itr->Position.z;

            double pt[3];
            pt[0] = endoscope_pose[0][0] * point[0] + endoscope_pose[1][0] * point[1] + endoscope_pose[2][0] * point[2];
            pt[1] = endoscope_pose[0][1] * point[0] + endoscope_pose[1][1] * point[1] + endoscope_pose[2][1] * point[2];
            pt[2] = endoscope_pose[0][2] * point[0] + endoscope_pose[1][2] * point[1] + endoscope_pose[2][2] * point[2];

            double t[3];
            t[0] = endoscope_pose[0][0] * Ptip[0] + endoscope_pose[1][0] * Ptip[1] + endoscope_pose[2][0] * Ptip[2];
            t[1] = endoscope_pose[0][1] * Ptip[0] + endoscope_pose[1][1] * Ptip[1] + endoscope_pose[2][1] * Ptip[2];
            t[2] = endoscope_pose[0][2] * Ptip[0] + endoscope_pose[1][2] * Ptip[1] + endoscope_pose[2][2] * Ptip[2];

            double xp = (pt[0] - t[0]) / (pt[2] - t[2]);
            double yp = (pt[1] - t[1]) / (pt[2] - t[2]);

            double Point[3];
            for (int i = 0; i < 3; i++)
            {
                Point[i] = endoscope_pose[0][i] * (point[0] - Ptip[0]) + endoscope_pose[1][i] * (point[1] - Ptip[1]) + endoscope_pose[2][i] * (point[2] - Ptip[2]);
            }

            const double focal_x = 396.7;
            const double focal_y = 396.9;
            const double u_x = 163.6;
            const double u_y = 157.1;
            double predicted[2];
            predicted[0] = focal_x * xp + u_x;
            predicted[1] = focal_y * yp + u_y;

            double error[2];
            error[0] = predicted[0] - (double)marker_itr->Point_Image.x;
            error[1] = predicted[1] - (double)marker_itr->Point_Image.x;

            double error_distance = sqrt(error[0] * error[0] + error[1] * error[1]);

            if (error_distance < 100.)
            {
                use_scene_counter++;
            }

            printf("#%d obs:[%d %d], predict[%0.3lf %0.3lf]\n", marker_itr->ID, (int)marker_itr->Point_Image.x, (int)marker_itr->Point_Image.y, predicted[0], predicted[1]);
            printf("points[%0.3lf %0.3lf %0.3lf] arm[%0.3lf %0.3lf %0.3lf]\n", point[0], point[1], point[2], Ptip[0], Ptip[1], Ptip[2]);
            printf("points - arm = [%0.3lf %0.3lf %0.3lf]\n", point[0] - Ptip[0], point[1] - Ptip[1], point[2] - Ptip[2]);
            printf("pt[%0.3lf %0.3lf %0.3lf], t[%0.3lf %0.3lf %0.3lf]\n", pt[0], pt[1], pt[2], t[0], t[1], t[2]);
            printf("Point[%0.3lf %0.3lf %0.3lf]\n", Point[0], Point[1], Point[2]);
            printf("xp,yp = [%0.3lf %0.3lf], focal*xp,yp = [%0.3lf %0.3lf]\n", xp, yp, focal_x * xp, focal_y * yp);
            endoscope_pose.print();
            // printf("rot[%0.3lf %0.3lf %0.3lf %0.3lf %0.3lf]\n", itr->joint[0], itr->joint[1], itr->joint[2], itr->joint[3], itr->joint[4]);
            std::cout << std::endl;
        }
    }
}

int Calib_Param::detect_marker(const cv::Mat &image, std::vector<Marker> *marker)
{
    if (image.empty())
    {
        std::cerr << "Image used for AR marker detection is empty." << std::endl;
        return 0;
    }

    // マーカーの検出
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);

    // マーカーが見つかんなかったら
    if (marker_ids.size() < 1)
    {
        std::cout << "Marker Not Found!" << std::endl;
        return 0;
    }
    std::cout << "I can detect " << marker_ids.size() << " markers!" << std::endl;

    // 検出したマーカーの描画
    marker_image = image.clone();
    cv::aruco::drawDetectedMarkers(marker_image, marker_corners, marker_ids);

    // 検出したマーカーの数だけmarkerに追加
    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        if (marker_ids[i] > 0 && marker_ids[i] < 20)
        {
            Marker new_marker;
            new_marker.ID = marker_ids[i];
            new_marker.setPosition(new_marker.ID);
            new_marker.Point_Image = marker_corners[i][0]; // (注)marker_cornersには4つのコーナー位置が左上から時計回り（左上、右上、右下、左下の順）で格納されている
            std::cout << "marker ID: " << new_marker.ID
                      << " Pos:[" << new_marker.Point_Image.x << ", " << new_marker.Point_Image.y << "]" << std::endl;
            marker->push_back(new_marker);
        }
    }

    // 終了処理
    marker_ids.clear();
    marker_corners.clear();
    return 1;
}

void Calib_Param::setStartFlag()
{
    flag_start = true;
}

void Calib_Param::setCalibrationFlag()
{
    flag_optimize = true;
}

int Calib_Param::getSceneNum()
{
    return scene_counter;
}

int Calib_Param::getUseSceneNum()
{
    return use_scene_counter;
}

void Calib_Param::getNowImage(cv::Mat *image)
{
    *image = now_image.clone();
}

void Calib_Param::getNewSceneImage(cv::Mat *image)
{
    *image = scene_image.clone();
}

void Calib_Param::getNewMarkerImage(cv::Mat *image)
{
    *image = marker_image.clone();
}

bool Calib_Param::getFinishFlag()
{
    return flag_finish;
}

void Calib_Param::clear()
{
    scene.clear();
    for (int i = 0; i < SCENE_NUM; i++)
    {
        new_Scene[i].marker.clear();
    }
    scene_counter = 0;
    flag_finish = true;
}

void Calib_Param::saveOffsetData()
{
    const char *fileName = "/home/takeyama/workspace/ros2_eyeexplorer/src/calibration/Output/offset.txt";

    // ファイル生成
    std::ofstream ofs(fileName);
    if (!ofs)
    {
        std::cout << "ファイルを開くことができませんでした" << std::endl;
        std::cin.get();
        return;
    }
    for (int i = 0; i < ADOF; i++)
        ofs << offset_output[i] << std::endl;
}

int Calib_Param::encoding2mat_type(const std::string &encoding)
{
    if (encoding == "mono8")
    {
        return CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
        return CV_8UC3;
    }
    else if (encoding == "mono16")
    {
        return CV_16SC1;
    }
    else if (encoding == "rgba8")
    {
        return CV_8UC4;
    }
    else if (encoding == "bgra8")
    {
        return CV_8UC4;
    }
    else if (encoding == "32FC1")
    {
        return CV_32FC1;
    }
    else if (encoding == "rgb8")
    {
        return CV_8UC3;
    }
    else
    {
        throw std::runtime_error("Unsupported encoding type");
    }
}

#include <fstream>
#include <omp.h>
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
    // printf("image #%s get\n", msg_image->header.frame_id.c_str());
    if (flag_start)
    {
        Calib_Param::input_image_data(msg_image);
        Calib_Param::setNewScene();
    }

    if (flag_optimize)
        Calib_Param::optimization_without_ceres();
}

void Calib_Param::topic_callback_joint_(const sensor_msgs::msg::JointState::SharedPtr msg_joint)
{
    // printf("joint #%s get\n", msg_joint->header.frame_id.c_str());
    if (flag_start)
    {
        Calib_Param::input_joint_data(msg_joint);
        // Calib_Param::setNewScene();
    }

    if (flag_optimize)
        Calib_Param::optimization_without_ceres();
}
void Calib_Param::input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    // 取り扱うsceneの決定
    this->decideHandleScene();

    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    new_Scene[handle_scene].Image = frame_image.clone();

    // マーカー位置検出
    bool detected = Calib_Param::detect_marker(frame_image, &new_Scene[handle_scene].marker);

    // マーカーが正しく検出された場合だけ
    if (detected)
    {
        new_Scene[handle_scene].set_image_flag = false;
    }
}

void Calib_Param::input_joint_data(const sensor_msgs::msg::JointState::SharedPtr msg_joint)
{
    // すべてのnew_Sceneで検索
    // #pragma omp parallel for
    for (int i = 0; i < SCENE_NUM; i++)
    {
        if (new_Scene[i].set_image_flag && new_Scene[i].set_joint_flag) // 画像も入っていないし角度が全部入りきっていないとき
        {
            // 角度情報を遅らせる必要があるため画像が入る前に角度情報を保持する必要がある
            Joint temp_joint;
            temp_joint.theta_0 = msg_joint->position[0];
            temp_joint.theta_1 = msg_joint->position[1];
            temp_joint.theta_2 = msg_joint->position[3];
            temp_joint.theta_3 = msg_joint->position[5];
            temp_joint.theta_4 = msg_joint->position[6];
            new_Scene[i].joint.push_back(temp_joint);

            if (new_Scene[i].joint.size() == MAX_DELAY_TIME + 1)
            {
                new_Scene[i].joint.erase(new_Scene[i].joint.begin());
                new_Scene[i].set_joint_flag = false;
            }
        }
        else if (!new_Scene[i].set_image_flag && new_Scene[i].set_joint_flag) // 画像は入っているけどまだ角度が全部入りきっていないとき
        {
            // 十分な角度情報を入手できなかったという意味なので、このnew_Sceneは削除する
            new_Scene[i].delete_flag = true;
        }
    }
}

void Calib_Param::decideHandleScene()
{
    while (true)
    {
        if (handle_scene >= SCENE_NUM)
            handle_scene = 0;

        if (new_Scene[handle_scene].marker.size() == 0)
        {
            // printf("Use Handlin No.%d\n", handle_scene);
            break;
        }
        else
        {
            // printf("Handlin No.%d marker is not empty\n", handle_scene);
            handle_scene++;
        }
    }
}

void Calib_Param::setNewScene()
{
    // すべてのnew_Sceneで検索
    // #pragma omp parallel for
    for (int i = 0; i < SCENE_NUM; i++)
    {
        if (!new_Scene[i].set_joint_flag && !new_Scene[i].set_image_flag) // 画像も角度も全部入っているとき
        {
            // 計算対象に追加
            scene.push_back(new_Scene[i]);
            scene_counter++;
            std::cout << "Scene#" << scene_counter << " was push_backed! " << std::endl;

            // new_Scene[i]の初期化
            new_Scene[i].delete_flag = true;
        }

        // std::cout << "new_Scene[" << i << "].set_image_flag = " << new_Scene[i].set_image_flag << " , set_joint" << new_Scene[i].set_joint_flag << std::endl;
        // std::cout << "new_Scene[" << i << "].joint.size() = " << new_Scene[i].joint.size() << std::endl;
        // std::cout << "new_Scene[" << i << "].joint[0] = " << new_Scene[i].joint[0].theta_0 << std::endl;
        // std::cout << "new_Scene[" << i << "].joint[end] = " << new_Scene[i].joint[MAX_DELAY_TIME].theta_0 << std::endl;
        if (new_Scene[i].delete_flag)
            new_Scene[i].clear();
    }
}

bool Calib_Param::detect_marker(const cv::Mat &image, std::vector<Marker> *marker)
{
    if (image.empty())
    {
        std::cerr << "Image used for AR marker detection is empty." << std::endl;
        return false;
    }

    // マーカーの検出
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);

    // マーカーが見つかんなかったら
    if (marker_ids.size() < 1)
    {
        // std::cout << "Marker Not Found!" << std::endl;
        return false;
    }
    // std::cout << "I can detect " << marker_ids.size() << " markers!" << std::endl;

    // 検出したマーカーの描画
    marker_image = image.clone();
    cv::aruco::drawDetectedMarkers(marker_image, marker_corners, marker_ids);

    // 検出したマーカーの数だけmarkerに追加
    bool boolean_detectMarker = false;
    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        if (marker_ids[i] > 0 && marker_ids[i] < 20)
        {
            Marker new_marker;
            new_marker.ID = marker_ids[i];
            new_marker.setPosition(new_marker.ID);
            new_marker.Point_Image = marker_corners[i][0]; // (注)marker_cornersには4つのコーナー位置が左上から時計回り（左上、右上、右下、左下の順）で格納されている
            // std::cout << "marker ID: " << new_marker.ID
            //           << " Pos:[" << new_marker.Point_Image.x << ", " << new_marker.Point_Image.y << "]" << std::endl;
            marker->push_back(new_marker);
            boolean_detectMarker = true;
        }
    }

    // マーカー３次元点を画像平面に再投影した点を表示
    // this->drawReprojectedMarkerPoint();

    // 終了処理
    marker_ids.clear();
    marker_corners.clear();
    return boolean_detectMarker;
}

void Calib_Param::optimization_without_ceres()
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

    // 投影誤差の総和を遅れ時間1msごとに計算しpush_back
    std::vector<float> distance_container;
    std::vector<float> error_container;
    for (int i = 0; i < MAX_DELAY_TIME; i++)
    {
        float error_distance = 0;
        for (auto itr = scene.begin(); itr != scene.end(); itr++)
        {
            // 運動学
            PassiveArm passivearm;
            passivearm.q[0] = (double)itr->joint[i].theta_0;
            passivearm.q[1] = (double)itr->joint[i].theta_1;
            passivearm.q[2] = (double)itr->joint[i].theta_2;
            passivearm.q[3] = (double)itr->joint[i].theta_3;
            passivearm.q[4] = (double)itr->joint[i].theta_4;
            passivearm.forward_kinematics();

            Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                                       Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
            Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;     // 内視鏡姿勢行列
            Ktl::Vector<3> n = endoscope_pose.column(2);                     // 内視鏡の向き
            Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;

            // 見つけたマーカーの数だけ処理する
            for (auto marker_itr = itr->marker.begin(); marker_itr != itr->marker.end(); marker_itr++)
            {
                //　3つめの透視射影
                float point[3]; // マーカーの三次元点(ワールド座標系)
                point[0] = marker_itr->Position.x;
                point[1] = marker_itr->Position.y;
                point[2] = marker_itr->Position.z;

                // カメラ座標系
                float Point[3];
                for (int j = 0; j < 3; j++)
                {
                    Point[j] = (float)endoscope_pose[0][j] * (point[0] - (float)Ptip[0]) + (float)endoscope_pose[1][j] * (point[1] - (float)Ptip[1]) + (float)endoscope_pose[2][j] * (point[2] - (float)Ptip[2]);
                }

                // カメラ正規化座標系
                float xp = Point[0] / Point[2];
                float yp = Point[1] / Point[2];

                // 最終投影位置の計算
                const float focal_x = 396.7;
                const float focal_y = 396.9;
                const float u_x = 163.6;
                const float u_y = 157.1;
                float predicted_x = focal_x * xp + u_x;
                float predicted_y = focal_y * yp + u_y;

                float observed_x = marker_itr->Point_Image.x;
                float observed_y = marker_itr->Point_Image.y;

                float error[2];
                error[0] = predicted_x - observed_x;
                error[1] = predicted_y - observed_y;
                error_distance += sqrt(error[0] * error[0] + error[1] * error[1]);
                // printf("point[%0.1lf %0.1lf %0.1lf], Ptip[%0.1lf %0.1lf %0.1lf]\n", point[0], point[1], point[2], Ptip[0], Ptip[1], Ptip[2]);
                // endoscope_pose.print();
                // printf("Point[%0.1lf %0.1lf %0.1lf], image_point[%0.1lf %0.1lf]\n", Point[0], Point[1], Point[2], xp, yp);
                // printf("obs[%0.1lf %0.1lf], pre[%0.1lf %0.1lf]\n", observed_x, observed_y, predicted_x, predicted_y);
                error_container.push_back(error_distance);
            }
        }
        std::cout << "#" << i << " error_distance: " << error_distance << std::endl;
        distance_container.push_back(error_distance);
        error_distance = 0;
    }

    // 投影誤差の最小値でソート
    auto min_iter = std::min_element(distance_container.begin(), distance_container.end());
    std::cout << "min: " << *min_iter << " ave: " << *min_iter / error_container.size() * MAX_DELAY_TIME << std::endl;
    std::cout << std::distance(distance_container.begin(), min_iter) << std::endl;
    delay = std::distance(distance_container.begin(), min_iter);

    // 結果をファイルに保存
    Calib_Param::saveOffsetData();

    // 終了処理
    flag_optimize = false;
    flag_finish = true;
    flag_start = false;
    Calib_Param::clear();
}

void Calib_Param::drawReprojectedMarkerPoint(const Scene &scene)
{
    // time_delay [ms]だけ /joint_states の時刻を遅らせたときの再投影位置をmarker_imageに書き込む

    // 運動学
    PassiveArm passivearm;
    passivearm.q[0] = scene.joint[DELAY_TIME].theta_0;
    passivearm.q[1] = scene.joint[DELAY_TIME].theta_1;
    passivearm.q[2] = scene.joint[DELAY_TIME].theta_2;
    passivearm.q[3] = scene.joint[DELAY_TIME].theta_3;
    passivearm.q[4] = scene.joint[DELAY_TIME].theta_4;
    passivearm.forward_kinematics();

    Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                               Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
    Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;     // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_pose.column(2);                     // 内視鏡の向き
    Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;

    // 見つけたマーカーの数だけ処理する
    for (auto marker_itr = scene.marker.begin(); marker_itr != scene.marker.end(); marker_itr++)
    {
        cv::Point3f point = marker_itr->Position;

        float Point[3];
        for (int i = 0; i < 3; i++)
            Point[i] = endoscope_pose[0][i] * (point.x - Ptip[0]) + endoscope_pose[1][i] * (point.y - Ptip[1]) + endoscope_pose[2][i] * (point.z - Ptip[2]);

        cv::Point2f im_point;
        im_point.x = Point[0] / Point[2];
        im_point.y = Point[1] / Point[2];

        // 最終投影位置の計算
        const float focal_x = 396.7;
        const float focal_y = 396.9;
        const float u_x = 163.6;
        const float u_y = 157.1;

        cv::Point2f predicted_point;
        predicted_point.x = focal_x * im_point.x + u_x;
        predicted_point.y = focal_y * im_point.y + u_y;
        std::cout << "predict pt:" << predicted_point << std::endl;

        //点（円）を描写する関数
        cv::Scalar color = cv::Scalar(0, 0, 255);
        cv::circle(marker_image, predicted_point, 10, color, 1, cv::LINE_AA);
    }
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
}

void Calib_Param::saveOffsetData()
{
    const char *fileName = "/home/takeyama/workspace/ros2_eyeexplorer/src/calibration_delay/Output/delay.txt";

    // ファイル生成
    std::ofstream ofs(fileName);
    if (!ofs)
    {
        std::cout << "ファイルを開くことができませんでした" << std::endl;
        std::cin.get();
        return;
    }
    ofs << delay;
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

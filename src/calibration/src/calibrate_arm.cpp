#include <rclcpp/rclcpp.hpp>
#include <ceres/ceres.h>
#include "../include/calibrate_arm.hpp"
#include "../include/cost_function.hpp"

void Marker::setPosition(int marker_id)
{
    // マーカーIDを受け取ればそれに対応する三次元位置をPositionに格納する
    switch (marker_id)
    {
    case 1:
        this->Position.x = 100.;
        this->Position.x = 100.;
        this->Position.x = 100.;
        break;

    case 2:
        this->Position.x = 200.;
        this->Position.y = 200.;
        this->Position.z = 200.;
        break;

    case 3:
        this->Position.x = 300.;
        this->Position.y = 300.;
        this->Position.z = 300.;
        break;
    default:
        std::cerr << "marker_id is unsupported" << std::endl;
        break;
    }
}

Calib_Param::Calib_Param()
    : scene_counter(0), flag_set(false), flag_set_image(false), flag_set_joint(false),
      flag_finish(false), flag_optimize(false)
{
    std::cout << "Welcome to Calibration node!" << std::endl;

    // Aruco Marker Detectionのdictionary生成
    dictionary_name = cv::aruco::DICT_4X4_50;
    dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
}

void Calib_Param::topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    now_image = frame_image.clone();

    if (flag_set && flag_set_image)
        Calib_Param::input_image_data(msg_image);

    if (flag_optimize)
        Calib_Param::optimization();
}

void Calib_Param::topic_callback_joint_(const sensor_msgs::msg::JointState::SharedPtr msg_joint)
{
    if (flag_set && flag_set_joint)
        Calib_Param::input_joint_data(msg_joint);

    if (flag_optimize)
        Calib_Param::optimization();
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

void Calib_Param::optimization()
{
    if (scene_counter < 5)
    {
        std::cout << "More than 5 Scenes are needed for Calibration" << std::endl;
        flag_optimize = false;
        return;
    }
    std::cout << "Start Calibration" << std::endl
              << scene_counter << " scenes are used for this calibration" << std::endl
              << "Optimizing..." << std::endl;
    sleep(5);
    std::cout << "Optimizing Finished!!!" << std::endl;
    flag_optimize = false;
    Calib_Param::clear();
    return;

    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;

    // 最適化用パラメータ
    double mutable_link_param[21]; // リンクの長さ
    double mutable_angle_param[5]; // 角度オフセット量

    // リンクパラメータの初期値を代入(シーン毎に変化しない)
    mutable_link_param[0] = 9.;     // [mm]
    mutable_link_param[1] = 0.;     // [mm]
    mutable_link_param[2] = 32.;    // [mm]
    mutable_link_param[3] = 32.32;  // [mm]
    mutable_link_param[4] = 0.;     // [mm]
    mutable_link_param[5] = 17.68;  // [mm]
    mutable_link_param[6] = 200.;   // [mm]
    mutable_link_param[7] = 0.;     // [mm]
    mutable_link_param[8] = 0.;     // [mm]
    mutable_link_param[9] = 41.5;   // [mm]
    mutable_link_param[10] = 0.5;   // [mm]
    mutable_link_param[11] = -19.;  // [mm]
    mutable_link_param[12] = 0.;    // [mm]
    mutable_link_param[13] = 0.;    // [mm]
    mutable_link_param[14] = -200.; // [mm]
    mutable_link_param[15] = 94.9;  // [mm]
    mutable_link_param[16] = 0.;    // [mm]
    mutable_link_param[17] = 0.;    // [mm]
    mutable_link_param[18] = 0.;    // [mm]
    mutable_link_param[19] = 0.;    // [mm]
    mutable_link_param[20] = -110.; // [mm]

    // 角度量とコスト関数生成
    for (auto itr = scene.begin(); itr != scene.end(); itr++)
    {
        for (int i = 0; i < 5; i++)
        {
            mutable_angle_param[i] = itr->joint[i]; // [rad]
        }
        // 見つけたマーカーの数だけ処理する
        for (auto marker_itr = itr->marker.begin(); marker_itr != itr->marker.end(); marker_itr++)
        {
            ceres::CostFunction *cost_function = NewProjectionErrorCostFuctor::Create((double)marker_itr->Point_Image.x, (double)marker_itr->Point_Image.y,
                                                                                      (double)marker_itr->Position.x, (double)marker_itr->Position.y, (double)marker_itr->Position.z);
            problem.AddResidualBlock(cost_function, NULL, mutable_link_param, mutable_angle_param);
        }
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    flag_optimize = false;
    Calib_Param::clear();
    rclcpp::shutdown();
}

void Calib_Param::input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    std::cout << "input image data #" << msg_image->header.frame_id.c_str() << std::endl;
    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    new_Scene.Image = frame_image.clone();

    // マーカー位置検出
    Calib_Param::detect_marker(new_Scene.Image, &new_Scene.marker);

    flag_set_image = false;

    if (!flag_set_joint)
        Calib_Param::setNewScene();
}

void Calib_Param::input_joint_data(const sensor_msgs::msg::JointState::SharedPtr msg_joint)
{
    std::cout << "input joint data" << std::endl;
    // 角度
    for (int i = 0; i < 5; i++)
    {
        new_Scene.joint[i] = msg_joint->position[i];
    }

    flag_set_joint = false;

    if (!flag_set_image)
        Calib_Param::setNewScene();
}

void Calib_Param::setNewScene()
{
    scene.push_back(new_Scene);
    scene_counter++;
    std::cout << "Scene#" << scene_counter << " was push_backed! " << std::endl;
    new_Scene.marker.clear();
    flag_set = false;
}

void Calib_Param::setCaptureFlag()
{
    flag_set = true;
    flag_set_image = true;
    flag_set_joint = true;
}

void Calib_Param::setCalibrationFlag()
{
    flag_optimize = true;
}
void Calib_Param::detect_marker(const cv::Mat &image, std::vector<Marker> *marker)
{
    if (image.empty())
    {
        std::cerr << "Image used for AR marker detection is empty." << std::endl;
        return;
    }

    // マーカーの検出
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);

    // マーカーが見つかんなかったら
    if (marker_ids.size() < 1)
    {
        std::cout << "Marker Not Found!" << std::endl;
        return;
    }

    // 検出したマーカーの描画
    cv::aruco::drawDetectedMarkers(marker_image, marker_corners, marker_ids);

    // 検出したマーカーの数だけmarkerに追加
    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        Marker new_marker;
        new_marker.ID = marker_ids[i];
        new_marker.setPosition(new_marker.ID);
        new_marker.Point_Image = marker_corners[i][0]; // (注)marker_cornersには4つのコーナー位置が左上から時計回り（左上、右上、右下、左下の順）で格納されている
        marker->push_back(new_marker);
    }

    // 終了処理
    marker_ids.clear();
    marker_corners.clear();
}

int Calib_Param::getSceneNum()
{
    return scene_counter;
}

void Calib_Param::getNewSceneImage(cv::Mat *image)
{
    *image = now_image.clone();
}

void Calib_Param::getNewMarkerImage(cv::Mat *image)
{
    *image = marker_image.clone();
}

void Calib_Param::clear()
{
    scene.clear();
    new_Scene.marker.clear();
    scene_counter = 0;
    flag_set = false;
    flag_set_image = false;
    flag_set_joint = false;
    flag_finish = true;
}

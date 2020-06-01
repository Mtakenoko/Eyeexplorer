#include <opencv2/aruco.hpp>
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
    : scene_counter(0), flag_set(false), flag_finish(false), flag_optimize(false)
{
    std::cout << "Welcome to Calibration node!" << std::endl;
}

void Calib_Param::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                  const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_arm)
{
    if (flag_set)
    {
        std::cout << "input data #" << scene_counter << std::endl;
        Calib_Param::input_data(msg_image, msg_arm);
    }
    if (flag_optimize)
    {
        std::cout << "Start Calibration" << std::endl
                  << scene_counter << "'s scenes are used for this calibration" << std::endl
                  << "Optimizing..." << std::endl;
        Calib_Param::optimization();
    }
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
}

void Calib_Param::input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                             const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_joint)
{
    // 新規シーンのオブジェクト生成
    Scene new_scene;

    // 角度
    for (int i = 0; i < 5; i++)
    {
        new_scene.joint[i] = msg_joint->position[i];
    }

    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    new_scene.Image = frame_image.clone();

    // マーカー位置検出
    Calib_Param::detect_marker(new_scene.Image, &new_scene.marker);

    scene.push_back(new_scene);

    // setフラグを下げる
    scene_counter++;
    flag_set = false;
}

void Calib_Param::detect_marker(const cv::Mat &image, std::vector<Marker> *marker)
{
    if (image.empty())
    {
        std::cerr << "Image is empty." << std::endl;
        return;
    }

    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    // マーカーの検出
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);

    // 検出したマーカーの描画
    cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
    cv::imshow("marker_detection", image);
    cv::waitKey(10);

    for (auto itr = marker_ids.begin(); itr != marker_ids.end(); itr++)
    {
        Marker new_marker;
        new_marker.ID = *itr;
        new_marker.setPosition(new_marker.ID);
        //TODO: Point_imageについても格納（ただこのイテレータ内では処理できないのでは…？）
        // マーカーデータのpush_back
        marker->push_back(new_marker);
    }
}

int Calib_Param::getSceneNum()
{
    return scene_counter;
}

cv::Mat Calib_Param::getNewSceneImage()
{
    cv::Mat image = scene.back().Image;
    return image;
}

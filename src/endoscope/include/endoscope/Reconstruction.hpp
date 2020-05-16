#ifndef RECONSTRUCTION_HPP__
#define RECONSTRUCTION_HPP__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "CameraInfo.hpp"
#include "KeyFrame.hpp"
#include "Bundler.hpp"
#include "../../../HTL/include/transform.h"
#include "../../../HTL/include/msg_converter.h"

Transform transform;
Converter converter;

class Reconstruction
{
public:
    Reconstruction();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);

public:
    // std::unique_ptr<FrameDatabase> frame_data = std::make_unique<FrameDatabase>();
    FrameDatabase frame_data;
    FrameDatabase keyframe_data;
    std::vector<FrameDatabase> keyframe_database;
    cv::Mat point3D, point3D_arm, point3D_BA;
    cv::Mat matching_image;

private:
    void initialize();
    int encoding2mat_type(const std::string &encoding);
    std::vector<cv::Point2f> keypoint2Point(std::vector<cv::KeyPoint> kp);
    void input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                    const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm);
    void knn_matching();
    void BF_matching();
    void knn_outlier_remover();
    void BF_outlier_remover();
    void triangulation();
    void triangulation_est();
    void bundler();
    void setFirstFrame();
    void setKeyFrame();
    void chooseKeyFrame();
    bool checkKeyFrame();
    void keyframe_detector();
    void estimate_move();
    void process();
    void showImage();
    void publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);

private:
    const cv::Mat Rotation_eye = cv::Mat::eye(3, 3, CV_32F);
    const cv::Mat Transform_zeros = cv::Mat::zeros(3, 1, CV_32F);
    int num_keyframe;
    bool flag_reconstruction;
    bool flag_setFirstFrame;
    bool flag_showImage;
    bool flag_estimate_move;
    bool isKeyFrame;
    unsigned long int counter;
    const float fovx = 396.7, fovy = 396.9, u0 = 163.6, v0 = 157.1;
    const cv::Mat CameraMat = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0,
                               0.0, fovy, v0,
                               0.0, 0.0, 1.0);

    float threshold_knn_ratio = 0.7f;
    float threshold_ransac = 5.0;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::DMatch> dmatch, inliners_matches;
    std::vector<cv::Point2f> matched_point1, matched_point2;
    size_t match_num;
    cv::Mat cameraMatrix;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat R_est, t_est;
};
#endif

Reconstruction::Reconstruction()
    : frame_data(), keyframe_data()
{
    counter = 0;
    flag_reconstruction = true;
    flag_setFirstFrame = true;
    flag_showImage = true;
    flag_estimate_move = true;
    frame_data.camerainfo.CameraMatrix = CameraMat.clone();

    if (flag_showImage)
    {
        cv::namedWindow("matching_image", cv::WINDOW_AUTOSIZE);
    }
}

void Reconstruction::initialize()
{
    counter++;
    knn_matches.clear();
    dmatch.clear();
    inliners_matches.clear();
    frame_data.extractor.point.clear();
    frame_data.extractor.keypoints.clear();
    keyframe_data.extractor.point.clear();
    keyframe_data.extractor.keypoints.clear();
    matched_point1.clear();
    matched_point2.clear();
    frame_data.camerainfo.CameraMatrix = CameraMat.clone();
}

void Reconstruction::setFirstFrame()
{
    frame_data.camerainfo.Rotation = Rotation_eye.clone();
    frame_data.camerainfo.Transform = Transform_zeros.clone();
    keyframe_database.push_back(frame_data);
}

void Reconstruction::chooseKeyFrame()
{
    if (keyframe_database.size() >= 10)
    {
        for (auto itr = keyframe_database.end() - 1; itr != keyframe_database.begin() - 1; --itr)
        {
            // 判定1: Z方向の変化が少ない
            // カメラ座標の計算
            cv::Mat t_endo = itr->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->camerainfo.Transform_world);
            if (abs(t_endo.at<float>(2)) < 5.)
            {
                // 判定2: xy方向の変化or仰角の変化が一定範囲内にある
                // xy方向の移動量
                cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
                bool moving_xy = cv::norm(t_move_xy) < 8. && cv::norm(t_move_xy) > 2.;

                // 仰角
                float phi = transform.RevFromRotMat(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
                bool moving_phi = abs(phi) < 0.1 && abs(phi) > 0.001;
                if (moving_xy && moving_phi)
                {
                    // printf("あったぞ＾＾(xy: %f, phi: %f)\n", cv::norm(t_move_xy), abs(phi));
                    keyframe_data = *itr;
                    return;
                }
            }
            // printf("t_endo_z = %f\n", abs(t_endo.at<float>(2)));
            // printf("move_xy = %f, phi = %f\n", cv::norm(t_move_xy), abs(phi));
            // printf("戻るぞ\n");
        }
        // 何一つ当てはまるのがなければ
        // 三次元復元は行わず、KFの候補であれば登録だけはする
        printf("一つもなかったよ；；\n");
        flag_reconstruction = false;
        this->setKeyFrame();
        return;
    }
    else
    {
        keyframe_data = keyframe_database[keyframe_database.size() - 1];
    }
}

void Reconstruction::setKeyFrame()
{
    if (this->checkKeyFrame())
    {
        printf("KeyFrame was setted!\n");
        frame_data.camerainfo.Rotation = Rotation_eye.clone();
        frame_data.camerainfo.Transform = Transform_zeros.clone();
        keyframe_database.push_back(frame_data);
    }
}

bool Reconstruction::checkKeyFrame()
{
    // KeyFrameに新たに登録してからのフレーム間隔
    auto itr_endo = keyframe_database.end() - 1;
    int frame_span = frame_data.camerainfo.frame_num - itr_endo->camerainfo.frame_num;

    // xy方向の移動量
    cv::Mat t_move = itr_endo->camerainfo.Rotation_world.t() * frame_data.camerainfo.Transform_world;
    cv::Point2f t_move_xy;
    t_move_xy.x = t_move.at<float>(0);
    t_move_xy.y = t_move.at<float>(1);
    float t_move_norm = cv::norm(t_move_xy);

    // 仰角
    float phi = transform.RevFromRotMat(itr_endo->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);

    if (frame_span > 10 && (t_move_norm > 3. || phi > M_PI / 540.))
        return true;
    return false;
}

int Reconstruction::encoding2mat_type(const std::string &encoding)
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
std::vector<cv::Point2f> Reconstruction::keypoint2Point(std::vector<cv::KeyPoint> kp)
{
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < kp.size(); i++)
    {
        points.push_back(kp[i].pt);
    }
    return points;
}
void Reconstruction::input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm)
{
    // Subscribeした画像
    cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    if (msg_image->encoding == "rgb8")
    {
        cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
    }
    frame_data.extractor.image = frame_image.clone();

    // 運動学で求めたグローバル座標からみたカメラの位置姿勢
    cv::Mat R_frame_world = transform.QuaternionToRotMat(msg_arm->rotation.x, msg_arm->rotation.y, msg_arm->rotation.z, msg_arm->rotation.w);
    cv::Mat t_frame_world = (cv::Mat_<float>(3, 1) << msg_arm->translation.x * 1000., msg_arm->translation.y * 1000., msg_arm->translation.z * 1000.);
    frame_data.camerainfo.Rotation_world = R_frame_world.clone();
    frame_data.camerainfo.Transform_world = t_frame_world.clone();

    // 画像のID情報
    frame_data.camerainfo.frame_num = atoi(msg_image->header.frame_id.c_str());

    // 特徴点検出・特徴量記述
    frame_data.extractor.extractAndcompute(Extractor::DetectorType::AKAZE);
}

void Reconstruction::knn_matching()
{
    if (keyframe_database.empty())
    {
        printf("    keyframe_database is empty!\n");
        return;
    }
    // descriptorはCV_32FじゃないとFLANNでのマッチングはできないらしい
    if (frame_data.extractor.descirptors.type() != CV_32F)
        frame_data.extractor.descirptors.convertTo(frame_data.extractor.descirptors, CV_32F);
    if (keyframe_data.extractor.descirptors.type() != CV_32F)
        keyframe_data.extractor.descirptors.convertTo(keyframe_data.extractor.descirptors, CV_32F);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    matcher->knnMatch(frame_data.extractor.descirptors, keyframe_data.extractor.descirptors, knn_matches, 2);
}

void Reconstruction::BF_matching()
{
    if (keyframe_database.empty())
    {
        printf("    keyframe_database is empty!\n");
        return;
    }
    // descriptorはCV_32FじゃないとFLANNでのマッチングはできないらしい
    if (frame_data.extractor.descirptors.type() != CV_32F)
        frame_data.extractor.descirptors.convertTo(frame_data.extractor.descirptors, CV_32F);
    if (keyframe_data.extractor.descirptors.type() != CV_32F)
        keyframe_data.extractor.descirptors.convertTo(keyframe_data.extractor.descirptors, CV_32F);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    matcher->match(frame_data.extractor.descirptors, keyframe_data.extractor.descirptors, dmatch);
}

void Reconstruction::knn_outlier_remover()
{
    // 誤対応除去①：knnマッチングでなるべく差が大きいものだけを選択
    // 距離が小さいほうがよりマッチング度合いが高い
    std::vector<cv::KeyPoint> matched1_keypoints, matched2_keypoints;
    std::vector<int> matched1_keypoints_idx, matched2_keypoints_idx;
    matched1_keypoints_idx.clear();
    matched2_keypoints_idx.clear();

    if (knn_matches.size() == 0)
    {
        printf("knn_matches.size() == 0\n");
        return;
    }

    std::vector<cv::Point2f> match_point1, match_point2;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < threshold_knn_ratio * knn_matches[i][1].distance)
        {
            match_point1.push_back(frame_data.extractor.keypoints[knn_matches[i][0].queryIdx].pt);
            match_point2.push_back(keyframe_data.extractor.keypoints[knn_matches[i][0].trainIdx].pt);
            dmatch.push_back(knn_matches[i][0]);
        }
    }

    // 誤対応除去②：ホモグラフィ変換を行うときのRANSACを用いる
    cv::Mat homography, inliner_mask;
    std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    std::vector<int> inliners_idx;
    homography = cv::findHomography(match_point1, match_point2, cv::RANSAC, threshold_ransac, inliner_mask);
    inliners_idx.clear();
    for (int i = 0; i < inliner_mask.rows; i++)
    {
        if (inliner_mask.at<uchar>(i))
        {
            // inliners1_keypoints.push_back(matched1_keypoints[i]);
            // inliners2_keypoints.push_back(matched2_keypoints[i]);
            inliners_matches.push_back(dmatch[i]);
            inliners_idx.push_back(i); // インデックスを整理した後にもオリジナルのインデックスを参照できるように保存
            matched_point1.push_back(frame_data.extractor.keypoints[dmatch[i].queryIdx].pt);
            matched_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].trainIdx].pt);
        }
    }
    match_num = inliners_matches.size();
}

void Reconstruction::BF_outlier_remover()
{
    if (dmatch.size() < 5)
    {
        printf("dmatch.size() == %zu\n", dmatch.size());
        return;
    }

    std::vector<cv::Point2f> match_point1, match_point2;
    for (size_t i = 0; i < dmatch.size(); ++i)
    {
        match_point1.push_back(frame_data.extractor.keypoints[dmatch[i].queryIdx].pt);
        match_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].trainIdx].pt);
    }

    // 誤対応除去：ホモグラフィ変換を行うときのRANSACを用いる
    std::vector<int> matched1_keypoints_idx, matched2_keypoints_idx, inliners_idx;
    cv::Mat homography, inliner_mask;
    std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    matched1_keypoints_idx.clear();
    matched2_keypoints_idx.clear();
    inliners_idx.clear();
    homography = cv::findHomography(match_point1, match_point2, inliner_mask, cv::RANSAC, 5.);
    for (int i = 0; i < inliner_mask.rows; i++)
    {
        if (inliner_mask.at<uchar>(i))
        {
            inliners_matches.push_back(dmatch[i]);
            inliners_idx.push_back(i); // インデックスを整理した後にもオリジナルのインデックスを参照できるように保存
            matched_point1.push_back(frame_data.extractor.keypoints[dmatch[i].queryIdx].pt);
            matched_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].trainIdx].pt);
        }
    }
    match_num = inliners_matches.size();
}

void Reconstruction::triangulation()
{
    // カメラ座標の計算
    cv::Mat R_endo = keyframe_data.camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world;
    cv::Mat t_endo = keyframe_data.camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - keyframe_data.camerainfo.Transform_world);
    frame_data.camerainfo.Rotation = R_endo.clone();
    frame_data.camerainfo.Transform = t_endo.clone();
    frame_data.camerainfo.setData();
    keyframe_data.camerainfo.setData();
    frame_data.extractor.match2point_query(inliners_matches);
    keyframe_data.extractor.match2point_train(inliners_matches);

    // 三角測量
    cv::Mat point4D(4, match_num, CV_32FC1);
    cv::Mat M = (cv::Mat_<float>(3, 3) << 1., 0., 0.,
                 0., 1., 0.,
                 0., 0., -1.);
    cv::Mat p3, p3_arm;
    for (size_t i = 0; i < match_num; i++)
    {
        cv::Mat point3D_result, point3D_result_arm;
        cv::triangulatePoints(keyframe_data.camerainfo.ProjectionMatrix, frame_data.camerainfo.ProjectionMatrix,
                              cv::Mat(keyframe_data.extractor.point[i]), cv::Mat(frame_data.extractor.point[i]),
                              point4D);
        cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
        point3D_result_arm = keyframe_data.camerainfo.Rotation_world * M * point3D_result.reshape(1, 3) + keyframe_data.camerainfo.Transform_world;
        p3.push_back(point3D_result);
        p3_arm.push_back(point3D_result_arm.reshape(3, 1));
    }
    point3D = p3.clone();
    point3D_arm = p3_arm.clone();
    // std::cout << "R_frame" << frame_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "t_frame" << frame_data.camerainfo.Transform_world << std::endl;
    // std::cout << "R_endo" << frame_data.camerainfo.Rotation << std::endl;
    // std::cout << "t_endo" << frame_data.camerainfo.Transform << std::endl;
    // std::cout << "Rt_f" << frame_data.camerainfo.CameraPose << std::endl;
    // std::cout << "Rt_kf" << keyframe_data.camerainfo.CameraPose << std::endl;
    // std::cout << "Prj_kf" << keyframe_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "Prj_f" << frame_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "p1" << cv::Mat(keyframe_data.extractor.point) << std::endl;
    // std::cout << "R_keyframe" << keyframe_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "t_keyframe" << keyframe_data.camerainfo.Transform_world << std::endl;
    // std::cout << "point1" << frame_data.extractor.point << std::endl;
}

void Reconstruction::triangulation_est()
{
    // カメラ座標の計算
    cv::Mat R_endo, t_endo;
    R_est.convertTo(R_endo, CV_32FC1);
    t_est.convertTo(t_endo, CV_32FC1);
    frame_data.camerainfo.Rotation = R_endo.clone();
    frame_data.camerainfo.Transform = t_endo.clone();
    frame_data.camerainfo.setData();
    keyframe_data.camerainfo.setData();
    frame_data.extractor.match2point_query(inliners_matches);
    keyframe_data.extractor.match2point_train(inliners_matches);

    // 三角測量
    cv::Mat point4D(4, match_num, CV_32FC1);
    cv::Mat M = (cv::Mat_<float>(3, 3) << 1., 0., 0.,
                 0., 1., 0.,
                 0., 0., -1.);
    point3D.zeros(match_num, 1, CV_32F);
    point3D_arm.zeros(match_num, 1, CV_32F);
    for (size_t i = 0; i < match_num; i++)
    {
        cv::Mat point3D_result, point3D_result_arm;
        cv::triangulatePoints(keyframe_data.camerainfo.ProjectionMatrix, frame_data.camerainfo.ProjectionMatrix,
                              cv::Mat(keyframe_data.extractor.point[i]), cv::Mat(frame_data.extractor.point[i]),
                              point4D);
        cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
        point3D_result_arm = keyframe_data.camerainfo.Rotation_world * M * point3D_result.reshape(1, 3) + keyframe_data.camerainfo.Transform_world;
        point3D.push_back(point3D_result);
        point3D_arm.push_back(point3D_result_arm.reshape(3, 1));
    }
    // std::cout << "R_frame" << frame_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "t_frame" << frame_data.camerainfo.Transform_world << std::endl;
    // std::cout << "R_endo" << frame_data.camerainfo.Rotation << std::endl;
    // std::cout << "t_endo" << frame_data.camerainfo.Transform << std::endl;
    // std::cout << "Rt_f" << frame_data.camerainfo.CameraPose << std::endl;
    // std::cout << "Rt_kf" << keyframe_data.camerainfo.CameraPose << std::endl;
    // std::cout << "Prj_kf" << keyframe_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "Prj_f" << frame_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "p1" << cv::Mat(keyframe_data.extractor.point) << std::endl;
    // std::cout << "R_keyframe" << keyframe_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "t_keyframe" << keyframe_data.camerainfo.Transform_world << std::endl;
    // std::cout << "point1" << frame_data.extractor.point << std::endl;
}

void Reconstruction::bundler()
{
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;
    //バンドル調整用パラメータ
    double mutable_camera_for_observations[2][6];
    double **mutable_point_for_observations = new double *[match_num * 2];
    for (size_t i = 0; i < match_num * 2; i++)
    {
        mutable_point_for_observations[i] = new double[3];
    }

    cv::Mat rvec_keyframe, rvec_frame;
    cv::Rodrigues(keyframe_data.camerainfo.Rotation_world, rvec_keyframe);
    cv::Rodrigues(frame_data.camerainfo.Rotation_world, rvec_frame);

    //KeyFrameの方の情報
    mutable_camera_for_observations[0][0] = (double)rvec_keyframe.at<float>(0);
    mutable_camera_for_observations[0][1] = (double)rvec_keyframe.at<float>(1);
    mutable_camera_for_observations[0][2] = (double)rvec_keyframe.at<float>(2);
    mutable_camera_for_observations[0][3] = (double)keyframe_data.camerainfo.Transform_world.at<float>(0);
    mutable_camera_for_observations[0][4] = (double)keyframe_data.camerainfo.Transform_world.at<float>(1);
    mutable_camera_for_observations[0][5] = (double)keyframe_data.camerainfo.Transform_world.at<float>(2);
    //Frameの方の情報
    mutable_camera_for_observations[1][0] = (double)rvec_frame.at<float>(0);
    mutable_camera_for_observations[1][1] = (double)rvec_frame.at<float>(1);
    mutable_camera_for_observations[1][2] = (double)rvec_frame.at<float>(2);
    mutable_camera_for_observations[1][3] = (double)frame_data.camerainfo.Transform_world.at<float>(0);
    mutable_camera_for_observations[1][4] = (double)frame_data.camerainfo.Transform_world.at<float>(1);
    mutable_camera_for_observations[1][5] = (double)frame_data.camerainfo.Transform_world.at<float>(2);

    for (size_t i = 0; i < match_num; i++)
    {
        mutable_point_for_observations[i][0] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[0];
        mutable_point_for_observations[i][1] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[1];
        mutable_point_for_observations[i][2] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[2];
        mutable_point_for_observations[i + match_num][0] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[0];
        mutable_point_for_observations[i + match_num][1] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[1];
        mutable_point_for_observations[i + match_num][2] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[2];

        //コスト関数
        ceres::CostFunction *cost_function_train = SnavelyReprojectionError::Create((double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.x,
                                                                                    (double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.y);
        ceres::CostFunction *cost_function_query = SnavelyReprojectionError::Create((double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.x,
                                                                                    (double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.y);

        problem.AddResidualBlock(cost_function_train, NULL, mutable_camera_for_observations[0], mutable_point_for_observations[i]);
        problem.AddResidualBlock(cost_function_query, NULL, mutable_camera_for_observations[1], mutable_point_for_observations[i + match_num]);
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //レポートなど出力
    // std::cout << summary.FullReport() << "\n";
    // std::cout << "match_num:" << match_num << std::endl;
    // std::cout << "rvec_x:" << rvec_frame.at<float>(0) << "->" << mutable_camera_for_observations[1][0] << std::endl;
    // std::cout << "rvec_y:" << rvec_frame.at<float>(1) << "->" << mutable_camera_for_observations[1][1] << std::endl;
    // std::cout << "rvec_z:" << rvec_frame.at<float>(2) << "->" << mutable_camera_for_observations[1][2] << std::endl;
    // std::cout << "x     :" << t_frame.at<float>(0) << "->" << mutable_camera_for_observations[1][3] << std::endl;
    // std::cout << "y     :" << t_frame.at<float>(1) << "->" << mutable_camera_for_observations[1][4] << std::endl;
    // std::cout << "z     :" << t_frame.at<float>(2) << "->" << mutable_camera_for_observations[1][5] << std::endl;

    // std::cout << "point_x:" << point3D_arm.at<cv::Vec3f>(0, 0)[0] << "->" << mutable_point_for_observations[0][0] << std::endl;
    // std::cout << "point_y:" << point3D_arm.at<cv::Vec3f>(0, 0)[1] << "->" << mutable_point_for_observations[0][1] << std::endl;
    // std::cout << "point_z:" << point3D_arm.at<cv::Vec3f>(0, 0)[2] << "->" << mutable_point_for_observations[0][2] << std::endl;

    cv::Mat p3_BA;
    for (size_t i = 0; i < 2 * match_num; ++i)
    {
        cv::Point3f p_xyz;
        p_xyz.x = (float)mutable_point_for_observations[i][0];
        p_xyz.y = (float)mutable_point_for_observations[i][1];
        p_xyz.z = (float)mutable_point_for_observations[i][2];
        p3_BA.push_back(p_xyz);
    }
    point3D_BA = p3_BA.clone();

    /// delete
    for (size_t i = 0; i < match_num * 2; i++)
    {
        delete[] mutable_point_for_observations[i];
        mutable_point_for_observations[i] = 0;
    }
    delete[] mutable_point_for_observations;
    mutable_point_for_observations = 0;
}

void Reconstruction::estimate_move()
{
    // 眼球の移動を検知したらフラグを管理する
    if (!flag_estimate_move)
        return;

    // 5点アルゴリズムやる以上、5点以上の点が必要
    if (matched_point1.size() < 5)
    {
        return;
    }
    // 5点アルゴリズム//１つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat essential_mask;
    cv::Point2f principle_point(u0, v0);
    cv::Mat EssentialMat = cv::findEssentialMat(matched_point1, matched_point2, (fovx + fovy) / 2., principle_point, cv::RANSAC, 0.9999, 0.003, essential_mask);
    cv::recoverPose(EssentialMat, matched_point1, matched_point2, R_est, t_est, (fovx + fovy) / 2., principle_point, essential_mask);
}

void Reconstruction::showImage()
{
    // マッチングの様子を図示
    cv::Scalar match_line_color = cv::Scalar(255, 0, 0);
    cv::Scalar match_point_color = cv::Scalar(255, 255, 0);
    cv::drawMatches(frame_data.extractor.image, frame_data.extractor.keypoints,
                    keyframe_data.extractor.image, keyframe_data.extractor.keypoints,
                    inliners_matches, matching_image, match_line_color, match_point_color);

    // カメラのuv方向への移動量を矢印で追加で図示
    cv::Point2f image_center = cv::Point2f(frame_data.extractor.image.rows / 2., frame_data.extractor.image.cols / 2.);
    cv::Point2f image_center2 = cv::Point2f(frame_data.extractor.image.rows * 3. / 2., frame_data.extractor.image.cols / 2.);
    cv::Scalar color_arrow = cv::Scalar(0, 255, 0);
    cv::Point2f center_t_arm = cv::Point2f(frame_data.camerainfo.Transform.at<float>(0) * 20 + image_center.x,
                                           frame_data.camerainfo.Transform.at<float>(1) * 20 + image_center.y);
    cv::Point2f center_t_arm_z = cv::Point2f(image_center2.x,
                                             frame_data.camerainfo.Transform.at<float>(2) * 20 + image_center.y);
    cv::arrowedLine(matching_image, image_center, center_t_arm, color_arrow, 2, 8, 0, 0.5);
    cv::arrowedLine(matching_image, image_center2, center_t_arm_z, color_arrow, 2, 8, 0, 0.5);

    // 表示
    cv::imshow("matching_image", matching_image);
    cv::waitKey(5);
}

void Reconstruction::publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    if (flag_setFirstFrame)
    {
        flag_setFirstFrame = false;
        return;
    }

    // cv::Mat pointCloud(match_num, 1, CV_32FC3);
    // point3D /= 1000.;
    // point3D.convertTo(pointCloud, CV_32FC3);
    // auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // converter.cvMat_to_msgPointCloud2(pointCloud, *msg_cloud_pub, 0);
    // pub_pointcloud->publish(std::move(msg_cloud_pub));

    // cv::Mat pointCloud_arm(match_num, 1, CV_32FC3);
    // point3D_arm /= 1000.;
    // point3D_arm.convertTo(pointCloud_arm, CV_32FC3);
    // auto msg_cloud_arm_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // converter.cvMat_to_msgPointCloud2(pointCloud_arm, *msg_cloud_arm_pub, 0);
    // pub_pointcloud->publish(std::move(msg_cloud_arm_pub));

    cv::Mat pointCloud_BA(match_num, 1, CV_32FC3);
    point3D_BA /= 1000.;
    point3D_BA.convertTo(pointCloud_BA, CV_32FC3);
    auto msg_cloud_BA_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    converter.cvMat_to_msgPointCloud2(pointCloud_BA, *msg_cloud_BA_pub, 0);
    pub_pointcloud->publish(std::move(msg_cloud_BA_pub));
}

void Reconstruction::process()
{
    flag_reconstruction = true;

    if (flag_setFirstFrame)
    {
        this->setFirstFrame();
        printf("FirstFrame was setted!\n");
        return;
    }

    // どのKFを使うか選択しkeyframe_dataに入力
    this->chooseKeyFrame();

    // 現在のフレームがKeyFrameの候補であれば
    this->setKeyFrame();

    if (!flag_reconstruction)
        return;

    // 特徴点マッチング
    this->BF_matching();

    // 誤対応除去
    this->BF_outlier_remover();

    if (match_num < 5)
        return;

    // もし眼球移動を検知すれば
    this->estimate_move();

    // 三角測量
    this->triangulation();
    // this->triangulation_est();

    // バンドル調整
    this->bundler();

    // 図示
    this->showImage();
}

void Reconstruction::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                     const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    printf("Received image #%s\n", msg_image->header.frame_id.c_str());
    // 初期化
    this->initialize();
    // Subscribeしたものをframe_dataに入力
    this->input_data(msg_image, msg_arm);
    // メインの処理
    this->process();
    // Publish
    this->publish(pub_pointcloud);
}